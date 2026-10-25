#!/usr/bin/env python3
"""Tables for the stationary TVV configs (tvv.ini: TvvValidation*, SMO1, SMO1BigCluster, SMO2).

    python3 tvvstats.py validation results/TvvValidation-*.sca
    python3 tvvstats.py sweep N results/SMO1-*.sca          # N = iteration variable of the sweep
    python3 tvvstats.py sweep B results/SMO2-*.sca

Each run (seed) gives one value per metric (mean over its nodes or totals over the network); the table shows the
mean over seeds and the 95 % confidence interval (Student t, seeds - 1 degrees of freedom).
"""
import collections, math, re, shlex, sys
from scipy.stats import t as student_t


def read_sca(path):
    itervars, scalars = {}, collections.defaultdict(dict)
    with open(path) as f:
        for line in f:
            if line.startswith('itervar '):
                _, k, v = line.split(None, 2)
                itervars[k] = v.strip().strip('"')
            elif line.startswith('attr repetition '):
                itervars['#'] = line.split()[2].strip('"')
            elif line.startswith('scalar '):
                _, module, name, value = shlex.split(line)
                try:
                    scalars[module][name] = float(value)
                except ValueError:
                    scalars[module][name] = math.nan
    return itervars, scalars


def node_index(module):
    m = re.search(r'\.node\[(\d+)\]', module)
    return int(m.group(1)) if m else None


def per_node(scalars):
    nodes = collections.defaultdict(dict)
    for module, vals in scalars.items():
        i = node_index(module)
        if i is None:
            continue
        part = 'mac' if module.endswith('.mac') else 'routing' if module.endswith('.routing') else 'radio' if '.radio' in module else None
        if part:
            for k, v in vals.items():
                nodes[i][part + '.' + k] = v
    return [nodes[i] for i in sorted(nodes)]


def ci(values):
    v = [x for x in values if not math.isnan(x)]
    n = len(v)
    if n == 0:
        return math.nan, math.nan, 0
    m = sum(v) / n
    if n < 2:
        return m, math.nan, n
    s = math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1))
    return m, student_t.ppf(0.975, n - 1) * s / math.sqrt(n), n


def wmean(pairs):
    """Mean of per-node means weighted by their counts: (mean, count) pairs."""
    tot = sum(c for m, c in pairs if c > 0 and not math.isnan(m))
    return sum(m * c for m, c in pairs if c > 0 and not math.isnan(m)) / tot if tot else math.nan


def run_metrics(nodes):
    N = len(nodes)
    g = lambda n, k: n.get(k, math.nan)
    pdr_rx = []
    for n in nodes:
        heard, lost = g(n, 'routing.heardDirect:sum'), g(n, 'routing.seqLost:sum')
        pdr_rx.append(100 * heard / (heard + lost) if heard + lost > 0 else math.nan)
    pdr_rx = [x for x in pdr_rx if not math.isnan(x)]
    heard = sum(g(n, 'routing.heardDirect:sum') for n in nodes)
    lost = sum(g(n, 'routing.seqLost:sum') for n in nodes)
    rx_ok = sum(g(n, 'mac.rxOk:sum') for n in nodes)
    rx_crc = sum(g(n, 'mac.rxLost:sum') for n in nodes)
    frames = sum(g(n, 'mac.frameSent:count') for n in nodes)
    fwd = sum(g(n, 'mac.frameSent:sum') for n in nodes)
    addr_sent = sum(g(n, 'routing.addressedSent:sum') for n in nodes)
    addr_rx = sum(g(n, 'routing.addressedReceived:sum') for n in nodes)
    addr_u = sum(g(n, 'routing.addressedUnique:sum') for n in nodes)
    own = frames - fwd
    return {
        'PDR per receiver (%)': sum(pdr_rx) / len(pdr_rx) if pdr_rx else math.nan,
        'PDR worst receiver (%)': min(pdr_rx) if pdr_rx else math.nan,
        'Frames lost, of expected (%)': 100 * lost / (heard + lost) if heard + lost else math.nan,
        'CRC errors (collisions) / node / h': rx_crc / N,
        'CRC errors, % of RxDone': 100 * rx_crc / (rx_ok + rx_crc) if rx_ok + rx_crc else math.nan,
        'Channel access delay (s)': wmean([(g(n, 'mac.channelAccessDelay:mean'), g(n, 'mac.frameSent:count')) for n in nodes]),
        'Channel access delay max (s)': max(g(n, 'mac.channelAccessDelay:max') for n in nodes),
        'Tx interval (s)': wmean([(g(n, 'mac.txInterval:mean'), g(n, 'mac.frameSent:count')) for n in nodes]),
        'Tx interval max (s)': max(g(n, 'mac.txInterval:max') for n in nodes),
        'Frames sent / node / h': frames / N,
        'Forwards, % of frames': 100 * fwd / frames if frames else math.nan,
        'Own beacon interval (s)': MEASURED_S * N / own if own else math.nan,
        'E2E delay direct (s)': wmean([(g(n, 'routing.beaconDelayDirect:mean'), g(n, 'routing.beaconDelayDirect:count')) for n in nodes]),
        'E2E delay forwarded copy (s)': wmean([(g(n, 'routing.forwardedCopyDelay:mean'), g(n, 'routing.forwardedCopyDelay:count')) for n in nodes]),
        'E2E delay forwarded copy max (s)': max(g(n, 'routing.forwardedCopyDelay:max') for n in nodes),
        'Forwarded copies at dest / addressed pkt': sum(g(n, 'routing.forwardedCopyDelay:count') for n in nodes) / addr_sent if addr_sent else math.nan,
        'Delivered first via forward / node / h': sum(g(n, 'routing.beaconDelayForwarded:count') for n in nodes) / N,
        'Addressed delivered, unique (%)': 100 * addr_u / addr_sent if addr_sent else math.nan,
        'Addressed read again (dup leak) / addr pkt': (addr_rx - addr_u) / addr_sent if addr_sent else math.nan,
        'Peers in cluster (mean)': sum(g(n, 'routing.clusterSize:mean') for n in nodes) / N,
        'Peers in cluster (max)': max(g(n, 'routing.clusterSize:max') for n in nodes),
        'Cluster full / node / h': sum(g(n, 'routing.clusterFull:sum') for n in nodes) / N,
        'Peers added / node / h': sum(g(n, 'routing.peerAdded:sum') for n in nodes) / N,
        'Peers removed / node / h': sum(g(n, 'routing.peerRemoved:sum') for n in nodes) / N,
    }


def sweep(var, files):
    groups = collections.defaultdict(list)
    for p in files:
        iv, sc = read_sca(p)
        groups[iv.get(var, '?')].append(run_metrics(per_node(sc)))
    keys = sorted(groups, key=lambda k: float(re.sub(r'[^\d.]', '', k) or 0))
    # counts per hour of the measured period: scaled by the caller's period (see PER_H)
    metrics = list(next(iter(groups.values()))[0].keys())
    print(f'{"metric":38s}' + ''.join(f'{var + "=" + k:>22s}' for k in keys))
    for m in metrics:
        row = f'{m:38s}'
        for k in keys:
            mean, h, n = ci([r[m] for r in groups[k]])
            scale = PER_H if '/ h' in m else 1
            row += f'{fmt(mean * scale, h * scale):>22s}'
        print(row)
    print(f'{"seeds":38s}' + ''.join(f'{len(groups[k]):>22d}' for k in keys))


def fmt(m, h):
    if math.isnan(m):
        return 'n/a'
    d = 3 if abs(m) < 10 else 2 if abs(m) < 100 else 1
    return f'{m:.{d}f} ± {h:.{d}f}' if not math.isnan(h) else f'{m:.{d}f}'


def validation(files):
    groups = collections.defaultdict(list)
    for p in files:
        iv, sc = read_sca(p)
        nodes = per_node(sc)
        a, b = nodes
        # PDR of each direction: frames heard / beacons the other node sent
        for rx, tx in ((a, b), (b, a)):
            groups[float(re.sub(r'[^\d.]', '', iv['D']))].append(
                (rx['routing.heardDirect:sum'], tx['routing.beaconsSent'], rx.get('routing.rxPower:mean', math.nan)))
    print(f'{"D (m)":>8s} {"PDR (%)":>16s} {"pooled":>8s} {"heard/sent":>12s} {"mean Pr (dBm)":>14s}')
    for d in sorted(groups):
        g = groups[d]
        mean, h, n = ci([100 * x / y for x, y, _ in g])
        hs, ss = sum(x for x, _, _ in g), sum(y for _, y, _ in g)
        prs = [p for x, _, p in g if x > 0 and not math.isnan(p)]
        print(f'{d:8.0f} {fmt(mean, h):>16s} {100 * hs / ss:8.1f} {f"{hs:.0f}/{ss:.0f}":>12s} '
              f'{(sum(prs) / len(prs)) if prs else math.nan:14.1f}')


PER_H = 1.0
MEASURED_S = 3300.0
if __name__ == '__main__':
    if sys.argv[1] == 'validation':
        validation(sys.argv[2:])
    else:
        # counts are per measured period: scale to per hour (sim-time-limit - warmup-period, 3300 s by default)
        MEASURED_S = float(__import__('os').environ.get('TVV_MEASURED_S', 3300))
        PER_H = 3600.0 / MEASURED_S
        sweep(sys.argv[2], sys.argv[3:])
