"""Per-packet extraction of the UCT rugby field runs (as tests/test_analysis.m), for channel calibration."""
import csv, math, os, sys
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tests", "uct rugby field")
COLS = ['ms', 'event', 'src', 'date', 'time', 'lat', 'lon', 'alt', 'speed', 'course', 'status', 'tasks',
        'battery', 'msg', 'seq', 'srcRssi', 'pktRssi', 'snr', 'freqErr', 'utc', 'air', 'sync', 'ttl']


def tod(h):
    if h == 2000000:
        return math.nan
    return (h // 1000000) * 3600 + (h // 10000) % 100 * 60 + (h % 10000) / 100


def read_vanet(path):
    rows = []
    with open(path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) != 23:
                continue
            try:
                r = {k: (p[i] if k == 'event' else float(p[i])) for i, k in enumerate(COLS)}
            except ValueError:
                continue
            r['t'] = tod(int(r['time']))
            if r['sync'] > 0 and r['utc'] > 0:
                r['t'] = (r['utc'] / 1e6 + 7200) % 86400
            if r['lat'] == 0 and r['lon'] == 0:
                r['lat'] = r['lon'] = math.nan
            rows.append(r)
    return rows


def read_sensor(path):
    t, lat, lon = [], [], []
    with open(path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) < 17:
                continue
            try:
                tt = tod(int(p[2])); la = float(p[3]); lo = float(p[4]); sats = float(p[8])
            except ValueError:
                continue
            if sats > 0 and not (la == 0 and lo == 0) and not math.isnan(tt):
                t.append(tt); lat.append(la); lon.append(lo)
    t = np.array(t); o = np.argsort(t, kind='stable'); t = t[o]
    u, iu = np.unique(t, return_index=True)
    return u, np.array(lat)[o][iu], np.array(lon)[o][iu]


def local(lat, lon, lat0, lon0):
    R = 6371000.0
    return math.radians(lon - lon0) * math.cos(math.radians(lat0)) * R, math.radians(lat - lat0) * R


def link(run_dir, sid, rid):
    """Packets sent by sid in the run, received by rid (full logs)? Returns list of dicts."""
    Vs = read_vanet(os.path.join(run_dir, f'vanet_log{sid}.txt'))
    Vr = read_vanet(os.path.join(ROOT, f'vanet_log{rid}.txt'))
    St, Slat, Slon = read_sensor(os.path.join(ROOT, f'sensor_log{rid}.txt'))
    tx, seen = [], set()
    for r in Vs:
        if r['event'] == 'TX' and r['src'] == sid and r['seq'] not in seen:
            seen.add(r['seq']); tx.append(r)
    t0, t1 = min(r['t'] for r in tx), max(r['t'] for r in tx)
    rx = {}
    for r in Vr:
        if r['event'] == 'RX' and r['src'] == sid and t0 - 5 <= r['t'] <= t1 + 60 and r['seq'] not in rx:
            rx[r['seq']] = r
    # receiver's own transmissions (half duplex): [start, end] in s
    rtx = [(r['t'], r['t'] + r['air'] / 1e6) for r in Vr if r['event'] in ('TX', 'FW') and r['sync'] > 0
           and t0 - 5 <= r['t'] <= t1 + 5]
    out = []
    for r in tx:
        d = math.nan
        if len(St) >= 2 and not math.isnan(r['lat']):
            i = int(np.argmin(np.abs(St - r['t'])))
            if abs(St[i] - r['t']) <= 5:
                e, n = local(r['lat'], r['lon'], Slat[i], Slon[i])
                d = math.hypot(e, n)
        s, e_ = r['t'], r['t'] + r['air'] / 1e6
        overlap = any(a < e_ and b > s for a, b in rtx)
        g = rx.get(r['seq'])
        out.append(dict(seq=int(r['seq']), t=r['t'], dist=d, delivered=g is not None,
                        rssi=g['pktRssi'] if g else math.nan, snr=g['snr'] if g else math.nan,
                        overlap=overlap, synced=r['sync'] > 0, air=r['air']))
    return out


if __name__ == '__main__':
    allp = []
    for run in ('run 1', 'run 2', 'run 3'):
        for sid, rid in ((2, 1), (1, 2)):
            L = link(os.path.join(ROOT, run), sid, rid)
            for p in L:
                p['run'] = run; p['dir'] = f'{sid}->{rid}'
            allp += L
            n = len(L); k = sum(p['delivered'] for p in L)
            ov = sum(p['overlap'] for p in L)
            print(f'{run} {sid}->{rid}: {k}/{n} = {100*k/n:.1f}%  overlapped own Tx: {ov}  '
                  f'max rx dist {max([p["dist"] for p in L if p["delivered"]] or [0]):.1f}')
    with open(sys.argv[1] if len(sys.argv) > 1 else 'packets.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(allp[0].keys()))
        w.writeheader(); w.writerows(allp)
