"""
TUKZIE V2X VANET (TVV) - SYNTHETIC TEST LOGS

Builds SYNTHETIC logs (not measured) from the real logs in tests/, as the current node firmware (and the current
simulated host, HOST_MSG = 4) would have logged the same tests:

  gw parking lot/run 1, run 2 (from tests/gw parking lot)
    Bad terrain (code 3): the current firmware flags |az - 1 g| >= TERRAIN_G (0.8 g) at ROAD_MIN_SPEED (5 km/h) or more,
    sampled at 100 Hz. The sensor log only holds a 1 s snapshot, which misses most of a jolt's peak, so each jolt is
    scaled up to JOLT_G in its logged sample:
      - samples already showing |az - 1 g| >= LIKELY_JOLT_G (a real jolt, under-recorded), and
      - in run 1, the largest sample before each beacon the old firmware flagged with a terrain code (4, potholes)
    Every beacon after a jolt carries code 3 (otherwise 0), and the sensor log's message column is the code in the last
    beacon (vanet.lastMsg). Run 2 had no VANET log: its beacons are made every BEACON_PERIOD_S from the sensor log.

  shd test (from tests/shd test)
    The host sends code 4 (serial monitor "m=5" -> "m=4"). Each node's myself log holds 4 from when its host sends it
    until its next beacon takes it (current main.cpp), that beacon carries 4, and low battery (code 2) is flagged while
    the host's battery is <= LOW_BATTERY_THRESHOLD. The codes are carried into the peer's RX/FW lines, the sensor logs
    (last beacon's code) and the serial monitor's RX lines (node's last beacon, and each peer's packet).

Everything else (times, positions, RSSI, status, tasks, battery, forwards) is the real data.

Run: python -I make_synthetic.py   (writes next to this script)
"""
import os, re, bisect

HERE = os.path.dirname(os.path.abspath(__file__))
REAL = os.path.dirname(HERE)

# Current firmware (system.h)
TERRAIN_G = 0.8
ROAD_MIN_SPEED = 5.0
LOW_BATTERY_THRESHOLD = 50
HOST_MSG = 4
TERRAIN_MSG = 3
LOW_BATTERY_MSG = 2
AIR_S = 1.483                   # SF12 airtime
BEACON_PERIOD_S = 15.0 + AIR_S  # BEACON_INTVL from the end of one transmission to the next

# Synthesis
LIKELY_JOLT_G = 0.5             # |az - 1 g| in a 1 s snapshot treated as a real jolt
JOLT_G = 0.9                    # |az - 1 g| written for a jolt (over TERRAIN_G)
JOLT_G_AT = {('run 1', '13545900'): 0.85}   # per-jolt |az - 1 g| (run, sensor time), still over TERRAIN_G
TRIM = {'run 1': '13:54:00'}    # run start (SAST): lines before it are left out (run 1: the az dip at 13:53:36)
OLD_TERRAIN_CODES = (3, 4)      # terrain codes of the firmware used for the real tests


def tod(t):
    t = int(t)
    return t // 1000000 * 3600 + t // 10000 % 100 * 60 + t % 10000 / 100


def hhmmsscc(s):
    s = round(s * 100)
    return f'{s // 360000:02d}{s // 6000 % 60:02d}{s // 100 % 60:02d}{s % 100:02d}'


def read(path):
    """Lines as [fields, line ending]"""
    with open(path, newline='') as fh:
        out = []
        for line in fh:
            body = line.rstrip('\r\n')
            out.append([body.split(','), line[len(body):]])
        return out


def write(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as fh:
        fh.writelines(','.join(f) + end for f, end in rows)


def tx_time(f):
    """VANET TX start (SAST s): utc when synced, else the line's time"""
    if int(f[21]) > 0 and int(f[19]) > 0:
        return (int(f[19]) / 1e6 + 7200) % 86400
    return tod(f[4])


# ---- gw parking lot ----------------------------------------------------------------------------------------------

def jolt(f, sign, g):
    """Scales a sensor line's sample into a jolt of g (vertical), with the lateral axes doubled"""
    f[12] = f'{1 + sign * g:.3f}'
    f[10] = f'{float(f[10]) * 2:.3f}'
    f[11] = f'{float(f[11]) * 2:.3f}'


def gw_run(name):
    src = os.path.join(REAL, 'gw parking lot', name)
    dst = os.path.join(HERE, 'gw parking lot', name)
    S = read(os.path.join(src, 'sensor_log1.txt'))
    V = read(os.path.join(src, 'vanet_log1.txt'))
    M = read(os.path.join(src, 'myself_log1.txt'))
    if name in TRIM:
        h, m, s = (int(x) for x in (TRIM[name] + ':0').split(':')[:3])
        start = h * 3600 + m * 60 + s
        S = [r for r in S if tod(r[0][2]) >= start]
        V = [r for r in V if tx_time(r[0]) >= start]
        M = [r for r in M if tod(r[0][3]) >= start]
    st = [tod(f[2]) for f, _ in S]

    # Beacons: the real ones, or every BEACON_PERIOD_S from the sensor log (time and position at the copy)
    if not V:
        end = S[0][1]
        t0, seq = st[0] + 9.0, 100
        k = 0
        while t0 <= st[-1]:
            i = max(j for j in range(len(st)) if st[j] <= t0)
            f = S[i][0]
            ms = int(f[0]) + round((t0 - st[i]) * 1000)
            utc = round(((t0 - 7200) % 86400) * 1e6)
            V.append([[str(ms), 'TX', '1', f[1], f[2], f[3], f[4], f[5], f[6], f[7], '0', '0', '100', '0',
                       str(seq + k), '-112', '0.0', '0.00', '0', str(utc), '1482990', '1', '2'], end])
            t0 += BEACON_PERIOD_S
            k += 1
    tx = [tx_time(f) for f, _ in V]

    # Jolts: under-recorded ones, and the peak before each beacon the old firmware flagged with a terrain code
    jolts = set(i for i, (f, _) in enumerate(S) if abs(float(f[12]) - 1) >= LIKELY_JOLT_G)
    for b, (f, _) in enumerate(V):
        if int(f[13]) in OLD_TERRAIN_CODES:
            prev = tx[b - 1] if b > 0 else st[0] - 1
            win = [i for i in range(len(S)) if prev < st[i] <= tx[b]]
            jolts.add(max(win, key=lambda i: abs(float(S[i][0][12]) - 1)))
    jolts = sorted(i for i in jolts if float(S[i][0][6]) >= ROAD_MIN_SPEED)
    for i in jolts:
        jolt(S[i][0], 1 if float(S[i][0][12]) >= 1 else -1, JOLT_G_AT.get((name, S[i][0][2]), JOLT_G))

    # Current firmware: a jolt flags bad terrain, the next beacon carries it
    flagged = [st[i] for i in range(len(S)) if abs(float(S[i][0][12]) - 1) >= TERRAIN_G and
               float(S[i][0][6]) >= ROAD_MIN_SPEED]
    prev = -1
    for b, (f, _) in enumerate(V):
        f[13] = str(TERRAIN_MSG if any(prev < t <= tx[b] for t in flagged) else 0)
        prev = tx[b]

    # Sensor log: code in the last beacon (after it is sent)
    for i, (f, _) in enumerate(S):
        last = [int(V[b][0][13]) for b in range(len(V)) if tx[b] + AIR_S <= st[i]]
        f[16] = str(last[-1] if last else 0)

    write(os.path.join(dst, 'sensor_log1.txt'), S)
    write(os.path.join(dst, 'vanet_log1.txt'), V)
    write(os.path.join(dst, 'myself_log1.txt'), M)
    print(f'gw parking lot/{name}: jolts at ' + ', '.join(S[i][0][2] for i in jolts) +
          '; beacons with code 3 at ' + ', '.join(V[b][0][4] for b in range(len(V)) if V[b][0][13] == '3'))


# ---- shd test ----------------------------------------------------------------------------------------------------

STATUS = ['Parked', 'Charging', 'Delivering', 'Taxi']
SERIAL = re.compile(r'^(\d+):(\d+):([\d.]+) > ([AB]) (TX|RX)\s+(?:(ME|P\d)\s+(\d+)\s+)?(\w+)\s+t=(\d+)\s+b=([\d.]+)%\s+'
                    r'm=(\d+)(?:\s+to=(\d+))?(?:\s+seq=(\d+))?')


def host_offset(myself, txh):
    """Serial clock minus node clock at which myself's status/tasks/battery best follow the host's packets
    (as test_analysis.m aligns them)"""
    best, best_score = 0.0, -1.0
    times = [t for t, _ in txh]
    for k in range(-600, 601):
        off = k / 10
        hit = n = 0
        for f, _ in myself:
            ts = tod(f[3]) + off
            j = bisect.bisect_right(times, ts) - 1
            if j < 0 or ts - times[j] <= 1.5:
                continue
            p = txh[j][1]
            n += 1
            hit += (int(f[9]), int(f[10]), int(f[11])) == p
        if n and hit / n > best_score:
            best, best_score = off, hit / n
    return best


def shd():
    src = os.path.join(REAL, 'shd test')
    dst = os.path.join(HERE, 'shd test')
    M = {n: read(os.path.join(src, f'myself_log{n}.txt')) for n in (1, 2)}
    S = {n: read(os.path.join(src, f'sensor_log{n}.txt')) for n in (1, 2)}
    V = {n: read(os.path.join(src, f'vanet_log{n}.txt')) for n in (1, 2)}
    with open(os.path.join(src, 'serial_monitor.txt'), newline='') as fh:
        lines = fh.readlines()

    # Serial monitor: host messages sent as code 4
    parsed = []
    for i, line in enumerate(lines):
        m = SERIAL.match(line)
        if m and m.group(5) == 'TX' and m.group(11) != '0':
            lines[i] = line.replace(f'm={m.group(11)} ', f'm={HOST_MSG} ', 1)
            m = SERIAL.match(lines[i])
        parsed.append(m)
    secs = lambda m: int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
    host_of = {int(m.group(7)): m.group(4) for m in parsed if m and m.group(6) == 'ME'}

    code = {}       # (node, seq) -> message code of its beacon
    for n, h in host_of.items():
        txh = [(secs(m), (STATUS.index(m.group(8)), int(m.group(9)), round(float(m.group(10)))))
               for m in parsed if m and m.group(4) == h and m.group(5) == 'TX']
        off = host_offset(M[n], txh)
        events = [secs(m) - off for m in parsed if m and m.group(4) == h and m.group(5) == 'TX' and m.group(11) != '0']

        # Current firmware, in time order: host message pending until a beacon takes it, low battery flagged while
        # the battery is at or below the threshold, the beacon takes the host message first, else the lowest flag
        timeline = [(t, 0, -1) for t in events]
        timeline += [(tod(f[3]), 1, i) for i, (f, _) in enumerate(M[n])]
        timeline += [(tx_time(f), 2, i) for i, (f, _) in enumerate(V[n]) if f[1] == 'TX' and f[2] == str(n)]
        host_pending = low_battery = False
        for t, kind, i in sorted(timeline):
            if kind == 0:
                host_pending = True
            elif kind == 1:
                f = M[n][i][0]
                if int(f[11]) <= LOW_BATTERY_THRESHOLD:
                    low_battery = True
                f[12] = str(HOST_MSG if host_pending else 0)
            else:
                f = V[n][i][0]
                if host_pending:
                    c, host_pending = HOST_MSG, False
                elif low_battery:
                    c, low_battery = LOW_BATTERY_MSG, False
                else:
                    c = 0
                f[13] = str(c)
                code[(n, int(f[14]))] = (c, t)
        print(f'shd test: node {n} (host {h}, serial clock {off:+.1f} s), {len(events)} host messages, beacons carrying 4: '
              + ', '.join(str(s) for (k, s), (c, _) in sorted(code.items()) if k == n and c == HOST_MSG))

    # Peer's RX and FW lines carry the sender's code
    for n in (1, 2):
        for f, _ in V[n]:
            if f[1] in ('RX', 'FW') and (int(f[2]), int(f[14])) in code:
                f[13] = str(code[(int(f[2]), int(f[14]))][0])

    # Sensor logs: code in the node's last beacon (after it is sent)
    for n in (1, 2):
        sent = sorted((t + AIR_S, c) for (k, _), (c, t) in code.items() if k == n)
        for f, _ in S[n]:
            last = [c for t, c in sent if t <= tod(f[2])]
            f[16] = str(last[-1] if last else 0)

    # Serial monitor RX lines: the node's last beacon (ME) and each peer's packet (Pn) carry the new codes
    for i, m in enumerate(parsed):
        if m and m.group(5) == 'RX':
            key = (int(m.group(7)), int(m.group(13)))
            if key in code:
                lines[i] = re.sub(r'm=\d+ ', f'm={code[key][0]} ', lines[i], count=1)

    for n in (1, 2):
        write(os.path.join(dst, f'myself_log{n}.txt'), M[n])
        write(os.path.join(dst, f'sensor_log{n}.txt'), S[n])
        write(os.path.join(dst, f'vanet_log{n}.txt'), V[n])
    with open(os.path.join(dst, 'serial_monitor.txt'), 'w', newline='') as fh:
        fh.writelines(lines)


if __name__ == '__main__':
    gw_run('run 1')
    gw_run('run 2')
    shd()
