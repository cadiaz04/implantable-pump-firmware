#!/usr/bin/env python3
"""
logger.py: headless CSV logger for the BioPump controller (Mac / Linux /
Raspberry Pi). Connects to the ESP32's WebSocket and writes every telemetry
frame to a CSV file. Python 3.8+, standard library only (no pip installs).

    python3 tools/logger.py                         # biopump.local → biopump-YYYYmmdd-HHMMSS.csv
    python3 tools/logger.py --host 192.168.4.1 --out run1.csv
    python3 tools/logger.py --every 5               # keep 1 of every 5 frames (5 Hz → 1 Hz)

Columns come from the firmware's signal registry (GET /api/state → signals),
so new sensors (e.g. pressure) show up automatically. Reconnects on its own
and appends to the same file; a "# reconnect" comment marks gaps.

The logger does NOT send watchdog heartbeats by default: an unattended run
should be started with "Keep running if disconnected". --heartbeat makes the
logger keep ordinary runs alive too; only use it if you understand that the
pump will then keep running as long as this script does.
"""
import argparse, base64, json, os, socket, ssl, struct, sys, time, urllib.request
from datetime import datetime, timezone


def ws_connect(host, port, path='/ws', timeout=10):
    s = socket.create_connection((host, port), timeout=timeout)
    key = base64.b64encode(os.urandom(16)).decode()
    req = (f'GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
           f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n')
    s.sendall(req.encode())
    resp = b''
    while b'\r\n\r\n' not in resp:
        chunk = s.recv(1024)
        if not chunk:
            raise ConnectionError('handshake failed')
        resp += chunk
    if b' 101 ' not in resp.split(b'\r\n')[0]:
        raise ConnectionError('server refused WebSocket: ' + resp.split(b'\r\n')[0].decode(errors='replace'))
    return s, resp.split(b'\r\n\r\n', 1)[1]


class WsReader:
    def __init__(self, sock, pending=b''):
        self.s, self.buf = sock, pending

    def _need(self, n):
        while len(self.buf) < n:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError('connection closed')
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv_text(self):
        message = b''
        while True:
            b0, b1 = self._need(2)
            op, n = b0 & 0x0F, b1 & 0x7F
            if n == 126:
                n = struct.unpack('>H', self._need(2))[0]
            elif n == 127:
                n = struct.unpack('>Q', self._need(8))[0]
            mask = self._need(4) if b1 & 0x80 else None
            data = self._need(n)
            if mask:
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
            if op == 8:
                raise ConnectionError('server closed the WebSocket')
            if op == 9:   # ping → pong
                self.send(data, opcode=0xA)
                continue
            if op in (0, 1):
                message += data
                if b0 & 0x80:
                    return message.decode('utf-8', errors='replace')

    def send(self, payload, opcode=0x1):
        if isinstance(payload, str):
            payload = payload.encode()
        mask = os.urandom(4)
        n = len(payload)
        hdr = bytes([0x80 | opcode])
        if n < 126:
            hdr += bytes([0x80 | n])
        elif n < 65536:
            hdr += bytes([0x80 | 126]) + struct.pack('>H', n)
        else:
            hdr += bytes([0x80 | 127]) + struct.pack('>Q', n)
        self.s.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))


def get_state(host, port):
    with urllib.request.urlopen(f'http://{host}:{port}/api/state', timeout=10) as r:
        return json.loads(r.read())


def unit_tag(u):
    return ''.join(ch for ch in u.replace('/', 'per').replace('%', 'pct') if ch.isalnum())


def fmt(v):
    if v is None:
        return ''
    if isinstance(v, float):
        return f'{v:.6g}'
    return str(v)


def main():
    ap = argparse.ArgumentParser(description='BioPump headless CSV logger')
    ap.add_argument('--host', default='biopump.local', help='ESP32 address (default biopump.local; AP mode: 192.168.4.1)')
    ap.add_argument('--port', type=int, default=80)
    ap.add_argument('--out', default=None, help='CSV file (default biopump-<timestamp>.csv)')
    ap.add_argument('--every', type=int, default=1, help='keep 1 of every N telemetry frames')
    ap.add_argument('--duration', type=float, default=0, help='stop after this many hours (0 = until Ctrl-C)')
    ap.add_argument('--heartbeat', action='store_true', help='also send watchdog heartbeats (keeps ordinary runs alive!)')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    out = a.out or datetime.now().strftime('biopump-%Y%m%d-%H%M%S.csv')
    t_end = time.time() + a.duration * 3600 if a.duration > 0 else None
    cols = None
    nframes = nrows = 0
    f = None
    print(f'Logging {a.host} → {out}  (Ctrl-C to stop)')
    try:
        while t_end is None or time.time() < t_end:
            try:
                st = get_state(a.host, a.port)
                signals = st['signals']
                nch = len(st['channels'])
                if cols is None:
                    cols = [s['id'] for s in signals]
                    header = (['time_ms', 'iso_time_utc'] + [f"{s['id']}_{s['name'].replace(' ', '_')}_{unit_tag(s['unit'])}" for s in signals] +
                              [x for i in range(1, nch + 1) for x in (f'ch{i}_state', f'ch{i}_mode', f'ch{i}_air_in_line', f'ch{i}_sensor_flags',
                                                                      f'ch{i}_alarms', f'ch{i}_run_volume_uL', f'ch{i}_profile')])
                    new = not os.path.exists(out) or os.path.getsize(out) == 0
                    f = open(out, 'a', buffering=1)
                    if new:
                        f.write('# BioPump log, firmware ' + str(st.get('version')) + (' (SIMULATION)' if st.get('sim') else '') + '\n')
                        f.write(','.join(header) + '\n')
                    else:
                        f.write('# reconnect ' + datetime.now(timezone.utc).isoformat() + '\n')
                sock, pending = ws_connect(a.host, a.port)
                ws = WsReader(sock, pending)
                if a.heartbeat:
                    ws.send(json.dumps({'type': 'time', 'epochMs': int(time.time() * 1000)}))
                sock.settimeout(30)
                last_hb = 0
                while t_end is None or time.time() < t_end:
                    m = json.loads(ws.recv_text())
                    if a.heartbeat and time.time() - last_hb > 2:
                        ws.send('{"type":"hb"}')
                        last_hb = time.time()
                    if m.get('type') == 'event':
                        f.write(f"# event {m.get('t')} ch{m.get('ch')} level{m.get('level')}: {m.get('msg')}\n")
                        if not a.quiet:
                            print(f"[event] ch{m.get('ch')} {m.get('msg')}")
                        continue
                    if m.get('type') != 'tel':
                        continue
                    nframes += 1
                    if nframes % a.every:
                        continue
                    t = m['t']
                    iso = datetime.fromtimestamp(t / 1000, timezone.utc).isoformat() if m.get('synced') else ''
                    row = [str(t), iso] + [fmt(m['sig'].get(c)) for c in cols]
                    for c in m['ch']:
                        row += [c['state'], c['mode'], '1' if c['alarms'] & 1 else '0', fmt(c['flags']), fmt(c['alarms']),
                                fmt(c.get('runVolume_uL')), c.get('profileId', '')]
                    f.write(','.join(row) + '\n')
                    nrows += 1
                    if not a.quiet and nrows % 50 == 0:
                        flows = ' '.join(f"ch{i + 1}={fmt(c['flow_avg_uLmin'])}uL/min {c['state']}" for i, c in enumerate(m['ch']))
                        print(f'{datetime.now():%H:%M:%S} rows={nrows} {flows}')
            except (OSError, ConnectionError, ValueError, KeyError) as e:
                print(f'connection problem ({e}); retrying in 3 s', file=sys.stderr)
                time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        if f:
            f.close()
    print(f'\nWrote {nrows} rows to {out}')


if __name__ == '__main__':
    main()
