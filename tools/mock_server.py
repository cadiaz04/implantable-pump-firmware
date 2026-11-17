#!/usr/bin/env python3
"""
mock_server.py: a laptop-only stand-in for the ESP32 web API, for working on
the web UI without hardware (no PlatformIO, no board).

    python3 tools/mock_server.py            # → http://localhost:8080

It serves data/www/ and emulates the REST API + WebSocket telemetry with a
crude pump/sensor model. It is NOT the firmware: control behaviour, limits and
error messages are approximations. Use the esp32dev_sim firmware build for
anything that matters. Standard library only.
"""
import base64, hashlib, json, math, os, random, socket, struct, sys, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'www')
LOCK = threading.RLock()
MAXFLOW = 50 * 1000 / 60

def now_ms(): return int(time.time() * 1000)

PUMPS = {
    '28byj48-syringe': {'id': '28byj48-syringe', 'name': '28BYJ-48 Syringe pump (example)', 'pumpType': 'syringe', 'notes': '', 'fluid': 'water',
        'motor': {'driverType': 'uln2003', 'stepsPerRev': 4096, 'microstep': 1, 'gearRatio': 1, 'maxCommand': 15, 'minCommand': 0, 'accel': 10,
                  'invertDir': False, 'decay': 'fast', 'minDuty': 0, 'kickMs': 0, 'kickDuty': 80, 'releaseWhenStopped': True, 'commandUnit': 'RPM'},
        'syringe': {'innerDiameter_mm': 4.78, 'leadscrewPitch_mm': 0.8}, 'pid': {'kp': 0.3, 'ki': 0.05, 'kd': 0, 'loopHz': 1, 'filterWindowS': 10, 'filter': 'ma', 'outputLimitPct': 100, 'integralLimitPct': 100},
        'calibrations': []},
    'dc-diaphragm': {'id': 'dc-diaphragm', 'name': 'DC Diaphragm pump (example)', 'pumpType': 'diaphragm', 'notes': '', 'fluid': 'water',
        'motor': {'driverType': 'drv8871', 'stepsPerRev': 200, 'microstep': 1, 'gearRatio': 1, 'maxCommand': 100, 'minCommand': 15, 'accel': 50,
                  'invertDir': False, 'decay': 'fast', 'minDuty': 12, 'kickMs': 150, 'kickDuty': 60, 'releaseWhenStopped': True, 'commandUnit': 'Duty %'},
        'syringe': {'innerDiameter_mm': 0, 'leadscrewPitch_mm': 0}, 'pid': {'kp': 0.3, 'ki': 0.05, 'kd': 0, 'loopHz': 1, 'filterWindowS': 20, 'filter': 'ma', 'outputLimitPct': 100, 'integralLimitPct': 100},
        'calibrations': []},
}
FAMILY = {'uln2003': 'stepper', 'stepdir': 'stepper', 'drv8871': 'dc', 'pwmdir': 'dc'}
LABELS = {'uln2003': ['IN1', 'IN2', 'IN3', 'IN4'], 'stepdir': ['STEP', 'DIR', 'EN'], 'drv8871': ['IN1', 'IN2'], 'pwmdir': ['PWM', 'DIR', 'EN']}
DEFAULTS = {'uln2003': [[16, 17, 18, 19], [13, 14, 27, 26]], 'stepdir': [[16, 17, 18], [13, 14, 27]], 'drv8871': [[16, 17], [13, 14]], 'pwmdir': [[16, 17, 18], [13, 14, 27]]}
EVENTS, HISTORY = [], []
STATE = {'estop': False, 'hb': time.time(), 'settings': {'watchdogS': 10, 'telemetryHz': 5, 'flowUnit': 'uL/min', 'pressureUnit': 'kPa', 'sensorFaultAction': 'pause', 'hostname': 'biopump',
         'wifi': {'ssid': '', 'passwordSet': False, 'apSsid': 'BioPump-MOCK', 'apIp': '127.0.0.1', 'staConnected': False, 'staIp': '', 'apClients': 1}},
         'epoch_synced': True}

def event(ch, level, msg):
    EVENTS.append({'seq': len(EVENTS) + 1, 't': now_ms(), 'ch': ch, 'level': level, 'msg': msg})

class Ch:
    def __init__(self, n, driver, pump):
        self.n, self.driver, self.pump = n, driver, pump
        self.pins = DEFAULTS[driver][n - 1][:]
        self.state, self.mode, self.cmd, self.actual, self.sp = 'IDLE', 'none', 0.0, 0.0, None
        self.flow, self.lag, self.vol, self.runvol, self.integ = 0.0, 0.0, 0.0, 0.0, 0.0
        self.req, self.keep, self.zero, self.alarms, self.elapsed = {}, False, 0.0, 0, 0.0
        self.samples, self.fault, self.bubble, self.gain = [], False, 0, (20.0 if FAMILY[driver] == 'stepper' else 6.0)
        self.cal = {'phase': 'idle', 'points': [], 'index': -1, 't0': 0, 'running': False, 'cfg': {}}
        self.pressure = 'mprls' if n == 1 else 'none'
        self.pabs, self.pzero, self.plimit, self.occluded = 100.8, None, 0.0, False
    @property
    def unit(self): return 'RPM' if FAMILY[self.driver] == 'stepper' else 'Duty %'
    def plant(self, c):
        if FAMILY[self.driver] == 'dc':
            u = c - 12
            return 0 if u <= 0 else 6 * u * (self.gain / 6) + 0.02 * u * u
        return self.gain * c
    def inverse(self, f):
        p = PUMPS[self.pump]; cal = next((c for c in p['calibrations'] if c['active']), None)
        lo, hi = p['motor']['minCommand'], p['motor']['maxCommand']
        if cal:
            c0, c1, c2 = cal['coefficients']
            g = lambda x: c0 + c1 * x + c2 * x * x
        else:
            k = self.req.get('initialCommand') or 0
            g = (lambda x: x * (self.sp / k)) if k else (lambda x: x * 20)
        a, b = lo, hi
        for _ in range(40):
            m = (a + b) / 2
            if g(m) < f: a = m
            else: b = m
        return (a + b) / 2
    def step(self, dt):
        t = time.time()
        p = PUMPS[self.pump]
        if self.state == 'RUNNING':
            self.elapsed += dt
            m = self.mode
            if m == 'manual': self.cmd = self.req.get('command', 0)
            elif m in ('closed_loop', 'volume', 'open_loop'):
                avg = self.avg()
                if m != 'open_loop' and not self.req.get('openLoop') and avg is not None:
                    self.integ = max(-self.sp, min(self.sp, self.integ + 0.05 * (self.sp - avg) * dt))
                self.cmd = self.inverse(self.sp + (0 if m == 'open_loop' else self.integ))
                if m == 'volume' and self.runvol >= self.req['volume_uL']:
                    self.state, self.mode = 'IDLE', 'none'; event(self.n, 0, 'Target volume delivered')
            elif m == 'program':
                self.cmd = self.req['program']['steps'][0]['value'] if self.req['program']['steps'] else 0
            if not self.keep and t - STATE['hb'] > STATE['settings']['watchdogS']:
                self.state, self.mode, self.alarms = 'IDLE', 'none', 16; event(self.n, 2, 'Browser heartbeat lost: run stopped by watchdog')
        elif self.state == 'CALIBRATING':
            self.cal_step(t)
        else:
            self.cmd = 0
        if self.state not in ('RUNNING', 'CALIBRATING'): self.cmd = 0
        acc = p['motor']['accel'] * dt
        self.actual += max(-acc, min(acc, self.cmd - self.actual))
        true = self.plant(self.actual)
        self.lag += (true - self.lag) * (1 - math.exp(-dt / 4))
        meas = self.lag + 8 + random.gauss(0, 5)
        if self.bubble > 0: self.bubble -= dt; meas *= 0.1
        self.flow = meas - self.zero
        self.samples.append((t, self.flow)); self.samples = [s for s in self.samples if s[0] > t - p['pid']['filterWindowS']]
        r = 0.02 * (40 if self.occluded else 1)
        self.pabs += (100.8 + r * true - self.pabs) * (1 - math.exp(-dt / 1.5))
        if self.occluded: self.flow *= 0.05
        gauge = self.pabs - (self.pzero or 101.325)
        if self.plimit > 0 and self.pressure == 'mprls' and gauge > self.plimit and self.state == 'RUNNING':
            self.state, self.alarms = 'PAUSED', 128; event(self.n, 2, f'Paused: pressure {gauge:.1f} kPa > limit {self.plimit:.1f} kPa (occlusion?)')
        if self.actual > 0:
            self.vol += self.flow * dt / 60
            if self.mode == 'volume': self.runvol += self.flow * dt / 60
    def avg(self):
        return sum(s[1] for s in self.samples) / len(self.samples) if self.samples else None
    def std(self):
        if len(self.samples) < 2: return None
        a = self.avg(); return math.sqrt(sum((s[1] - a) ** 2 for s in self.samples) / (len(self.samples) - 1))
    def cal_step(self, t):
        c = self.cal; cfg = c['cfg']; i = c['index']
        el = t - c['t0']
        per = (cfg.get('settleS', 60) + cfg.get('averageS', 60)) / 20   # mock runs 20× faster
        if c['phase'] == 'settle' and el > per / 2: c['phase'], c['t0'] = 'average', t
        elif c['phase'] == 'average' and el > per / 2:
            pt = c['points'][i]; pt.update(status='ok', flow=self.plant(pt['command']) + random.gauss(0, 1.5), std=5 + random.random(), temp=23.1, n=500)
            if i + 1 < len(c['points']):
                c['index'] = i + 1; c['points'][i + 1]['status'] = 'active'; c['phase'], c['t0'] = 'settle', t
                self.cmd = c['points'][i + 1]['command']
            else:
                c['phase'], c['running'] = 'done', False; self.state, self.mode = 'IDLE', 'none'
                event(self.n, 0, 'Calibration sweep finished')
        if c['running']: self.cmd = c['points'][c['index']]['command']
    def tel(self):
        a, s = self.avg(), self.std()
        al = self.alarms | (1 if self.bubble > 0 else 0) | (2 if self.fault else 0)
        return {'state': self.state, 'mode': self.mode, 'command': self.cmd, 'commandActual': self.actual, 'commandUnit': self.unit,
                'stepRate_Hz': self.actual * 4096 / 60 if self.driver == 'uln2003' else 0, 'setpoint': self.sp, 'manualCommand': self.req.get('command'),
                'reverse': False, 'flow_uLmin': None if self.fault else self.flow, 'flow_avg_uLmin': None if self.fault else a, 'flow_std': s, 'temp_C': 23.1,
                'volume_uL': self.vol, 'volumeEstimated': False, 'runVolume_uL': self.runvol,
                'plannedVolume_uL': (self.sp or 0) * self.elapsed / 60 if self.mode == 'volume' else None,
                'targetVolume_uL': self.req.get('volume_uL') if self.mode == 'volume' else None,
                'eta_s': (self.req['volume_uL'] - self.runvol) / self.sp * 60 if self.mode == 'volume' and self.sp else None,
                'elapsed_s': self.elapsed, 'flags': 1 if self.bubble > 0 else 0, 'alarms': al, 'sensor': 'FAULT' if self.fault else 'OK',
                'errors': {'crc': 0, 'i2c': 0, 'consecutive': 0, 'reinits': 0}, 'zeroOffset': self.zero, 'zeroing': False, 'zeroProgress': 0,
                'keepRunning': self.keep, 'programStep': 0 if self.mode == 'program' else -1, 'profileId': self.pump,
                'profileName': PUMPS[self.pump]['name'], 'profileMismatch': False, 'hasCalibration': any(c['active'] for c in PUMPS[self.pump]['calibrations']),
                'ff': None, 'corr': self.integ if self.mode in ('closed_loop', 'volume') else None,
                'cal': {'phase': self.cal['phase'], 'index': self.cal['index'], 'points': len(self.cal['points']), 'elapsed': 0, 'duration': 0},
                **({'pressure_kPa': self.pabs - (self.pzero or 101.325), 'pressureAbs_kPa': self.pabs, 'pressureTared': self.pzero is not None,
                    'pressureZero_kPa': self.pzero, 'pressureSensor': 'OK', 'pressureSaturated': False, 'pressureTaring': False,
                    'pressureTareProgress': 0, 'pressureAlarm_kPa': self.plimit, 'pressureErrors': {'frame': 0, 'i2c': 0, 'reinits': 0}}
                   if self.pressure == 'mprls' else {})}

CH = [Ch(1, 'uln2003', '28byj48-syringe'), Ch(2, 'drv8871', 'dc-diaphragm')]

def fits(ch):
    pts = [p for p in ch.cal['points'] if p['status'] in ('ok', 'not_steady') or p.get('ref') is not None]
    xs = [p['command'] for p in pts]; ys = [p['ref'] if p.get('ref') is not None else p['flow'] for p in pts]
    out = {}
    n = len(xs)
    if n >= 2:
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs) or 1
        b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx; a = my - b * mx
        res = [y - (a + b * x) for x, y in zip(xs, ys)]
        sst = sum((y - my) ** 2 for y in ys) or 1
        r2 = 1 - sum(r * r for r in res) / sst
        lin = {'fitType': 'linear', 'coefficients': [a, b, 0], 'r2': r2, 'rmse': math.sqrt(sum(r * r for r in res) / n),
               'maxErrPct': max((abs(r) / abs(y) * 100 for r, y in zip(res, ys) if abs(y) > 1), default=0), 'nonMonotone': False, 'usable': b > 0, 'residuals': res}
        out['linear'] = lin
        out['quadratic'] = dict(lin, fitType='quadratic')
        srt = sorted(zip(xs, ys))
        out['lookup'] = dict(lin, fitType='lookup', lookup={'x': [p[0] for p in srt], 'y': [p[1] for p in srt]})
    else:
        for k in ('linear', 'quadratic', 'lookup'): out[k] = {'error': 'need at least 2 valid points'}
    return out, n

def state():
    return {'ok': True, 'sim': True, 'version': 'mock', 'uptimeMs': now_ms() % 10 ** 7, 'time': {'epochMs': now_ms(), 'synced': True},
            'maxChannels': 2, 'limits': {'maxFlow_uLmin': MAXFLOW, 'minFlow_uLmin': 0}, 'heapFree': 150000,
            'history': {'capacity': 1777, 'size': len(HISTORY), 'periodMs': 1000}, 'estop': STATE['estop'], 'hbAge_s': time.time() - STATE['hb'],
            'settings': STATE['settings'],
            'channels': [{'n': c.n, 'hw': {'driver': c.driver, 'commandUnit': c.unit, 'pins': c.pins, 'pinLabels': LABELS[c.driver], 'requiredPins': 2,
                          'enActiveLow': True, 'sensor': 'slf3s', 'pressure': c.pressure, 'sda': [21, 32][c.n - 1], 'scl': [22, 33][c.n - 1]},
                          'profile': dict(PUMPS[c.pump], calibrations=None, activeCalibration=next((k for k in PUMPS[c.pump]['calibrations'] if k['active']), None)),
                          'status': dict(c.tel(), alarms=[])} for c in CH],
            'signals': [s for c in CH for s in (
                {'id': f'f{c.n}', 'name': f'Flow Ch{c.n}', 'unit': 'uL/min', 'channel': c.n}, {'id': f'fa{c.n}', 'name': f'Flow avg Ch{c.n}', 'unit': 'uL/min', 'channel': c.n},
                {'id': f'sp{c.n}', 'name': f'Setpoint Ch{c.n}', 'unit': 'uL/min', 'channel': c.n}, {'id': f'c{c.n}', 'name': f'Command Ch{c.n}', 'unit': c.unit, 'channel': c.n},
                {'id': f't{c.n}', 'name': f'Temp Ch{c.n}', 'unit': 'C', 'channel': c.n}, {'id': f'v{c.n}', 'name': f'Volume Ch{c.n}', 'unit': 'uL', 'channel': c.n})]
                + [{'id': f'p{c.n}', 'name': f'Pressure Ch{c.n}', 'unit': 'kPa', 'channel': c.n} for c in CH if c.pressure == 'mprls']}

def api(method, path, q, body):
    seg = [s for s in path.split('/')[2:] if s]
    if not seg: return 404, {'ok': False, 'error': 'not found'}
    s0 = seg[0]
    if s0 == 'state': return 200, state()
    if s0 in ('hb', 'time'): STATE['hb'] = time.time(); return 200, {'ok': True}
    if s0 == 'events': return 200, {'ok': True, 'events': EVENTS[-48:]}
    if s0 == 'estop':
        if len(seg) > 1:
            STATE['estop'] = False
            for c in CH: c.state, c.alarms = 'IDLE', 0
            event(0, 1, 'E-STOP reset')
        else:
            STATE['estop'] = True
            for c in CH: c.state, c.mode, c.cmd, c.actual, c.alarms = 'ESTOP', 'none', 0, 0, 32
            event(0, 2, 'E-STOP (API): all motors stopped')
        return 200, {'ok': True}
    if s0 == 'config':
        if method == 'GET': return 200, {'ok': True, 'config': STATE['settings']}
        for k in ('watchdogS', 'telemetryHz', 'flowUnit', 'sensorFaultAction', 'pressureUnit'):
            if k in body: STATE['settings'][k] = body[k]
        if 'wifi' in body and 'ssid' in body['wifi']: STATE['settings']['wifi']['ssid'] = body['wifi']['ssid']
        return 200, {'ok': True}
    if s0 == 'hardware':
        if method == 'GET':
            return 200, {'ok': True, 'channels': [{'driver': c.driver, 'pins': c.pins, 'pinLabels': LABELS[c.driver], 'enActiveLow': True, 'sensor': 'slf3s', 'pressure': c.pressure,
                         'sda': [21, 32][c.n - 1], 'scl': [22, 33][c.n - 1], 'profile': c.pump} for c in CH],
                         'driverTypes': [{'id': k, 'family': FAMILY[k], 'commandUnit': 'RPM' if FAMILY[k] == 'stepper' else 'Duty %', 'pinLabels': v,
                                          'requiredPins': 4 if k == 'uln2003' else 2, 'defaults': DEFAULTS[k]} for k, v in LABELS.items()]}
        used, errs = {}, []
        for i, c in enumerate(body['channels']):
            for lab, p in zip(LABELS[c['driver']], c['pins']):
                if p < 0: continue
                if p in (0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 15) or p >= 34: errs.append({'channel': i + 1, 'field': lab, 'gpio': p, 'reason': 'illegal pin'})
                elif p in used: errs.append({'channel': i + 1, 'field': lab, 'gpio': p, 'reason': 'conflicts with ' + used[p]})
                used[p] = f'Ch{i + 1} {lab}'
        if errs: return 400, {'ok': False, 'error': 'invalid pin configuration', 'pinErrors': errs}
        for c, b in zip(CH, body['channels']):
            c.driver, c.pins = b['driver'], b['pins']
            c.pressure = b.get('pressure', c.pressure)
            if b.get('profile'): c.pump = b['profile']
        return 200, {'ok': True}
    if s0 == 'pumps':
        if len(seg) == 1 and method == 'GET':
            return 200, {'ok': True, 'pumps': [{'id': p['id'], 'name': p['name'], 'pumpType': p['pumpType'], 'driverType': p['motor']['driverType'],
                         'family': FAMILY[p['motor']['driverType']], 'calibrations': len(p['calibrations']),
                         'activeCalibration': next((c['name'] for c in p['calibrations'] if c['active']), '')} for p in PUMPS.values()]}
        if len(seg) == 1 and method == 'POST':
            pid = body.get('id') or body['name'].lower().replace(' ', '-')[:28]
            while pid in PUMPS: pid += '-2'
            body['id'] = pid; body.setdefault('calibrations', []); PUMPS[pid] = body
            return 201, {'ok': True, 'id': pid}
        pid = seg[1]
        if pid not in PUMPS: return 404, {'ok': False, 'error': 'unknown profile'}
        if len(seg) == 3: n = dict(json.loads(json.dumps(PUMPS[pid])), name=PUMPS[pid]['name'] + ' (copy)', id=pid + '-copy'); PUMPS[n['id']] = n; return 201, {'ok': True}
        if method == 'GET': return 200, PUMPS[pid]
        if method == 'PUT': body['id'] = pid; PUMPS[pid] = body; return 200, {'ok': True}
        if method == 'DELETE':
            if any(c.pump == pid for c in CH): return 409, {'ok': False, 'error': 'profile is assigned to a channel'}
            del PUMPS[pid]; return 200, {'ok': True}
    if s0 == 'history':
        if method == 'DELETE': HISTORY.clear(); return 200, {'ok': True}
        n = int(q.get('ch', ['1'])[0])
        return 200, {'ok': True, 'ch': n, 'synced': True, 'cols': [], 'rows': [[h['t'], h['f'][n - 1], h['sp'][n - 1], h['c'][n - 1], 23.1, 0] for h in HISTORY]}
    if s0 in ('channels', 'calibration', 'sensors', 'sim') and len(seg) >= 2:
        c = CH[int(seg[1]) - 1]
        act = seg[2] if len(seg) > 2 else ''
        if s0 == 'sim':
            if 'bubble' in body: c.bubble = body['bubble']
            if 'fault' in body: c.fault = body['fault']
            if 'gainScale' in body: c.gain *= body['gainScale']
            if 'occlusion' in body: c.occluded = body['occlusion']
            return 200, {'ok': True}
        if s0 == 'sensors' and act == 'pressure-zero':
            c.pzero = None if body.get('clear') else c.pabs
            if c.pzero: event(c.n, 0, f'Pressure tare: atmosphere = {c.pzero:.2f} kPa')
            return 200, {'ok': True}
        if s0 == 'sensors' and act == 'pressure-alarm':
            c.plimit = float(body.get('kPa', 0)); return 200, {'ok': True}
        if s0 == 'sensors':
            if body.get('clear'): c.zero = 0
            else: c.zero = c.avg() or 0; event(c.n, 0, f'Zero complete: offset {c.zero:.2f} uL/min')
            return 200, {'ok': True}
        if s0 == 'calibration':
            if act == 'status':
                f, n = fits(c)
                cal = c.cal
                return 200, {'ok': True, 'status': {'phase': cal['phase'], 'running': cal['running'], 'index': cal['index'], 'phaseElapsed': 0, 'phaseDuration': 0,
                             'commandUnit': c.unit, 'zeroOffset': c.zero, 'remainingS': 0, 'config': cal['cfg'], 'validPoints': n,
                             'points': [dict(p, i=i) for i, p in enumerate(cal['points'])], 'fits': f}}
            if act == 'start':
                if STATE['estop']: return 409, {'ok': False, 'error': 'E-STOP is latched'}
                cmds = body.get('commands', [])
                c.cal = {'phase': 'settle', 'index': 0, 't0': time.time(), 'running': True, 'cfg': body,
                         'points': [{'command': x, 'pass': 0, 'status': 'active' if k == 0 else 'pending', 'flow': None, 'std': None, 'temp': None, 'ref': None, 'n': 0} for k, x in enumerate(cmds)]}
                c.state, c.mode = 'CALIBRATING', 'calibration'; event(c.n, 0, f'Calibration sweep started ({len(cmds)} points)')
                return 200, {'ok': True}
            if act == 'abort': c.cal['running'] = False; c.cal['phase'] = 'aborted'; c.state, c.mode = 'IDLE', 'none'; return 200, {'ok': True}
            if act == 'discard': c.cal = {'phase': 'idle', 'points': [], 'index': -1, 't0': 0, 'running': False, 'cfg': {}}; return 200, {'ok': True}
            if act == 'point':
                p = c.cal['points'][body['index']]
                p['ref'] = None if body.get('clear') else body.get('flow_uLmin', (body.get('volume_uL') or 0) / (body.get('duration_s') or 1) * 60)
                return 200, {'ok': True}
            if act == 'accept':
                f, n = fits(c); fit = f[body.get('fitType', 'linear')]
                for k in PUMPS[c.pump]['calibrations']: k['active'] = False
                PUMPS[c.pump]['calibrations'].append(dict(fit, id=f'c{int(time.time())}', name=body.get('name') or 'mock calibration', timestamp=int(time.time()),
                    fluid='water', tubingNote=body.get('tubingNote', ''), active=True, points=c.cal['points']))
                return 200, {'ok': True}
            return 200, {'ok': True}
        if act == 'run':
            if STATE['estop']: return 409, {'ok': False, 'error': 'E-STOP is latched; reset it first'}
            m = body.get('mode', 'manual')
            if m == 'manual' and not (0 <= body.get('command', 0) <= PUMPS[c.pump]['motor']['maxCommand']):
                return 409, {'ok': False, 'error': 'command outside 0..maxCommand of the pump profile'}
            if 'volume_mL' in body: body['volume_uL'] = body['volume_mL'] * 1000
            if 'duration_h' in body: body['duration_s'] = body['duration_h'] * 3600
            sp = body.get('flow_uLmin') if m in ('closed_loop', 'open_loop') else (body['volume_uL'] / body['duration_s'] * 60 if m == 'volume' else None)
            if sp is not None and not (0 <= sp <= MAXFLOW): return 409, {'ok': False, 'error': 'flow setpoint outside firmware limits'}
            if c.state != 'RUNNING' or c.mode != m: c.runvol, c.elapsed, c.integ = 0, 0, 0
            c.req, c.mode, c.state, c.sp, c.keep, c.alarms = body, m, 'RUNNING', sp, body.get('keepRunning', False), 0
            STATE['hb'] = time.time(); event(c.n, 0, f'Run started: {m}')
            return 200, {'ok': True}
        if act == 'stop': c.state, c.mode, c.sp = ('ESTOP' if STATE['estop'] else 'IDLE'), 'none', None; event(c.n, 0, 'Stopped'); return 200, {'ok': True}
        if act == 'pause': c.state = 'PAUSED'; return 200, {'ok': True}
        if act == 'resume': c.state = 'RUNNING'; return 200, {'ok': True}
        if act == 'volume-reset': c.vol = 0; return 200, {'ok': True}
        if act == 'profile': c.pump = body['id']; return 200, {'ok': True}
    return 404, {'ok': False, 'error': 'unknown endpoint ' + path}

CLIENTS = []

def ws_send(sock, text):
    data = text.encode()
    hdr = bytes([0x81])
    n = len(data)
    hdr += bytes([n]) if n < 126 else (bytes([126]) + struct.pack('>H', n) if n < 65536 else bytes([127]) + struct.pack('>Q', n))
    sock.sendall(hdr + data)

def ws_reader(sock):
    try:
        while True:
            h = sock.recv(2)
            if len(h) < 2: break
            n = h[1] & 0x7F
            if n == 126: n = struct.unpack('>H', sock.recv(2))[0]
            elif n == 127: n = struct.unpack('>Q', sock.recv(8))[0]
            mask = sock.recv(4)
            buf = b''
            while len(buf) < n: buf += sock.recv(n - len(buf))
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(buf))
            if h[0] & 0x0F == 8: break
            try: m = json.loads(payload)
            except Exception: continue
            STATE['hb'] = time.time()
            if m.get('type') == 'estop': api('POST', '/api/estop', {}, {})
    except OSError:
        pass
    with LOCK:
        if sock in CLIENTS: CLIENTS.remove(sock)

class H(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    def log_message(self, *a): pass
    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        return json.loads(self.rfile.read(n) or b'{}') if n else {}
    def _send(self, code, obj, ctype='application/json'):
        data = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        self.send_response(code); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(data))); self.end_headers()
        self.wfile.write(data)
    def handle_api(self, method):
        u = urlparse(self.path)
        with LOCK:
            code, obj = api(method, u.path, parse_qs(u.query), self._body() if method in ('POST', 'PUT') else {})
        self._send(code, obj)
    def do_POST(self): self.handle_api('POST')
    def do_PUT(self): self.handle_api('PUT')
    def do_DELETE(self): self.handle_api('DELETE')
    def do_GET(self):
        if self.path.startswith('/ws'):
            key = self.headers['Sec-WebSocket-Key']
            acc = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            self.send_response(101); self.send_header('Upgrade', 'websocket'); self.send_header('Connection', 'Upgrade')
            self.send_header('Sec-WebSocket-Accept', acc); self.end_headers()
            sock = self.connection
            ws_send(sock, json.dumps({'type': 'hello', 'sim': True, 'version': 'mock'}))
            with LOCK: CLIENTS.append(sock)
            ws_reader(sock)
            self.close_connection = True
            return
        if self.path.startswith('/api/'): return self.handle_api('GET')
        p = urlparse(self.path).path
        p = '/index.html' if p == '/' else p
        f = os.path.normpath(os.path.join(ROOT, p.lstrip('/')))
        if not f.startswith(os.path.normpath(ROOT)) or not os.path.isfile(f): return self._send(404, b'not found', 'text/plain')
        ct = {'.html': 'text/html', '.js': 'application/javascript', '.css': 'text/css'}.get(os.path.splitext(f)[1], 'application/octet-stream')
        self._send(200, open(f, 'rb').read(), ct)

def loop():
    last, lastHist = time.time(), 0
    while True:
        time.sleep(1 / max(0.5, STATE['settings']['telemetryHz']))
        t = time.time()
        with LOCK:
            for c in CH:
                c.step(t - last)
            last = t
            msg = {'type': 'tel', 't': now_ms(), 'up': now_ms(), 'synced': True, 'estop': STATE['estop'], 'hbAge': t - STATE['hb'],
                   'ch': [c.tel() for c in CH], 'sig': {}}
            for c in CH:
                d = c.tel(); n = c.n
                if 'pressure_kPa' in d: msg['sig'][f'p{n}'] = d['pressure_kPa']
                msg['sig'].update({f'f{n}': d['flow_uLmin'], f'fa{n}': d['flow_avg_uLmin'], f'sp{n}': d['setpoint'], f'c{n}': d['commandActual'], f't{n}': 23.1, f'v{n}': d['volume_uL']})
            if t - lastHist >= 1:
                lastHist = t
                HISTORY.append({'t': now_ms(), 'f': [c.flow for c in CH], 'sp': [c.sp for c in CH], 'c': [c.actual for c in CH]})
                del HISTORY[:-1800]
            text = json.dumps(msg)
            sent = len(EVENTS)
            for s in CLIENTS[:]:
                try:
                    ws_send(s, text)
                    for e in EVENTS[getattr(loop, 'seen', 0):]: ws_send(s, json.dumps(dict(e, type='event')))
                except OSError:
                    CLIENTS.remove(s)
            loop.seen = sent

if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    event(0, 0, 'Mock server started (not the firmware!)')
    threading.Thread(target=loop, daemon=True).start()
    print(f'Mock BioPump API on http://localhost:{port}/  (serving {os.path.normpath(ROOT)})')
    ThreadingHTTPServer(('', port), H).serve_forever()
