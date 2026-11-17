/* BioPump Controller — web UI (vanilla JS, no build step, no CDN). */
'use strict';

const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const isNum = (v) => typeof v === 'number' && isFinite(v);

const S = {
  state: null,          // GET /api/state
  tel: null,            // last telemetry frame
  ws: null,
  connected: false,
  unit: 'uL/min',
  cards: [],            // per-channel DOM + plot
  session: { rows: [], cols: null },
  backfilled: false,
  calCh: 0,
  calStatus: null,
  fit: 'linear',
  refMode: 'volume',
  pumps: [],
  editing: null,        // profile being edited
  hwMeta: null,
  lastHb: 0,
};

// ------------------------------------------------------------------ utils
const store = {
  get(k, d) { try { const v = localStorage.getItem('bp.' + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem('bp.' + k, JSON.stringify(v)); } catch { /* private mode */ } },
};

function toast(msg, kind = '') {
  const t = document.createElement('div');
  t.className = 'toast ' + kind;
  t.textContent = msg;
  $('#toasts').appendChild(t);
  setTimeout(() => t.remove(), kind === 'err' ? 6000 : 3500);
}

async function api(method, path, body) {
  const opt = { method, headers: {} };
  if (body !== undefined) {
    opt.headers['Content-Type'] = 'application/json';
    opt.body = JSON.stringify(body);
  }
  const r = await fetch('/api/' + path, opt);
  let j = null;
  try { j = await r.json(); } catch { /* not JSON */ }
  if (!r.ok || (j && j.ok === false)) {
    const e = new Error((j && j.error) || r.statusText || 'request failed');
    e.data = j;
    throw e;
  }
  return j;
}

// Runs an API action, toasting errors. Returns the result or undefined.
async function act(fn, okMsg) {
  try {
    const r = await fn();
    if (okMsg) toast(okMsg);
    return r;
  } catch (e) {
    toast(e.message, 'err');
    return undefined;
  }
}

const flowK = () => (S.unit === 'mL/min' ? 0.001 : 1);
const flowUnit = () => (S.unit === 'mL/min' ? 'mL/min' : 'µL/min');
// Pressure is always kPa on the wire; display unit is a preference.
const P_K = { kPa: 1, psi: 0.1450377, mmHg: 7.500617, cmH2O: 10.19716 };
const pK = () => P_K[S.punit] || 1;
const pUnit = () => (S.punit === 'cmH2O' ? 'cmH₂O' : S.punit || 'kPa');
function fmtP(kPa, withUnit = false) {
  if (!isNum(kPa)) return '—';
  const x = kPa * pK();
  const d = S.punit === 'psi' ? 3 : Math.abs(x) < 100 ? 2 : 1;
  return x.toFixed(d) + (withUnit ? ' ' + pUnit() : '');
}
function fmt(v, d = 1) { return isNum(v) ? v.toFixed(d) : '—'; }
function fmtFlow(v, withUnit = false) {
  if (!isNum(v)) return '—';
  const x = v * flowK();
  let d;
  if (S.unit === 'mL/min') d = Math.abs(x) < 1 ? 4 : 3;
  else d = Math.abs(x) < 100 ? 2 : 1;
  return x.toFixed(d) + (withUnit ? ' ' + flowUnit() : '');
}
function fmtVol(uL) {
  if (!isNum(uL)) return '—';
  return Math.abs(uL) < 1000 ? uL.toFixed(1) + ' µL' : (uL / 1000).toFixed(3) + ' mL';
}
function fmtDur(s) {
  if (!isNum(s)) return '—';
  s = Math.max(0, Math.round(s));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), x = s % 60;
  return (h ? h + ':' + String(m).padStart(2, '0') : m) + ':' + String(x).padStart(2, '0');
}
function fmtClock(ms) { return new Date(ms).toLocaleTimeString([], { hour12: false }); }
function download(name, text, type = 'text/plain') {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}
function stamp() { return new Date().toISOString().replace(/[:T]/g, '-').slice(0, 19); }
function cssVar(n) { return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }
function num(el) { const v = parseFloat(el.value); return isFinite(v) ? v : NaN; }
const ALARM_TEXT = {
  1: ['Air-in-line', ''], 2: ['Sensor FAULT', ''], 4: ['Sensor high-flow flag', 'warn'],
  8: ['Flow above firmware ceiling', ''], 16: ['Watchdog stopped run', ''], 32: ['E-STOP', ''], 64: ['No flow sensor', ''],
  128: ['Over-pressure (occlusion?)', ''], 256: ['Pressure sensor fault', ''],
};

// --------------------------------------------------------------- tabs
$$('.tabs button').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
function showTab(name) {
  $$('.tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
  $$('.tab').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + name));
  store.set('tab', name);
  if (name === 'cal') pollCal();
  if (name === 'pumps') loadPumps();
  if (name === 'settings') loadHardware();
  if (name === 'data') renderData();
  requestAnimationFrame(resizePlots);
}
new ResizeObserver(() => requestAnimationFrame(resizePlots)).observe(document.querySelector('main'));

// --------------------------------------------------------------- WebSocket
function connect() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
  S.ws = ws;
  ws.onopen = () => {
    setConn(true);
    send({ type: 'time', epochMs: Date.now() });
    loadState();
  };
  ws.onclose = () => {
    setConn(false);
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === 'tel') onTel(m);
    else if (m.type === 'event') onEvent(m);
    else if (m.type === 'hello') $('#simBadge').hidden = !m.sim;
    else if (m.type === 'ack' && !m.ok) toast(m.error, 'err');
  };
}
function send(o) { if (S.ws && S.ws.readyState === 1) S.ws.send(JSON.stringify(o)); }
function heartbeat() {
  const now = Date.now();
  if (now - S.lastHb < 1000) return;
  S.lastHb = now;
  send({ type: 'hb' });
}
// Timers are throttled in background tabs, so we also answer every telemetry
// frame with a heartbeat (incoming messages are not throttled).
setInterval(heartbeat, 1500);

function setConn(on) {
  S.connected = on;
  $('#connDot').className = 'dot ' + (on ? 'on' : 'off');
  $('#connText').textContent = on ? (S.state && S.state.sim ? 'Connected (sim)' : 'Connected') : 'Disconnected, retrying…';
}

// --------------------------------------------------------------- state
let stateTimer = null;
function refreshStateSoon(ms = 600) {
  clearTimeout(stateTimer);
  stateTimer = setTimeout(loadState, ms);
}

async function loadState() {
  let st;
  try { st = await api('GET', 'state'); } catch (e) { return; }
  S.state = st;
  if (!store.get('unitSet', false)) S.unit = st.settings.flowUnit === 'mL/min' ? 'mL/min' : 'uL/min';
  if (!store.get('punitSet', false) && P_K[st.settings.pressureUnit]) S.punit = st.settings.pressureUnit;
  $('#simBadge').hidden = !st.sim;
  $('#simPanel').hidden = !st.sim;
  setConn(S.connected);
  if (S.cards.length !== st.channels.length) buildCards();
  try { S.pumps = (await api('GET', 'pumps')).pumps; } catch { /* keep old */ }
  st.channels.forEach((c, i) => updateCardStatic(i));
  renderSettings();
  renderCalHeader();
  renderEstop(st.estop);
  if (!S.backfilled) {
    S.backfilled = true;
    backfill();
    loadEvents();
  }
}
setInterval(() => { if (S.connected) loadState(); }, 15000);

async function loadEvents() {
  try {
    const r = await api('GET', 'events');
    r.events.forEach((e) => addEvent(e, false));
  } catch { /* ignore */ }
}

// --------------------------------------------------------------- E-STOP
$('#estop').addEventListener('click', async () => {
  send({ type: 'estop' });   // fastest path
  await act(() => api('POST', 'estop'));
  renderEstop(true);
});
$('#estopReset').addEventListener('click', async () => {
  if (!confirm('Reset E-STOP? Channels return to IDLE; nothing restarts automatically.')) return;
  await act(() => api('POST', 'estop/reset'), 'E-STOP reset');
  refreshStateSoon(200);
});
function renderEstop(on) { $('#estopBanner').hidden = !on; }

// --------------------------------------------------------------- events
const events = [];
function onEvent(m) {
  addEvent(m, true);
}
function addEvent(e, live) {
  if (events.some((x) => x.seq === e.seq)) return;
  events.push(e);
  if (events.length > 300) events.shift();
  const li = document.createElement('li');
  li.className = 'lv' + e.level;
  const t = document.createElement('time');
  t.textContent = fmtClock(e.t > 1e12 ? e.t : Date.now());
  const s = document.createElement('span');
  s.textContent = (e.ch ? 'Ch' + e.ch + ': ' : '') + e.msg;
  li.append(t, s);
  $('#eventList').prepend(li);
  $('#eventCount').textContent = '(' + events.length + ')';
  if (live && e.level >= 1) toast((e.ch ? 'Ch' + e.ch + ': ' : '') + e.msg, e.level >= 2 ? 'err' : 'warn');
  if (live) refreshStateSoon(800);
}

// --------------------------------------------------------------- channel cards
function chUi(i) {
  const k = 'ui' + i;
  const d = { mode: 'manual', cmd: 0, flow: 62.5, vol: 15, dur: 4, ol: false, keep: false, rev: false,
    program: [{ kind: 'flow', value: 62.5, durationS: 600 }, { kind: 'flow', value: 30, durationS: 600 }], loop: false };
  return Object.assign(d, store.get(k, {}));
}
function saveUi(i, patch) { store.set('ui' + i, Object.assign(chUi(i), patch)); }

function buildCards() {
  const host = $('#channels');
  host.innerHTML = '';
  S.cards.forEach((c) => c.plot && c.plot.destroy());
  S.cards = [];
  S.state.channels.forEach((ch, i) => {
    const el = $('#tplChannel').content.firstElementChild.cloneNode(true);
    host.appendChild(el);
    const card = { el, i, plot: null, buf: newBuf(), win: store.get('win' + i, 600), paused: false, dirty: true, extra: [] };
    S.cards.push(card);
    wireCard(card);
  });
  resizePlots();
}

function newBuf() { return { t: [], f: [], fa: [], sp: [], c: [], tc: [], p: [], x: {} }; }
const BUF_KEYS = ['t', 'f', 'fa', 'sp', 'c', 'tc', 'p'];

function wireCard(card) {
  const { el, i } = card;
  const ui = chUi(i);
  $('.ch-num', el).textContent = 'Ch ' + (i + 1);

  // mode tabs
  $$('.modes button', el).forEach((b) => b.addEventListener('click', () => setMode(card, b.dataset.mode)));
  setMode(card, ui.mode);

  // manual
  const mc = $('.m-cmd', el), ms = $('.m-slider', el);
  mc.value = ui.cmd;
  ms.value = ui.cmd;
  mc.addEventListener('input', () => { ms.value = mc.value; saveUi(i, { cmd: num(mc) }); });
  ms.addEventListener('input', () => { mc.value = (+ms.value).toFixed(2); saveUi(i, { cmd: num(mc) }); });
  $('.m-rev', el).checked = ui.rev;
  $('.m-rev', el).addEventListener('change', (e) => saveUi(i, { rev: e.target.checked }));

  // flows
  $('.c-flow', el).value = +(ui.flow * flowK()).toPrecision(4);
  $('.o-flow', el).value = +(ui.flow * flowK()).toPrecision(4);
  $('.c-flow', el).addEventListener('input', (e) => saveUi(i, { flow: num(e.target) / flowK() }));
  $('.o-flow', el).addEventListener('input', (e) => saveUi(i, { flow: num(e.target) / flowK() }));

  // volume
  $('.v-vol', el).value = ui.vol;
  $('.v-dur', el).value = ui.dur;
  $('.v-ol', el).checked = ui.ol;
  const upRate = () => {
    const v = num($('.v-vol', el)), h = num($('.v-dur', el));
    saveUi(i, { vol: v, dur: h, ol: $('.v-ol', el).checked });
    const r = v * 1000 / (h * 60);
    $('.v-rate', el).textContent = isNum(r) ? 'Target rate: ' + fmtFlow(r, true) + ' (' + fmt(v / h, 3) + ' mL/h)' : '';
  };
  ['.v-vol', '.v-dur', '.v-ol'].forEach((s) => $(s, el).addEventListener('input', upRate));
  $$('.presets .chip', el).forEach((c) => c.addEventListener('click', () => {
    $('.v-vol', el).value = c.dataset.v;
    $('.v-dur', el).value = c.dataset.h;
    upRate();
  }));
  upRate();

  // steppers (− / +)
  $$('.step', el).forEach((b) => b.addEventListener('click', () => {
    const inp = b.parentElement.querySelector('input');
    const v = num(inp) || 0;
    const step = inp.classList.contains('m-cmd') ? stepFor(card) : (S.unit === 'mL/min' ? 0.001 : 1);
    inp.value = +(Math.max(0, v + step * +b.dataset.delta)).toPrecision(6);
    inp.dispatchEvent(new Event('input'));
  }));

  // program
  card.program = ui.program.slice();
  $('.p-loop', el).checked = ui.loop;
  $('.p-loop', el).addEventListener('change', (e) => saveUi(i, { loop: e.target.checked }));
  $('.p-add', el).addEventListener('click', () => {
    card.program.push({ kind: 'flow', value: 62.5, durationS: 600 });
    renderProgram(card);
  });
  renderProgram(card);

  $('.keep-run', el).checked = ui.keep;
  $('.keep-run', el).addEventListener('change', (e) => saveUi(i, { keep: e.target.checked }));

  // actions
  $('.go', el).addEventListener('click', () => startRun(card));
  $('.stop', el).addEventListener('click', () => act(() => api('POST', `channels/${i + 1}/stop`)));
  $('.pause', el).addEventListener('click', () => {
    const st = S.tel && S.tel.ch[i] ? S.tel.ch[i].state : '';
    act(() => api('POST', `channels/${i + 1}/${st === 'PAUSED' ? 'resume' : 'pause'}`));
  });
  $('.vol-reset', el).addEventListener('click', () => {
    if (confirm('Reset the volume totalizer for Ch ' + (i + 1) + '?')) act(() => api('POST', `channels/${i + 1}/volume-reset`));
  });

  // profile select
  $('.ch-profile', el).addEventListener('change', async (e) => {
    const r = await act(() => api('POST', `channels/${i + 1}/profile`, { id: e.target.value }), 'Profile assigned');
    if (!r) updateCardStatic(i);
    refreshStateSoon(700);
  });

  // zero
  $('.z-go', el).addEventListener('click', () => {
    const sec = num($('.z-sec', el)) || 10;
    if (!confirm(`Zero Ch ${i + 1}? The pump must be stopped and the line full of liquid with no bubbles. Averages ${sec} s.`)) return;
    act(() => api('POST', `sensors/${i + 1}/zero`, { seconds: sec }), 'Zeroing…');
  });
  $('.z-clear', el).addEventListener('click', () => act(() => api('POST', `sensors/${i + 1}/zero`, { clear: true }), 'Offset cleared'));

  // pressure tare + occlusion limit
  $('.pz-go', el).addEventListener('click', () => {
    const sec = num($('.pz-sec', el)) || 5;
    if (!confirm(`Tare the pressure sensor on Ch ${i + 1}? Stop the pump and open the sensor port to air. Averages ${sec} s.`)) return;
    act(() => api('POST', `sensors/${i + 1}/pressure-zero`, { seconds: sec }), 'Taring pressure…');
  });
  $('.pz-clear', el).addEventListener('click', () => act(() => api('POST', `sensors/${i + 1}/pressure-zero`, { clear: true }), 'Pressure tare cleared (standard atmosphere used)'));
  $('.pa-set', el).addEventListener('click', async () => {
    const v = num($('.pa-val', el));
    if (!(v > 0)) return toast('Enter a limit above 0, or use "Turn off"', 'err');
    const r = await act(() => api('POST', `sensors/${i + 1}/pressure-alarm`, { kPa: v / pK() }), 'Occlusion limit set');
    if (r) refreshStateSoon(300);
  });
  $('.pa-off', el).addEventListener('click', async () => {
    const r = await act(() => api('POST', `sensors/${i + 1}/pressure-alarm`, { kPa: 0 }), 'Occlusion alarm off');
    if (r) { $('.pa-val', el).value = ''; refreshStateSoon(300); }
  });

  // plot controls
  $$('.win button', el).forEach((b) => {
    b.classList.toggle('active', +b.dataset.win === card.win);
    b.addEventListener('click', () => {
      card.win = +b.dataset.win;
      store.set('win' + i, card.win);
      $$('.win button', el).forEach((x) => x.classList.toggle('active', x === b));
      card.paused = false;
      $('.plot-pause', el).textContent = 'Pause plot';
      card.dirty = true;
    });
  });
  $('.plot-pause', el).addEventListener('click', () => {
    card.paused = !card.paused;
    $('.plot-pause', el).textContent = card.paused ? 'Resume plot' : 'Pause plot';
    card.dirty = true;
  });
  makePlot(card);
}

function stepFor(card) {
  const p = profileOf(card.i);
  const max = p ? p.motor.maxCommand : 10;
  return max > 50 ? 1 : max > 5 ? 0.1 : 0.01;
}

function profileOf(i) {
  const c = S.state && S.state.channels[i];
  return c ? c.profile : null;
}

function setMode(card, mode) {
  card.mode = mode;
  saveUi(card.i, { mode });
  $$('.modes button', card.el).forEach((b) => b.classList.toggle('active', b.dataset.mode === mode));
  $$('.mode-panel', card.el).forEach((p) => p.classList.toggle('active', p.dataset.panel === mode));
}

function renderProgram(card) {
  const tb = $('.prog tbody', card.el);
  tb.innerHTML = '';
  card.program.forEach((s, k) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${k + 1}</td>
      <td><select><option value="manual">Command</option><option value="flow">Flow (closed)</option><option value="flow_ol">Flow (open)</option></select></td>
      <td><input type="number" step="any"></td><td><input type="number" step="any" min="1"></td>
      <td><button class="link">✕</button></td>`;
    const [sel, val, dur] = [$('select', tr), $$('input', tr)[0], $$('input', tr)[1]];
    sel.value = s.kind;
    val.value = s.kind === 'manual' ? s.value : +(s.value * flowK()).toPrecision(4);
    val.title = s.kind === 'manual' ? 'command' : flowUnit();
    dur.value = s.durationS;
    const save = () => {
      s.kind = sel.value;
      s.value = s.kind === 'manual' ? num(val) : num(val) / flowK();
      s.durationS = num(dur);
      saveUi(card.i, { program: card.program });
    };
    [sel, val, dur].forEach((x) => x.addEventListener('input', save));
    $('button', tr).addEventListener('click', () => {
      card.program.splice(k, 1);
      saveUi(card.i, { program: card.program });
      renderProgram(card);
    });
    if (S.tel && S.tel.ch[card.i] && S.tel.ch[card.i].mode === 'program' && S.tel.ch[card.i].programStep === k) tr.className = 'active-row';
    tb.appendChild(tr);
  });
}

async function startRun(card) {
  const { el, i } = card;
  const keep = $('.keep-run', el).checked;
  let body;
  switch (card.mode) {
    case 'manual':
      body = { mode: 'manual', command: num($('.m-cmd', el)), reverse: $('.m-rev', el).checked };
      break;
    case 'closed_loop':
      body = { mode: 'closed_loop', flow_uLmin: num($('.c-flow', el)) / flowK() };
      if (isNum(num($('.c-init', el)))) body.initialCommand = num($('.c-init', el));
      break;
    case 'open_loop':
      body = { mode: 'open_loop', flow_uLmin: num($('.o-flow', el)) / flowK() };
      break;
    case 'volume':
      body = { mode: 'volume', volume_mL: num($('.v-vol', el)), duration_h: num($('.v-dur', el)), openLoop: $('.v-ol', el).checked };
      if (isNum(num($('.v-init', el)))) body.initialCommand = num($('.v-init', el));
      break;
    case 'program':
      body = { mode: 'program', program: { steps: card.program, loop: $('.p-loop', el).checked } };
      if (isNum(num($('.c-init', el)))) body.initialCommand = num($('.c-init', el));
      break;
  }
  body.keepRunning = keep;
  const r = await act(() => api('POST', `channels/${i + 1}/run`, body));
  if (r) heartbeat();
}

// Profile/hardware-dependent parts of a card (from /api/state).
function updateCardStatic(i) {
  const card = S.cards[i];
  if (!card) return;
  const ch = S.state.channels[i];
  const p = ch.profile;
  const unit = ch.hw.commandUnit;
  const fam = ch.hw.driver === 'drv8871' || ch.hw.driver === 'pwmdir' ? 'dc' : 'stepper';
  const sel = $('.ch-profile', card.el);
  sel.innerHTML = '';
  const compatible = S.pumps.filter((x) => x.family === fam);
  if (!p) sel.append(new Option('— select pump profile —', ''));
  compatible.forEach((x) => sel.append(new Option(x.name + (x.activeCalibration ? '' : ' (uncalibrated)'), x.id)));
  if (p) sel.value = p.id;
  $('.m-unit', card.el).textContent = unit;
  $('.cmd-lbl', card.el).textContent = unit === 'RPM' ? 'Motor RPM' : 'Duty %';
  $$('.flow-unit', card.el).forEach((u) => (u.textContent = flowUnit()));
  const max = p ? p.motor.maxCommand : 0;
  const ms = $('.m-slider', card.el);
  ms.min = 0;
  ms.max = max;
  ms.step = max > 50 ? 0.5 : 0.01;
  const hasCal = ch.status.hasCalibration;
  $$('.nocal', card.el).forEach((n) => (n.hidden = hasCal));
  $('[data-mode="open_loop"]', card.el).title = hasCal ? '' : 'needs an active calibration (or syringe geometry)';
  const hasP = ch.hw.pressure === 'mprls';
  $$('.p-only', card.el).forEach((n) => (n.hidden = !hasP));
  $('.p-unit', card.el).textContent = '(' + pUnit() + ')';
  const lim = ch.status.pressureAlarm_kPa;
  const pa = $('.pa-val', card.el);
  if (document.activeElement !== pa) pa.value = lim > 0 ? +(lim * pK()).toPrecision(4) : '';
  if (card.plotUnit !== unit || card.plotFlowUnit !== S.unit || card.plotHasP !== hasP || card.plotPUnit !== S.punit || extrasChanged(card)) makePlot(card);
}

function extrasChanged(card) {
  const ids = extraSignals(card.i).map((s) => s.id).join(',');
  return ids !== card.extra.map((s) => s.id).join(',');
}
function extraSignals(i) {
  if (!S.state) return [];
  const core = ['f', 'fa', 'sp', 'c', 't', 'v', 'p'].map((p) => p + (i + 1));
  return S.state.signals.filter((s) => s.channel === i + 1 && !core.includes(s.id));
}

// Telemetry → cards, plots, session log.
function onTel(m) {
  heartbeat();
  S.tel = m;
  renderEstop(m.estop);
  m.ch.forEach((c, i) => {
    const card = S.cards[i];
    if (!card) return;
    pushPoint(card, m, c);
    renderCard(card, c);
  });
  logSession(m);
  if (S.calCh < m.ch.length && $('#tab-cal').classList.contains('active')) renderCalLive(m.ch[S.calCh]);
}

function pushPoint(card, m, c) {
  const b = card.buf;
  const t = m.t / 1000;
  if (b.t.length && t <= b.t[b.t.length - 1]) return;
  b.t.push(t);
  b.f.push(c.flow_uLmin);
  b.fa.push(c.flow_avg_uLmin);
  b.sp.push(c.setpoint);
  b.c.push(c.commandActual);
  b.tc.push(c.temp_C);
  b.p.push(isNum(c.pressure_kPa) ? c.pressure_kPa : null);
  card.extra.forEach((s) => {
    (b.x[s.id] = b.x[s.id] || []).push(m.sig ? m.sig[s.id] : null);
  });
  if (b.t.length > 120000) {
    const cut = 20000;
    for (const k of BUF_KEYS) b[k].splice(0, cut);
    for (const k in b.x) b.x[k].splice(0, cut);
  }
  card.dirty = true;
}

function renderCard(card, c) {
  const el = card.el;
  const st = $('.pill.state', el);
  st.textContent = c.state + (c.mode !== 'none' && c.state !== 'IDLE' ? ' · ' + c.mode.replace('_', '-') : '');
  st.className = 'pill state st-' + c.state;
  const sn = $('.pill.sensor', el);
  sn.textContent = 'Sensor ' + c.sensor + (c.zeroing ? ' (zeroing ' + Math.round(c.zeroProgress * 100) + '%)' : '');
  sn.className = 'pill sensor sn-' + c.sensor;

  const al = $('.alarms', el);
  al.innerHTML = '';
  for (const bit in ALARM_TEXT) {
    if (c.alarms & bit) {
      const d = document.createElement('span');
      d.className = 'alarm ' + ALARM_TEXT[bit][1];
      d.textContent = ALARM_TEXT[bit][0];
      al.appendChild(d);
    }
  }
  if (c.profileMismatch) {
    const d = document.createElement('span');
    d.className = 'alarm';
    d.textContent = 'Profile does not match this channel\'s driver';
    al.appendChild(d);
  }

  $('.flow', el).textContent = fmtFlow(c.flow_avg_uLmin) + ' ' + flowUnit();
  $('.noise', el).textContent = isNum(c.flow_std) ? '± ' + fmtFlow(c.flow_std) + ' (1σ)' : '';
  $('.flowi', el).textContent = fmtFlow(c.flow_uLmin);
  $('.cmd', el).textContent = fmt(c.commandActual, c.commandUnit === 'RPM' ? 3 : 1);
  $('.cmdt', el).textContent = 'target ' + fmt(c.command, c.commandUnit === 'RPM' ? 3 : 1) + (c.stepRate_Hz ? ' · ' + fmt(c.stepRate_Hz, 0) + ' steps/s' : '');
  $('.temp', el).textContent = fmt(c.temp_C, 2) + ' °C';
  $('.vol', el).textContent = fmtVol(c.volume_uL) + (c.volumeEstimated ? ' *' : '');
  $('.vol', el).title = c.volumeEstimated ? 'includes estimated volume (sensor unavailable)' : '';
  const vm = c.mode === 'volume';
  $$('.run-only', el).forEach((x) => (x.hidden = !(vm || isNum(c.eta_s))));
  $('.delivered', el).textContent = vm ? fmtVol(c.runVolume_uL) + ' / ' + fmtVol(c.plannedVolume_uL) : '—';
  const pct = vm && c.targetVolume_uL > 0 ? (100 * c.runVolume_uL) / c.targetVolume_uL : NaN;
  $('.delivered-sub', el).textContent = vm ? 'target ' + fmtVol(c.targetVolume_uL) + (isNum(pct) ? ' · ' + pct.toFixed(1) + ' % done' : '') : '';
  $('.eta', el).textContent = isNum(c.eta_s) ? fmtDur(c.eta_s) : '—';
  $('.elapsed', el).textContent = isNum(c.eta_s) ? 'elapsed ' + fmtDur(c.elapsed_s) : '';

  const running = c.state === 'RUNNING' || c.state === 'PAUSED';
  const locked = c.state === 'ESTOP' || c.state === 'CALIBRATING';
  $('.go', el).textContent = running ? 'Apply' : 'Start';
  $('.go', el).disabled = locked || c.state === 'FAULT';
  $('.pause', el).textContent = c.state === 'PAUSED' ? 'Resume' : 'Pause';
  $('.pause', el).disabled = !running;
  $('.ch-profile', el).disabled = running || locked || c.zeroing;
  if (running && card.mode !== c.mode && !card.modeSynced) setMode(card, c.mode);
  card.modeSynced = running;
  if (c.mode === 'program' && card.lastStep !== c.programStep) {
    card.lastStep = c.programStep;
    renderProgram(card);
  }
  if ('pressure_kPa' in c) {
    $('.pres', el).textContent = fmtP(c.pressure_kPa) + ' ' + pUnit();
    $('.pres-sub', el).textContent = (c.pressureSaturated ? 'OVER RANGE · ' : '') + (c.pressureTared ? 'abs ' + fmtP(c.pressureAbs_kPa, true) : 'untared: tare with the line open to air') +
      (c.pressureAlarm_kPa > 0 ? ' · limit ' + fmtP(c.pressureAlarm_kPa, true) : '');
    const ps = $('.pill.psensor', el);
    ps.textContent = 'Pressure ' + c.pressureSensor + (c.pressureTaring ? ' (taring ' + Math.round(c.pressureTareProgress * 100) + '%)' : '');
    ps.className = 'pill psensor p-only sn-' + c.pressureSensor;
    $('.p-info', el).textContent = `tare (atmosphere) ${c.pressureTared ? fmtP(c.pressureZero_kPa, true) : 'not set: standard atmosphere assumed'} · ` +
      `occlusion limit ${c.pressureAlarm_kPa > 0 ? fmtP(c.pressureAlarm_kPa, true) : 'off'} · frame errors ${c.pressureErrors.frame} · I²C errors ${c.pressureErrors.i2c} · re-inits ${c.pressureErrors.reinits}`;
  }
  $('.sensor-info', el).textContent =
    `offset ${fmtFlow(c.zeroOffset, true)} · CRC errors ${c.errors.crc} · I²C errors ${c.errors.i2c} · re-inits ${c.errors.reinits}` +
    (isNum(c.ff) ? ` · feed-fwd ${fmt(c.ff, 3)} · correction ${fmtFlow(c.corr, true)}` : '');
}

// --------------------------------------------------------------- plots
function makePlot(card) {
  const host = $('.plot', card.el);
  if (card.plot) card.plot.destroy();
  host.innerHTML = '';
  const ch = S.state && S.state.channels[card.i];
  const unit = ch ? ch.hw.commandUnit : '';
  card.plotUnit = unit;
  card.plotFlowUnit = S.unit;
  card.extra = extraSignals(card.i);
  const k = flowK();
  const pk = pK();
  const hasP = !!(ch && ch.hw.pressure === 'mprls');
  card.plotHasP = hasP;
  card.plotPUnit = S.punit;
  const fv = (u, v) => (isNum(v) ? (v * k).toPrecision(4) : '—');
  const muted = cssVar('--muted'), line = cssVar('--line');
  const series = [
    {},
    { label: 'Flow', scale: 'flow', stroke: cssVar('--s-flow'), width: 1, value: fv, alpha: 0.5 },
    { label: 'Avg', scale: 'flow', stroke: cssVar('--s-avg'), width: 2, value: fv },
    { label: 'Setpoint', scale: 'flow', stroke: cssVar('--s-sp'), width: 1.5, dash: [6, 4], value: fv },
    { label: unit || 'Cmd', scale: 'cmd', stroke: cssVar('--s-cmd'), width: 1.5 },
    { label: '°C', scale: 'temp', stroke: cssVar('--s-temp'), width: 1, show: false },
  ];
  if (hasP) series.push({ label: 'Pressure', scale: 'pres', stroke: cssVar('--s-pres'), width: 1.5, value: (u, v) => (isNum(v) ? (v * pk).toPrecision(4) : '—') });
  const palette = ['#0891b2', '#be185d', '#4d7c0f', '#9333ea'];
  card.extra.forEach((s, n) => series.push({ label: s.name + ' (' + s.unit + ')', scale: 'u_' + s.unit, stroke: palette[n % 4], width: 1.5 }));
  const opts = {
    // Hidden tabs report 0 width; fall back to the viewport and fix on resize.
    width: Math.max(260, host.clientWidth || Math.min(card.el.clientWidth - 30, window.innerWidth - 64) || 300),
    height: 240,
    series,
    scales: { x: { time: true } },
    axes: [
      { stroke: muted, grid: { stroke: line, width: 1 }, ticks: { stroke: line } },
      { scale: 'flow', stroke: muted, grid: { stroke: line, width: 1 }, ticks: { stroke: line }, size: 56,
        values: (u, sp) => sp.map((v) => +(v * k).toPrecision(4)), label: flowUnit(), labelSize: 18 },
      { scale: 'cmd', side: 1, stroke: muted, grid: { show: false }, ticks: { stroke: line }, size: 50, label: unit, labelSize: 18 },
    ].concat(hasP ? [{ scale: 'pres', side: 1, stroke: cssVar('--s-pres'), grid: { show: false }, ticks: { stroke: line }, size: 52,
      values: (u, sp) => sp.map((v) => +(v * pk).toPrecision(3)), label: pUnit(), labelSize: 18 }] : []),
    cursor: { drag: { x: true, y: false, setScale: true } },
    legend: { live: true },
    hooks: {
      setSelect: [() => { card.paused = true; $('.plot-pause', card.el).textContent = 'Resume plot'; }],
    },
  };
  card.plot = new uPlot(opts, plotData(card), host);
  card.dirty = true;
}

function plotData(card) {
  const b = card.buf;
  const d = [b.t, b.f, b.fa, b.sp, b.c, b.tc];
  if (card.plotHasP) d.push(b.p);
  card.extra.forEach((s) => {
    const arr = b.x[s.id] || [];
    while (arr.length < b.t.length) arr.unshift(null);
    b.x[s.id] = arr;
    d.push(arr);
  });
  return d;
}

function resizePlots() {
  S.cards.forEach((c) => {
    if (!c.plot) return;
    const w = $('.plot', c.el).clientWidth;
    if (w > 0) c.plot.setSize({ width: w, height: 240 });
  });
  if (S.calPlot) drawCal();
}
window.addEventListener('resize', () => requestAnimationFrame(resizePlots));

// Redraw at ≤ 2 Hz to keep phones cool.
setInterval(() => {
  if (!$('#tab-dash').classList.contains('active')) return;
  S.cards.forEach((card) => {
    if (!card.plot || !card.dirty || card.paused) return;
    card.dirty = false;
    const b = card.buf;
    card.plot.setData(plotData(card), false);
    if (b.t.length) {
      const max = b.t[b.t.length - 1];
      // Until the buffer spans the window, fit the data instead of squashing it.
      const min = card.win > 0 ? Math.max(max - card.win, b.t[0]) : b.t[0];
      card.plot.setScale('x', { min, max });
    }
  });
}, 500);

async function backfill() {
  if (!S.state) return;
  for (let i = 0; i < S.cards.length; i++) {
    try {
      const h = await api('GET', 'history?ch=' + (i + 1));
      if (!h.synced) continue;   // timestamps are uptime; don't mix with wall-clock
      const b = S.cards[i].buf;
      const first = b.t.length ? b.t[0] : Infinity;
      const pre = newBuf();
      h.rows.forEach((r) => {
        const t = r[0] / 1000;
        if (t >= first) return;
        pre.t.push(t); pre.f.push(r[1]); pre.fa.push(r[1]); pre.sp.push(r[2]); pre.c.push(r[3]); pre.tc.push(r[4]);
        pre.p.push(r.length > 6 ? r[6] : null);
      });
      for (const k of BUF_KEYS) b[k] = pre[k].concat(b[k]);
      S.cards[i].dirty = true;
    } catch { /* history is optional */ }
  }
}

// --------------------------------------------------------------- session log
function logSession(m) {
  const sigs = S.state ? S.state.signals : [];
  if (!S.session.cols) {
    S.session.cols = sigs.map((s) => s.id);
    S.session.header = ['time_ms', 'iso_time']
      .concat(sigs.map((s) => `${s.id}_${s.name.replace(/\s+/g, '_')}_${s.unit.replace('/', 'per').replace('%', 'pct').replace(/[^A-Za-z0-9]/g, '')}`))
      .concat(m.ch.flatMap((_, i) => [`ch${i + 1}_state`, `ch${i + 1}_mode`, `ch${i + 1}_air_in_line`, `ch${i + 1}_sensor_flags`, `ch${i + 1}_alarms`]));
  }
  const row = [m.t].concat(S.session.cols.map((id) => (m.sig && isNum(m.sig[id]) ? m.sig[id] : null)));
  m.ch.forEach((c) => row.push(c.state, c.mode, (c.alarms & 1) ? 1 : 0, c.flags, c.alarms));
  S.session.rows.push(row);
  if ($('#tab-data').classList.contains('active') && S.session.rows.length % 10 === 0) renderData();
}

function sessionCsv() {
  const lines = [S.session.header.join(',')];
  for (const r of S.session.rows) {
    const t = r[0];
    const iso = t > 1e12 ? new Date(t).toISOString() : '';
    lines.push([t, iso].concat(r.slice(1).map((v) => (v === null || v === undefined ? '' : typeof v === 'number' ? +v.toPrecision(7) : v))).join(','));
  }
  return lines.join('\n') + '\n';
}

function renderData() {
  const n = S.session.rows.length;
  const dur = n > 1 ? (S.session.rows[n - 1][0] - S.session.rows[0][0]) / 1000 : 0;
  $('#sessionInfo').textContent = `${n} rows · ${fmtDur(dur)} recorded · ${S.session.header ? S.session.header.length : 0} columns`;
  const h = S.state ? S.state.history : null;
  $('#historyInfo').textContent = h ? `${h.size} / ${h.capacity} samples at 1 Hz (≈ ${Math.round(h.capacity / 60)} min ring buffer)` : '';
}
$('#dlSession').addEventListener('click', () => {
  if (!S.session.rows.length) return toast('Session log is empty');
  download(`biopump-session-${stamp()}.csv`, sessionCsv(), 'text/csv');
});
$('#clearSession').addEventListener('click', () => {
  if (!confirm('Clear the session log in this browser?')) return;
  S.session = { rows: [], cols: null };
  renderData();
});
$('#clearHistory').addEventListener('click', async () => {
  if (!confirm('Clear the ESP32 history buffer (all clients)?')) return;
  await act(() => api('DELETE', 'history'), 'History cleared');
  loadState().then(renderData);
});

// --------------------------------------------------------------- calibration
const calInputs = ['calMin', 'calMax', 'calPoints', 'calSettle', 'calAvg', 'calRepeats', 'calList', 'calSsWin', 'calSsTimeout', 'calSsSlope', 'calSsStd', 'calSsStdFloor', 'calZeroS'];
calInputs.forEach((id) => {
  const v = store.get('cal.' + id, null);
  if (v !== null) $('#' + id).value = v;
  $('#' + id).addEventListener('input', () => { store.set('cal.' + id, $('#' + id).value); estimateCal(); });
});
$('#calCh').addEventListener('change', (e) => {
  S.calCh = +e.target.value;
  S.calStatus = null;
  renderCalHeader();
  pollCal();
});
$$('#fitSeg button').forEach((b) => b.addEventListener('click', () => {
  S.fit = b.dataset.fit;
  $$('#fitSeg button').forEach((x) => x.classList.toggle('active', x === b));
  renderCalStatus();
}));

function renderCalHeader() {
  if (!S.state) return;
  const sel = $('#calCh');
  if (sel.options.length !== S.state.channels.length) {
    sel.innerHTML = '';
    S.state.channels.forEach((c, i) => sel.append(new Option('Ch ' + (i + 1), i)));
    const sc = $('#simCh');
    sc.innerHTML = sel.innerHTML;
  }
  sel.value = S.calCh;
  const ch = S.state.channels[S.calCh];
  const p = ch.profile;
  $('#calProfile').textContent = p ? `${p.name} · ${ch.hw.commandUnit} · max ${p.motor.maxCommand}` : 'No pump profile on this channel';
  $$('[data-cal-unit]').forEach((u) => (u.textContent = '(' + ch.hw.commandUnit + ')'));
  if (p && !$('#calMax').value) $('#calMax').value = p.motor.maxCommand;
  if (p && !$('#calMin').value) $('#calMin').value = p.motor.minCommand || 0;
  $('#calHint').textContent = 'Prime the line and remove bubbles first. Calibration runs on the ESP32, so it keeps going if your phone locks.' +
    (p && p.pumpType === 'diaphragm' ? ' Pulsatile pump: use averaging ≥ 60 s.' : '') +
    ' Points that read below ~50 µL/min are dominated by the sensor error floor; add manual references there.';
  estimateCal();
}

function calCommands() {
  const list = $('#calList').value.trim();
  if (list) return list.split(/[,\s]+/).map(parseFloat).filter(isFinite);
  const lo = num($('#calMin')), hi = num($('#calMax')), n = Math.round(num($('#calPoints')));
  if (!(hi > lo) || !(n >= 2)) return [];
  return Array.from({ length: n }, (_, k) => +(lo + (hi - lo) * k / (n - 1)).toPrecision(6));
}

function estimateCal() {
  const n = calCommands().length * (num($('#calRepeats')) || 1) * ($('#calUpDown').checked ? 2 : 1);
  const t = n * ((num($('#calSettle')) || 0) + (num($('#calAvg')) || 0)) + ($('#calZero').checked ? 15 + (num($('#calZeroS')) || 30) : 0);
  $('#calEstimate').textContent = n ? `${n} points · ≈ ${fmtDur(t)} (plus steady-state waits)` : '';
}
['calUpDown', 'calZero'].forEach((id) => $('#' + id).addEventListener('change', estimateCal));

$('#calStart').addEventListener('click', async () => {
  const cmds = calCommands();
  if (cmds.length < 2) return toast('Need at least 2 command points', 'err');
  const body = {
    commands: cmds,
    settleS: num($('#calSettle')), averageS: num($('#calAvg')), repeats: num($('#calRepeats')) || 1,
    upDown: $('#calUpDown').checked, zeroFirst: $('#calZero').checked, keepRunning: $('#calKeep').checked,
    reverse: $('#calReverse').checked, zeroS: num($('#calZeroS')),
    steadyWindowS: num($('#calSsWin')), steadyTimeoutS: num($('#calSsTimeout')),
    maxSlopePctPerMin: num($('#calSsSlope')), maxStdPct: num($('#calSsStd')), stdFloor_uLmin: num($('#calSsStdFloor')),
  };
  const r = await act(() => api('POST', `calibration/${S.calCh + 1}/start`, body), 'Sweep started');
  if (r) pollCal();
});
$('#calAbort').addEventListener('click', () => {
  if (confirm('Abort the sweep? The motor stops; measured points are kept.')) act(() => api('POST', `calibration/${S.calCh + 1}/abort`)).then(pollCal);
});
$('#calDiscard').addEventListener('click', () => {
  if (confirm('Discard all sweep data for this channel?')) act(() => api('POST', `calibration/${S.calCh + 1}/discard`)).then(pollCal);
});
$('#calAccept').addEventListener('click', async () => {
  const fit = S.calStatus && S.calStatus.fits[S.fit];
  if (fit && fit.nonMonotone && !confirm('The data is not monotone; the lookup table was repaired. Accept anyway?')) return;
  const r = await act(() => api('POST', `calibration/${S.calCh + 1}/accept`, {
    fitType: S.fit, name: $('#calName').value.trim(), tubingNote: $('#calNote').value.trim(),
  }), 'Calibration saved and set active');
  if (r) { pollCal(); refreshStateSoon(800); }
});
$('#calExportCsv').addEventListener('click', () => {
  const s = S.calStatus;
  if (!s || !s.points.length) return toast('No calibration data');
  const lines = ['index,command,pass,status,sensor_flow_uLmin,sensor_std_uLmin,temp_C,n_samples,reference_flow_uLmin,used_flow_uLmin'];
  s.points.forEach((p) => lines.push([p.i + 1, p.command, p.pass ? 'down' : 'up', p.status, p.flow ?? '', p.std ?? '', p.temp ?? '', p.n, p.ref ?? '', p.ref ?? p.flow ?? ''].join(',')));
  const f = s.fits[S.fit];
  if (f && f.coefficients) lines.push('', `# fit,${S.fit},c0,${f.coefficients[0]},c1,${f.coefficients[1]},c2,${f.coefficients[2]},r2,${f.r2},rmse,${f.rmse},maxErrPct,${f.maxErrPct}`);
  download(`calibration-ch${S.calCh + 1}-${stamp()}.csv`, lines.join('\n') + '\n', 'text/csv');
});
$('#calExportJson').addEventListener('click', () => {
  if (!S.calStatus) return toast('No calibration data');
  download(`calibration-ch${S.calCh + 1}-${stamp()}.json`, JSON.stringify(S.calStatus, null, 2), 'application/json');
});

let calTimer = null;
async function pollCal() {
  clearTimeout(calTimer);
  if (!$('#tab-cal').classList.contains('active')) return;
  try {
    const r = await api('GET', `calibration/${S.calCh + 1}/status`);
    S.calStatus = r.status;
    renderCalStatus();
  } catch { /* retry */ }
  calTimer = setTimeout(pollCal, S.calStatus && S.calStatus.running ? 2000 : 6000);
}

function renderCalLive(c) {
  // cheap per-telemetry update of the progress line
  if (!S.calStatus || !S.calStatus.running) return;
  const k = c.cal;
  $('#calPhase').textContent = `${k.phase.replace('_', ' ')} · point ${k.index + 1}/${k.points} · ${fmtDur(k.elapsed)} / ${fmtDur(k.duration)} · flow ${fmtFlow(c.flow_avg_uLmin, true)} at ${fmt(c.commandActual, 3)} ${c.commandUnit}`;
}

function renderCalStatus() {
  const s = S.calStatus;
  const tb = $('#calTable tbody');
  tb.innerHTML = '';
  if (!s) return;
  const done = s.points.filter((p) => p.status !== 'pending' && p.status !== 'active').length;
  $('#calBar').style.width = (s.points.length ? (100 * done) / s.points.length : 0) + '%';
  if (!s.running) {
    $('#calPhase').textContent = s.points.length
      ? `${s.phase} · ${s.validPoints} valid of ${s.points.length} points · zero offset ${fmtFlow(s.zeroOffset, true)}`
      : 'No sweep data.';
  } else {
    $('#calPhase').textContent = `${s.phase.replace('_', ' ')} · point ${s.index + 1}/${s.points.length} · ≈ ${fmtDur(s.remainingS)} remaining`;
  }
  $('#calStart').disabled = s.running;
  $('#calAbort').disabled = !s.running;
  $('#calAccept').disabled = s.running || s.validPoints < 2;

  const refHead = S.refMode === 'volume' ? 'µL collected / s' : flowUnit();
  $('#calTable thead th:nth-child(8)').innerHTML = `Manual reference <button class="link" id="refToggle">(${refHead} ⇄)</button>`;
  $('#refToggle').onclick = () => { S.refMode = S.refMode === 'volume' ? 'flow' : 'volume'; renderCalStatus(); };
  s.points.forEach((p) => {
    const tr = document.createElement('tr');
    if (p.status === 'active') tr.className = 'active-row';
    const canRef = p.status !== 'pending' && p.status !== 'active';
    tr.innerHTML = `<td>${p.i + 1}</td><td class="num">${fmt(p.command, 3)}</td><td>${p.pass ? '↓' : '↑'}</td>
      <td class="st-${p.status}">${p.status.replace('_', ' ')}</td>
      <td class="num">${fmtFlow(p.flow)}</td><td class="num">${fmtFlow(p.std)}</td><td class="num">${fmtFlow(p.ref)}</td>
      <td><div class="ref-cell"></div></td><td></td>`;
    const cell = $('.ref-cell', tr);
    if (canRef && !s.running) {
      const a = document.createElement('input');
      a.type = 'number'; a.step = 'any';
      a.placeholder = S.refMode === 'volume' ? 'µL' : flowUnit();
      cell.append(a);
      let b = null;
      if (S.refMode === 'volume') {
        b = document.createElement('input');
        b.type = 'number'; b.step = 'any'; b.placeholder = 's';
        cell.append(b);
      }
      const set = document.createElement('button');
      set.className = 'btn small';
      set.textContent = 'Set';
      set.onclick = () => {
        const body = { index: p.i };
        if (b) { body.volume_uL = num(a); body.duration_s = num(b); } else body.flow_uLmin = num(a) / flowK();
        act(() => api('POST', `calibration/${S.calCh + 1}/point`, body)).then(pollCal);
      };
      cell.append(set);
      if (isNum(p.ref)) {
        const clr = document.createElement('button');
        clr.className = 'link';
        clr.textContent = '✕';
        clr.title = 'clear reference';
        clr.onclick = () => act(() => api('POST', `calibration/${S.calCh + 1}/point`, { index: p.i, clear: true })).then(pollCal);
        cell.append(clr);
      }
      const retry = document.createElement('button');
      retry.className = 'btn small';
      retry.textContent = 'Retry';
      retry.onclick = () => act(() => api('POST', `calibration/${S.calCh + 1}/retry`, { index: p.i, keepRunning: $('#calKeep').checked })).then(pollCal);
      tr.lastElementChild.append(retry);
    }
    tb.appendChild(tr);
  });

  const f = s.fits[S.fit];
  const unit = s.commandUnit;
  if (f && f.coefficients) {
    const c = f.coefficients;
    let eq;
    if (S.fit === 'linear') eq = `flow = ${c[0].toPrecision(4)} + ${c[1].toPrecision(4)}·x`;
    else if (S.fit === 'quadratic') eq = `flow = ${c[0].toPrecision(4)} + ${c[1].toPrecision(4)}·x + ${c[2].toPrecision(4)}·x²`;
    else eq = `lookup table, ${f.lookup.x.length} points`;
    $('#fitStats').innerHTML = `${eq} (µL/min, x in ${unit})<br>R² ${fmt(f.r2, 5)} · RMSE ${fmtFlow(f.rmse, true)} · max err ${fmt(f.maxErrPct, 1)} %` +
      (f.nonMonotone ? ' · <span class="st-not_steady">non-monotone data repaired</span>' : '') +
      (f.usable === false ? ' · <span class="st-air">not usable (not increasing)</span>' : '');
  } else {
    $('#fitStats').textContent = f && f.error ? f.error : '';
  }
  S.calPlot = true;
  drawCal();
}

// Minimal canvas XY plotter for the calibration scatter/fit and residuals.
function drawXY(host, cfg) {
  let cv = host.querySelector('canvas');
  if (!cv) { cv = document.createElement('canvas'); host.appendChild(cv); }
  const W = Math.max(280, host.clientWidth), H = cfg.height || 260, dpr = window.devicePixelRatio || 1;
  cv.width = W * dpr; cv.height = H * dpr; cv.style.width = W + 'px'; cv.style.height = H + 'px';
  const g = cv.getContext('2d');
  g.scale(dpr, dpr);
  g.clearRect(0, 0, W, H);
  const pts = cfg.series.flatMap((s) => s.pts);
  if (!pts.length) {
    g.fillStyle = cssVar('--muted'); g.font = '13px system-ui';
    g.fillText(cfg.empty || 'No data yet', 12, 24);
    return;
  }
  let [x0, x1] = [Math.min(...pts.map((p) => p[0])), Math.max(...pts.map((p) => p[0]))];
  let [y0, y1] = [Math.min(...pts.map((p) => p[1])), Math.max(...pts.map((p) => p[1]))];
  if (cfg.zeroLine) { y0 = Math.min(y0, 0); y1 = Math.max(y1, 0); }
  if (x1 === x0) { x0 -= 1; x1 += 1; }
  if (y1 === y0) { y0 -= 1; y1 += 1; }
  const py = (y1 - y0) * 0.08; y0 -= py; y1 += py;
  const L = 56, R = 12, T = 10, B = 34;
  const X = (x) => L + ((x - x0) / (x1 - x0)) * (W - L - R);
  const Y = (y) => H - B - ((y - y0) / (y1 - y0)) * (H - T - B);
  g.strokeStyle = cssVar('--line'); g.fillStyle = cssVar('--muted'); g.font = '11px system-ui'; g.lineWidth = 1;
  const ticks = (a, b, n) => { const st = Math.pow(10, Math.floor(Math.log10((b - a) / n))); const m = [1, 2, 5, 10].find((k) => (b - a) / (k * st) <= n) * st; const out = []; for (let v = Math.ceil(a / m) * m; v <= b; v += m) out.push(+v.toPrecision(6)); return out; };
  ticks(x0, x1, 6).forEach((v) => { g.beginPath(); g.moveTo(X(v), T); g.lineTo(X(v), H - B); g.stroke(); g.fillText(v, X(v) - 8, H - B + 14); });
  ticks(y0, y1, 5).forEach((v) => { g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(W - R, Y(v)); g.stroke(); g.fillText(+(v * (cfg.yk || 1)).toPrecision(4), 4, Y(v) + 4); });
  if (cfg.zeroLine) { g.strokeStyle = cssVar('--muted'); g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(W - R, Y(0)); g.stroke(); }
  g.fillText(cfg.xlabel || '', W / 2 - 30, H - 4);
  g.save(); g.translate(12, T + 60); g.rotate(-Math.PI / 2); g.restore();
  cfg.series.forEach((s) => {
    g.strokeStyle = s.color; g.fillStyle = s.color; g.lineWidth = 2;
    if (s.line) {
      g.beginPath();
      s.pts.forEach((p, k) => (k ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))));
      g.stroke();
    } else {
      s.pts.forEach((p) => {
        g.beginPath();
        if (s.square) g.rect(X(p[0]) - 4, Y(p[1]) - 4, 8, 8); else g.arc(X(p[0]), Y(p[1]), 4, 0, 7);
        s.hollow ? g.stroke() : g.fill();
      });
    }
  });
  let lx = L + 8;
  cfg.series.forEach((s) => {
    if (!s.label) return;
    g.fillStyle = s.color; g.fillRect(lx, T + 4, 10, 10);
    g.fillStyle = cssVar('--text'); g.fillText(s.label, lx + 14, T + 13);
    lx += g.measureText(s.label).width + 30;
  });
}

function evalFit(f, x) {
  if (f.fitType !== 'lookup') return f.coefficients[0] + f.coefficients[1] * x + f.coefficients[2] * x * x;
  const xs = f.lookup.x, ys = f.lookup.y, n = xs.length;
  let i = 0;
  if (x >= xs[n - 1]) i = n - 2; else while (i < n - 2 && x > xs[i + 1]) i++;
  const dx = xs[i + 1] - xs[i];
  return dx > 0 ? ys[i] + ((ys[i + 1] - ys[i]) * (x - xs[i])) / dx : ys[i];
}

function drawCal() {
  const s = S.calStatus;
  if (!s) return;
  const k = flowK();
  const sensor = s.points.filter((p) => isNum(p.flow)).map((p) => [p.command, p.flow]);
  const refs = s.points.filter((p) => isNum(p.ref)).map((p) => [p.command, p.ref]);
  const bad = s.points.filter((p) => isNum(p.flow) && !['ok', 'not_steady'].includes(p.status) && !isNum(p.ref)).map((p) => [p.command, p.flow]);
  const series = [
    { label: 'Sensor', pts: sensor, color: cssVar('--s-avg') },
    { label: 'Reference', pts: refs, color: cssVar('--s-sp'), square: true },
    { label: 'Invalid', pts: bad, color: cssVar('--danger'), hollow: true },
  ];
  const f = s.fits[S.fit];
  if (f && f.coefficients && sensor.length + refs.length > 1) {
    const xs = s.points.map((p) => p.command);
    const a = Math.min(...xs), b = Math.max(...xs);
    const line = [];
    for (let j = 0; j <= 60; j++) { const x = a + ((b - a) * j) / 60; line.push([x, evalFit(f, x)]); }
    series.push({ label: 'Fit', pts: line, color: cssVar('--s-flow'), line: true });
  }
  drawXY($('#calPlot'), { series, xlabel: 'command (' + s.commandUnit + ')', yk: k, empty: 'Start a sweep to see flow vs command' });
  const used = s.points.filter((p) => isNum(p.ref) || ['ok', 'not_steady'].includes(p.status));
  const res = f && f.residuals ? f.residuals.map((r, j) => [used[j] ? used[j].command : j, r]) : [];
  drawXY($('#resPlot'), { series: [{ label: 'Residual (' + flowUnit() + ')', pts: res, color: cssVar('--s-cmd') }], xlabel: 'command', yk: k, zeroLine: true, height: 180, empty: 'Residuals appear after ≥ 2 valid points' });
}

// --------------------------------------------------------------- pumps
async function loadPumps() {
  try { S.pumps = (await api('GET', 'pumps')).pumps; } catch (e) { return toast(e.message, 'err'); }
  const host = $('#pumpList');
  host.innerHTML = '';
  const assigned = {};
  (S.state ? S.state.channels : []).forEach((c, i) => { if (c.profile) (assigned[c.profile.id] = assigned[c.profile.id] || []).push('Ch' + (i + 1)); });
  S.pumps.forEach((p) => {
    const d = document.createElement('div');
    d.className = 'pump-item';
    d.innerHTML = `<div class="name"></div><div class="meta"></div>
      <div class="row wrap"><button class="btn small e">Edit</button><button class="btn small d">Duplicate</button>
      <button class="btn small x">Export</button><button class="btn small danger r">Delete</button></div>`;
    $('.name', d).textContent = p.name;
    $('.meta', d).textContent = `${p.pumpType} · ${p.driverType} · ${p.calibrations} calibration(s)` +
      (p.activeCalibration ? ` · active: ${p.activeCalibration}` : ' · uncalibrated') + (assigned[p.id] ? ' · on ' + assigned[p.id].join(', ') : '');
    $('.e', d).onclick = () => editPump(p.id);
    $('.d', d).onclick = () => act(() => api('POST', `pumps/${p.id}/duplicate`), 'Duplicated').then(() => setTimeout(loadPumps, 400));
    $('.x', d).onclick = async () => {
      const j = await act(() => api('GET', 'pumps/' + p.id));
      if (j) download(`pump-${p.id}.json`, JSON.stringify(j, null, 2), 'application/json');
    };
    $('.r', d).onclick = () => {
      if (confirm(`Delete "${p.name}" and all its calibrations?`)) act(() => api('DELETE', 'pumps/' + p.id), 'Deleted').then(() => setTimeout(loadPumps, 400));
    };
    host.appendChild(d);
  });
}

$('#pumpNew').addEventListener('click', () => {
  editPumpData({
    name: 'New pump', pumpType: 'syringe', notes: '', fluid: 'water',
    motor: { driverType: 'stepdir', stepsPerRev: 200, microstep: 16, gearRatio: 1, maxCommand: 2, minCommand: 0, accel: 5, invertDir: false, decay: 'fast', minDuty: 0, kickMs: 0, kickDuty: 80, releaseWhenStopped: false },
    syringe: { innerDiameter_mm: 0, leadscrewPitch_mm: 0 },
    pid: { kp: 0.3, ki: 0.05, kd: 0, loopHz: 1, filterWindowS: 10, filter: 'ma', outputLimitPct: 100, integralLimitPct: 100 },
    calibrations: [],
  }, true);
});
$('#pumpImport').addEventListener('change', async (e) => {
  const f = e.target.files[0];
  e.target.value = '';
  if (!f) return;
  let j;
  try { j = JSON.parse(await f.text()); } catch { return toast('Not a JSON file', 'err'); }
  const r = await act(() => api('POST', 'pumps', j), 'Imported');
  if (r) setTimeout(loadPumps, 400);
});
$('#pumpCancel').addEventListener('click', () => { $('#pumpForm').hidden = true; S.editing = null; });

async function editPump(id) {
  const j = await act(() => api('GET', 'pumps/' + id));
  if (j) editPumpData(j, false);
}

function getPath(o, p) { return p.split('.').reduce((a, k) => (a == null ? a : a[k]), o); }
function setPath(o, p, v) { const ks = p.split('.'); let a = o; ks.slice(0, -1).forEach((k) => (a = a[k] = a[k] || {})); a[ks[ks.length - 1]] = v; }

function editPumpData(p, isNew) {
  S.editing = { data: JSON.parse(JSON.stringify(p)), isNew };
  const f = $('#pumpForm');
  f.hidden = false;
  $('#pumpFormTitle').textContent = isNew ? 'New profile' : 'Edit profile: ' + p.name;
  $$('[name]', f).forEach((el) => {
    const v = getPath(p, el.name);
    if (el.type === 'checkbox') el.checked = !!v;
    else el.value = v === undefined || v === null ? '' : v;
  });
  updatePumpFormMode();
  renderCalList();
  f.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function updatePumpFormMode() {
  const f = $('#pumpForm');
  const drv = f.elements['motor.driverType'].value;
  const dc = drv === 'drv8871' || drv === 'pwmdir';
  f.classList.toggle('dc-mode', dc);
  f.classList.toggle('stepper-mode', !dc);
  $$('[data-motor-unit]', f).forEach((u) => (u.textContent = dc ? '(duty %)' : '(RPM)'));
  $$('[data-motor-unit-s]', f).forEach((u) => (u.textContent = dc ? '(%/s)' : '(RPM/s)'));
  const d = parseFloat(f.elements['syringe.innerDiameter_mm'].value), pitch = parseFloat(f.elements['syringe.leadscrewPitch_mm'].value);
  const th = d > 0 && pitch > 0 ? (Math.PI / 4) * d * d * pitch : NaN;
  $('#theory').textContent = isNum(th) ? `${th.toFixed(2)} µL/min per RPM` : '—';
  $('.pulsatile-hint', f).hidden = f.elements.pumpType.value !== 'diaphragm' && !dc;
}
$('#pumpForm').addEventListener('input', updatePumpFormMode);

function renderCalList() {
  const host = $('#calList2');
  host.innerHTML = '';
  const cals = S.editing.data.calibrations || [];
  if (!cals.length) host.innerHTML = '<p class="hint">No calibrations yet; run one from the Calibrate tab.</p>';
  cals.forEach((c, k) => {
    const d = document.createElement('div');
    d.className = 'cal-item';
    const when = c.timestamp ? new Date(c.timestamp * 1000).toLocaleString() : 'unknown date';
    d.innerHTML = `<label class="inline"><input type="radio" name="activeCal"> active</label>
      <span class="grow"><strong></strong><br><span class="muted"></span></span>
      <button type="button" class="link">delete</button>`;
    $('strong', d).textContent = c.name || c.id;
    $('.muted', d).textContent = `${when} · ${c.fitType} · R² ${fmt(c.r2, 4)} · max err ${fmt(c.maxErrPct, 1)} % · ${c.fluid}${c.tubingNote ? ' · ' + c.tubingNote : ''} · ${c.points.length} pts`;
    const r = $('input', d);
    r.checked = !!c.active;
    r.onchange = () => cals.forEach((x, j) => (x.active = j === k));
    $('button', d).onclick = () => { if (confirm('Delete this calibration?')) { cals.splice(k, 1); renderCalList(); } };
    host.appendChild(d);
  });
  if (cals.length) {
    const none = document.createElement('label');
    none.className = 'inline';
    none.innerHTML = '<input type="radio" name="activeCal"> no active calibration';
    none.firstChild.checked = !cals.some((c) => c.active);
    none.firstChild.onchange = () => cals.forEach((x) => (x.active = false));
    host.appendChild(none);
  }
}

$('#pumpForm').addEventListener('submit', async (e) => {
  e.preventDefault();
  const f = e.target;
  const out = S.editing.data;
  $$('[name]', f).forEach((el) => {
    if (el.name === 'activeCal') return;
    let v;
    if (el.type === 'checkbox') v = el.checked;
    else if (el.type === 'number') v = el.value === '' ? 0 : parseFloat(el.value);
    else v = el.value;
    setPath(out, el.name, v);
  });
  const r = S.editing.isNew
    ? await act(() => api('POST', 'pumps', out), 'Profile created')
    : await act(() => api('PUT', 'pumps/' + out.id, out), 'Profile saved');
  if (r) {
    f.hidden = true;
    S.editing = null;
    setTimeout(() => { loadPumps(); refreshStateSoon(100); }, 400);
  }
});

// --------------------------------------------------------------- settings
async function loadHardware() {
  try { S.hwMeta = await api('GET', 'hardware'); } catch (e) { return toast(e.message, 'err'); }
  try { S.pumps = (await api('GET', 'pumps')).pumps; } catch { /* keep */ }
  const host = $('#hwForms');
  host.innerHTML = '';
  $('#hwErrors').innerHTML = '';
  S.hwMeta.channels.forEach((c, i) => {
    const d = document.createElement('div');
    d.className = 'hw-card';
    d.innerHTML = `<h4>Channel ${i + 1} <span class="muted small">(flow sensor on I²C bus ${i})</span></h4>
      <div class="grid3">
        <label class="field">Driver<select class="drv"></select></label>
        <label class="field">Pump profile<select class="prof"></select></label>
        <label class="field">Flow sensor<select class="sen"><option value="slf3s">SLF3S-1300F</option><option value="none">none</option></select></label>
        <label class="field">Pressure sensor (same bus)<select class="psen"><option value="mprls">MPRLS / Qwiic MicroPressure (0x18)</option><option value="none">none</option></select></label>
      </div>
      <div class="grid3 pins"></div>
      <div class="grid3">
        <label class="field">SDA<input class="sda" type="number"></label>
        <label class="field">SCL<input class="scl" type="number"></label>
      </div>
      <div class="row wrap"><label class="inline en"><input class="enlow" type="checkbox"> EN active-low</label>
      <button class="btn small defs">Default pins</button></div>`;
    const drv = $('.drv', d);
    S.hwMeta.driverTypes.forEach((t) => drv.append(new Option({ uln2003: 'ULN2003 (28BYJ-48)', stepdir: 'STEP/DIR (A4988/DRV8825/TMC2209)', drv8871: 'DC: DRV8871', pwmdir: 'DC: PWM+DIR (L298N/TB6612)' }[t.id], t.id)));
    drv.value = c.driver;
    $('.sen', d).value = c.sensor;
    $('.psen', d).value = c.pressure || 'none';
    $('.sda', d).value = c.sda;
    $('.scl', d).value = c.scl;
    $('.enlow', d).checked = c.enActiveLow;
    const renderPins = (pins) => {
      const meta = S.hwMeta.driverTypes.find((t) => t.id === drv.value);
      const ph = $('.pins', d);
      ph.innerHTML = '';
      meta.pinLabels.forEach((lab, s) => {
        const l = document.createElement('label');
        l.className = 'field';
        l.textContent = lab + (s >= meta.requiredPins ? ' (optional, -1 = none)' : '');
        const inp = document.createElement('input');
        inp.type = 'number';
        inp.className = 'pin';
        inp.dataset.label = lab;
        inp.value = pins[s] ?? -1;
        l.appendChild(inp);
        ph.appendChild(l);
      });
      $('.en', d).hidden = drv.value !== 'stepdir';
      const fam = meta.family;
      const prof = $('.prof', d);
      const cur = prof.value || c.profile;
      prof.innerHTML = '';
      S.pumps.filter((p) => p.family === fam).forEach((p) => prof.append(new Option(p.name, p.id)));
      prof.value = cur;
      if (prof.selectedIndex < 0 && prof.options.length) prof.selectedIndex = 0;
    };
    const defaults = () => S.hwMeta.driverTypes.find((t) => t.id === drv.value).defaults[i];
    drv.addEventListener('change', () => renderPins(defaults()));
    $('.defs', d).addEventListener('click', () => renderPins(defaults()));
    renderPins(c.pins);
    host.appendChild(d);
  });
}

$('#hwApply').addEventListener('click', async () => {
  const chs = $$('#hwForms .hw-card').map((d) => ({
    driver: $('.drv', d).value,
    pins: $$('.pin', d).map((p) => (p.value === '' ? -1 : parseInt(p.value, 10))),
    enActiveLow: $('.enlow', d).checked,
    sensor: $('.sen', d).value,
    pressure: $('.psen', d).value,
    sda: parseInt($('.sda', d).value, 10),
    scl: parseInt($('.scl', d).value, 10),
    profile: $('.prof', d).value,
  }));
  $$('#hwForms input').forEach((x) => x.classList.remove('bad'));
  $('#hwErrors').innerHTML = '';
  try {
    await api('PUT', 'hardware', { channels: chs });
    toast('Hardware applied');
    setTimeout(() => { loadState(); loadHardware(); }, 900);
  } catch (e) {
    const errs = e.data && e.data.pinErrors;
    if (errs) {
      $('#hwErrors').innerHTML = '<ul>' + errs.map((x) => `<li>Ch${x.channel} ${x.field} (GPIO ${x.gpio}): ${x.reason}</li>`).join('') + '</ul>';
      errs.forEach((x) => {
        const card = $$('#hwForms .hw-card')[x.channel - 1];
        if (!card) return;
        const inp = x.field === 'SDA' ? $('.sda', card) : x.field === 'SCL' ? $('.scl', card) : $$('.pin', card).find((p) => p.dataset.label === x.field);
        if (inp) inp.classList.add('bad');
      });
    }
    toast(e.message, 'err');
  }
});

function renderSettings() {
  const st = S.state;
  if (!st) return;
  const s = st.settings;
  const focused = document.activeElement;
  const setIf = (id, v) => { if ($('#' + id) !== focused) $('#' + id).value = v; };
  setIf('setUnit', S.unit);
  setIf('setWd', s.watchdogS);
  setIf('setTel', s.telemetryHz);
  setIf('setFault', s.sensorFaultAction);
  setIf('setPUnit', S.punit);
  if ($('#wifiSsid') !== focused && !$('#wifiSsid').dataset.touched) $('#wifiSsid').value = s.wifi.ssid;
  const w = s.wifi;
  $('#wifiStatus').textContent = `AP "${w.apSsid}" at ${w.apIp} (${w.apClients} client${w.apClients === 1 ? '' : 's'}) · ` +
    (w.ssid ? (w.staConnected ? `joined "${w.ssid}" as ${w.staIp}` : `joining "${w.ssid}"…`) : 'not joining another network') + ` · http://${s.hostname}.local`;
  $('#timeStatus').textContent = `ESP32 clock ${st.time.synced ? 'synced: ' + new Date(st.time.epochMs).toLocaleString() : 'not synced (using uptime)'} · uptime ${fmtDur(st.uptimeMs / 1000)} · firmware ${st.version}${st.sim ? ' (SIM)' : ''} · free heap ${(st.heapFree / 1024).toFixed(0)} KB`;
}
$('#wifiSsid').addEventListener('input', (e) => (e.target.dataset.touched = '1'));
$('#setUnit').addEventListener('change', (e) => {
  S.unit = e.target.value;
  store.set('unitSet', true);
  store.set('unit', S.unit);
  S.cards.forEach((c, i) => updateCardStatic(i));
});
$('#setPUnit').addEventListener('change', (e) => {
  S.punit = e.target.value;
  store.set('punitSet', true);
  store.set('punit', S.punit);
  S.cards.forEach((c, i) => updateCardStatic(i));
});
$('#setSave').addEventListener('click', () => act(() => api('PUT', 'config', {
  watchdogS: num($('#setWd')), telemetryHz: num($('#setTel')), flowUnit: $('#setUnit').value, sensorFaultAction: $('#setFault').value,
  pressureUnit: $('#setPUnit').value,
}), 'Settings saved').then(() => refreshStateSoon(300)));
$('#wifiSave').addEventListener('click', () => {
  const body = { wifi: { ssid: $('#wifiSsid').value.trim() } };
  if ($('#wifiPass').value) body.wifi.password = $('#wifiPass').value;
  if ($('#apPass').value) body.apPassword = $('#apPass').value;
  act(() => api('PUT', 'config', body), 'Wi-Fi saved (AP password applies after reboot)').then(() => {
    $('#wifiPass').value = '';
    $('#apPass').value = '';
    delete $('#wifiSsid').dataset.touched;
    refreshStateSoon(3000);
  });
});
$('#timeSync').addEventListener('click', () => act(() => api('POST', 'time', { epochMs: Date.now() }), 'Time synced').then(() => refreshStateSoon(200)));
$('#reboot').addEventListener('click', () => {
  if (confirm('Reboot the ESP32? All channels must be stopped. Runs never resume after a reboot.')) act(() => api('POST', 'reboot'), 'Rebooting…');
});
$('#simBubble').addEventListener('click', () => act(() => api('POST', `sim/${+$('#simCh').value + 1}`, { bubble: 3 }), 'Bubble injected'));
$('#simFault').addEventListener('click', async () => {
  const ch = +$('#simCh').value;
  S.simFault = S.simFault || {};
  S.simFault[ch] = !S.simFault[ch];
  await act(() => api('POST', `sim/${ch + 1}`, { fault: S.simFault[ch] }), 'Sensor fault ' + (S.simFault[ch] ? 'ON' : 'OFF'));
});
$('#simOcclusion').addEventListener('click', async () => {
  const ch = +$('#simCh').value;
  S.simOcc = S.simOcc || {};
  S.simOcc[ch] = !S.simOcc[ch];
  await act(() => api('POST', `sim/${ch + 1}`, { occlusion: S.simOcc[ch] }), 'Occlusion ' + (S.simOcc[ch] ? 'ON' : 'OFF'));
});
$('#simPFault').addEventListener('click', async () => {
  const ch = +$('#simCh').value;
  S.simPF = S.simPF || {};
  S.simPF[ch] = !S.simPF[ch];
  await act(() => api('POST', `sim/${ch + 1}`, { pressureFault: S.simPF[ch] }), 'Pressure-sensor fault ' + (S.simPF[ch] ? 'ON' : 'OFF'));
});
$('#simWeaker').addEventListener('click', () => act(() => api('POST', `sim/${+$('#simCh').value + 1}`, { gainScale: 0.8 }), 'Simulated pump is now 20 % weaker'));

// --------------------------------------------------------------- boot
S.unit = store.get('unitSet', false) ? store.get('unit', 'uL/min') : 'uL/min';
S.punit = store.get('punitSet', false) ? store.get('punit', 'kPa') : 'kPa';
showTab(store.get('tab', 'dash'));
connect();
