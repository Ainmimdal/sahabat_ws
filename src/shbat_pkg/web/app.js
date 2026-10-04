'use strict';
// Sahabat browser console. Talks to web_console.py: SSE for state, POST for commands.

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;

function store(key, value) {
  try {
    if (value === undefined) return localStorage.getItem(key);
    localStorage.setItem(key, value);
  } catch (_e) { /* storage unavailable */ }
  return null;
}
function randomId() {
  let s = '';
  for (let i = 0; i < 32; i++) s += Math.floor(Math.random() * 16).toString(16);
  return s;
}
const clientId = store('sahabat.client') || randomId();
store('sahabat.client', clientId);

const S = {
  connected: false,
  status: null,
  map: null, mapImg: null,
  scan: null, plan: null,
  tags: null, startup: '',
  lease: { held: false, client: '' },
  mapping: { active: false, saving: '' },
  server: null,          // last waypoints payload from robot
  draft: [],             // editable waypoint copy
  dirty: false,
  serverChanged: false,
  selected: null,        // waypoint id
  layers: { scan: true, plan: true, routes: true, labels: true, tags: true, grid: false },
  tool: 'pan',
  follow: false,
};
const iHaveControl = () => S.lease.held && S.lease.client === clientId;

// ------------------------------------------------------------------ helpers
function toast(text, kind = '') {
  const el = document.createElement('div');
  el.className = `toast ${kind}`;
  el.textContent = text;
  $('toasts').appendChild(el);
  setTimeout(() => el.remove(), kind === 'err' ? 6000 : 3000);
}

async function cmd(op, args = {}, quiet = false) {
  try {
    const res = await fetch('api/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ op, client: clientId, ...args }),
    });
    const body = await res.json();
    if (!body.ok) throw new Error(body.error || res.statusText);
    if (!quiet && typeof body.result === 'string') toast(body.result, 'ok');
    return body.result;
  } catch (err) {
    if (!quiet) toast(`${op}: ${err.message}`, 'err');
    throw err;
  }
}
const safe = (p) => p.catch(() => undefined);

function dialog({ title, text = '', input = null, okText = 'OK', danger = false }) {
  const dlg = $('dlg');
  $('dlg-title').textContent = title;
  $('dlg-text').textContent = text;
  const inp = $('dlg-input');
  inp.classList.toggle('hidden', input === null);
  inp.value = input ?? '';
  $('dlg-ok').textContent = okText;
  $('dlg-ok').style.background = danger ? 'var(--bad)' : '';
  $('dlg-ok').style.borderColor = danger ? 'var(--bad)' : '';
  return new Promise((resolve) => {
    dlg.onclose = () => resolve(dlg.returnValue === 'ok' ? (input === null ? true : inp.value.trim()) : null);
    dlg.showModal();
    if (input !== null) inp.focus();
  });
}

// --------------------------------------------------------------- SSE stream
function connect() {
  const es = new EventSource('api/stream');
  es.onopen = () => { S.connected = true; renderHeader(); };
  es.onerror = () => { S.connected = false; renderHeader(); };
  const on = (name, fn) => es.addEventListener(name, (e) => fn(JSON.parse(e.data)));
  on('status', (d) => { S.status = d; renderHeader(); renderPanels(); if (S.follow) centerOnRobot(); draw(); });
  on('map', (d) => {
    const img = new Image();
    img.onload = () => {
      const first = !S.mapImg;
      S.map = d; S.mapImg = img; S.mapBounds = knownBounds(img, d);
      if (first) fitMap();
      draw();
      renderMapping();
    };
    img.src = `api/map.png?v=${d.version}`;
  });
  on('scan', (d) => { S.scan = d; draw(); });
  on('plan', (d) => { S.plan = d; draw(); });
  on('tags', (d) => { S.tags = d; renderTags(); draw(); });
  on('startup', (d) => { S.startup = d.text; renderPanels(); });
  on('lease', (d) => { S.lease = d; renderHeader(); renderPanels(); });
  on('waypoints', onWaypoints);
  on('mapping', (d) => {
    const started = d.active && !S.mapping.active;
    S.mapping = d;
    renderHeader(); renderMapping();
    if (started) { document.querySelector('[data-tab="mapping"]').click(); loadMapFiles(); }
  });
}

setInterval(() => {
  if (iHaveControl()) safe(cmd('heartbeat', {}, true));
}, 1000);

// ------------------------------------------------------------------ header
function chip(el, text, cls) {
  el.textContent = text;
  el.className = `chip ${cls || ''}`;
}
function renderHeader() {
  $('conn').classList.toggle('on', S.connected);
  const st = S.status;
  if (st) {
    chip($('chip-map'), st.active_map || 'no map');
    chip($('chip-mode'), st.mode);
    chip($('chip-nav'), st.operation ? `${st.navigation_state}: ${st.operation}` : st.navigation_state,
      st.navigation_state === 'failed' ? 'bad' : '');
    const localized = st.localized && st.frame === 'map';
    if (S.mapping.active) chip($('chip-loc'), 'Mapping', 'ok');
    else chip($('chip-loc'), localized ? 'Localized' : 'Not localized', localized ? 'ok' : 'warn');
    chip($('chip-scan'), st.scan_ok ? 'Lidar' : 'Lidar down', st.scan_ok ? 'ok' : 'bad');
    chip($('chip-motor'), st.motor_enabled ? 'Motors on' : 'Motors off', st.motor_enabled ? 'ok' : 'warn');
    const b = st.battery;
    chip($('chip-batt'), b == null ? 'Battery —' : `Battery ${b.toFixed(0)}%`,
      b == null ? '' : b < 20 ? 'bad' : b < 40 ? 'warn' : 'ok');
    $('estop-banner').classList.toggle('hidden', !st.emergency_stop);
  }
  const btn = $('btn-control');
  if (iHaveControl()) {
    btn.textContent = 'In control · Release';
    btn.className = 'btn owned';
  } else {
    const owner = st && st.control_owner;
    btn.textContent = owner ? `Take over from ${owner}` : 'Take control';
    btn.className = 'btn';
  }
}

$('btn-control').onclick = async () => {
  if (iHaveControl()) return safe(cmd('release_control'));
  let name = store('sahabat.name');
  if (!name) {
    name = await dialog({ title: 'Operator name', text: 'Shown to others as the control owner.', input: 'operator' });
    if (!name) return;
    store('sahabat.name', name);
  }
  safe(cmd('take_control', { name }));
};
$('btn-estop').onclick = () => { stopDriving(); safe(cmd('estop')); };
$('btn-estop-clear').onclick = async () => {
  const ok = await dialog({
    title: 'Clear E-stop?',
    text: 'Only clear the E-stop after a person has confirmed the area around the robot is safe.',
    okText: 'Area is safe, clear', danger: true,
  });
  if (ok) safe(cmd('estop_clear', { confirmation: 'CLEAR' }));
};

// ----------------------------------------------------------------- map view
const canvas = $('map');
const ctx = canvas.getContext('2d');
// World centre (m), pixels per metre, and view rotation (rad, CCW).
const view = { cx: 0, cy: 0, scale: 40, rot: Number(store('sahabat.rot')) || 0 };
let dpr = 1;

function resize() {
  dpr = window.devicePixelRatio || 1;
  const r = canvas.getBoundingClientRect();
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  if (!S.fitted && S.mapBounds && r.width > 0) { S.fitted = true; fitMap(); }
  draw();
}
new ResizeObserver(resize).observe(canvas);

const W = () => canvas.width / dpr;
const H = () => canvas.height / dpr;
function toScreen(x, y) {
  const c = Math.cos(view.rot), s = Math.sin(view.rot);
  const dx = x - view.cx, dy = y - view.cy;
  return [(dx * c - dy * s) * view.scale + W() / 2, -(dx * s + dy * c) * view.scale + H() / 2];
}
function toWorld(sx, sy) {
  const c = Math.cos(view.rot), s = Math.sin(view.rot);
  const rx = (sx - W() / 2) / view.scale, ry = -(sy - H() / 2) / view.scale;
  return [rx * c + ry * s + view.cx, -rx * s + ry * c + view.cy];
}

// World-space bounding box of the mapped (non-transparent) cells.
function knownBounds(img, m) {
  const c = document.createElement('canvas');
  c.width = img.width; c.height = img.height;
  const g = c.getContext('2d');
  g.drawImage(img, 0, 0);
  const px = g.getImageData(0, 0, c.width, c.height).data;
  let x0 = c.width, x1 = -1, y0 = c.height, y1 = -1;
  for (let y = 0; y < c.height; y++) {
    for (let x = 0; x < c.width; x++) {
      if (px[(y * c.width + x) * 4 + 3]) {
        if (x < x0) x0 = x; if (x > x1) x1 = x;
        if (y < y0) y0 = y; if (y > y1) y1 = y;
      }
    }
  }
  if (x1 < 0) {
    return { x0: m.origin_x, y0: m.origin_y, x1: m.origin_x + m.width * m.resolution, y1: m.origin_y + m.height * m.resolution };
  }
  const r = m.resolution;
  return {
    x0: m.origin_x + x0 * r, x1: m.origin_x + (x1 + 1) * r,
    y0: m.origin_y + (m.height - y1 - 1) * r, y1: m.origin_y + (m.height - y0) * r,
  };
}

function fitMap() {
  const b = S.mapBounds;
  if (!b || !W() || !H()) return;
  // Extent of the known area after view rotation.
  const c = Math.abs(Math.cos(view.rot)), s = Math.abs(Math.sin(view.rot));
  const bw = b.x1 - b.x0, bh = b.y1 - b.y0;
  const w = Math.max(1, bw * c + bh * s), h = Math.max(1, bw * s + bh * c);
  view.cx = (b.x0 + b.x1) / 2;
  view.cy = (b.y0 + b.y1) / 2;
  view.scale = Math.min(W() / w, H() / h) * 0.9;
  draw();
}
function centerOnRobot() {
  if (!S.status) return;
  view.cx = S.status.x; view.cy = S.status.y;
}
function zoomAt(factor, sx = W() / 2, sy = H() / 2) {
  const [wx, wy] = toWorld(sx, sy);
  view.scale = Math.max(2, Math.min(600, view.scale * factor));
  const [nx, ny] = toWorld(sx, sy);
  view.cx += wx - nx; view.cy += wy - ny;
  draw();
}

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const darkTheme = () => matchMedia('(prefers-color-scheme: dark)').matches;

let drawQueued = false;
function draw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(() => { drawQueued = false; render(); });
}

function arrow(x, y, yaw, len, color, width = 2) {
  const [sx, sy] = toScreen(x, y);
  const a0 = yaw + view.rot;
  const ex = sx + Math.cos(a0) * len, ey = sy - Math.sin(a0) * len;
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
  ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
  const a = Math.atan2(ey - sy, ex - sx);
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 9 * Math.cos(a - 0.45), ey - 9 * Math.sin(a - 0.45));
  ctx.lineTo(ex - 9 * Math.cos(a + 0.45), ey - 9 * Math.sin(a + 0.45));
  ctx.closePath(); ctx.fill();
}

function polyline(points, color, width, dash = []) {
  if (!points || points.length < 4) return;
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
  ctx.beginPath();
  for (let i = 0; i < points.length; i += 2) {
    const [sx, sy] = toScreen(points[i], points[i + 1]);
    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
  }
  ctx.stroke(); ctx.setLineDash([]);
}

function render() {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W(), H());
  const accent = css('--accent');
  const text = css('--text');

  if (S.mapImg && S.map) {
    const m = S.map;
    const [sx, sy] = toScreen(m.origin_x, m.origin_y + m.height * m.resolution);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-view.rot);
    ctx.imageSmoothingEnabled = view.scale * m.resolution < 2;
    if (darkTheme()) ctx.filter = 'invert(0.9) hue-rotate(180deg)';
    ctx.drawImage(S.mapImg, 0, 0, m.width * m.resolution * view.scale, m.height * m.resolution * view.scale);
    ctx.restore();
  }

  if (S.layers.grid) {
    ctx.strokeStyle = 'rgba(128,128,128,.25)'; ctx.lineWidth = 1;
    const corners = [toWorld(0, 0), toWorld(W(), 0), toWorld(0, H()), toWorld(W(), H())];
    const xs = corners.map((p) => p[0]), ys = corners.map((p) => p[1]);
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    const step = view.scale > 30 ? 1 : 5;
    const seg = (ax, ay, bx, by) => { const a = toScreen(ax, ay), b = toScreen(bx, by); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); };
    ctx.beginPath();
    for (let x = Math.ceil(x0 / step) * step; x < x1; x += step) seg(x, y0, x, y1);
    for (let y = Math.ceil(y0 / step) * step; y < y1; y += step) seg(x0, y, x1, y);
    ctx.stroke();
  }

  if (S.layers.scan && S.scan && S.scan.frame === (S.status?.frame || 'map')) {
    ctx.fillStyle = '#ef4444';
    const p = S.scan.points, r = Math.max(1.5, Math.min(3, view.scale * 0.03));
    for (let i = 0; i < p.length; i += 2) {
      const [sx, sy] = toScreen(p[i], p[i + 1]);
      ctx.fillRect(sx - r / 2, sy - r / 2, r, r);
    }
  }

  if (S.layers.plan && S.plan) polyline(S.plan.points, '#22c55e', 3);

  // Waypoint tour order and preferred routes.
  const wps = S.draft;
  if (wps.length > 1) {
    const pts = [];
    wps.filter((w) => w.enabled).forEach((w) => pts.push(w.x, w.y));
    polyline(pts, 'rgba(37,99,235,.45)', 2, [6, 6]);
  }
  if (S.layers.routes && S.server) {
    const byId = Object.fromEntries(wps.map((w) => [w.id, w]));
    for (const seg of S.server.segments || []) {
      const a = byId[seg.from], b = byId[seg.to];
      if (!a || !b) continue;
      const pts = [a.x, a.y];
      seg.via.forEach((v) => pts.push(v.x, v.y));
      pts.push(b.x, b.y);
      polyline(pts, seg.enabled ? 'rgba(234,179,8,.9)' : 'rgba(128,128,128,.6)', 3);
    }
  }

  if (S.layers.tags && S.tags) {
    for (const t of S.tags.tags) {
      if (!t.saved) continue;
      const [sx, sy] = toScreen(t.x, t.y);
      ctx.fillStyle = t.visible ? '#a855f7' : 'rgba(168,85,247,.5)';
      ctx.fillRect(sx - 6, sy - 6, 12, 12);
      arrow(t.x, t.y, t.yaw, 16, ctx.fillStyle, 2);
      if (S.layers.labels) { ctx.fillStyle = text; ctx.font = '11px system-ui'; ctx.fillText(`tag ${t.id}`, sx + 9, sy + 14); }
    }
  }

  const dock = S.server?.dock;
  if (dock) {
    const [sx, sy] = toScreen(dock.x, dock.y);
    ctx.strokeStyle = '#0ea5e9'; ctx.lineWidth = 2.5;
    ctx.strokeRect(sx - 9, sy - 9, 18, 18);
    arrow(dock.x, dock.y, dock.yaw, 22, '#0ea5e9');
    if (S.layers.labels) { ctx.fillStyle = text; ctx.font = '11px system-ui'; ctx.fillText('dock', sx + 12, sy - 10); }
  }

  wps.forEach((w, i) => {
    const sel = w.id === S.selected;
    const [sx, sy] = toScreen(w.x, w.y);
    arrow(w.x, w.y, w.yaw, sel ? 34 : 24, w.enabled ? accent : '#94a3b8', sel ? 3 : 2);
    ctx.beginPath(); ctx.arc(sx, sy, sel ? 12 : 10, 0, 2 * Math.PI);
    ctx.fillStyle = w.enabled ? accent : '#94a3b8'; ctx.fill();
    if (sel) { ctx.strokeStyle = '#fbbf24'; ctx.lineWidth = 3; ctx.stroke(); }
    ctx.fillStyle = '#fff'; ctx.font = 'bold 11px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(String(i + 1), sx, sy);
    ctx.textAlign = 'start'; ctx.textBaseline = 'alphabetic';
    if (S.layers.labels && view.scale > 12) { ctx.fillStyle = text; ctx.font = '12px system-ui'; ctx.fillText(w.name, sx + 14, sy - 10); }
    if (sel) {
      const [hx, hy] = headingHandle(w);
      ctx.beginPath(); ctx.arc(hx, hy, 7, 0, 2 * Math.PI);
      ctx.fillStyle = '#fbbf24'; ctx.fill();
    }
  });

  const st = S.status;
  if (st) {
    const [sx, sy] = toScreen(st.x, st.y);
    const r = Math.max(8, 0.3 * view.scale);
    ctx.beginPath(); ctx.arc(sx, sy, r, 0, 2 * Math.PI);
    ctx.fillStyle = st.frame === 'map' ? 'rgba(16,185,129,.35)' : 'rgba(245,158,11,.35)';
    ctx.fill();
    ctx.strokeStyle = st.frame === 'map' ? '#10b981' : '#f59e0b'; ctx.lineWidth = 2; ctx.stroke();
    arrow(st.x, st.y, st.yaw, r + 14, ctx.strokeStyle, 3);
  }

  if (drag && drag.kind === 'aim') {
    const color = { pose: '#10b981', goal: '#22c55e', add: accent }[drag.tool];
    const [sx, sy] = toScreen(drag.x, drag.y);
    ctx.beginPath(); ctx.arc(sx, sy, 8, 0, 2 * Math.PI); ctx.fillStyle = color; ctx.fill();
    arrow(drag.x, drag.y, drag.yaw, 44, color, 3);
  }
}

function headingHandle(w) {
  const [sx, sy] = toScreen(w.x, w.y);
  const a = w.yaw + view.rot;
  return [sx + Math.cos(a) * 34, sy - Math.sin(a) * 34];
}

// --------------------------------------------------------- map interaction
const pointers = new Map();
let drag = null;
let pinch = null;

function pickWaypoint(sx, sy) {
  const sel = S.draft.find((w) => w.id === S.selected);
  if (sel) {
    const [hx, hy] = headingHandle(sel);
    if (Math.hypot(sx - hx, sy - hy) < 12) return { w: sel, handle: true };
  }
  for (let i = S.draft.length - 1; i >= 0; i--) {
    const w = S.draft[i];
    const [px, py] = toScreen(w.x, w.y);
    if (Math.hypot(sx - px, sy - py) < 13) return { w, handle: false };
  }
  return null;
}

function localXY(e) {
  const r = canvas.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
}

canvas.addEventListener('pointerdown', (e) => {
  try { canvas.setPointerCapture(e.pointerId); } catch (_e) { /* synthetic pointer */ }
  const [sx, sy] = localXY(e);
  pointers.set(e.pointerId, [sx, sy]);
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    pinch = { dist: Math.hypot(a[0] - b[0], a[1] - b[1]) };
    drag = null; draw();
    return;
  }
  const [wx, wy] = toWorld(sx, sy);
  if (S.tool !== 'pan') {
    if (S.tool !== 'add' && !iHaveControl()) { toast('Take control first', 'err'); setTool('pan'); return; }
    drag = { kind: 'aim', tool: S.tool, x: wx, y: wy, yaw: 0, sx, sy };
    draw();
    return;
  }
  const hit = pickWaypoint(sx, sy);
  if (hit) {
    select(hit.w.id);
    drag = { kind: hit.handle ? 'rotate' : 'move', w: hit.w, sx, sy, moved: false };
  } else {
    drag = { kind: 'pan', sx, sy, wx, wy, moved: false };
  }
});

canvas.addEventListener('pointermove', (e) => {
  const [sx, sy] = localXY(e);
  const [wx, wy] = toWorld(sx, sy);
  $('readout').textContent = `${wx.toFixed(2)}, ${wy.toFixed(2)}`;
  if (!pointers.has(e.pointerId)) return;
  pointers.set(e.pointerId, [sx, sy]);
  if (pinch && pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
    zoomAt(d / pinch.dist, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2);
    pinch.dist = d;
    return;
  }
  if (!drag) return;
  if (Math.hypot(sx - drag.sx, sy - drag.sy) > 4) drag.moved = true;
  if (drag.kind === 'pan') {
    if (!drag.moved) return;
    S.follow = false; $('follow').classList.remove('active');
    // Keep the grabbed world point under the pointer.
    view.cx += drag.wx - wx;
    view.cy += drag.wy - wy;
  } else if (drag.kind === 'aim') {
    if (Math.hypot(sx - drag.sx, sy - drag.sy) > 6) drag.yaw = Math.atan2(wy - drag.y, wx - drag.x);
  } else if (drag.kind === 'move' && drag.moved) {
    drag.w.x = round3(wx); drag.w.y = round3(wy); markDirty();
  } else if (drag.kind === 'rotate') {
    drag.w.yaw = round3(Math.atan2(wy - drag.w.y, wx - drag.w.x)); markDirty();
  }
  draw();
});

function endPointer(e) {
  pointers.delete(e.pointerId);
  if (pointers.size < 2) pinch = null;
  if (!drag || pointers.size > 0) { if (pointers.size === 0) drag = null; return; }
  const d = drag;
  drag = null;
  if (d.kind === 'pan' && !d.moved) select(null);
  if (d.kind === 'aim' && e.type === 'pointerup') commitAim(d);
  if (d.kind === 'move' || d.kind === 'rotate') renderWaypoints();
  draw();
}
canvas.addEventListener('pointerup', endPointer);
canvas.addEventListener('pointercancel', endPointer);
canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const [sx, sy] = localXY(e);
  zoomAt(Math.exp(-e.deltaY * 0.0015), sx, sy);
}, { passive: false });

const round3 = (v) => Math.round(v * 1000) / 1000;

function commitAim(d) {
  const x = round3(d.x), y = round3(d.y), yaw = round3(d.yaw);
  if (d.tool === 'pose') safe(cmd('initial_pose', { x, y, yaw }));
  else if (d.tool === 'goal') safe(cmd('goal', { x, y, yaw }));
  else if (d.tool === 'add') {
    const w = { id: randomId(), name: nextName(), x, y, yaw, dwell: 0, enabled: true };
    const idx = S.selected ? S.draft.findIndex((v) => v.id === S.selected) + 1 : S.draft.length;
    S.draft.splice(idx || S.draft.length, 0, w);
    S.selected = w.id;
    markDirty();
    renderWaypoints();
    return; // stay in add mode for quick placement
  }
  setTool('pan');
}
function nextName() {
  let n = S.draft.length + 1;
  const names = new Set(S.draft.map((w) => w.name));
  while (names.has(`waypoint_${n}`)) n++;
  return `waypoint_${n}`;
}

const hints = {
  pan: '',
  pose: 'Press where the robot is, drag toward where it faces, release.',
  goal: 'Press the target, drag toward the final heading, release. The robot will drive there.',
  add: 'Press to place a waypoint, drag to set heading. Esc to finish.',
};
function setTool(t) {
  S.tool = t;
  document.querySelectorAll('[data-tool]').forEach((b) => b.classList.toggle('active', b.dataset.tool === t));
  canvas.classList.toggle('crosshair', t !== 'pan');
  $('hint').textContent = hints[t];
}
document.querySelectorAll('[data-tool]').forEach((b) => { b.onclick = () => setTool(b.dataset.tool); });
$('zoom-in').onclick = () => zoomAt(1.4);
$('zoom-out').onclick = () => zoomAt(1 / 1.4);
$('zoom-fit').onclick = () => { S.follow = false; $('follow').classList.remove('active'); fitMap(); };
$('follow').onclick = () => {
  S.follow = !S.follow;
  $('follow').classList.toggle('active', S.follow);
  if (S.follow) { centerOnRobot(); draw(); }
};

// ---------------------------------------------------------------- waypoints
function onWaypoints(d) {
  const sameSet = S.server && S.server.set_id === d.set_id && S.server.map_id === d.map_id;
  S.server = d;
  if (S.dirty && sameSet) {
    S.serverChanged = d.revision !== S.baseRevision;
  } else {
    resetDraft();
  }
  renderSets();
  renderWaypoints();
  draw();
}
function resetDraft() {
  S.draft = (S.server?.waypoints || []).map((w) => ({ ...w }));
  S.baseRevision = S.server?.revision || 0;
  S.dirty = false;
  S.serverChanged = false;
  if (!S.draft.some((w) => w.id === S.selected)) S.selected = null;
}
function markDirty() { S.dirty = true; renderDirty(); }
function renderDirty() {
  const el = $('dirty');
  el.className = S.dirty ? 'dirty' : 'muted';
  el.textContent = S.serverChanged ? 'Unsaved, and changed on robot. Revert to reload.'
    : S.dirty ? 'Unsaved changes' : 'Saved';
}

function renderSets() {
  const sel = $('set-select');
  const d = S.server;
  sel.innerHTML = '';
  for (const s of d?.sets || []) {
    const o = document.createElement('option');
    o.value = s.id; o.textContent = `${s.name} (${s.count})`;
    sel.appendChild(o);
  }
  if (d) sel.value = d.set_id;
  const dock = d?.dock;
  $('dock-info').textContent = dock ? `Dock at ${dock.x.toFixed(2)}, ${dock.y.toFixed(2)}, ${(dock.yaw * DEG).toFixed(0)}°` : 'No dock saved for this map';
  const n = d?.segments?.length || 0;
  $('segments-note').textContent = n ? `${n} preferred route segment(s) are kept when you save. Edit routes in RViz for now.` : '';
}

function renderWaypoints() {
  const list = $('wp-list');
  list.innerHTML = '';
  if (!S.draft.length) {
    const li = document.createElement('li');
    li.className = 'empty';
    li.textContent = 'No waypoints yet. Use + Waypoint on the map.';
    list.appendChild(li);
  }
  S.draft.forEach((w, i) => {
    const li = document.createElement('li');
    li.className = (w.id === S.selected ? 'sel ' : '') + (w.enabled ? '' : 'off');
    li.innerHTML = `<span class="num"></span><span class="name"></span>
      <button class="mini" data-a="up" title="Move earlier">↑</button>
      <button class="mini" data-a="down" title="Move later">↓</button>
      <button class="mini" data-a="del" title="Delete">✕</button>`;
    li.querySelector('.num').textContent = i + 1;
    li.querySelector('.name').textContent = w.name;
    li.onclick = (e) => {
      const a = e.target.dataset?.a;
      if (a === 'up' && i > 0) { [S.draft[i - 1], S.draft[i]] = [S.draft[i], S.draft[i - 1]]; markDirty(); }
      else if (a === 'down' && i < S.draft.length - 1) { [S.draft[i + 1], S.draft[i]] = [S.draft[i], S.draft[i + 1]]; markDirty(); }
      else if (a === 'del') { S.draft.splice(i, 1); if (S.selected === w.id) S.selected = null; markDirty(); }
      else { select(w.id); centerOn(w); return; }
      renderWaypoints(); draw();
    };
    list.appendChild(li);
  });
  renderDirty();
  renderDetail();
}
function centerOn(w) { if (view.scale) { view.cx = w.x; view.cy = w.y; S.follow = false; $('follow').classList.remove('active'); draw(); } }
function select(id) {
  S.selected = id;
  renderWaypoints();
  draw();
}

function renderDetail() {
  const w = S.draft.find((v) => v.id === S.selected);
  $('wp-detail').classList.toggle('hidden', !w);
  if (!w) return;
  const set = (id, v) => { if (document.activeElement !== $(id)) $(id).value = v; };
  set('wp-name', w.name);
  set('wp-dwell', w.dwell);
  set('wp-x', w.x.toFixed(3));
  set('wp-y', w.y.toFixed(3));
  set('wp-yaw', (w.yaw * DEG).toFixed(1));
  $('wp-enabled').checked = w.enabled;
}
function bindField(id, apply) {
  $(id).addEventListener('input', () => {
    const w = S.draft.find((v) => v.id === S.selected);
    if (!w) return;
    apply(w, $(id));
    markDirty();
    const li = $('wp-list').querySelector('li.sel .name');
    if (li) li.textContent = w.name;
    draw();
  });
}
bindField('wp-name', (w, el) => { w.name = el.value; });
bindField('wp-dwell', (w, el) => { w.dwell = Math.max(0, Number(el.value) || 0); });
bindField('wp-x', (w, el) => { if (el.value !== '') w.x = Number(el.value); });
bindField('wp-y', (w, el) => { if (el.value !== '') w.y = Number(el.value); });
bindField('wp-yaw', (w, el) => { if (el.value !== '') w.yaw = Number(el.value) / DEG; });
$('wp-enabled').onchange = () => {
  const w = S.draft.find((v) => v.id === S.selected);
  if (w) { w.enabled = $('wp-enabled').checked; markDirty(); renderWaypoints(); draw(); }
};
$('wp-here').onclick = () => {
  const w = S.draft.find((v) => v.id === S.selected);
  const st = S.status;
  if (!w || !st) return;
  if (st.frame !== 'map') return toast('Robot is not localized on the map', 'err');
  w.x = round3(st.x); w.y = round3(st.y); w.yaw = round3(st.yaw);
  markDirty(); renderDetail(); draw();
};
$('wp-go').onclick = () => {
  if (S.dirty) return toast('Save waypoints before navigating', 'err');
  if (!S.selected) return;
  safe(cmd('patrol', { command: 'navigate', waypoint_id: S.selected, set_id: S.server?.set_id || '' }));
};

$('wp-save').onclick = async () => {
  if (!S.server) return;
  const names = S.draft.map((w) => w.name.trim());
  if (names.some((n) => !n)) return toast('Every waypoint needs a name', 'err');
  try {
    await cmd('save_waypoints', {
      set_id: S.server.set_id,
      revision: S.baseRevision,
      waypoints: S.draft,
      segments: S.server.segments || [],
    });
    S.dirty = false;
    renderDirty();
  } catch (_e) { /* toast already shown */ }
};
$('wp-revert').onclick = () => { resetDraft(); renderWaypoints(); draw(); };

$('set-select').onchange = async () => {
  if (S.dirty && !(await dialog({ title: 'Discard unsaved changes?', okText: 'Discard', danger: true }))) {
    $('set-select').value = S.server.set_id; return;
  }
  S.dirty = false;
  safe(cmd('waypoint_set', { action: 'select', set_id: $('set-select').value }));
};
$('set-new').onclick = async () => {
  const name = await dialog({ title: 'New waypoint set', input: 'New Tour' });
  if (name) safe(cmd('waypoint_set', { action: 'create', name }));
};
$('set-rename').onclick = async () => {
  const cur = S.server?.sets.find((s) => s.id === S.server.set_id);
  const name = await dialog({ title: 'Rename set', input: cur?.name || '' });
  if (name) safe(cmd('waypoint_set', { action: 'rename', set_id: S.server.set_id, name }));
};
$('set-delete').onclick = async () => {
  const cur = S.server?.sets.find((s) => s.id === S.server.set_id);
  if (await dialog({ title: `Delete “${cur?.name}”?`, text: 'This removes the waypoint set file from the robot.', okText: 'Delete', danger: true })) {
    safe(cmd('waypoint_set', { action: 'delete', set_id: S.server.set_id }));
  }
};

const patrol = (command) => () => {
  if (command === 'start' && S.dirty) return toast('Save waypoints before starting', 'err');
  safe(cmd('patrol', { command, set_id: S.server?.set_id || '', loop: $('patrol-loop').checked }));
};
$('patrol-start').onclick = patrol('start');
$('patrol-pause').onclick = patrol('pause');
$('patrol-resume').onclick = patrol('resume');
$('patrol-stop').onclick = patrol('stop');

$('dock-save').onclick = async () => {
  const st = S.status;
  if (!st || st.frame !== 'map') return toast('Robot is not localized on the map', 'err');
  const ok = await dialog({
    title: 'Save dock here?',
    text: `Dock = current robot pose (${st.x.toFixed(2)}, ${st.y.toFixed(2)}, ${(st.yaw * DEG).toFixed(0)}°). Startup localization searches around this pose, so only save it after you've checked the robot is correctly localized.`,
    okText: 'Save dock',
  });
  if (ok) safe(cmd('save_dock', { x: st.x, y: st.y, yaw: st.yaw }));
};
$('dock-go').onclick = () => safe(cmd('patrol', { command: 'navigate', waypoint_id: '__map_dock__' }));

// ---------------------------------------------------------------- localize
function renderTags() {
  const t = S.tags;
  $('tags-status').textContent = t ? `${t.status || ''}${t.auto_init ? ' · auto-init on' : ''}` : 'No AprilTag manager running.';
  const body = $('tags-body');
  body.innerHTML = '';
  for (const tag of t?.tags || []) {
    const tr = document.createElement('tr');
    const state = tag.capturing ? `capturing ${tag.samples}` : tag.saved ? (tag.visible ? 'saved · seen' : 'saved') : 'seen';
    tr.innerHTML = `<td></td><td><input type="text"></td><td></td><td></td><td></td>`;
    tr.children[0].textContent = tag.id;
    const inp = tr.querySelector('input');
    inp.value = tag.name || '';
    inp.placeholder = `tag_${tag.id}`;
    tr.children[2].textContent = state;
    tr.children[3].textContent = tag.visible && tag.distance > 0 ? `${tag.distance.toFixed(1)} m` : '';
    const cell = tr.children[4];
    if (tag.visible) {
      const b = document.createElement('button');
      b.className = 'btn small'; b.textContent = tag.saved ? 'Recapture' : 'Capture';
      b.onclick = () => safe(cmd('tag', { action: 'capture', tag_id: tag.id, name: inp.value }));
      cell.appendChild(b);
    }
    if (tag.saved) {
      const b = document.createElement('button');
      b.className = 'btn small danger-text'; b.textContent = '✕'; b.title = 'Delete saved tag';
      b.onclick = async () => {
        if (await dialog({ title: `Delete tag ${tag.id}?`, okText: 'Delete', danger: true })) safe(cmd('tag', { action: 'delete', tag_id: tag.id }));
      };
      cell.appendChild(b);
    }
    body.appendChild(tr);
  }
  if (!t?.tags?.length) body.innerHTML = '<tr><td colspan="5" class="muted">No detected or saved tags.</td></tr>';
}
$('loc-pose').onclick = () => setTool('pose');
$('loc-tags').onclick = () => safe(cmd('tag', { action: 'localize' }));
$('tags-reload').onclick = () => safe(cmd('tag', { action: 'reload' }));
$('loc-recover').onclick = async () => {
  if (await dialog({ title: 'Start global relocalization?', text: 'The robot will rotate in place. Make sure the area around it is clear.', okText: 'Start' })) {
    safe(cmd('recovery', { start: true }));
  }
};
$('loc-recover-stop').onclick = () => safe(cmd('recovery', { start: false }));

// -------------------------------------------------------------- side panels
function kv(el, rows) {
  el.innerHTML = '';
  for (const [k, v, cls] of rows) {
    const a = document.createElement('div'); a.className = 'k'; a.textContent = k;
    const b = document.createElement('div'); b.className = `v ${cls || ''}`; b.textContent = v;
    el.append(a, b);
  }
}
const yes = (b, t = 'ok', f = 'no') => [b ? t : f, b ? 'ok' : 'bad'];
function renderPanels() {
  const st = S.status;
  if (!st) return;
  const pose = `${st.x.toFixed(2)}, ${st.y.toFixed(2)}, ${(st.yaw * DEG).toFixed(0)}° (${st.frame})`;
  kv($('loc-kv'), [
    ['Localized', ...yes(st.localized && st.frame === 'map', 'yes', 'no')],
    ['Pose', pose],
    ['Startup', S.startup || '—'],
    ['Recovery', st.recovery_active ? `running: ${st.recovery_status}` : st.recovery_status || '—'],
  ]);
  kv($('drive-kv'), [
    ['Control', iHaveControl() ? 'you' : st.control_owner || 'nobody', iHaveControl() ? 'ok' : ''],
    ['E-stop', ...yes(!st.emergency_stop, 'clear', 'ACTIVE')],
    ['Motors', ...yes(st.motor_enabled, 'enabled', 'disabled')],
    ['Velocity', `${st.linear.toFixed(2)} m/s, ${st.angular.toFixed(2)} rad/s`],
  ]);
  kv($('sys-kv'), [
    ['Diagnostic', st.diagnostic_message || st.diagnostic, st.diagnostic === 'ok' ? 'ok' : st.diagnostic === 'warn' ? 'warn' : 'bad'],
    ['Mode', st.mode],
    ['Navigation', st.navigation_state],
    ['Map', ...yes(st.map_ok)],
    ['Lidar', ...yes(st.scan_ok)],
    ['TF', ...yes(st.tf_ok)],
    ['Control owner', st.control_owner || 'nobody'],
    ['Pose', pose],
  ]);
  $('joy').classList.toggle('disabled', !iHaveControl() || st.emergency_stop);
}

document.querySelectorAll('[data-tab]').forEach((b) => {
  b.onclick = () => {
    document.querySelectorAll('[data-tab]').forEach((x) => x.classList.toggle('active', x === b));
    document.querySelectorAll('[data-panel]').forEach((p) => p.classList.toggle('hidden', p.dataset.panel !== b.dataset.tab));
    if (b.dataset.tab === 'system') loadMaps();
    if (b.dataset.tab === 'mapping') { loadMapFiles(); renderMapping(); }
    store('sahabat.tab', b.dataset.tab);
  };
});
document.querySelectorAll('[data-layer]').forEach((c) => {
  c.onchange = () => { S.layers[c.dataset.layer] = c.checked; draw(); };
});

async function loadMaps() {
  try {
    const r = await cmd('list_maps', {}, true);
    const sel = $('map-select');
    sel.innerHTML = '';
    for (const m of r.maps) {
      const o = document.createElement('option');
      o.value = m.id; o.textContent = m.name && m.name !== m.id ? `${m.name} (${m.id})` : m.id;
      sel.appendChild(o);
    }
    sel.value = r.active;
  } catch (_e) { /* backend offline */ }
}
$('map-load').onclick = async () => {
  const id = $('map-select').value;
  if (!id || id === S.status?.active_map) return;
  if (await dialog({ title: `Load map “${id}”?`, text: 'The robot must be stationary. You will need to localize again afterwards.', okText: 'Load' })) {
    safe(cmd('load_map', { map_id: id }));
  }
};

// ------------------------------------------------------------------ mapping
function renderMapping() {
  const m = S.mapping;
  const el = $('mapping-state');
  if (m.saving) {
    el.className = 'notice warn'; el.textContent = `Saving map “${m.saving}”…`;
  } else if (m.active) {
    el.className = 'notice ok'; el.textContent = 'SLAM mapping is running. Drive the robot to build the map.';
  } else {
    el.className = 'notice'; el.textContent = 'Not mapping. Start the “Sahabat New Mapping (Web Console)” shortcut on the robot to build a new map.';
  }
  const map = S.map;
  const b = S.mapBounds;
  kv($('mapping-kv'), [
    ['Live map', map ? `${(map.width * map.resolution).toFixed(1)} × ${(map.height * map.resolution).toFixed(1)} m grid` : '—'],
    ['Mapped area', b ? `${(b.x1 - b.x0).toFixed(1)} × ${(b.y1 - b.y0).toFixed(1)} m` : '—'],
    ['Resolution', map ? `${map.resolution.toFixed(2)} m/cell` : '—'],
    ['Folder', m.directory || '—'],
  ]);
  $('map-save').disabled = !m.active || !!m.saving;
}

async function loadMapFiles() {
  try {
    const r = await cmd('list_map_files', {}, true);
    const body = $('maps-body');
    body.innerHTML = '';
    for (const m of r.maps) {
      const tr = document.createElement('tr');
      tr.innerHTML = '<td></td><td></td><td class="muted"></td>';
      tr.children[0].textContent = m.id;
      tr.children[1].textContent = m.session ? 'yes' : '—';
      tr.children[2].textContent = new Date(m.modified * 1000).toLocaleString();
      tr.onclick = () => { $('map-name').value = m.id; };
      body.appendChild(tr);
    }
    if (!r.maps.length) body.innerHTML = '<tr><td colspan="3" class="muted">No saved maps.</td></tr>';
  } catch (_e) { /* console offline */ }
}

$('map-save').onclick = async () => {
  const name = $('map-name').value.trim();
  let check;
  try { check = await cmd('map_name_check', { name }, true); } catch (e) { return toast(e.message, 'err'); }
  let overwrite = false;
  if (check.existing.length) {
    overwrite = await dialog({
      title: `Replace map “${name}”?`,
      text: `These files exist: ${check.existing.join(', ')}. They will be moved to maps/.archive, not deleted. Waypoint sets and AprilTags for this map stay in place but may no longer line up with the new map.`,
      okText: 'Replace', danger: true,
    });
    if (!overwrite) return;
  }
  $('map-save').disabled = true;
  try {
    await cmd('save_map', { name, overwrite, session: $('map-session').checked });
    loadMapFiles();
  } catch (_e) { /* toast shown */ }
  renderMapping();
};

// ------------------------------------------------------------------- teleop
const drive = { linear: 0, angular: 0, active: false, timer: null, source: '' };
const lin = $('lin-max'), ang = $('ang-max');
const syncLimits = () => { $('lin-val').textContent = Number(lin.value).toFixed(2); $('ang-val').textContent = Number(ang.value).toFixed(1); };
lin.oninput = ang.oninput = () => { syncLimits(); store('sahabat.lin', lin.value); store('sahabat.ang', ang.value); };
lin.value = store('sahabat.lin') || lin.value;
ang.value = store('sahabat.ang') || ang.value;
syncLimits();

function sendDrive(deadman) {
  return cmd('teleop', { linear: drive.linear, angular: drive.angular, deadman }, true)
    .catch((e) => { stopDriving(); toast(`Drive: ${e.message}`, 'err'); });
}
function startDriving(source) {
  if (!iHaveControl()) { toast('Take control first', 'err'); return false; }
  if (S.status?.emergency_stop) { toast('E-stop is active', 'err'); return false; }
  drive.active = true; drive.source = source;
  clearInterval(drive.timer);
  drive.timer = setInterval(() => sendDrive(true), 100);
  return true;
}
function stopDriving() {
  if (!drive.active) return;
  drive.active = false; drive.linear = 0; drive.angular = 0;
  clearInterval(drive.timer);
  safe(sendDrive(false));
  $('joy').classList.remove('active');
  $('joy-knob').style.transform = '';
}
window.addEventListener('blur', stopDriving);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopDriving(); });

const joy = $('joy');
function joyUpdate(e) {
  const r = joy.getBoundingClientRect();
  const R = r.width / 2 - 35;
  let dx = e.clientX - (r.left + r.width / 2);
  let dy = e.clientY - (r.top + r.height / 2);
  const m = Math.hypot(dx, dy);
  if (m > R) { dx *= R / m; dy *= R / m; }
  $('joy-knob').style.transform = `translate(${dx}px, ${dy}px)`;
  const fx = dx / R, fy = -dy / R;
  // Small dead zone so a tap does not creep the robot.
  drive.linear = Math.abs(fy) < 0.08 ? 0 : fy * Number(lin.value);
  drive.angular = Math.abs(fx) < 0.08 ? 0 : -fx * Number(ang.value);
}
joy.addEventListener('pointerdown', (e) => {
  if (!startDriving('joy')) return;
  joy.classList.add('active');
  joyUpdate(e);
  try { joy.setPointerCapture(e.pointerId); } catch (_e) { /* synthetic pointer */ }
});
joy.addEventListener('pointermove', (e) => { if (drive.active && drive.source === 'joy') joyUpdate(e); });
joy.addEventListener('pointerup', stopDriving);
joy.addEventListener('pointercancel', stopDriving);

const keys = new Set();
const driveKeys = { w: 1, arrowup: 1, s: 1, arrowdown: 1, a: 1, arrowleft: 1, d: 1, arrowright: 1 };
function typing() {
  const t = document.activeElement?.tagName;
  return t === 'INPUT' || t === 'SELECT' || t === 'TEXTAREA';
}
function keyDrive() {
  const f = (keys.has('w') || keys.has('arrowup') ? 1 : 0) - (keys.has('s') || keys.has('arrowdown') ? 1 : 0);
  const t = (keys.has('a') || keys.has('arrowleft') ? 1 : 0) - (keys.has('d') || keys.has('arrowright') ? 1 : 0);
  drive.linear = f * Number(lin.value);
  drive.angular = t * Number(ang.value);
  if (!keys.size) stopDriving();
}
document.addEventListener('keydown', (e) => {
  if (typing() || $('dlg').open) return;
  const k = e.key.toLowerCase();
  if (k === ' ') { e.preventDefault(); $('btn-estop').click(); return; }
  if (k === 'escape') { setTool('pan'); return; }
  if (!$('kbd-drive').checked || !driveKeys[k]) return;
  e.preventDefault();
  if (e.repeat) return;
  if (!drive.active && !startDriving('kbd')) return;
  keys.add(k); keyDrive();
});
document.addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  if (!keys.delete(k)) return;
  keyDrive();
});

// ------------------------------------------------------------ view rotation
const gimbal = $('gimbal');
// Free rotation with soft detents: within 1.5° of a 15° step it settles on
// the step; any other angle stays exactly where it is put. Shift disables.
const DETENT_STEP = 15 / DEG, DETENT_ZONE = 1.5 / DEG;
function detent(rad, free) {
  if (free) return rad;
  const q = Math.round(rad / DETENT_STEP) * DETENT_STEP;
  return Math.abs(rad - q) < DETENT_ZONE ? q : rad;
}
function setRotation(rad) {
  const r = Math.atan2(Math.sin(rad), Math.cos(rad));
  view.rot = r;
  store('sahabat.rot', String(r));
  $('gimbal-dial').style.transform = `rotate(${-r * DEG}deg)`;
  const deg = r * DEG;
  $('gimbal-deg').textContent = `${Number.isInteger(Math.round(deg * 10) / 10) ? Math.round(deg) : deg.toFixed(1)}°`;
  draw();
}
let gimbalDrag = null;
const gimbalAngle = (e) => {
  const r = gimbal.getBoundingClientRect();
  return Math.atan2(e.clientY - (r.top + r.height / 2), e.clientX - (r.left + r.width / 2));
};
gimbal.addEventListener('pointerdown', (e) => {
  try { gimbal.setPointerCapture(e.pointerId); } catch (_e) { /* synthetic pointer */ }
  gimbalDrag = { a: gimbalAngle(e), rot: view.rot, moved: false };
});
gimbal.addEventListener('pointermove', (e) => {
  if (!gimbalDrag) return;
  let d = gimbalAngle(e) - gimbalDrag.a;
  d = Math.atan2(Math.sin(d), Math.cos(d));
  if (Math.abs(d) > 0.03) gimbalDrag.moved = true;
  if (gimbalDrag.moved) setRotation(detent(gimbalDrag.rot - d, e.shiftKey));
});
// Wheel on the dial: 1° per notch, 0.1° with Shift.
gimbal.addEventListener('wheel', (e) => {
  e.preventDefault();
  const step = (e.shiftKey ? 0.1 : 1) / DEG;
  setRotation(view.rot + (e.deltaY > 0 ? -step : step));
}, { passive: false });
const gimbalEnd = () => {
  if (gimbalDrag && !gimbalDrag.moved) setRotation(0);
  gimbalDrag = null;
};
gimbal.addEventListener('pointerup', gimbalEnd);
gimbal.addEventListener('pointercancel', () => { gimbalDrag = null; });
setRotation(view.rot);

// ------------------------------------------------------------------- camera
const cam = $('cam');
function updateCamera() {
  const want = !document.hidden && !$('camera-card').classList.contains('hidden')
    && !document.querySelector('[data-panel="localize"]').classList.contains('hidden');
  const on = cam.dataset.on === '1';
  if (want && !on) {
    cam.src = `api/camera.mjpg?fps=${$('cam-fps').value}&t=${Date.now()}`;
    cam.dataset.on = '1';
  } else if (!want && on) {
    cam.removeAttribute('src');
    cam.src = 'data:,'; // closes the MJPEG connection so the robot stops encoding
    cam.dataset.on = '0';
  }
}
$('cam-toggle').onchange = () => {
  $('camera-card').classList.toggle('hidden', !$('cam-toggle').checked);
  store('sahabat.cam', $('cam-toggle').checked ? '1' : '0');
  updateCamera();
};
$('cam-toggle').checked = store('sahabat.cam') !== '0';
$('cam-fps').value = store('sahabat.camfps') || '15';
$('cam-fps').onchange = () => {
  store('sahabat.camfps', $('cam-fps').value);
  cam.dataset.on = '0';
  updateCamera();
};
$('camera-card').classList.toggle('hidden', !$('cam-toggle').checked);
document.addEventListener('visibilitychange', updateCamera);
document.querySelectorAll('[data-tab]').forEach((b) => b.addEventListener('click', updateCamera));

// --------------------------------------------------------------------- init
const savedTab = store('sahabat.tab');
if (savedTab) document.querySelector(`[data-tab="${savedTab}"]`)?.click();
setTool('pan');
renderMapping();
connect();
