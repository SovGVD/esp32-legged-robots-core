import './style.css';
import { Cli } from './cli.ts';
import { loadConfig } from './config.ts';
import { Connection, type ConnectionState } from './connection.ts';
import { DemoSocket, installDemoConfig } from './demo.ts';
import { Joystick } from './joystick.ts';
import {
  NAV_STATES,
  TS,
  batteryState,
  encodeCli,
  encodeGait,
  encodeGoto,
  encodeScan,
  encodeMapRequest,
  encodeNavRequest,
  encodeLevel,
  encodeMove,
  encodeTelemetryRequest,
  encodeTofRequest,
  zeroVector,
  type NavInfo,
  type Telemetry,
} from './protocol.ts';
import { MapPanel } from './map.ts';
import { RemoteToggle } from './toggle.ts';
import { TofPanel } from './tof.ts';

const MOVE_INTERVAL_MS = 50; // the firmware failsafe expects a move package at least this often
const TELEMETRY_INTERVAL_MS = 1000;
// ToF frames while the View tab is open: the sensors give 15 frames/s, 8/s is
// plenty to look at and keeps WiFi traffic at ~1 KB/s per sensor
const TOF_INTERVAL_MS = 125;
// how long a toggle waits for telemetry to confirm a switch before showing
// the robot's state again; a gait change waits until the robot stands still
const GAIT_CONFIRM_MS = 15000;
const LEVEL_CONFIRM_MS = 2500;
// Map tab (lidar / ToF room scans): pose, path and scan 4 times a second, the map (10 KB) once a second
const NAV_INTERVAL_MS = 250;
const MAP_INTERVAL_MS = 1000;

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

/** localStorage that may not be there (private mode, blocked storage). */
const store = {
  get(key: string): string | null {
    try {
      return localStorage.getItem(key);
    } catch {
      return null;
    }
  },
  set(key: string, value: string): void {
    try {
      localStorage.setItem(key, value);
    } catch {
      /* not remembered */
    }
  },
};

if (__DEMO__) installDemoConfig();
const config = loadConfig();
const vector = zeroVector();
let telemetry: Telemetry | null = null;
let nav: NavInfo | null = null;

const ui = {
  conn: $('conn'),
  power: $('power'),
  rotate: $<HTMLInputElement>('rotate'),
  control: $('control'),
  alerts: $('alerts'),
  cards: $('cards'),
  modeChip: $<HTMLButtonElement>('mode-chip'),
  rightHint: $('right-hint'),
  cliInput: $<HTMLInputElement>('cli-input'),
};

// ---- tabs: one full-size view at a time, the sticks stay on top of all of them

type Tab = 'view' | 'map' | 'status' | 'cli' | 'settings';
const tabButtons = [...document.querySelectorAll<HTMLButtonElement>('#tabs button')];
const panes = [...document.querySelectorAll<HTMLElement>('.pane')];
const available = (t: Tab) => (t === 'view' ? config.tofSensors > 0 : t === 'map' ? config.lidar : true);
let tab: Tab = 'status';

function showTab(next: Tab): void {
  if (!available(next)) next = 'status';
  tab = next;
  store.set('tab', next);
  tabButtons.forEach((b) => b.classList.toggle('on', b.dataset.tab === next));
  panes.forEach((p) => (p.hidden = p.dataset.pane !== next));
  document.body.dataset.tab = next;
  tof.show(next === 'view');
  if (next === 'map') {
    connection.send(encodeMapRequest());
    connection.send(encodeNavRequest());
    map.draw();
  }
}
tabButtons.forEach((b) => {
  b.hidden = !available(b.dataset.tab as Tab);
  b.addEventListener('click', () => showTab(b.dataset.tab as Tab));
});

// ---- connection

const stateText: Record<ConnectionState, string> = {
  connecting: 'Connecting',
  online: 'Online',
  offline: 'Offline',
  replaced: 'Taken over',
};

const connection = new Connection(
  config.wsUrl,
  {
    onState(state) {
      document.body.dataset.state = state;
      ui.conn.textContent = stateText[state];
      if (state !== 'online') {
        telemetry = null;
        nav = null;
        tof.clear();
        map.clear();
      }
      renderTelemetry();
    },
    onMessage(msg) {
      if (msg.type === 'telemetry') {
        telemetry = msg.telemetry;
        renderTelemetry();
      } else if (msg.type === 'tof') {
        if (tab === 'view') tof.update(msg.sensors);
      } else if (msg.type === 'map') {
        if (tab === 'map') map.setMap(msg.map);
      } else if (msg.type === 'nav') {
        nav = msg.nav;
        if (tab === 'map') map.setNav(msg.nav);
      } else if (msg.type === 'cli') {
        cli.write(msg.text);
      }
    },
  },
  __DEMO__ ? () => new DemoSocket() as unknown as WebSocket : undefined,
);

const cli = new Cli($('cli-output'), ui.cliInput, $<HTMLFormElement>('cli-form'), (cmd) => connection.send(encodeCli(cmd)));

// ask for telemetry right away so the toggle confirms without waiting a second
function sendSetting(data: Uint8Array): void {
  connection.send(data);
  connection.send(encodeTelemetryRequest());
}

const walk = new RemoteToggle($<HTMLInputElement>('walk'), (on) => sendSetting(encodeGait(on)), GAIT_CONFIRM_MS);
const level = new RemoteToggle($<HTMLInputElement>('level'), (on) => sendSetting(encodeLevel(on)), LEVEL_CONFIRM_MS);
$('walk-label').hidden = !config.gaitToggle;
$('level-label').hidden = !config.levelToggle;

const tof = new TofPanel($('tof'), $('pane-view'), config.tofSensors, config.tofRange, config.tofMounts);
const map = new MapPanel(
  $('map'),
  (x, y) => connection.send(encodeGoto(x, y)),
  () => connection.send(encodeScan()),
);

// ---- status: a HUD line in the header, alerts on top of every tab, cards on the Status tab

const batteryText = { unknown: '', ok: '', low: 'Battery low', critical: 'Battery empty: the robot sat down' } as const;

function card(title: string): (value: string, sub?: string, warn?: boolean) => void {
  const el = document.createElement('div');
  el.className = 'card';
  el.innerHTML = '<div class="dim small"></div><div class="value mono"></div><div class="small dim"></div>';
  const [t, v, s] = el.children as unknown as HTMLElement[];
  t.textContent = title;
  ui.cards.append(el);
  return (value, sub = '', warn = false) => {
    v.textContent = value;
    s.textContent = sub;
    el.classList.toggle('warn', warn);
  };
}
const cards = {
  battery: card('Battery'),
  gait: card('Gait'),
  tilt: card('Body tilt'),
  heading: card('Heading'),
  loop: card('Motion loop'),
  terrain: card('Terrain'),
  ik: card('IK limits'),
  nav: config.lidar ? card('Map position') : null,
};

function alert(text: string, level: 'info' | 'warn', action?: [string, () => void]): HTMLElement {
  const el = document.createElement('div');
  el.className = `alert ${level}`;
  el.textContent = text;
  if (action) {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = action[0];
    b.addEventListener('click', action[1]);
    el.append(b);
  }
  return el;
}

let alertKey = '';
function renderAlerts(list: [string, 'info' | 'warn', ([string, () => void] | undefined)?][]): void {
  const key = list.map((a) => a[0]).join('|');
  if (key === alertKey) return; // keep the buttons under a finger stable
  alertKey = key;
  ui.alerts.replaceChildren(...list.map(([t, l, a]) => alert(t, l, a)));
}

const dash = '—';

function renderTelemetry(): void {
  const t = telemetry;
  const st = t?.statuses ?? 0;
  const battery = t ? batteryState(st) : 'unknown';
  const lowBattery = battery === 'low' || battery === 'critical';
  ui.power.textContent = t ? `${t.voltage.toFixed(2)} V · ${t.current.toFixed(1)} A` : '';
  ui.power.classList.toggle('warn', lowBattery);

  const alerts: Parameters<typeof renderAlerts>[0] = [];
  const state = connection.state;
  if (state === 'replaced') alerts.push(['Another device took control', 'warn', ['Take control', () => connection.connect()]]);
  else if (state !== 'online') alerts.push(['Connecting to the robot…', 'info']);
  if (batteryText[battery]) alerts.push([batteryText[battery], 'warn']);
  if (t && st & TS.gapBlocked) alerts.push(['Gap ahead: stopped', 'warn']);
  renderAlerts(alerts);

  const sitting = !!t && (st & TS.sitting) !== 0;
  if (!t) {
    Object.values(cards).forEach((c) => c?.(dash));
  } else {
    cards.battery(`${t.voltage.toFixed(2)} V`, `${t.current.toFixed(2)} A${batteryText[battery] ? ' · ' + battery : ''}`, lowBattery);
    const gait = sitting ? 'resting' : !(st & TS.hal) ? 'HAL off' : st & TS.gaitStatic ? 'walk' : 'trot';
    const gaitSub = [st & TS.gaitActive ? 'stepping' : 'standing', st & TS.leveling ? 'level on' : ''].filter(Boolean).join(' · ');
    cards.gait(gait, gaitSub);
    const imu = t.pitch !== undefined && t.roll !== undefined && st & TS.imu;
    cards.tilt(imu ? `${t.pitch!.toFixed(1)}° ${t.roll!.toFixed(1)}°` : dash, imu ? 'nose · sideways' : 'no IMU');
    if (t.heading !== undefined && t.heading !== null) {
      const hold = st & TS.headingHold && t.headingTarget != null ? `holding ${Math.round(t.headingTarget)}°` : '';
      cards.heading(`${Math.round(t.heading)}°`, [st & TS.headingAbsolute ? 'compass' : 'turned since start', hold].filter(Boolean).join(' · '));
    } else cards.heading(dash, 'no heading');
    const overrun = (st & TS.overrun) !== 0;
    cards.loop(`${t.loopTime} µs`, `missed ${t.missedTicks ?? 0}${overrun ? ' · overrun' : ''}`, overrun || !!t.missedTicks);
    const gap = (st & TS.gapBlocked) !== 0;
    cards.terrain(st & TS.terrain ? (st & TS.tof ? 'sensing' : 'no ToF') : 'open loop', gap ? 'gap ahead: stopped' : '', gap);
    const ik = t.ik;
    const ikSum = ik ? ik.reach + ik.jointLimit + ik.servoRange : 0;
    cards.ik(ik ? `${ik.reach} / ${ik.jointLimit} / ${ik.servoRange}` : dash, 'reach / joint / servo', ikSum > 0);
  }
  if (cards.nav) {
    cards.nav(nav ? `${(nav.x / 1000).toFixed(2)}, ${(nav.y / 1000).toFixed(2)} m` : dash, nav ? NAV_STATES[nav.state] ?? '' : 'open the Map tab');
  }

  walk.sync(t ? (st & TS.gaitStatic) !== 0 : null, !sitting);
  level.sync(t ? (st & TS.leveling) !== 0 : null, !!t && (st & TS.imu) !== 0 && !sitting);
  $('tof').classList.toggle('gap', !!t && (st & TS.gapBlocked) !== 0);
}

// ---- sticks: always on top. Left: turn. Right: walk, or tilt the body in tilt mode.

function stop(): void {
  sticks.forEach((s) => s.release());
  Object.assign(vector, zeroVector());
}

const sticks = [
  new Joystick($('stick-left'), (v) => {
    vector.rotate.yaw = v.x;
  }),
  new Joystick($('stick-right'), (v) => {
    if (ui.rotate.checked) {
      vector.rotate.roll = v.y;
      vector.rotate.pitch = v.x;
    } else {
      vector.move.x = v.x;
      vector.move.y = v.y;
    }
  }),
];

function syncTiltMode(): void {
  stop();
  ui.modeChip.classList.toggle('on', ui.rotate.checked);
  ui.rightHint.textContent = ui.rotate.checked ? 'tilt' : 'move';
}
ui.rotate.addEventListener('change', syncTiltMode);
ui.modeChip.addEventListener('click', () => {
  ui.rotate.checked = !ui.rotate.checked;
  syncTiltMode();
});

const sizeButtons = [...document.querySelectorAll<HTMLButtonElement>('#stick-size button')];
function setStickSize(size: string): void {
  if (!['s', 'm', 'l'].includes(size)) size = 'm';
  document.body.dataset.stick = size;
  sizeButtons.forEach((b) => b.classList.toggle('on', b.dataset.size === size));
  store.set('stick', size);
}
sizeButtons.forEach((b) => b.addEventListener('click', () => setStickSize(b.dataset.size!)));

// the phone keyboard needs the space: hide the sticks while typing a command
ui.cliInput.addEventListener('focus', () => {
  stop();
  document.body.classList.add('typing');
});
ui.cliInput.addEventListener('blur', () => document.body.classList.remove('typing'));

document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'hidden') {
    // tab hidden or phone locked: stop the robot right away instead of
    // waiting for the firmware failsafe timeout
    stop();
    connection.send(encodeMove(vector, false));
  }
});

const visible = () => document.visibilityState === 'visible';
setInterval(() => visible() && connection.send(encodeMove(vector)), MOVE_INTERVAL_MS);
setInterval(() => connection.send(encodeTelemetryRequest()), TELEMETRY_INTERVAL_MS);
setInterval(() => tab === 'view' && visible() && connection.send(encodeTofRequest()), TOF_INTERVAL_MS);
setInterval(() => tab === 'map' && visible() && connection.send(encodeNavRequest()), NAV_INTERVAL_MS);
setInterval(() => tab === 'map' && visible() && connection.send(encodeMapRequest()), MAP_INTERVAL_MS);
window.addEventListener('resize', () => tab === 'map' && map.draw());

const about = $('about');
about.textContent = `Web UI v${config.version} · ${config.wsUrl}`;
if (__DEMO__) {
  // demo build: say so, and let the dog (1 ToF) and the hexapod (2 ToF) be swapped
  const dog = config.tofSensors === 1;
  about.textContent = `Demo: a mock ${dog ? 'dog' : 'hexapod'} runs inside this page, no robot needed. `;
  const a = document.createElement('a');
  a.href = `#${dog ? 'hexapod' : 'dog'}`;
  a.textContent = `Try the ${dog ? 'hexapod' : 'dog'} (${dog ? 2 : 1} ToF)`;
  about.append(a);
}
setStickSize(store.get('stick') ?? 'm');
showTab((store.get('tab') as Tab) ?? 'view');
syncTiltMode();
renderTelemetry();
connection.connect();
