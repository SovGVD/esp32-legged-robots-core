import './style.css';
import { Cli } from './cli.ts';
import { loadConfig } from './config.ts';
import { Connection, type ConnectionState } from './connection.ts';
import { Joystick } from './joystick.ts';
import {
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
  type Telemetry,
} from './protocol.ts';
import { MapPanel } from './map.ts';
import { RemoteToggle } from './toggle.ts';
import { TofPanel } from './tof.ts';

const MOVE_INTERVAL_MS = 50; // the firmware failsafe expects a move package at least this often
const TELEMETRY_INTERVAL_MS = 1000;
// ToF frames while the ToF view is open: the sensors give 15 frames/s, 8/s is
// plenty to look at and keeps WiFi traffic at ~1 KB/s per sensor
const TOF_INTERVAL_MS = 125;
// how long a toggle waits for telemetry to confirm a switch before showing
// the robot's state again; a gait change waits until the robot stands still
const GAIT_CONFIRM_MS = 15000;
const LEVEL_CONFIRM_MS = 2500;
// map view (lidar / ToF room scans): pose, path and scan 4 times a second, the map (10 KB) once a second
const NAV_INTERVAL_MS = 250;
const MAP_INTERVAL_MS = 1000;

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

const config = loadConfig();
const vector = zeroVector();
let telemetry: Telemetry | null = null;

const ui = {
  conn: $('conn'),
  power: $('power'),
  loop: $('loop'),
  status: $('status'),
  rotate: $<HTMLInputElement>('rotate'),
  cliToggle: $<HTMLInputElement>('cli-toggle'),
  takeControl: $<HTMLButtonElement>('take-control'),
  control: $('control'),
  cli: $('cli'),
  tofToggle: $<HTMLInputElement>('tof-toggle'),
  tof: $('tof'),
  mapToggle: $<HTMLInputElement>('map-toggle'),
  map: $('map'),
};

const stateText: Record<ConnectionState, string> = {
  connecting: 'Connecting…',
  online: 'Online',
  offline: 'Offline',
  replaced: 'Another device took control',
};

const connection = new Connection(config.wsUrl, {
  onState(state) {
    document.body.dataset.state = state;
    ui.conn.textContent = stateText[state];
    ui.takeControl.hidden = state !== 'replaced';
    if (state !== 'online') {
      telemetry = null;
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
      if (!ui.tof.hidden) tof.update(msg.sensors);
    } else if (msg.type === 'map') {
      if (!ui.map.hidden) map.setMap(msg.map);
    } else if (msg.type === 'nav') {
      if (!ui.map.hidden) map.setNav(msg.nav);
    } else if (msg.type === 'cli') {
      cli.write(msg.text);
    }
  },
});

const cli = new Cli($('cli-output'), $<HTMLInputElement>('cli-input'), $<HTMLFormElement>('cli-form'), (cmd) =>
  connection.send(encodeCli(cmd)),
);

// ask for telemetry right away so the toggle confirms without waiting a second
function sendSetting(data: Uint8Array): void {
  connection.send(data);
  connection.send(encodeTelemetryRequest());
}

const walk = new RemoteToggle($<HTMLInputElement>('walk'), (on) => sendSetting(encodeGait(on)), GAIT_CONFIRM_MS);
const level = new RemoteToggle($<HTMLInputElement>('level'), (on) => sendSetting(encodeLevel(on)), LEVEL_CONFIRM_MS);
$('walk-label').hidden = !config.gaitToggle;
$('level-label').hidden = !config.levelToggle;

const tof = new TofPanel(ui.tof, ui.control, config.tofSensors, config.tofRange, config.tofMounts);
$('tof-label').hidden = config.tofSensors === 0;
function showTof(on: boolean): void {
  ui.tofToggle.checked = on;
  ui.tof.hidden = !on;
  tof.show(on);
}

// the map and the ToF view share the space above the sticks: one at a time
const map = new MapPanel(
  ui.map,
  (x, y) => connection.send(encodeGoto(x, y)),
  () => connection.send(encodeScan()),
);
$('map-label').hidden = !config.lidar;
function showMap(on: boolean): void {
  ui.mapToggle.checked = on;
  ui.map.hidden = !on;
  if (on) {
    connection.send(encodeMapRequest());
    connection.send(encodeNavRequest());
  }
}
ui.tofToggle.addEventListener('change', () => {
  if (ui.tofToggle.checked) showMap(false);
  showTof(ui.tofToggle.checked);
});
ui.mapToggle.addEventListener('change', () => {
  if (ui.mapToggle.checked) showTof(false);
  showMap(ui.mapToggle.checked);
});

const batteryText = { unknown: '', ok: '', low: 'battery low', critical: 'battery empty' } as const;

function renderTelemetry(): void {
  const t = telemetry;
  const battery = t ? batteryState(t.statuses) : 'unknown';
  ui.power.textContent = t ? `${t.voltage.toFixed(2)} V  ${t.current.toFixed(2)} A` : '— V  — A';
  ui.power.classList.toggle('warn', battery === 'low' || battery === 'critical');
  ui.loop.textContent = t ? `loop ${t.loopTime} µs` : '';
  ui.loop.classList.toggle('warn', !!t && (t.statuses & TS.overrun) !== 0);

  const parts: string[] = [];
  let warn = false;
  if (t) {
    if (batteryText[battery]) {
      parts.push(batteryText[battery]);
      warn = true;
    }
    if (t.statuses & TS.sitting) parts.push('resting');
    else if (t.statuses & TS.hal) {
      parts.push(t.statuses & TS.gaitStatic ? 'walk' : 'trot');
      if (t.statuses & TS.gaitActive) parts.push('stepping');
    } else parts.push('HAL off');
    if (t.statuses & TS.leveling) parts.push('level');
    if (t.statuses & TS.terrain) parts.push(t.statuses & TS.tof ? 'terrain' : 'terrain (no ToF)');
    if (t.statuses & TS.gapBlocked) {
      parts.push('GAP AHEAD: stopped');
      warn = true;
    }
    if (t.heading !== undefined && t.heading !== null) {
      let h = `${t.statuses & TS.headingAbsolute ? 'heading' : 'turned'} ${Math.round(t.heading)}°`;
      if (t.statuses & TS.headingHold && t.headingTarget !== undefined && t.headingTarget !== null) {
        h += ` → ${Math.round(t.headingTarget)}°`;
      }
      parts.push(h);
    }
    if (t.pitch !== undefined && t.roll !== undefined && t.statuses & TS.imu) {
      parts.push(`tilt ${t.pitch.toFixed(1)}° ${t.roll.toFixed(1)}°`);
    }
    if (t.missedTicks) parts.push(`missed ${t.missedTicks}`);
    if (t.ik && t.ik.reach + t.ik.jointLimit + t.ik.servoRange > 0) {
      parts.push(`IK ${t.ik.reach}/${t.ik.jointLimit}/${t.ik.servoRange}`);
    }
  }
  ui.status.textContent = parts.join(' · ');

  const sitting = !!t && (t.statuses & TS.sitting) !== 0;
  walk.sync(t ? (t.statuses & TS.gaitStatic) !== 0 : null, !sitting);
  level.sync(t ? (t.statuses & TS.leveling) !== 0 : null, !!t && (t.statuses & TS.imu) !== 0 && !sitting);
  ui.status.classList.toggle('warn', warn);
  ui.tof.classList.toggle('gap', !!t && (t.statuses & TS.gapBlocked) !== 0);
}

function stop(): void {
  sticks.forEach((s) => s.release());
  Object.assign(vector, zeroVector());
}

// Left stick: turn. Right stick: walk, or tilt the body in "rotate" mode.
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

ui.rotate.addEventListener('change', stop);
ui.cliToggle.addEventListener('change', () => {
  const on = ui.cliToggle.checked;
  stop();
  ui.cli.hidden = !on;
  ui.control.hidden = on;
  if (on) cli.focus();
});
ui.takeControl.addEventListener('click', () => connection.connect());

document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'hidden') {
    // tab hidden or phone locked: stop the robot right away instead of
    // waiting for the firmware failsafe timeout
    stop();
    connection.send(encodeMove(vector, false));
  }
});

setInterval(() => {
  if (document.visibilityState === 'visible') connection.send(encodeMove(vector));
}, MOVE_INTERVAL_MS);
setInterval(() => connection.send(encodeTelemetryRequest()), TELEMETRY_INTERVAL_MS);
setInterval(() => {
  if (!ui.tof.hidden && document.visibilityState === 'visible') connection.send(encodeTofRequest());
}, TOF_INTERVAL_MS);

setInterval(() => {
  if (!ui.map.hidden && document.visibilityState === 'visible') connection.send(encodeNavRequest());
}, NAV_INTERVAL_MS);
setInterval(() => {
  if (!ui.map.hidden && document.visibilityState === 'visible') connection.send(encodeMapRequest());
}, MAP_INTERVAL_MS);

$('version').textContent = `v${config.version}`;
renderTelemetry();
connection.connect();
