import './style.css';
import { Cli } from './cli.ts';
import { loadConfig } from './config.ts';
import { Connection, type ConnectionState } from './connection.ts';
import { Joystick } from './joystick.ts';
import { encodeCli, encodeMove, encodeTelemetryRequest, zeroVector, type Telemetry } from './protocol.ts';

const MOVE_INTERVAL_MS = 50; // the firmware failsafe expects a move package at least this often
const TELEMETRY_INTERVAL_MS = 1000;

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

const config = loadConfig();
const vector = zeroVector();
let telemetry: Telemetry | null = null;

const ui = {
  conn: $('conn'),
  power: $('power'),
  loop: $('loop'),
  rotate: $<HTMLInputElement>('rotate'),
  cliToggle: $<HTMLInputElement>('cli-toggle'),
  takeControl: $<HTMLButtonElement>('take-control'),
  control: $('control'),
  cli: $('cli'),
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
    if (state !== 'online') telemetry = null;
    renderTelemetry();
  },
  onMessage(msg) {
    if (msg.type === 'telemetry') {
      telemetry = msg.telemetry;
      renderTelemetry();
    } else if (msg.type === 'cli') {
      cli.write(msg.text);
    }
  },
});

const cli = new Cli($('cli-output'), $<HTMLInputElement>('cli-input'), $<HTMLFormElement>('cli-form'), (cmd) =>
  connection.send(encodeCli(cmd)),
);

function renderTelemetry(): void {
  ui.power.textContent = telemetry ? `${telemetry.voltage.toFixed(2)} V  ${telemetry.current.toFixed(2)} A` : '— V  — A';
  ui.loop.textContent = telemetry ? `loop ${telemetry.loopTime} µs` : '';
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

$('version').textContent = `v${config.version}`;
renderTelemetry();
connection.connect();
