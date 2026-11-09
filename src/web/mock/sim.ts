/**
 * Mock robot that speaks the firmware's websocket protocol, without Node or
 * an ESP32. The dev server wraps it in a real websocket (mock/robot.ts); the
 * demo build runs it inside the page (src/demo.ts).
 *
 * It is a toy, not the MuJoCo sim: the robot slides around a two room flat
 * with the sticks, the ToF zones are rays marched against the floor, boxes,
 * a gap and the walls, the map fills in as the "lidar" sees the walls, and
 * the battery drains while it walks.
 */
import type { TofMount } from '../src/config.ts';
import { zoneDirection } from '../src/tofgeom.ts';
import {
  P_CLI,
  P_GAIT,
  P_GOTO,
  P_LEVEL,
  P_MAP,
  P_MOVE,
  P_NAV,
  P_SETUP,
  P_TELEMETRY,
  P_TOF,
  TOF_ZONES,
  SH,
  SO,
  TS,
  decodeAxis,
} from '../src/protocol.ts';

const FAILSAFE_MS = 500;
const SPEED = 300; // mm/s at full stick forward
const SIDE_SPEED = 200;
const YAW_RATE = 1.6; // rad/s at full stick
const RADIUS = 110; // robot footprint for the walls, mm
const FLOOR = -95; // ground under the body, mm
const BATTERY_FULL = 8.2;
const BATTERY_LOW = 6.8;
const BATTERY_CRITICAL = 6.4;
const SCAN_S = 4; // a ToF room scan: one turn in 4 s

/** Walls as segments, mm (map frame: x right, y forward at the start). */
export const WALLS: [number, number, number, number][] = [
  [-1600, -900, 1800, -900], [-1600, 4800, 1800, 4800], [-1600, -900, -1600, 4800], [1800, -900, 1800, 4800],
  [-1600, 2200, 600, 2200], [1400, 2200, 1800, 2200], [-1450, 0, -850, 0], [-850, 0, -850, 1400],
];
/** Boxes on the floor: x0, y0, x1, y1, height (mm). The robot walks over them. */
const BOXES: [number, number, number, number, number][] = [
  [-200, 600, 100, 800, 30], [900, 1200, 1300, 1500, 20], [-1200, 3200, -700, 3600, 40], [500, 3800, 900, 4100, 25],
];
/** A gap (stairs down): the robot stops in front of it, like the firmware's gap check. */
const GAPS: [number, number, number, number][] = [[200, 1700, 1700, 2000]];

const MAP_N = 200;
const MAP_CELL = 50;

// Magnetometer: earth field (horizontal north, vertical down) and the board's own
// distortion, so the compass is off until `heading cal` and the plot has something to fix
const EARTH_H = 22;
const EARTH_V = 40;
const HARD_IRON = [14, -9, 3];
const SOFT_IRON = [1.18, 0.86, 1];
const CAL_TURN = 0.5; // yaw command of the calibration turn, as HEADING_CAL_TURN
const JOINTS = ['alpha', 'beta', 'gamma'];

const wrap180 = (d: number) => ((((d + 180) % 360) + 360) % 360) - 180;

/** ToF mounting, same as the dev server's /c.js (model_data.h TOF_SENSORS, PR #25 angles). */
export function mockTofMounts(count: number): TofMount[] {
  const m = (x: number, y: number, z: number, yaw: number, tilt: number) => ({ x, y, z, yaw, tilt, fov: 0.785 });
  return count === 1 ? [m(0, 88, 0, 0, 0.44)] : [m(-25, 90, 10, 0.35, 0.61), m(25, 90, 10, -0.35, 0.38)];
}

/** /c.js for the mock: see src/config.ts for the keys. */
export function mockConfig(tofCount: number, ws: string) {
  const m = mockTofMounts(tofCount).map((s) => [s.x, s.y, s.z, s.yaw, s.tilt, s.fov]);
  return { v: 'mock', w: ws, c: 0, g: 1, l: 1, t: tofCount, r: 450, n: 1, m };
}

function segDist(px: number, py: number, [x0, y0, x1, y1]: [number, number, number, number]): number {
  const ex = x1 - x0;
  const ey = y1 - y0;
  const t = Math.max(0, Math.min(1, ((px - x0) * ex + (py - y0) * ey) / (ex * ex + ey * ey)));
  return Math.hypot(px - x0 - t * ex, py - y0 - t * ey);
}

const inside = (x: number, y: number, r: number[]) => x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3];

/** Height of whatever is at a map point, relative to the body (mm). */
export function heightAt(x: number, y: number): number {
  for (const w of WALLS) if (segDist(x, y, w) < 40) return 800;
  for (const b of BOXES) if (inside(x, y, b)) return FLOOR + b[4];
  for (const g of GAPS) if (inside(x, y, g)) return -400;
  return FLOOR;
}

/** Distance along a ray to the nearest wall, Infinity if none. */
function wallHit(x: number, y: number, dx: number, dy: number): number {
  let best = Infinity;
  for (const [x0, y0, x1, y1] of WALLS) {
    const ex = x1 - x0;
    const ey = y1 - y0;
    const den = dx * ey - dy * ex;
    if (Math.abs(den) < 1e-9) continue;
    const t = ((x0 - x) * ey - (y0 - y) * ex) / den;
    const u = ((x0 - x) * dy - (y0 - y) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1) best = Math.min(best, t);
  }
  return best;
}

interface Bytes {
  b: Uint8Array;
  v: DataView;
}
function alloc(n: number): Bytes {
  const b = new Uint8Array(n);
  return { b, v: new DataView(b.buffer) };
}

const encoder = new TextEncoder();
const textDecoder = new TextDecoder();

export function cliPackage(text: string): Uint8Array {
  const t = encoder.encode(text);
  const b = new Uint8Array(t.length + 1);
  b[0] = P_CLI;
  b.set(t, 1);
  return b;
}

export interface MockOptions {
  tofCount: number;
  /** nav flag 32: ToF room scans instead of a lidar (Scan button) */
  tofNav: boolean;
  log?: (...args: unknown[]) => void;
}

export class MockRobot {
  // pose in the map frame: mm, heading rad counter-clockwise, 0 = facing +y
  x = 0;
  y = 0;
  heading = 0;
  voltage = BATTERY_FULL;
  walk = false;
  level = false;
  goal: [number, number] | null = null;
  navState = 0; // idle, walking, arrived, no path
  scanUntil = 0;
  sitting = false;
  hal = true;
  terrain: boolean;
  feet = false;
  hold = false;
  holdTarget: number | null = null;
  /** magnetometer calibration: state as the firmware's HeadingCalState */
  cal = { state: 0, turned: 0, time: 0, min: [0, 0, 0], max: [0, 0, 0], n: 0 };
  magCal = { offset: [0, 0, 0], scale: [1, 1, 1], field: 0 };
  readonly legs: string[];
  readonly trims: number[][];
  private pending: { what: 'terrain' | 'feet'; on: boolean } | null = null;
  private outbox: Uint8Array[] = [];
  private frozen = false;

  private readonly opts: MockOptions;
  private readonly mounts: TofMount[];
  private readonly cells = new Uint8Array(MAP_N * MAP_N);
  private axes = [0, 0, 0, 0, 0, 0]; // x y z pitch roll yaw
  private lastMove = 0;
  private last = 0;
  private stepping = false;
  private gapBlocked = false;
  private current = 0.4;
  private frame = 0;
  private scans = 0;
  private readonly log: (...args: unknown[]) => void;

  constructor(opts: MockOptions) {
    this.opts = opts;
    this.log = opts.log ?? (() => {});
    this.mounts = mockTofMounts(opts.tofCount);
    // the dog has one ToF and no foot switches, the hexapod two ToF and a switch per foot
    this.legs = opts.tofCount === 1 ? ['LF', 'RF', 'LH', 'RH'] : ['LF', 'LM', 'LR', 'RF', 'RM', 'RR'];
    this.trims = this.legs.map((_, i) => [0, i % 3 ? 0 : 1.5, i === 1 ? -2 : 0]);
    this.terrain = opts.tofCount > 0;
  }

  private get dog(): boolean {
    return this.legs.length === 4;
  }

  /** Magnetometer reading in the body frame, uT, as the board sees it (distorted). */
  magRaw(): number[] {
    const h = -this.heading; // compass, clockwise
    const body = [-Math.sin(h) * EARTH_H, Math.cos(h) * EARTH_H, -EARTH_V];
    const noise = this.stepping ? 1.2 : 0.4;
    return body.map((b, i) => HARD_IRON[i] + b / SOFT_IRON[i] + (Math.random() - 0.5) * noise);
  }

  /** Compass heading the firmware would report: the magnetometer with the calibration in use. */
  compass(): number {
    const m = this.magRaw().map((r, i) => (r - this.magCal.offset[i]) * this.magCal.scale[i]);
    return ((Math.atan2(-m[0], m[1]) * 180) / Math.PI + 360) % 360;
  }

  private say(text: string): void {
    this.outbox.push(cliPackage(text));
  }

  /** Answer one package from the page; returns the reply packages. */
  receive(raw: Uint8Array, now = Date.now()): Uint8Array[] {
    this.tick(now);
    if (raw.length === 0) return [];
    // replies that came later (IMU calibration, a switch once standing)
    if (this.outbox.length) return [...this.outbox.splice(0), ...this.answer(raw, now)];
    return this.answer(raw, now);
  }

  private answer(raw: Uint8Array, now: number): Uint8Array[] {
    const v = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
    switch (raw[0]) {
      case P_MOVE: {
        if (raw.length < 14) return [];
        this.lastMove = now;
        this.axes = raw[1] === 1 ? [2, 4, 6, 8, 10, 12].map((o) => decodeAxis(v.getUint16(o))) : [0, 0, 0, 0, 0, 0];
        if (raw[1] === 0) this.log('[mock robot] client sent failsafe');
        return [];
      }
      case P_TELEMETRY:
        return [this.telemetry()];
      case P_SETUP:
        return [this.setup(now)];
      case P_TOF:
        return [this.tof(now)];
      case P_MAP:
        return [this.mapPackage()];
      case P_NAV:
        return [this.nav(now)];
      case P_GOTO:
        if (raw.length === 2 && raw[1] === 0x53) {
          this.scanUntil = now + SCAN_S * 1000;
          this.log('[mock robot] scan');
        } else if (raw.length >= 5) {
          this.goal = [v.getInt16(1), v.getInt16(3)];
          this.navState = 1;
          this.log(`[mock robot] goto ${this.goal[0]} ${this.goal[1]} mm`);
        } else {
          this.navState = 0;
          this.goal = null;
          this.log('[mock robot] goto stop');
        }
        return [];
      case P_GAIT:
        this.walk = raw[1] === 1;
        this.log(`[mock robot] gait ${this.walk ? 'walk' : 'trot'}`);
        return [];
      case P_LEVEL:
        this.level = raw[1] === 1;
        this.log(`[mock robot] leveling ${this.level ? 'on' : 'off'}`);
        return [];
      case P_CLI: {
        const end = raw.indexOf(0);
        const command = textDecoder.decode(raw.subarray(1, end < 0 ? raw.length : end)).trim();
        return [cliPackage(`> ${command}\n${this.cli(command)}`)];
      }
      default:
        this.log('[mock robot] unknown package', raw[0]);
        return [];
    }
  }

  private cli(command: string): string {
    const [cmd, arg = '', a2 = '', a3 = ''] = command.split(/\s+/);
    const onOff = (set: (on: boolean) => void) => (arg === 'on' ? set(true) : arg === 'off' ? set(false) : undefined);
    switch (cmd) {
      case 'help':
        return [
          ' - [hal     ] [on|off|state] leg calculation',
          ' - [level   ] [on|off|state] IMU leveling',
          ' - [gait    ] Gait [trot|walk|state]',
          ' - [terrain ] [on|off|state] ToF terrain map',
          ' - [feet    ] [on|off|state] foot switches',
          ' - [heading ] [on|off|state|cal|hand] heading',
          ' - [turnto  ] Turn to compass heading <deg>',
          ' - [calib   ] Calibrate [imu|leg]',
          ' - [trim    ] Trim servos <leg> <joint> <deg>',
          ' - [show    ] Display current settings',
          ' - [save    ] Save settings',
          ' - [tof     ] [state|print] ToF sensors',
          ' - [lidar   ] [state|scan|reset|freeze|stop]',
          ' - [loop    ] Motion loop timing',
          ' - [wifi    ] WiFi AP information',
          'Mock only: state, battery [V], home, ping',
          '',
        ].join('\n');
      case 'ping':
        return 'pong\n';
      case 'state':
        return `pose ${(this.x / 1000).toFixed(2)}, ${(this.y / 1000).toFixed(2)} m, heading ${Math.round((this.heading * 180) / Math.PI)} deg\nbattery ${this.voltage.toFixed(2)} V, ${this.walk ? 'walk' : 'trot'}, level ${this.level ? 'on' : 'off'}\n`;
      case 'battery':
        this.voltage = arg ? Math.max(5, Math.min(9, Number(arg) || BATTERY_FULL)) : BATTERY_FULL;
        this.sitting = this.voltage < BATTERY_CRITICAL;
        return `battery ${this.voltage.toFixed(2)} V\n`;
      case 'home':
        this.x = this.y = this.heading = 0;
        this.navState = 0;
        return 'back at the start\n';
      case 'hal':
        onOff((on) => (this.hal = on));
        return `HAL: ${this.hal ? 'on' : 'off'}\n`;
      case 'level':
        onOff((on) => (this.level = on));
        return `Leveling: ${this.level ? 'on' : 'off'}, IMU MPU9250 (mock)\n`;
      case 'gait':
        if (arg === 'walk' || arg === 'trot') this.walk = arg === 'walk';
        else if (arg !== 'state') return 'Usage: gait trot|walk|state\n';
        return `Gait: ${this.walk ? 'walk' : 'trot'}\n`;
      case 'terrain':
        onOff((on) => (this.pending = { what: 'terrain', on }));
        return `Terrain: ${this.terrain ? 'on' : 'off'}${this.pending?.what === 'terrain' ? ' (switch pending: once the robot stands still)' : ''}\nToF: ${this.mounts.length} running\n`;
      case 'feet':
        if (this.dog) return 'No foot switches: set FOOT_SWITCH_PINS in the model\'s config.h.\n';
        onOff((on) => (this.pending = { what: 'feet', on }));
        return `Foot switches: ${this.feet ? 'on' : 'off'}${this.pending?.what === 'feet' ? ' (switch pending)' : ''}\n${this.legs.map((l, i) => `  ${l}: ${this.footBits() & (1 << i) ? 'pressed' : 'released'}`).join('\n')}\n`;
      case 'heading':
        if (arg === 'on' || arg === 'off') {
          this.hold = arg === 'on';
          this.holdTarget = null;
        } else if (arg === 'cal' || arg === 'hand') {
          this.cal = { state: arg === 'cal' ? 1 : 2, turned: 0, time: 0, min: [0, 0, 0], max: [0, 0, 0], n: 0 };
          return arg === 'cal' ? 'Turning two full turns in place on flat ground, away from steel. Stop: `hal off`.\n' : 'Turn the robot slowly two full turns on flat ground (60 s). Mock: turn it with the left stick.\n';
        }
        return `Heading ${this.compass().toFixed(1)} deg (magnetic north)\nMagnetometer: yes, calibration ${['not run', 'turning', 'turn it by hand', 'saved', 'failed', 'stopped'][this.cal.state]}\nHold: ${this.hold ? 'on' : 'off'}${this.holdTarget !== null ? `, target ${this.holdTarget.toFixed(1)} deg` : ''}\n`;
      case 'turnto': {
        const deg = ((Number(arg) % 360) + 360) % 360;
        this.hold = true;
        this.holdTarget = deg;
        return `Turning to ${deg.toFixed(1)} deg and holding it (\`heading off\` to stop).\n`;
      }
      case 'calib':
        if (arg === 'imu') {
          setTimeout(() => this.say('IMU calibrated.\n'), 5000);
          return 'IMU: calibrating in 5 s, keep the robot still on a flat level surface.\n';
        }
        if (arg === 'leg') return 'Servos at the calibration pose (`hal off` first, or the gait moves them back).\n';
        return 'Usage: calib imu|leg\n';
      case 'trim': {
        const leg = this.legs.indexOf(arg);
        const j = JOINTS.indexOf(a2);
        if (j < 0) return `Unknown angle [${a2}].\n`;
        if (leg < 0) return `Leg is not valid.\nLegs: ${this.legs.map((l) => `[${l}]`).join(',')},\n`;
        this.trims[leg][j] = Number(a3) || 0;
        return `Angle ${((this.trims[leg][j] * Math.PI) / 180).toFixed(6)} rad, ${this.trims[leg][j].toFixed(6)} deg.\n`;
      }
      case 'show':
        return this.trimText();
      case 'save':
        return `Settings saved.\n${this.trimText()}`;
      case 'tof':
        return this.mounts.map((_, i) => `ToF ${i} (VL53L5CX): ok`).join('\n') + (arg === 'print' ? '\n  (mock: zones in the View tab)' : '') + '\n';
      case 'lidar':
        if (arg === 'reset') {
          this.cells.fill(0);
          return 'Map cleared: the robot is at 0, 0 facing +y.\n';
        }
        if (arg === 'freeze') this.frozen = !this.frozen;
        if (arg === 'stop') {
          this.navState = 0;
          this.goal = null;
          return 'Stopped walking to the goal.\n';
        }
        if (arg === 'scan') this.scanUntil = Date.now() + SCAN_S * 1000;
        return `Map: ${this.scans} scans${this.frozen ? ' (frozen)' : ''}\nPose: x ${(this.x / 1000).toFixed(2)} m, y ${(this.y / 1000).toFixed(2)} m\n`;
      case 'loop':
        return 'Motion loop, last second: 232 ticks, 0 missed\n  total avg 1310 us, worst 1890 us (LOOP_TIME 4310)\nMissed since boot: 0\n';
      case 'wifi':
        return 'WiFi AP: SmallDogV1, IP 192.168.1.1, 1 client (mock)\n';
      case 'hil':
        return 'Not a HIL build (pio run -e robot-dog-hil / robot-hexapod-hil).\n';
      default:
        return `Unknown command: ${command} (try help)\n`;
    }
  }

  private trimText(): string {
    return this.legs.map((l, i) => `[${l}] trim {${this.trims[i].map((t) => t.toFixed(2)).join(', ')}}`).join('\n') + '\n';
  }

  /** Feet on the ground: all while standing, a tripod at a time while the hexapod walks. */
  private footBits(now = Date.now()): number {
    if (this.dog) return 0;
    if (!this.stepping || this.sitting) return 0b111111;
    return Math.floor(now / 300) % 2 ? 0b101010 : 0b010101;
  }

  /** Settings tab state (firmware pSetup). */
  setup(now = Date.now()): Uint8Array {
    const dof = 3;
    const legBytes = this.legs.reduce((n, l) => n + 1 + l.length + dof * 2, 0);
    const { b, v } = alloc(34 + legBytes);
    const raw = this.magRaw();
    const field = Math.hypot(...raw.map((r, i) => (r - this.magCal.offset[i]) * this.magCal.scale[i]));
    b[0] = P_SETUP;
    b[1] = 1;
    b[2] = SH.mag | SH.walk | SH.imu | SH.raw | (this.opts.tofCount ? SH.terrain : 0) | (this.dog ? 0 : SH.feet) | (this.hal ? SH.hal : 0);
    b[3] =
      (this.terrain ? SO.terrain : 0) |
      (this.pending?.what === 'terrain' ? SO.terrainPending : 0) |
      (this.feet ? SO.feet | SO.feetGait : 0) |
      (this.hold ? SO.hold : 0) |
      (this.level ? SO.level : 0) |
      (this.walk ? SO.walk : 0) |
      (this.magCal.field === 0 || Math.abs(field - this.magCal.field) < 0.25 * this.magCal.field ? SO.magTrusted : 0);
    b[4] = this.cal.state;
    b[5] = Math.min(255, Math.round(this.cal.turned / 4));
    b[6] = Math.min(255, Math.round(this.cal.time));
    b[7] = (1 << this.mounts.length) - 1;
    v.setUint16(8, this.footBits(now));
    raw.forEach((r, i) => v.setInt16(10 + i * 2, Math.round(r * 10)));
    this.magCal.offset.forEach((o, i) => v.setInt16(16 + i * 2, Math.round(o * 10)));
    this.magCal.scale.forEach((k, i) => v.setUint16(22 + i * 2, Math.round(k * 1000)));
    v.setUint16(28, Math.round(this.magCal.field * 10));
    v.setUint16(30, Math.round(field * 10));
    b[32] = this.legs.length;
    b[33] = dof;
    let o = 34;
    this.legs.forEach((l, i) => {
      b[o++] = l.length;
      for (const ch of l) b[o++] = ch.charCodeAt(0);
      for (const t of this.trims[i]) {
        v.setInt16(o, Math.round(t * 100));
        o += 2;
      }
    });
    return b;
  }

  /** Magnetometer calibration turn: collect min/max like MagCalibrator, fit after two turns. */
  private calStep(dt: number, turnedDeg: number): void {
    const c = this.cal;
    c.time += dt;
    c.turned += Math.abs(turnedDeg);
    const m = this.magRaw();
    for (let i = 0; i < 3; i++) {
      if (c.n === 0 || m[i] < c.min[i]) c.min[i] = m[i];
      if (c.n === 0 || m[i] > c.max[i]) c.max[i] = m[i];
    }
    c.n++;
    if (c.turned < 720 && c.time < 60) return;
    const rx = (c.max[0] - c.min[0]) / 2;
    const ry = (c.max[1] - c.min[1]) / 2;
    if (c.turned < 720 || c.n < 50 || rx < 5 || ry < 5) {
      c.state = 4;
      this.say('Magnetometer calibration failed (not two full turns in 60 s)\n');
      return;
    }
    const r = (rx + ry) / 2;
    const offset = [(c.max[0] + c.min[0]) / 2, (c.max[1] + c.min[1]) / 2, 0];
    const scale = [r / rx, r / ry, 1];
    const h = r;
    const vz = ((c.max[2] + c.min[2]) / 2 - offset[2]) * scale[2];
    this.magCal = { offset, scale, field: Math.hypot(h, vz) };
    c.state = 3;
    this.say(`Magnetometer calibration saved (turned ${Math.round(c.turned)} deg)\n`);
  }

  /** Move the robot from the last stick command or toward the goal. */
  tick(now = Date.now()): void {
    const dt = this.last ? Math.min(0.2, (now - this.last) / 1000) : 0;
    this.last = now;
    if (now - this.lastMove > FAILSAFE_MS) this.axes = [0, 0, 0, 0, 0, 0];
    let [sx, sy] = this.axes;
    let yaw = this.axes[5];
    const tilting = this.axes[3] !== 0 || this.axes[4] !== 0;
    const manual = sx !== 0 || sy !== 0 || yaw !== 0;
    if (manual && this.navState === 1) this.navState = 0; // the sticks win over goto

    // a switch asked for while walking waits until the robot stands still, like the firmware
    if (this.pending && !this.stepping) {
      if (this.pending.what === 'terrain') this.terrain = this.pending.on;
      else this.feet = this.pending.on;
      this.say(`${this.pending.what === 'terrain' ? 'Terrain mode' : 'Foot switches'} ${this.pending.on ? 'on' : 'off'}\n`);
      this.pending = null;
    }
    const compassNow = this.compass();
    if (this.cal.state === 1) {
      sx = sy = 0;
      yaw = this.hal && !this.sitting && now - this.lastMove < FAILSAFE_MS ? CAL_TURN : 0;
    } else if (this.hold && !(this.goal && this.navState === 1) && now >= this.scanUntil) {
      // heading hold: the stick turns and moves the target, let go and it holds
      if (yaw !== 0) this.holdTarget = null;
      else {
        if (this.holdTarget === null) this.holdTarget = compassNow;
        const err = wrap180(this.holdTarget - compassNow);
        yaw = Math.abs(err) < 2 ? 0 : Math.max(-0.6, Math.min(0.6, err / 30));
      }
    }
    if (now < this.scanUntil) {
      sx = sy = 0;
      yaw = (2 * Math.PI) / SCAN_S / YAW_RATE;
    } else if (!manual && this.goal && this.navState === 1) {
      // walk to the goal: turn toward it, then forward
      const dx = this.goal[0] - this.x;
      const dy = this.goal[1] - this.y;
      const d = Math.hypot(dx, dy);
      if (d < 80) {
        this.navState = 2;
      } else {
        let err = Math.atan2(-dx, dy) - this.heading;
        err = Math.atan2(Math.sin(err), Math.cos(err));
        yaw = -Math.max(-1, Math.min(1, err * 2));
        sy = Math.abs(err) < 0.5 ? Math.min(1, d / 300) * 0.7 : 0;
      }
    }
    if (this.sitting || !this.hal) sx = sy = yaw = 0;
    const speed = this.walk ? 0.4 : 1;

    const fwd = sy * SPEED * speed;
    const side = sx * SIDE_SPEED * speed;
    // stop in front of a gap, like GaitEngine::isBlocked()
    const c = Math.cos(this.heading);
    const s = Math.sin(this.heading);
    const ahead = (bx: number, by: number) => [this.x + bx * c - by * s, this.y + bx * s + by * c];
    const [gx, gy] = ahead(Math.sign(side) * 60, Math.sign(fwd) * 200);
    this.gapBlocked = this.terrain && (fwd !== 0 || side !== 0) && heightAt(gx, gy) < FLOOR - 60;
    if (!this.gapBlocked) {
      const [nx, ny] = ahead(side * dt, fwd * dt);
      const free = WALLS.every((w) => segDist(nx, ny, w) > RADIUS);
      if (free) {
        this.x = nx;
        this.y = ny;
      } else if (this.navState === 1) {
        this.navState = 3; // no path (the mock does not plan)
      }
    } else if (this.navState === 1) {
      this.navState = 3;
    }
    const turn = yaw * YAW_RATE * speed * dt;
    this.heading -= turn;
    this.heading = Math.atan2(Math.sin(this.heading), Math.cos(this.heading));
    if (this.cal.state === 1 || this.cal.state === 2) this.calStep(dt, (turn * 180) / Math.PI);

    const effort = Math.min(1, Math.hypot(sx, sy) + Math.abs(yaw) * 0.5);
    this.stepping = effort > 0.05;
    this.current = this.sitting ? 0.1 : 0.4 + effort * 1.8 + (tilting ? 0.3 : 0) + (Math.random() - 0.5) * 0.1;
    this.voltage -= dt * (0.0006 + effort * 0.004);
    if (this.voltage < BATTERY_CRITICAL && !this.sitting) {
      this.sitting = true;
      this.log('[mock robot] battery empty: sitting down');
    }
  }

  telemetry(): Uint8Array {
    const { b, v } = alloc(26);
    const battery = this.voltage < BATTERY_CRITICAL ? 3 : this.voltage < BATTERY_LOW ? 2 : 1;
    let st = TS.imu | TS.power | TS.headingAbsolute | (battery << 7);
    if (this.hal) st |= TS.hal;
    if (this.hold) st |= TS.headingHold;
    if (this.stepping) st |= TS.gaitActive;
    if (this.walk) st |= TS.gaitStatic;
    if (this.level) st |= TS.leveling;
    if (this.sitting) st |= TS.sitting;
    if (this.opts.tofCount) st |= TS.tof;
    if (this.terrain) st |= TS.terrain;
    if (this.gapBlocked) st |= TS.gapBlocked;
    b[0] = P_TELEMETRY;
    b[1] = 3;
    v.setUint16(2, st);
    v.setUint16(4, Math.round(this.voltage * 1000));
    v.setInt16(6, Math.round(this.current * 1000));
    v.setUint16(8, 1250 + Math.round(Math.random() * 200));
    // body tilt from the right stick in tilt mode; leveling holds it flat
    const keep = this.level ? 0.2 : 1;
    const noise = () => (Math.random() - 0.5) * (this.stepping ? 1.5 : 0.3);
    v.setInt16(10, Math.round((this.axes[3] * 12 * keep + noise()) * 100));
    v.setInt16(12, Math.round((this.axes[4] * 12 * keep + noise()) * 100));
    v.setUint16(14, Math.random() < 0.02 ? 1 : 0);
    v.setUint16(16, this.gapBlocked ? 2 : 0);
    v.setUint16(22, Math.round(this.compass() * 10) % 3600);
    v.setUint16(24, this.hold && this.holdTarget !== null ? Math.round(this.holdTarget * 10) % 3600 : 0xffff);
    return b;
  }

  /** ToF frames: zone rays marched against the flat from the robot's pose. */
  tof(now = Date.now(), range = 450): Uint8Array {
    const count = this.mounts.length;
    const size = 2 + TOF_ZONES * 2;
    const { b, v } = alloc(2 + count * size);
    b[0] = P_TOF;
    b[1] = count;
    this.frame = Math.floor(now / 66) & 0xff;
    const c = Math.cos(this.heading);
    const s = Math.sin(this.heading);
    // the body bobs a little while stepping
    const bob = this.stepping ? Math.sin(now / 90) * 4 : 0;
    for (let i = 0; i < count; i++) {
      const o = 2 + i * size;
      const m = this.mounts[i];
      b[o] = 1; // running
      b[o + 1] = this.frame;
      for (let z = 0; z < TOF_ZONES; z++) {
        const d = zoneDirection(m, z);
        let mm = 0;
        for (let r = 20; r <= range; r += 4) {
          const px = m.x + d[0] * r;
          const py = m.y + d[1] * r;
          if (m.z + bob + d[2] * r <= heightAt(this.x + px * c - py * s, this.y + px * s + py * c)) {
            mm = r + (Math.random() - 0.5) * 6;
            break;
          }
        }
        v.setUint16(o + 2 + z * 2, Math.max(0, Math.round(mm)));
      }
    }
    return b;
  }

  /** Map: unknown until the scan rays have seen it. */
  mapPackage(): Uint8Array {
    const { b, v } = alloc(6 + (MAP_N * MAP_N) / 4);
    b[0] = P_MAP;
    b[1] = 1;
    v.setUint16(2, MAP_N);
    v.setUint16(4, MAP_CELL);
    for (let i = 0; i < MAP_N * MAP_N; i++) b[6 + (i >> 2)] |= this.cells[i] << ((i & 3) * 2);
    return b;
  }

  private mark(x: number, y: number, code: number): void {
    const ix = Math.floor(x / MAP_CELL) + MAP_N / 2;
    const iy = Math.floor(y / MAP_CELL) + MAP_N / 2;
    if (ix < 0 || iy < 0 || ix >= MAP_N || iy >= MAP_N) return;
    const i = iy * MAP_N + ix;
    if (code === 3 || this.cells[i] === 0) this.cells[i] = code;
  }

  nav(now = Date.now()): Uint8Array {
    const scanning = now < this.scanUntil;
    // a lidar sees all around; ToF room scans only see while scanning (and a little ahead)
    const full = !this.opts.tofNav || scanning;
    const scan: [number, number][] = [];
    const c = Math.cos(-this.heading);
    const s = Math.sin(-this.heading);
    for (let a = 0; a < 360; a += 4) {
      if (!full && a > 30 && a < 330) continue;
      const r = (a * Math.PI) / 180 + this.heading;
      const dx = -Math.sin(r);
      const dy = Math.cos(r);
      const hit = wallHit(this.x, this.y, dx, dy);
      const reach = Math.min(hit, this.opts.tofNav ? 2500 : 6000);
      for (let k = 0; k < reach - MAP_CELL; k += MAP_CELL / 2) this.mark(this.x + dx * k, this.y + dy * k, 1);
      if (hit > reach) continue;
      this.mark(this.x + dx * hit, this.y + dy * hit, 3);
      const wx = dx * hit;
      const wy = dy * hit;
      scan.push([wx * c - wy * s, wx * s + wy * c]); // robot frame
    }
    if (scan.length) this.scans++;
    const path = this.goal && this.navState === 1 ? [[this.x, this.y], this.goal] : [];
    const { b, v } = alloc(19 + path.length * 4 + scan.length * 4);
    b[0] = P_NAV;
    b[1] = 1 | (this.opts.tofNav ? 32 : 0) | (scanning ? 16 : 0) | (this.gapBlocked ? 4 : 0);
    b[2] = this.navState;
    b[3] = 1;
    v.setInt16(4, Math.round(this.x));
    v.setInt16(6, Math.round(this.y));
    v.setInt16(8, Math.round((this.heading * 18000) / Math.PI));
    v.setInt16(10, Math.round(this.goal?.[0] ?? 0));
    v.setInt16(12, Math.round(this.goal?.[1] ?? 0));
    v.setUint16(14, this.scans & 0xffff);
    b[16] = path.length;
    let o = 17;
    for (const [x, y] of path) {
      v.setInt16(o, Math.round(x));
      v.setInt16(o + 2, Math.round(y));
      o += 4;
    }
    v.setUint16(o, scan.length);
    o += 2;
    for (const [x, y] of scan) {
      v.setInt16(o, Math.round(x));
      v.setInt16(o + 2, Math.round(y));
      o += 4;
    }
    return b;
  }
}
