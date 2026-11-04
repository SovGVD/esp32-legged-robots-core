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
  P_TELEMETRY,
  P_TOF,
  TOF_ZONES,
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
  }

  /** Answer one package from the page; returns the reply packages. */
  receive(raw: Uint8Array, now = Date.now()): Uint8Array[] {
    this.tick(now);
    if (raw.length === 0) return [];
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
    const [cmd, arg] = command.split(/\s+/);
    switch (cmd) {
      case 'help':
        return 'Mock robot commands:\n  state         pose, battery, gait\n  battery [V]   set the battery voltage (default full)\n  home          back to the start\n  ping\n';
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
      default:
        return `Unknown command: ${command} (try help)\n`;
    }
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
    if (this.sitting) sx = sy = yaw = 0;
    const speed = this.walk ? 0.4 : 1;

    const fwd = sy * SPEED * speed;
    const side = sx * SIDE_SPEED * speed;
    // stop in front of a gap, like GaitEngine::isBlocked()
    const c = Math.cos(this.heading);
    const s = Math.sin(this.heading);
    const ahead = (bx: number, by: number) => [this.x + bx * c - by * s, this.y + bx * s + by * c];
    const [gx, gy] = ahead(Math.sign(side) * 60, Math.sign(fwd) * 200);
    this.gapBlocked = (fwd !== 0 || side !== 0) && heightAt(gx, gy) < FLOOR - 60;
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
    this.heading -= yaw * YAW_RATE * speed * dt;
    this.heading = Math.atan2(Math.sin(this.heading), Math.cos(this.heading));

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
    let st = TS.hal | TS.imu | TS.power | TS.headingAbsolute | (battery << 7);
    if (this.stepping) st |= TS.gaitActive;
    if (this.walk) st |= TS.gaitStatic;
    if (this.level) st |= TS.leveling;
    if (this.sitting) st |= TS.sitting;
    if (this.opts.tofCount) st |= TS.terrain | TS.tof;
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
    const compass = (((-this.heading * 180) / Math.PI) % 360 + 360) % 360;
    v.setUint16(22, Math.round(compass * 10) % 3600);
    v.setUint16(24, 0xffff);
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
