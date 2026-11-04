/**
 * Fake robot for `npm run dev`: speaks the firmware's websocket protocol on
 * /ws of the Vite dev server, so the UI can be developed without an ESP32.
 */
import type { Server as HttpServer } from 'node:http';
import { WebSocketServer, type WebSocket } from 'ws';
import type { TofMount } from '../src/config.ts';
import { zoneDirection } from '../src/tofgeom.ts';
import { P_CLI, P_GAIT, P_GOTO, P_LEVEL, P_MAP, P_MOVE, P_NAV, P_TELEMETRY, P_TOF, TOF_ZONES, TS, decodeAxis } from '../src/protocol.ts';

const FAILSAFE_MS = 500;

function cliPackage(text: string): Buffer {
  return Buffer.concat([Buffer.of(P_CLI), Buffer.from(text, 'utf8')]);
}

function telemetryPackage(voltage: number, current: number, loopTime: number, extra = 0): Buffer {
  const b = Buffer.alloc(26);
  b[0] = P_TELEMETRY;
  b[1] = 3; // version
  b.writeUInt16BE(TS.hal | TS.imu | TS.power | (1 << 7) | extra, 2); // battery ok
  b.writeUInt16BE(Math.round(voltage * 1000), 4);
  b.writeInt16BE(Math.round(current * 1000), 6);
  b.writeUInt16BE(loopTime, 8);
  b.writeInt16BE(Math.round((Math.random() - 0.5) * 200), 10);
  b.writeInt16BE(Math.round((Math.random() - 0.5) * 200), 12);
  b.writeUInt16BE(Math.round(Math.random() * 3599), 22); // heading
  b.writeUInt16BE(0xffff, 24); // no hold target
  return b;
}

/** ToF mounting, same as the dev server's /c.js (model_data.h TOF_SENSORS). */
export function mockTofMounts(count: number): TofMount[] {
  const m = (x: number, y: number, z: number, yaw: number, tilt: number) => ({ x, y, z, yaw, tilt, fov: 0.785 });
  return count === 1 ? [m(0, 88, 0, 0, 0.7)] : [m(-25, 90, 10, 0.35, 0.61), m(25, 90, 10, -0.35, 0.61)];
}

/** Mock ground: floor 95 mm below the body, a 30 mm box sweeping left and right, a gap further out. */
function groundAt(x: number, y: number, t: number): number {
  const boxX = Math.sin(t * 0.8) * 90;
  if (Math.abs(x - boxX) < 35 && y > 170 && y < 230) return -65;
  if (y > 290 && y < 340) return -400;
  return -95;
}

/** Fake ToF frames: zone rays marched against the mock ground, like the sim's legsim/tof.py. */
export function tofPackage(count: number, frame: number, t = Date.now() / 1000, range = 450): Buffer {
  const mounts = mockTofMounts(count);
  const b = Buffer.alloc(2 + count * (2 + TOF_ZONES * 2));
  b[0] = P_TOF;
  b[1] = count;
  for (let i = 0; i < count; i++) {
    const o = 2 + i * (2 + TOF_ZONES * 2);
    b[o] = 1; // running
    b[o + 1] = frame & 0xff;
    for (let z = 0; z < TOF_ZONES; z++) {
      const m = mounts[i];
      const d = zoneDirection(m, z);
      let mm = 0;
      for (let r = 20; r <= range; r += 3) {
        if (m.z + d[2] * r <= groundAt(m.x + d[0] * r, m.y + d[1] * r, t)) {
          mm = r + (Math.random() - 0.5) * 6;
          break;
        }
      }
      b.writeUInt16BE(Math.max(0, Math.round(mm)), o + 2 + z * 2);
    }
  }
  return b;
}

/** Mock flat for the lidar view: two rooms (5 cm cells, 200 x 200), walls as segments in mm. */
const WALLS: [number, number, number, number][] = [
  [-1600, -900, 1800, -900], [-1600, 4800, 1800, 4800], [-1600, -900, -1600, 4800], [1800, -900, 1800, 4800],
  [-1600, 2200, 600, 2200], [1400, 2200, 1800, 2200], [-1450, 0, -850, 0], [-850, 0, -850, 1400],
];
const MAP_N = 200;
const MAP_CELL = 50;

export function mapPackage(): Buffer {
  const b = Buffer.alloc(6 + (MAP_N * MAP_N) / 4);
  b[0] = P_MAP;
  b[1] = 1;
  b.writeUInt16BE(MAP_N, 2);
  b.writeUInt16BE(MAP_CELL, 4);
  const set = (x: number, y: number, code: number) => {
    const ix = Math.floor(x / MAP_CELL) + MAP_N / 2;
    const iy = Math.floor(y / MAP_CELL) + MAP_N / 2;
    if (ix < 0 || iy < 0 || ix >= MAP_N || iy >= MAP_N) return;
    const i = iy * MAP_N + ix;
    b[6 + (i >> 2)] = (b[6 + (i >> 2)] & ~(3 << ((i & 3) * 2))) | (code << ((i & 3) * 2));
  };
  for (let x = -1600; x < 1800; x += MAP_CELL) for (let y = -900; y < 4800; y += MAP_CELL) set(x, y, 1);
  for (const [x0, y0, x1, y1] of WALLS) {
    const n = Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 25);
    for (let k = 0; k <= n; k++) set(x0 + ((x1 - x0) * k) / n, y0 + ((y1 - y0) * k) / n, 3);
  }
  return b;
}

/** Robot walking straight toward the goal (no planning), with a fake scan of the nearest walls. */
export class MockNav {
  scanUntil = 0;
  x = 0;
  y = 0;
  heading = 0;
  goal: [number, number] | null = null;
  state = 0;
  private last = Date.now();

  step(): void {
    const now = Date.now();
    const dt = (now - this.last) / 1000;
    this.last = now;
    if (!this.goal || this.state !== 1) return;
    const dx = this.goal[0] - this.x;
    const dy = this.goal[1] - this.y;
    const d = Math.hypot(dx, dy);
    if (d < 100) {
      this.state = 2;
      return;
    }
    this.heading = Math.atan2(-dx, dy);
    const v = Math.min(d, 200 * dt);
    this.x += (dx / d) * v;
    this.y += (dy / d) * v;
  }

  package(): Buffer {
    this.step();
    const scan: [number, number][] = [];
    for (let a = 0; a < 360; a += 4) {
      const r = (a * Math.PI) / 180;
      const [dx, dy] = [-Math.sin(r), Math.cos(r)];
      let best = Infinity;
      for (const [x0, y0, x1, y1] of WALLS) {
        const ex = x1 - x0;
        const ey = y1 - y0;
        const den = dx * ey - dy * ex;
        if (Math.abs(den) < 1e-9) continue;
        const t = ((x0 - this.x) * ey - (y0 - this.y) * ex) / den;
        const u = ((x0 - this.x) * dy - (y0 - this.y) * dx) / den;
        if (t > 0 && u >= 0 && u <= 1) best = Math.min(best, t);
      }
      if (best > 8000) continue;
      // robot frame
      const c = Math.cos(-this.heading);
      const s = Math.sin(-this.heading);
      scan.push([best * (dx * c - dy * s), best * (dx * s + dy * c)]);
    }
    const path = this.goal && this.state === 1 ? [[this.x, this.y], this.goal] : [];
    const b = Buffer.alloc(19 + path.length * 4 + scan.length * 4);
    b[0] = P_NAV;
    // flags: data; TOF_NAV=1 env: ToF room scans (Scan button), scanning for 3 s after one
    b[1] = 1 | (process.env.TOF_NAV ? 32 : 0) | (Date.now() < this.scanUntil ? 16 : 0);
    b[2] = this.state;
    b[3] = 1;
    b.writeInt16BE(Math.round(this.x), 4);
    b.writeInt16BE(Math.round(this.y), 6);
    b.writeInt16BE(Math.round((this.heading * 18000) / Math.PI), 8);
    b.writeInt16BE(Math.round(this.goal?.[0] ?? 0), 10);
    b.writeInt16BE(Math.round(this.goal?.[1] ?? 0), 12);
    b[16] = path.length;
    let o = 17;
    for (const [x, y] of path) {
      b.writeInt16BE(Math.round(x), o);
      b.writeInt16BE(Math.round(y), o + 2);
      o += 4;
    }
    b.writeUInt16BE(scan.length, o);
    o += 2;
    for (const [x, y] of scan) {
      b.writeInt16BE(Math.round(x), o);
      b.writeInt16BE(Math.round(y), o + 2);
      o += 4;
    }
    return b;
  }
}

export function attachMockRobot(httpServer: HttpServer, log = console.log): void {
  const wss = new WebSocketServer({ noServer: true });
  let controller: WebSocket | null = null;

  httpServer.on('upgrade', (req, socket, head) => {
    if (new URL(req.url ?? '/', 'http://x').pathname !== '/ws') return; // leave Vite's HMR socket alone
    wss.handleUpgrade(req, socket, head, (ws) => wss.emit('connection', ws));
  });

  wss.on('connection', (ws) => {
    // like the firmware: the newest client takes over
    controller?.close(1000, 'Another client took control');
    controller = ws;
    log('[mock robot] client connected');
    ws.send('Ok');
    ws.send(cliPackage('Mock robot ready. Try "help".\n'));

    let lastMove = 0;
    let walk = false;
    let level = false;
    let moving = false;
    const nav = new MockNav();
    const watchdog = setInterval(() => {
      if (moving && Date.now() - lastMove > FAILSAFE_MS) {
        moving = false;
        log('[mock robot] failsafe: no move packages');
      }
    }, 100);

    ws.on('message', (raw: Buffer, isBinary: boolean) => {
      if (!isBinary || raw.length === 0) return;
      switch (raw[0]) {
        case P_MOVE: {
          if (raw.length < 14) return log('[mock robot] short move package', raw.length);
          lastMove = Date.now();
          const axes = [2, 4, 6, 8, 10, 12].map((o) => decodeAxis(raw.readUInt16BE(o)).toFixed(2));
          const active = raw[1] === 1 && axes.some((a) => Number(a) !== 0);
          if (raw[1] === 0) log('[mock robot] client sent failsafe');
          else if (active) log(`[mock robot] move x=${axes[0]} y=${axes[1]} z=${axes[2]} pitch=${axes[3]} roll=${axes[4]} yaw=${axes[5]}`);
          moving = raw[1] === 1;
          break;
        }
        case P_TELEMETRY:
          ws.send(
            telemetryPackage(
              7.4 + Math.random() * 0.1,
              1.2 + Math.random() * 0.3,
              2000 + Math.round(Math.random() * 300),
              (walk ? TS.gaitStatic : 0) | (level ? TS.leveling : 0),
            ),
          );
          break;
        case P_TOF:
          ws.send(tofPackage(Number(process.env.TOF ?? 2), Math.floor(Date.now() / 66)));
          break;
        case P_MAP:
          ws.send(mapPackage());
          break;
        case P_NAV:
          ws.send(nav.package());
          break;
        case P_GOTO:
          if (raw.length === 2 && raw[1] === 0x53) {
            nav.scanUntil = Date.now() + 3000;
            log('[mock robot] scan');
          } else if (raw.length >= 5) {
            nav.goal = [raw.readInt16BE(1), raw.readInt16BE(3)];
            nav.state = 1;
            log(`[mock robot] goto ${nav.goal[0]} ${nav.goal[1]} mm`);
          } else {
            nav.state = 0;
            log('[mock robot] goto stop');
          }
          break;
        case P_GAIT:
          walk = raw[1] === 1;
          log(`[mock robot] gait ${walk ? 'walk' : 'trot'}`);
          break;
        case P_LEVEL:
          level = raw[1] === 1;
          log(`[mock robot] leveling ${level ? 'on' : 'off'}`);
          break;
        case P_CLI: {
          const end = raw.indexOf(0);
          const command = raw.subarray(1, end < 0 ? raw.length : end).toString('utf8').trim();
          const reply = command === 'help' ? 'Commands: help, ping\n' : command === 'ping' ? 'pong\n' : `Unknown command: ${command}\n`;
          ws.send(cliPackage(`> ${command}\n${reply}`));
          break;
        }
        default:
          log('[mock robot] unknown package', raw[0]);
      }
    });
    ws.on('close', () => {
      clearInterval(watchdog);
      if (controller === ws) controller = null;
      log('[mock robot] client disconnected');
    });
  });
}
