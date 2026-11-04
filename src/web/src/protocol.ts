/**
 * Binary WebSocket protocol shared with the firmware
 * (src/esp32-legged-robot/packages*.{ino,cpp} and def/def.h).
 *
 * Every package starts with a one byte ASCII id. Multi-byte numbers are
 * big-endian.
 *
 * Client -> robot
 *   'M' move       [0x4D, enabled, x, y, z, pitch, roll, yaw]  14 bytes
 *                  enabled: 1 = drive, 0 = failsafe (robot stops)
 *                  each axis: uint16 = round((value + 1) * 10000), value in [-1, 1]
 *   'T' telemetry  [0x54, 1] request, answered with a telemetry package
 *   'C' cli        [0x43, ...utf8 command, '\n', 0]
 *   'G' gait       [0x47, walk]  1 = statically stable walk, 0 = default gait (trot);
 *                  applied once the robot stands still
 *   'L' level      [0x4C, on]    IMU leveling on/off (ignored without an IMU)
 *   'D' tof        [0x44] request, answered with a ToF package (ToF builds only)
 *   'V' / 'v'      start / stop camera stream (camera builds only)
 *   'P' map        [0x50] request, answered with a map package (lidar builds only)
 *   'N' nav        [0x4E] request, answered with a nav package (lidar builds only)
 *   'W' goto       [0x57, x:i16, y:i16] walk to a map point (mm); [0x57] alone stops
 *
 * Robot -> client
 *   text "Ok"      sent once after connecting
 *   'T' telemetry  [0x54, version:u8, statuses:u16, voltage*1000:u16, current*1000:i16, loopTime(us):u16,
 *                  version 2 adds: pitch*100:i16, roll*100:i16 (deg), missedTicks:u16,
 *                  ikReach:u16, ikJointLimit:u16, ikServoRange:u16]  22 bytes
 *                  version 3 adds: heading*10:u16, headingTarget*10:u16 (deg, compass, 0xFFFF none)  26 bytes
 *   'D' tof        [0x44, count:u8, per sensor: running:u8, frame:u8, 64 x mm:u16]  2 + 130 * count bytes
 *                  zones row-major, row 0 = top of the sensor's view, col 0 = left; 0 mm = no target
 *   'P' map        [0x50, running:u8, n:u16, cell(mm):u16, n*n/4 bytes] 2 bits per cell, first cell
 *                  in the low bits, rows from map y = min up: 0 unknown, 1 free, 2 seen once, 3 occupied
 *   'N' nav        [0x4E, flags:u8, state:u8, match:u8, x:i16, y:i16, heading*100:i16, goalX:i16, goalY:i16,
 *                  scans:u16, p:u8, p x (x:i16, y:i16), s:u16, s x (x:i16, y:i16)]
 *                  mm and deg (counter-clockwise, 0 = facing +y) in the map frame; scan points in the
 *                  robot frame (x right, y forward). flags: 1 lidar data, 2 frozen, 4 blocked, 8 no memory
 *   'C' cli        [0x43, ...utf8 output]
 */

export const P_MOVE = 0x4d; // 'M'
export const P_TELEMETRY = 0x54; // 'T'
export const P_CLI = 0x43; // 'C'
export const P_GAIT = 0x47; // 'G'
export const P_LEVEL = 0x4c; // 'L'
export const P_TOF = 0x44; // 'D'
export const P_MAP = 0x50; // 'P'
export const P_NAV = 0x4e; // 'N'
export const P_GOTO = 0x57; // 'W'
export const P_START_VIDEO = 0x56; // 'V'
export const P_STOP_VIDEO = 0x76; // 'v'

export const MOVE_LEN = 14;
/** Firmware CLI input buffer is 254 bytes; leave room for '\n' and 0. */
export const CLI_MAX_BYTES = 250;

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface Rotation {
  pitch: number;
  roll: number;
  yaw: number;
}

export interface MoveVector {
  move: Vec3;
  rotate: Rotation;
}

/** Telemetry status bits (TS_* in the firmware's def.h). */
export const TS = {
  hal: 0x0001,
  failsafe: 0x0002,
  gaitActive: 0x0004,
  gaitStatic: 0x0008,
  leveling: 0x0010,
  imu: 0x0020,
  power: 0x0040,
  sitting: 0x0200,
  overrun: 0x0400,
  terrain: 0x0800,
  gapBlocked: 0x1000,
  headingHold: 0x2000,
  headingAbsolute: 0x4000,
  tof: 0x8000,
} as const;
const TS_BATTERY_SHIFT = 7;

export type BatteryState = 'unknown' | 'ok' | 'low' | 'critical';
const batteryStates: BatteryState[] = ['unknown', 'ok', 'low', 'critical'];

export function batteryState(statuses: number): BatteryState {
  return batteryStates[(statuses >> TS_BATTERY_SHIFT) & 3];
}

export interface Telemetry {
  statuses: number;
  voltage: number;
  current: number;
  loopTime: number;
  /** version 2 firmware only */
  pitch?: number;
  roll?: number;
  missedTicks?: number;
  ik?: { reach: number; jointLimit: number; servoRange: number };
  /** version 3 firmware only: compass heading and hold target, deg; null = none */
  heading?: number | null;
  headingTarget?: number | null;
}

export const TOF_ZONES = 64;

export interface TofSensorFrame {
  /** false while the sensor starts up or when it was not found */
  running: boolean;
  /** low byte of the frame counter: unchanged = no new frame */
  frame: number;
  /** mm per zone, row-major from the top-left of the view; 0 = no target */
  mm: Uint16Array;
}

export interface LidarMap {
  running: boolean;
  /** cells per side */
  n: number;
  /** mm */
  cell: number;
  /** n*n codes, row 0 = lowest map y: 0 unknown, 1 free, 2 seen once, 3 occupied */
  cells: Uint8Array;
}

export const NAV_STATES = ['idle', 'walking', 'arrived', 'no path', 'no memory'] as const;

export interface NavInfo {
  flags: number;
  state: number;
  match: number;
  x: number;
  y: number;
  /** rad, counter-clockwise, 0 = facing +y */
  heading: number;
  goal: [number, number];
  scans: number;
  path: [number, number][];
  /** robot frame, mm */
  scan: [number, number][];
}

export type Incoming =
  | { type: 'telemetry'; telemetry: Telemetry }
  | { type: 'map'; map: LidarMap }
  | { type: 'nav'; nav: NavInfo }
  | { type: 'tof'; sensors: TofSensorFrame[] }
  | { type: 'cli'; text: string }
  | { type: 'unknown'; id: number };

export const zeroVector = (): MoveVector => ({
  move: { x: 0, y: 0, z: 0 },
  rotate: { pitch: 0, roll: 0, yaw: 0 },
});

/** Map [-1, 1] to the firmware's uint16 encoding. NaN counts as 0. */
export function encodeAxis(value: number): number {
  const v = Number.isFinite(value) ? Math.min(1, Math.max(-1, value)) : 0;
  return Math.round((v + 1) * 10000);
}

/** Inverse of encodeAxis, same formula as the firmware's pMove(). */
export function decodeAxis(raw: number): number {
  return raw / 10000 - 1;
}

export function encodeMove(v: MoveVector, enabled = true): Uint8Array {
  const buf = new Uint8Array(MOVE_LEN);
  const view = new DataView(buf.buffer);
  buf[0] = P_MOVE;
  buf[1] = enabled ? 1 : 0;
  const axes = [v.move.x, v.move.y, v.move.z, v.rotate.pitch, v.rotate.roll, v.rotate.yaw];
  axes.forEach((a, i) => view.setUint16(2 + i * 2, encodeAxis(a)));
  return buf;
}

export function encodeTelemetryRequest(): Uint8Array {
  return Uint8Array.of(P_TELEMETRY, 1);
}

export function encodeGait(walk: boolean): Uint8Array {
  return Uint8Array.of(P_GAIT, walk ? 1 : 0);
}

export function encodeLevel(on: boolean): Uint8Array {
  return Uint8Array.of(P_LEVEL, on ? 1 : 0);
}

export function encodeTofRequest(): Uint8Array {
  return Uint8Array.of(P_TOF);
}

export function encodeMapRequest(): Uint8Array {
  return Uint8Array.of(P_MAP);
}

export function encodeNavRequest(): Uint8Array {
  return Uint8Array.of(P_NAV);
}

/** Walk to map point (x, y) mm; no point: stop. */
export function encodeGoto(x?: number, y?: number): Uint8Array {
  if (x === undefined || y === undefined) return Uint8Array.of(P_GOTO);
  const b = new Uint8Array(5);
  const v = new DataView(b.buffer);
  const c = (a: number) => Math.max(-32768, Math.min(32767, Math.round(a)));
  b[0] = P_GOTO;
  v.setInt16(1, c(x));
  v.setInt16(3, c(y));
  return b;
}

export function encodeVideo(start: boolean): Uint8Array {
  return Uint8Array.of(start ? P_START_VIDEO : P_STOP_VIDEO);
}

const encoder = new TextEncoder();
const decoder = new TextDecoder();

/**
 * Encode a CLI command. Commands longer than CLI_MAX_BYTES are cut (on a
 * character boundary) so they never overflow the firmware input buffer.
 */
export function encodeCli(command: string): Uint8Array {
  let bytes = encoder.encode(command.replace(/[\r\n\0]/g, ''));
  if (bytes.length > CLI_MAX_BYTES) {
    let end = CLI_MAX_BYTES;
    while (end > 0 && (bytes[end] & 0xc0) === 0x80) end--; // don't split a UTF-8 sequence
    bytes = bytes.subarray(0, end);
  }
  const buf = new Uint8Array(bytes.length + 3);
  buf[0] = P_CLI;
  buf.set(bytes, 1);
  buf[bytes.length + 1] = 10; // new line
  buf[bytes.length + 2] = 0; // end of package
  return buf;
}

export function decodeTelemetry(data: Uint8Array): Telemetry | null {
  if (data.length < 10) return null;
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const t: Telemetry = {
    statuses: view.getUint16(2),
    voltage: view.getUint16(4) / 1000,
    current: view.getInt16(6) / 1000,
    loopTime: view.getUint16(8),
  };
  if (data[1] >= 2 && data.length >= 22) {
    t.pitch = view.getInt16(10) / 100;
    t.roll = view.getInt16(12) / 100;
    t.missedTicks = view.getUint16(14);
    t.ik = { reach: view.getUint16(16), jointLimit: view.getUint16(18), servoRange: view.getUint16(20) };
  }
  if (data[1] >= 3 && data.length >= 26) {
    const deg = (raw: number) => (raw === 0xffff ? null : raw / 10);
    t.heading = deg(view.getUint16(22));
    t.headingTarget = deg(view.getUint16(24));
  }
  return t;
}

export function decodeTof(data: Uint8Array): TofSensorFrame[] | null {
  if (data.length < 2) return null;
  const count = data[1];
  const size = 2 + TOF_ZONES * 2;
  if (data.length < 2 + count * size) return null;
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const sensors: TofSensorFrame[] = [];
  for (let i = 0; i < count; i++) {
    const o = 2 + i * size;
    const mm = new Uint16Array(TOF_ZONES);
    for (let z = 0; z < TOF_ZONES; z++) mm[z] = view.getUint16(o + 2 + z * 2);
    sensors.push({ running: data[o] === 1, frame: data[o + 1], mm });
  }
  return sensors;
}

export function decodeMap(data: Uint8Array): LidarMap | null {
  if (data.length < 2) return null;
  if (data[1] !== 1) return { running: false, n: 0, cell: 0, cells: new Uint8Array(0) };
  if (data.length < 6) return null;
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const n = view.getUint16(2);
  if (data.length < 6 + (n * n) / 4) return null;
  const cells = new Uint8Array(n * n);
  for (let i = 0; i < n * n; i++) cells[i] = (data[6 + (i >> 2)] >> ((i & 3) * 2)) & 3;
  return { running: true, n, cell: view.getUint16(4), cells };
}

export function decodeNav(data: Uint8Array): NavInfo | null {
  if (data.length < 19) return null;
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  let o = 17;
  const points = (count: number, size: number): [number, number][] | null => {
    if (data.length < o + count * 4 + size) return null;
    const out: [number, number][] = [];
    for (let i = 0; i < count; i++, o += 4) out.push([v.getInt16(o), v.getInt16(o + 2)]);
    return out;
  };
  const path = points(data[16], 2);
  if (!path) return null;
  const count = v.getUint16(o);
  o += 2;
  const scan = points(count, 0);
  if (!scan) return null;
  return {
    flags: data[1],
    state: data[2],
    match: data[3],
    x: v.getInt16(4),
    y: v.getInt16(6),
    heading: (v.getInt16(8) / 100) * (Math.PI / 180),
    goal: [v.getInt16(10), v.getInt16(12)],
    scans: v.getUint16(14),
    path,
    scan,
  };
}

export function decode(data: Uint8Array): Incoming | null {
  if (data.length === 0) return null;
  switch (data[0]) {
    case P_TELEMETRY: {
      const telemetry = decodeTelemetry(data);
      return telemetry ? { type: 'telemetry', telemetry } : null;
    }
    case P_TOF: {
      const sensors = decodeTof(data);
      return sensors ? { type: 'tof', sensors } : null;
    }
    case P_MAP: {
      const map = decodeMap(data);
      return map ? { type: 'map', map } : null;
    }
    case P_NAV: {
      const nav = decodeNav(data);
      return nav ? { type: 'nav', nav } : null;
    }
    case P_CLI:
      return { type: 'cli', text: decoder.decode(data.subarray(1), { stream: true }) };
    default:
      return { type: 'unknown', id: data[0] };
  }
}
