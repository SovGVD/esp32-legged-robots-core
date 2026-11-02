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
 *   'V' / 'v'      start / stop camera stream (camera builds only)
 *
 * Robot -> client
 *   text "Ok"      sent once after connecting
 *   'T' telemetry  [0x54, version:u8, statuses:u16, voltage*1000:u16, current*1000:i16, loopTime(us):u16,
 *                  version 2 adds: pitch*100:i16, roll*100:i16 (deg), missedTicks:u16,
 *                  ikReach:u16, ikJointLimit:u16, ikServoRange:u16]  22 bytes
 *   'C' cli        [0x43, ...utf8 output]
 */

export const P_MOVE = 0x4d; // 'M'
export const P_TELEMETRY = 0x54; // 'T'
export const P_CLI = 0x43; // 'C'
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
}

export type Incoming =
  | { type: 'telemetry'; telemetry: Telemetry }
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
  return t;
}

export function decode(data: Uint8Array): Incoming | null {
  if (data.length === 0) return null;
  switch (data[0]) {
    case P_TELEMETRY: {
      const telemetry = decodeTelemetry(data);
      return telemetry ? { type: 'telemetry', telemetry } : null;
    }
    case P_CLI:
      return { type: 'cli', text: decoder.decode(data.subarray(1), { stream: true }) };
    default:
      return { type: 'unknown', id: data[0] };
  }
}
