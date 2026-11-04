import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  CLI_MAX_BYTES,
  MOVE_LEN,
  P_CLI,
  P_MOVE,
  P_TELEMETRY,
  P_TOF,
  TS,
  batteryState,
  decode,
  encodeAxis,
  encodeCli,
  encodeMove,
  encodeTelemetryRequest,
  encodeTofRequest,
  zeroVector,
} from '../src/protocol.ts';

/** Same as the firmware's pMove(): float(byte1 << 8 | byte2) / 10000 - 1 */
const firmwareAxis = (b: Uint8Array, o: number) => ((b[o] << 8) | b[o + 1]) / 10000 - 1;

/** The previous client's encoder (s.js), for compatibility checks. */
function legacyMove(values: number[]): number[] {
  const out = [77, 1];
  for (const v of values) {
    const n = (v + 1) * 10000;
    out.push((n >> 8) & 255, n & 255);
  }
  return out;
}

test('move package layout matches the firmware decoder', () => {
  const v = zeroVector();
  v.move = { x: 0.5, y: -1, z: 0 };
  v.rotate = { pitch: 1, roll: -0.25, yaw: 0.1234 };
  const p = encodeMove(v);
  assert.equal(p.length, MOVE_LEN);
  assert.equal(p[0], P_MOVE);
  assert.equal(p[1], 1);
  const decoded = [2, 4, 6, 8, 10, 12].map((o) => firmwareAxis(p, o));
  const expected = [0.5, -1, 0, 1, -0.25, 0.1234];
  decoded.forEach((d, i) => assert.ok(Math.abs(d - expected[i]) < 1e-4, `axis ${i}: ${d}`));
});

test('move package is byte-identical to the previous client for exact values', () => {
  const values = [0, 0.5, -0.5, 1, -1, 0.25];
  const v = zeroVector();
  [v.move.x, v.move.y, v.move.z, v.rotate.pitch, v.rotate.roll, v.rotate.yaw] = values;
  assert.deepEqual([...encodeMove(v)], legacyMove(values));
});

test('disabled move package carries the failsafe flag', () => {
  assert.equal(encodeMove(zeroVector(), false)[1], 0);
});

test('axis values are clamped and NaN is treated as zero', () => {
  assert.equal(encodeAxis(5), 20000);
  assert.equal(encodeAxis(-5), 0);
  assert.equal(encodeAxis(Number.NaN), 10000);
});

test('telemetry request', () => {
  assert.deepEqual([...encodeTelemetryRequest()], [P_TELEMETRY, 1]);
});

test('cli package is id + utf8 + newline + terminator', () => {
  const p = encodeCli('servo 1 90');
  assert.equal(p[0], P_CLI);
  assert.equal(new TextDecoder().decode(p.subarray(1, -2)), 'servo 1 90');
  assert.deepEqual([...p.subarray(-2)], [10, 0]);
});

test('cli strips control characters that would end the command early', () => {
  const p = encodeCli('a\nb\0c');
  assert.equal(new TextDecoder().decode(p.subarray(1, -2)), 'abc');
});

test('long cli commands fit the firmware buffer and keep valid utf8', () => {
  const p = encodeCli('я'.repeat(200)); // 400 bytes
  const body = p.subarray(1, -2);
  assert.ok(body.length <= CLI_MAX_BYTES);
  assert.equal(new TextDecoder('utf-8', { fatal: true }).decode(body), 'я'.repeat(body.length / 2));
});

test('telemetry package decodes like the firmware encodes it', () => {
  const b = new Uint8Array(14);
  const view = new DataView(b.buffer);
  b[0] = P_TELEMETRY;
  view.setUint16(4, 7412);
  view.setInt16(6, -150);
  view.setUint16(8, 2345);
  const msg = decode(b);
  assert.deepEqual(msg, { type: 'telemetry', telemetry: { statuses: 0, voltage: 7.412, current: -0.15, loopTime: 2345 } });
});

test('version 2 telemetry adds tilt, missed ticks, IK counters and battery state', () => {
  const b = new Uint8Array(22);
  const view = new DataView(b.buffer);
  b[0] = P_TELEMETRY;
  b[1] = 2;
  view.setUint16(2, TS.hal | TS.gaitActive | TS.leveling | (2 << 7)); // battery low
  view.setUint16(4, 6712);
  view.setInt16(10, -125);
  view.setInt16(12, 340);
  view.setUint16(14, 3);
  view.setUint16(16, 1);
  view.setUint16(18, 2);
  view.setUint16(20, 65535);
  const msg = decode(b);
  assert.ok(msg && msg.type === 'telemetry');
  const t = msg.telemetry;
  assert.equal(t.pitch, -1.25);
  assert.equal(t.roll, 3.4);
  assert.equal(t.missedTicks, 3);
  assert.deepEqual(t.ik, { reach: 1, jointLimit: 2, servoRange: 65535 });
  assert.equal(batteryState(t.statuses), 'low');
  assert.ok(t.statuses & TS.leveling);
});

test('version 3 telemetry adds the heading and the hold target', () => {
  const b = new Uint8Array(26);
  const view = new DataView(b.buffer);
  b[0] = P_TELEMETRY;
  b[1] = 3;
  view.setUint16(2, TS.hal | TS.terrain | TS.gapBlocked | TS.headingHold | TS.headingAbsolute);
  view.setUint16(22, 2705);
  view.setUint16(24, 0xffff);
  const msg = decode(b);
  assert.ok(msg && msg.type === 'telemetry');
  assert.equal(msg.telemetry.heading, 270.5);
  assert.equal(msg.telemetry.headingTarget, null);
  assert.ok(msg.telemetry.statuses & TS.gapBlocked);
  // a version 2 package has no heading
  b[1] = 2;
  const v2 = decode(b);
  assert.ok(v2 && v2.type === 'telemetry');
  assert.equal(v2.telemetry.heading, undefined);
});

test('short telemetry package is ignored', () => {
  assert.equal(decode(Uint8Array.of(P_TELEMETRY, 0, 0)), null);
});

test('cli output is decoded as text', () => {
  const b = new Uint8Array([P_CLI, ...new TextEncoder().encode('ok\n')]);
  assert.deepEqual(decode(b), { type: 'cli', text: 'ok\n' });
});

test('unknown and empty packages', () => {
  assert.deepEqual(decode(Uint8Array.of(0x99)), { type: 'unknown', id: 0x99 });
  assert.equal(decode(new Uint8Array()), null);
});

test('gait and level packages match the firmware ids', async () => {
  const { encodeGait, encodeLevel } = await import('../src/protocol.ts');
  assert.deepEqual([...encodeGait(true)], [0x47, 1]);
  assert.deepEqual([...encodeGait(false)], [0x47, 0]);
  assert.deepEqual([...encodeLevel(true)], [0x4c, 1]);
  assert.deepEqual([...encodeLevel(false)], [0x4c, 0]);
});

test('tof package decodes like the firmware pTof() writes it', () => {
  const zones = 64;
  const count = 2;
  const buf = new Uint8Array(2 + count * (2 + zones * 2));
  buf[0] = P_TOF;
  buf[1] = count;
  const o1 = 2 + 2 + zones * 2;
  buf[2] = 1; // sensor 0 running
  buf[3] = 7; // frame
  buf[4] = 0x01; // zone 0 = 0x0123 = 291 mm
  buf[5] = 0x23;
  buf[o1] = 0; // sensor 1 not running
  const msg = decode(buf);
  assert.ok(msg && msg.type === 'tof');
  assert.equal(msg.sensors.length, 2);
  assert.equal(msg.sensors[0].running, true);
  assert.equal(msg.sensors[0].frame, 7);
  assert.equal(msg.sensors[0].mm[0], 291);
  assert.equal(msg.sensors[0].mm[1], 0);
  assert.equal(msg.sensors[1].running, false);
  assert.deepEqual([...encodeTofRequest()], [P_TOF]);
});

test('short tof package is rejected', () => {
  assert.equal(decode(Uint8Array.of(P_TOF, 1, 1, 0, 0)), null);
});

test('goto, map and nav packages', async () => {
  const { encodeGoto, encodeMapRequest, encodeNavRequest, P_GOTO, P_MAP, P_NAV } = await import('../src/protocol.ts');
  assert.deepEqual([...encodeGoto(-1500, 2300)], [P_GOTO, 0xfa, 0x24, 0x08, 0xfc]);
  assert.deepEqual([...encodeGoto()], [P_GOTO]);
  assert.deepEqual([...encodeMapRequest()], [P_MAP]);
  assert.deepEqual([...encodeNavRequest()], [P_NAV]);

  // map: 4 x 4 cells, codes 0..3 packed 4 per byte, first cell in the low bits
  const m = new Uint8Array(6 + 4);
  m.set([P_MAP, 1, 0, 4, 0, 50]);
  m[6] = 0b11100100; // cells 0..3 = 0, 1, 2, 3
  const map = decode(m);
  assert.equal(map?.type, 'map');
  if (map?.type === 'map') {
    assert.equal(map.map.n, 4);
    assert.equal(map.map.cell, 50);
    assert.deepEqual([...map.map.cells.subarray(0, 4)], [0, 1, 2, 3]);
  }
  assert.deepEqual(decode(Uint8Array.of(P_MAP, 0)), { type: 'map', map: { running: false, n: 0, cell: 0, cells: new Uint8Array(0) } });

  // nav: pose, goal, one path point, two scan points
  const n = new Uint8Array(19 + 4 + 8);
  const v = new DataView(n.buffer);
  n.set([P_NAV, 5, 1, 1]);
  v.setInt16(4, -120);
  v.setInt16(6, 3400);
  v.setInt16(8, 9000);
  v.setInt16(10, 500);
  v.setInt16(12, -700);
  v.setUint16(14, 42);
  n[16] = 1;
  v.setInt16(17, 10);
  v.setInt16(19, 20);
  v.setUint16(21, 2);
  v.setInt16(23, 100);
  v.setInt16(25, -200);
  v.setInt16(27, 300);
  v.setInt16(29, 400);
  const nav = decode(n);
  assert.equal(nav?.type, 'nav');
  if (nav?.type === 'nav') {
    assert.equal(nav.nav.x, -120);
    assert.equal(nav.nav.y, 3400);
    assert.ok(Math.abs(nav.nav.heading - Math.PI / 2) < 1e-6);
    assert.deepEqual(nav.nav.goal, [500, -700]);
    assert.equal(nav.nav.scans, 42);
    assert.deepEqual(nav.nav.path, [[10, 20]]);
    assert.deepEqual(nav.nav.scan, [[100, -200], [300, 400]]);
  }
  assert.equal(decode(n.subarray(0, 25)), null); // cut short
});
