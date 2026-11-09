import assert from 'node:assert/strict';
import { test } from 'node:test';
import { MockRobot } from '../mock/sim.ts';
import { correct, dialAngle, minMaxFit, roundness, type P2 } from '../src/compass.ts';
import { P_SETUP, SH, SO, decode, encodeSetupRequest } from '../src/protocol.ts';

/** A setup package laid out like the firmware's pSetup() (packages.cpp). */
function firmwareSetup(): Uint8Array {
  const legs = ['LF', 'RF', 'LH', 'RH'];
  const b = new Uint8Array(34 + legs.length * (1 + 2 + 3 * 2));
  const v = new DataView(b.buffer);
  b.set([P_SETUP, 1, SH.terrain | SH.mag | SH.imu | SH.hal | SH.raw, SO.terrain | SO.hold | SO.magTrusted, 1, 90, 12, 0b01]);
  v.setUint16(8, 0b0101);
  [123, -456, -400].forEach((x, i) => v.setInt16(10 + i * 2, x));
  [140, -90, 0].forEach((x, i) => v.setInt16(16 + i * 2, x));
  [1180, 860, 1000].forEach((x, i) => v.setUint16(22 + i * 2, x));
  v.setUint16(28, 455);
  v.setUint16(30, 448);
  b[32] = legs.length;
  b[33] = 3;
  let o = 34;
  legs.forEach((l, i) => {
    b[o++] = 2;
    b[o++] = l.charCodeAt(0);
    b[o++] = l.charCodeAt(1);
    [i * 100, -150, 5].forEach((t) => {
      v.setInt16(o, t);
      o += 2;
    });
  });
  return b;
}

test('setup request is one byte', () => {
  assert.deepEqual([...encodeSetupRequest()], [0x53]);
});

test('decodes the firmware setup package', () => {
  const m = decode(firmwareSetup());
  assert.equal(m?.type, 'setup');
  if (m?.type !== 'setup') return;
  const s = m.setup;
  assert.ok(s.has & SH.terrain && !(s.has & SH.feet));
  assert.ok(s.on & SO.hold);
  assert.deepEqual(s.cal, { state: 1, turned: 360, time: 12 });
  assert.equal(s.feet, 0b0101);
  assert.deepEqual(s.mag.raw, [12.3, -45.6, -40]);
  assert.deepEqual(s.mag.offset, [14, -9, 0]);
  assert.deepEqual(s.mag.scale, [1.18, 0.86, 1]);
  assert.equal(s.mag.calField, 45.5);
  assert.equal(s.dof, 3);
  assert.deepEqual(s.legs.map((l) => l.name), ['LF', 'RF', 'LH', 'RH']);
  assert.deepEqual(s.legs[2].trim, [2, -1.5, 0.05]);
});

test('no raw magnetometer reading decodes as null; a cut package is rejected', () => {
  const b = firmwareSetup();
  b[2] &= ~SH.raw;
  const m = decode(b);
  assert.ok(m?.type === 'setup' && m.setup.mag.raw === null);
  assert.equal(decode(b.subarray(0, b.length - 1)), null);
});

test('mock robot: trims, switches and the compass calibration go through the setup package', () => {
  const robot = new MockRobot({ tofCount: 2, tofNav: true });
  const cli = (c: string, t: number) => robot.receive(new Uint8Array([0x43, ...new TextEncoder().encode(c), 10, 0]), t);
  const setup = (t: number) => {
    const m = decode(robot.receive(encodeSetupRequest(), t).at(-1)!);
    assert.equal(m?.type, 'setup');
    return m!.type === 'setup' ? m!.setup : null!;
  };
  let t = 1000;
  cli('trim RM gamma -3.25', t);
  cli('heading on', t);
  let s = setup(t);
  assert.deepEqual(s.legs[4], { name: 'RM', trim: [0, 0, -3.25] });
  assert.ok(s.on & SO.hold && s.has & SH.feet && s.mag.calField === 0);
  // the calibration turn needs a client: keep sending (still) move packages
  const still = new Uint8Array([0x4d, 1, 39, 16, 39, 16, 39, 16, 39, 16, 39, 16, 39, 16]);
  cli('heading cal', t);
  for (let i = 0; i < 400 && setup(t).cal.state === 1; i++) {
    t += 100;
    robot.receive(still, t);
  }
  s = setup(t);
  assert.equal(s.cal.state, 3);
  assert.ok(Math.abs(s.mag.offset[0] - 14) < 1.5 && Math.abs(s.mag.offset[1] + 9) < 1.5, `offset ${s.mag.offset}`);
  assert.ok(Math.abs(s.mag.scale[0] / s.mag.scale[1] - 1.18 / 0.86) < 0.1, `scale ${s.mag.scale}`);
});

test('compass fit math matches the firmware MagCalibrator', () => {
  // an off-centre ellipse: 30 x 20 uT round (10, -5)
  const pts: P2[] = Array.from({ length: 72 }, (_, i) => [10 + 30 * Math.cos(i / 11.46), -5 + 20 * Math.sin(i / 11.46)]);
  const f = minMaxFit(pts)!;
  assert.ok(Math.abs(f.cx - 10) < 0.2 && Math.abs(f.cy + 5) < 0.2);
  const r = (f.rx + f.ry) / 2;
  const fixed = pts.map((p) => correct(p, [f.cx, f.cy, 0], [r / f.rx, r / f.ry, 1]));
  const rd = roundness(fixed)!;
  assert.ok(Math.abs(rd.r - 25) < 0.5 && rd.spread < 0.01);
  assert.ok(roundness(pts)!.spread > 0.1);
  assert.equal(minMaxFit(pts.slice(0, 5)), null);
  assert.equal(Math.round(dialAngle(1, 0)), 90);
  assert.equal(Math.round(dialAngle(0, 1)), 180);
  assert.equal(Math.round(dialAngle(-1, 0)), 270);
});
