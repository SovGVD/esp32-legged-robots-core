import assert from 'node:assert/strict';
import { test } from 'node:test';
import { zoneDirection, zonePoint } from '../src/tofgeom.ts';

const mount = { x: 0, y: 88, z: 0, yaw: 0, tilt: 0.7, fov: 0.785 };
const close = (a: number, b: number) => assert.ok(Math.abs(a - b) < 1e-6, `${a} != ${b}`);

test('zone rays are unit length and follow the firmware zone order', () => {
  for (let z = 0; z < 64; z++) close(Math.hypot(...zoneDirection(mount, z)), 1);
  const topLeft = zoneDirection(mount, 0);
  const bottomRight = zoneDirection(mount, 63);
  assert.ok(topLeft[0] < 0 && bottomRight[0] > 0, 'col 0 looks left');
  assert.ok(topLeft[2] > bottomRight[2], 'row 0 looks highest');
});

test('tilt turns the view down, yaw turns it left', () => {
  // rays 27/28/35/36 surround the axis; their mean points along the tilted axis
  const mean = [27, 28, 35, 36].map((z) => zoneDirection(mount, z)).reduce((a, d) => [a[0] + d[0], a[1] + d[1], a[2] + d[2]]);
  const len = Math.hypot(...mean);
  close(mean[0] / len, 0);
  close(mean[1] / len, Math.cos(0.7));
  close(mean[2] / len, -Math.sin(0.7));
  const left = zoneDirection({ ...mount, tilt: 0, yaw: 0.5 }, 27);
  assert.ok(left[0] < 0);
});

test('zone point: sensor position plus the ray, null without a target', () => {
  assert.equal(zonePoint(mount, 10, 0), null);
  const p = zonePoint(mount, 10, 200)!;
  const d = zoneDirection(mount, 10);
  close(p[1], 88 + d[1] * 200);
});
