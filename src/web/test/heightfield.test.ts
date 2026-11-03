import assert from 'node:assert/strict';
import { test } from 'node:test';
import { HF_COLS, HeightField } from '../src/heightfield.ts';

const at = (col: number, row: number, z: number): [number, number, number] => {
  const [x, y] = HeightField.cellCenter(col, row);
  return [x, y, z];
};

test('readings average over frames, an obstacle top replaces the floor at once', () => {
  const f = new HeightField();
  f.add([at(3, 4, -90)], 0);
  f.add([at(3, 4, -80)], 100);
  assert.equal(f.z[4 * HF_COLS + 3], -85);
  f.add([at(3, 4, -40)], 200);
  assert.equal(f.z[4 * HF_COLS + 3], -40);
});

test('cells nobody sees are forgotten, small holes are filled for drawing', () => {
  const f = new HeightField();
  f.add([at(1, 1, -90), at(3, 1, -90), at(2, 0, -90), at(2, 2, -70)], 0);
  assert.ok(Number.isNaN(f.z[1 * HF_COLS + 2]));
  assert.equal(f.filled()[1 * HF_COLS + 2], -85);
  f.add([], 2000);
  assert.ok(Number.isNaN(f.z[1 * HF_COLS + 1]));
});
