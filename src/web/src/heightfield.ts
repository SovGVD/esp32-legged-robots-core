import type { Vec } from './tofgeom.ts';

export const HF_CELL = 25; // mm
export const HF_X0 = -250; // mm, left edge (body frame, x right)
export const HF_Y0 = 50; // mm, near edge (y forward)
export const HF_COLS = 21; // 500 mm wide
export const HF_ROWS = 20; // to 550 mm ahead

const KEEP_MS = 700; // a cell nobody sees any more is forgotten after this (the robot moves)
const SMOOTHING = 0.5; // weight of a new reading on the same surface
const JUMP = 15; // mm: a reading this much higher replaces the cell at once (obstacle top)

/**
 * Ground heights on a regular grid in front of the robot, from the ToF zone
 * points of all sensors. Gives the first-person view one continuous mesh
 * instead of one patch per sensor: readings are averaged over a few frames,
 * obstacle tops win over the floor around them (like the firmware's terrain
 * map), and small holes between zones are filled from their neighbours.
 * Body frame, no odometry: cells are only kept for a moment.
 */
export class HeightField {
  readonly z = new Float32Array(HF_COLS * HF_ROWS).fill(NaN);
  private readonly seen = new Float64Array(HF_COLS * HF_ROWS);

  static cellCenter(col: number, row: number): [number, number] {
    return [HF_X0 + (col + 0.5) * HF_CELL, HF_Y0 + (row + 0.5) * HF_CELL];
  }

  /** Add one frame of points (body frame, mm). */
  add(points: Vec[], now: number): void {
    const sum = new Float32Array(HF_COLS * HF_ROWS);
    const n = new Uint16Array(HF_COLS * HF_ROWS);
    for (const p of points) {
      const col = Math.floor((p[0] - HF_X0) / HF_CELL);
      const row = Math.floor((p[1] - HF_Y0) / HF_CELL);
      if (col < 0 || row < 0 || col >= HF_COLS || row >= HF_ROWS) continue;
      sum[row * HF_COLS + col] += p[2];
      n[row * HF_COLS + col]++;
    }
    for (let i = 0; i < this.z.length; i++) {
      if (n[i]) {
        const v = sum[i] / n[i];
        const z = this.z[i];
        this.z[i] = Number.isNaN(z) || v > z + JUMP ? v : z + (v - z) * SMOOTHING;
        this.seen[i] = now;
      } else if (now - this.seen[i] > KEEP_MS) {
        this.z[i] = NaN;
      }
    }
  }

  clear(): void {
    this.z.fill(NaN);
  }

  /** Heights for drawing: unknown cells with at least 3 known neighbours get their average. */
  filled(): Float32Array {
    const out = this.z.slice();
    for (let r = 0; r < HF_ROWS; r++) {
      for (let c = 0; c < HF_COLS; c++) {
        if (!Number.isNaN(this.z[r * HF_COLS + c])) continue;
        let s = 0;
        let k = 0;
        for (let dr = -1; dr <= 1; dr++) {
          for (let dc = -1; dc <= 1; dc++) {
            const rr = r + dr;
            const cc = c + dc;
            if (rr < 0 || cc < 0 || rr >= HF_ROWS || cc >= HF_COLS) continue;
            const v = this.z[rr * HF_COLS + cc];
            if (!Number.isNaN(v)) {
              s += v;
              k++;
            }
          }
        }
        if (k >= 3) out[r * HF_COLS + c] = s / k;
      }
    }
    return out;
  }
}
