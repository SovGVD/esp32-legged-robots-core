import type { TofMount } from './config.ts';
import { HF_COLS, HF_ROWS, HeightField } from './heightfield.ts';
import type { TofSensorFrame } from './protocol.ts';
import { zonePoint, type Vec } from './tofgeom.ts';

const EYE: Vec = [0, -20, 60]; // mm, body frame: a bit behind and above the sensors
const PITCH = 0.42; // rad, looking down
const FOG = 650; // mm: the mesh fades out towards this distance
const OBSTACLE = 30; // mm above the floor that gets the hottest colour
const MAX_STEP = 90; // mm: quads with a bigger height step are drawn as an edge, not a wall

/** Neon hue by height above the floor: cyan floor, through green and yellow to red, magenta below. */
function hue(dz: number): number {
  if (dz < -25) return 300;
  return Math.round(190 - Math.min(1, Math.max(0, dz / OBSTACLE)) * 190);
}

/**
 * First-person view of the ground ahead: the ToF points of all sensors go
 * into a height field (one mesh across the sensors), drawn from just above
 * the robot's nose as a glowing wire mesh that fades with distance, with
 * distance marks on the floor. Fills the drive area behind the sticks and
 * takes no input, so driving works as usual.
 */
export class TofFirstPerson {
  readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly mounts: TofMount[];
  private readonly field = new HeightField();
  private floor = -90;
  private w = 0;
  private h = 0;

  constructor(mounts: TofMount[]) {
    this.mounts = mounts;
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'tof-fpv';
    this.ctx = this.canvas.getContext('2d')!;
  }

  update(sensors: TofSensorFrame[], now = performance.now()): void {
    const points: Vec[] = [];
    this.mounts.forEach((m, i) => {
      const s = sensors[i];
      if (!s || !s.running) return;
      for (let z = 0; z < s.mm.length; z++) {
        const p = zonePoint(m, z, s.mm[z]);
        if (p) points.push(p);
      }
    });
    if (sensors.length) this.field.add(points, now);
    else this.field.clear();
    this.draw();
  }

  /** Body frame -> [screen x, screen y, depth]; null behind the camera. */
  private project(p: Vec): Vec | null {
    const x = p[0] - EYE[0];
    const y = p[1] - EYE[1];
    const z = p[2] - EYE[2];
    const cp = Math.cos(PITCH);
    const sp = Math.sin(PITCH);
    const depth = y * cp - z * sp;
    if (depth < 15) return null;
    const up = y * sp + z * cp;
    const f = Math.max(this.w, this.h * 1.2) * 0.55;
    return [this.w / 2 + (f * x) / depth, this.h * 0.42 - (f * up) / depth, depth];
  }

  private resize(): void {
    const ratio = Math.min(2, window.devicePixelRatio || 1);
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (w && h && (w !== this.w || h !== this.h)) {
      this.w = w;
      this.h = h;
      this.canvas.width = Math.round(w * ratio);
      this.canvas.height = Math.round(h * ratio);
      this.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    }
  }

  draw(): void {
    this.resize();
    const { ctx, w, h } = this;
    if (!w || !h) return;
    const z = this.field.filled();

    // floor: a low percentile of the known heights, followed slowly
    const known = Array.from(z).filter((v) => !Number.isNaN(v));
    if (known.length > 8) {
      known.sort((a, b) => a - b);
      this.floor += (known[Math.floor(known.length * 0.3)] - this.floor) * 0.2;
    }

    // night sky to a faint horizon glow
    const horizon = this.project([0, 5000, EYE[2]])?.[1] ?? h * 0.2;
    const sky = ctx.createLinearGradient(0, 0, 0, h);
    sky.addColorStop(0, '#05070d');
    sky.addColorStop(Math.min(0.95, Math.max(0.05, horizon / h)), '#0f1a2a');
    sky.addColorStop(1, '#05070d');
    ctx.fillStyle = sky;
    ctx.fillRect(0, 0, w, h);

    this.drawDistanceMarks();

    // mesh, far to near: translucent faces, glowing edges, both fading with distance
    const quads: { pts: Vec[]; depth: number; hue: number }[] = [];
    for (let r = HF_ROWS - 2; r >= 0; r--) {
      for (let c = 0; c < HF_COLS - 1; c++) {
        const idx = [r * HF_COLS + c, r * HF_COLS + c + 1, (r + 1) * HF_COLS + c + 1, (r + 1) * HF_COLS + c];
        const zs = idx.map((i) => z[i]);
        if (zs.some((v) => Number.isNaN(v))) continue;
        if (Math.max(...zs) - Math.min(...zs) > MAX_STEP) continue;
        const cells: [number, number][] = [
          [c, r],
          [c + 1, r],
          [c + 1, r + 1],
          [c, r + 1],
        ];
        const proj = cells.map(([cc, rr], k) => {
          const [x, y] = HeightField.cellCenter(cc, rr);
          return this.project([x, y, zs[k]]);
        });
        if (proj.some((p) => !p)) continue;
        const pts = proj as Vec[];
        const avg = zs.reduce((a, v) => a + v, 0) / 4;
        quads.push({ pts, depth: pts.reduce((a, p) => a + p[2], 0) / 4, hue: hue(avg - this.floor) });
      }
    }
    quads.sort((a, b) => b.depth - a.depth);
    ctx.lineJoin = 'round';
    for (const q of quads) {
      const fade = Math.max(0, 1 - q.depth / FOG);
      if (fade <= 0) continue;
      ctx.beginPath();
      q.pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
      ctx.closePath();
      ctx.fillStyle = `hsla(${q.hue} 90% 45% / ${0.18 + 0.3 * fade})`;
      ctx.fill();
      ctx.strokeStyle = `hsla(${q.hue} 100% 65% / ${0.25 + 0.75 * fade})`;
      ctx.lineWidth = 0.6 + 1.2 * fade;
      ctx.stroke();
    }
  }

  /** Lines across the floor every 10 cm ahead of the nose, with labels. */
  private drawDistanceMarks(): void {
    const { ctx } = this;
    const nose = Math.max(0, ...this.mounts.map((m) => m.y));
    ctx.font = '11px ui-monospace, Menlo, monospace';
    ctx.textBaseline = 'bottom';
    for (let cm = 10; cm <= 50; cm += 10) {
      const y = nose + cm * 10;
      const a = this.project([-260, y, this.floor]);
      const b = this.project([260, y, this.floor]);
      if (!a || !b) continue;
      const fade = Math.max(0.15, 1 - a[2] / FOG);
      ctx.strokeStyle = `rgba(148,163,184,${0.35 * fade})`;
      ctx.setLineDash([4, 6]);
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(a[0], a[1]);
      ctx.lineTo(b[0], b[1]);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = `rgba(203,213,225,${0.8 * fade})`;
      ctx.fillText(`${cm} cm`, Math.min(b[0] + 4, this.w - 44), b[1]);
    }
  }
}
