import type { TofMount } from './config.ts';
import type { TofSensorFrame } from './protocol.ts';
import { zoneDirection, zonePoint, type Vec } from './tofgeom.ts';

const SIDE = 8;
const MAX_STEP = 120; // mm: neighbours further apart than this are not joined (foreground vs background)
const OBSTACLE = 40; // mm above the floor that gets the hottest colour
const DEFAULT_VIEW = { yaw: 0, pitch: 0.85, dist: 950 };

interface Face {
  pts: Vec[];
  depth: number;
  color: string;
}

/**
 * The ToF zones as a 3D surface in the robot's frame: every zone becomes a
 * point (sensor mounting + zone ray * distance, like the terrain map does),
 * neighbouring points are joined into shaded quads coloured by height above
 * the floor: blue floor, red obstacles, purple below the floor (a hole or a
 * gap). Drag to orbit, wheel to zoom, double click to reset the view.
 */
export class TofSurface {
  readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly mounts: TofMount[];
  private readonly range: number;
  private view = { ...DEFAULT_VIEW };
  private target: Vec = [0, 220, -80];
  private sensors: TofSensorFrame[] = [];
  private w = 0;
  private h = 0;

  constructor(mounts: TofMount[], range: number) {
    this.mounts = mounts;
    this.range = range;
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'tof3d';
    this.ctx = this.canvas.getContext('2d')!;
    this.drag();
  }

  update(sensors: TofSensorFrame[]): void {
    this.sensors = sensors;
    this.draw();
  }

  private drag(): void {
    const c = this.canvas;
    let last: { x: number; y: number } | null = null;
    c.addEventListener('pointerdown', (e) => {
      last = { x: e.clientX, y: e.clientY };
      c.setPointerCapture(e.pointerId);
    });
    c.addEventListener('pointermove', (e) => {
      if (!last) return;
      this.view.yaw -= (e.clientX - last.x) * 0.01;
      this.view.pitch = Math.min(1.5, Math.max(0.05, this.view.pitch + (e.clientY - last.y) * 0.01));
      last = { x: e.clientX, y: e.clientY };
      this.draw();
    });
    const up = () => (last = null);
    c.addEventListener('pointerup', up);
    c.addEventListener('pointercancel', up);
    c.addEventListener(
      'wheel',
      (e) => {
        e.preventDefault();
        this.view.dist = Math.min(2000, Math.max(250, this.view.dist * (e.deltaY > 0 ? 1.1 : 0.9)));
        this.draw();
      },
      { passive: false },
    );
    c.addEventListener('dblclick', () => {
      this.view = { ...DEFAULT_VIEW };
      this.draw();
    });
  }

  /** Body frame -> [screen x, screen y, depth]; null behind the camera. */
  private project(p: Vec): Vec | null {
    const { yaw, pitch, dist } = this.view;
    const x = p[0] - this.target[0];
    const y = p[1] - this.target[1];
    const z = p[2] - this.target[2];
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const x1 = x * cy + y * sy;
    const y1 = -x * sy + y * cy;
    const cp = Math.cos(pitch);
    const sp = Math.sin(pitch);
    const depth = y1 * cp - z * sp + dist;
    if (depth < 20) return null;
    const up = y1 * sp + z * cp;
    const f = this.h * 2.6;
    return [this.w / 2 + (f * x1) / depth, this.h / 2 - (f * up) / depth, depth];
  }

  private resize(): void {
    const ratio = Math.min(3, window.devicePixelRatio || 1);
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (w && h && (w !== this.w || h !== this.h || this.canvas.width !== Math.round(w * ratio))) {
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
    ctx.clearRect(0, 0, w, h);

    // zone points per sensor
    const grids: (Vec | null)[][] = this.mounts.map((m, i) => {
      const s = this.sensors[i];
      const g: (Vec | null)[] = [];
      for (let z = 0; z < SIDE * SIDE; z++) g.push(s && s.running ? zonePoint(m, z, s.mm[z]) : null);
      return g;
    });
    const all = grids.flat().filter((p): p is Vec => !!p);

    // floor: a low percentile of the heights (most zones see the floor)
    let floor = this.target[2];
    if (all.length > 4) {
      const zs = all.map((p) => p[2]).sort((a, b) => a - b);
      floor = zs[Math.floor(zs.length * 0.3)];
      const cx = all.reduce((a, p) => a + p[0], 0) / all.length;
      const cy = all.reduce((a, p) => a + p[1], 0) / all.length;
      // follow the points slowly so the view doesn't jump with every frame
      this.target = [this.target[0] + (cx - this.target[0]) * 0.1, this.target[1] + (cy - this.target[1]) * 0.1, this.target[2] + (floor - this.target[2]) * 0.1];
    }

    this.drawFloorGrid(floor);
    this.drawRobot();

    // surface quads, far to near
    const light: Vec = norm([0.4, -0.6, 1]);
    const faces: Face[] = [];
    for (const g of grids) {
      for (let r = 0; r < SIDE - 1; r++) {
        for (let c = 0; c < SIDE - 1; c++) {
          const q = [g[r * SIDE + c], g[r * SIDE + c + 1], g[(r + 1) * SIDE + c + 1], g[(r + 1) * SIDE + c]];
          if (q.some((p) => !p)) continue;
          const pts = q as Vec[];
          if (span(pts) > MAX_STEP) continue;
          const n = norm(cross(sub(pts[1], pts[0]), sub(pts[3], pts[0])));
          const shade = 0.55 + 0.45 * Math.abs(dot(n, light));
          const zAvg = pts.reduce((a, p) => a + p[2], 0) / 4;
          const proj = pts.map((p) => this.project(p));
          if (proj.some((p) => !p)) continue;
          faces.push({
            pts: proj as Vec[],
            depth: proj.reduce((a, p) => a + p![2], 0) / 4,
            color: heightColor(zAvg - floor, shade),
          });
        }
      }
    }
    faces.sort((a, b) => b.depth - a.depth);
    for (const f of faces) {
      ctx.beginPath();
      f.pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
      ctx.closePath();
      ctx.fillStyle = f.color;
      ctx.fill();
      ctx.strokeStyle = 'rgba(0,0,0,0.25)';
      ctx.lineWidth = 0.5;
      ctx.stroke();
    }

    // zones that are not part of a quad still show as dots
    for (const p of all) {
      const s = this.project(p);
      if (!s) continue;
      ctx.fillStyle = heightColor(p[2] - floor, 1);
      ctx.beginPath();
      ctx.arc(s[0], s[1], Math.max(1.5, 600 / s[2]), 0, Math.PI * 2);
      ctx.fill();
    }
  }

  private line(a: Vec, b: Vec): void {
    const p = this.project(a);
    const q = this.project(b);
    if (!p || !q) return;
    this.ctx.moveTo(p[0], p[1]);
    this.ctx.lineTo(q[0], q[1]);
  }

  private drawFloorGrid(z: number): void {
    const { ctx } = this;
    ctx.beginPath();
    for (let x = -200; x <= 200; x += 50) this.line([x, -100, z], [x, 500, z]);
    for (let y = -100; y <= 500; y += 50) this.line([-200, y, z], [200, y, z]);
    ctx.strokeStyle = 'rgba(128,128,128,0.25)';
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  /** Body outline, nose arrow, and each sensor's field of view out to its range. */
  private drawRobot(): void {
    const { ctx } = this;
    const front = Math.max(60, ...this.mounts.map((m) => m.y));
    const half = front * 0.5;
    ctx.beginPath();
    this.line([-half, -front, 0], [half, -front, 0]);
    this.line([half, -front, 0], [half, front, 0]);
    this.line([half, front, 0], [-half, front, 0]);
    this.line([-half, front, 0], [-half, -front, 0]);
    this.line([-half * 0.5, front - 30, 0], [0, front, 0]);
    this.line([0, front, 0], [half * 0.5, front - 30, 0]);
    ctx.strokeStyle = 'rgba(128,128,128,0.9)';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.beginPath();
    for (const m of this.mounts) {
      const o: Vec = [m.x, m.y, m.z];
      for (const zone of [0, SIDE - 1, SIDE * SIDE - 1, SIDE * (SIDE - 1)]) {
        const d = zoneDirection(m, zone);
        this.line(o, [m.x + d[0] * this.range, m.y + d[1] * this.range, m.z + d[2] * this.range]);
      }
    }
    ctx.strokeStyle = 'rgba(249,115,22,0.35)';
    ctx.lineWidth = 1;
    ctx.stroke();
  }
}

/** Blue on the floor, through green and yellow to red for obstacles, purple below the floor. */
export function heightColor(dz: number, shade: number): string {
  const l = Math.round(50 * shade);
  if (dz < -25) return `hsl(275 60% ${l}%)`;
  const f = Math.min(1, Math.max(0, dz / OBSTACLE));
  return `hsl(${Math.round(210 - f * 210)} 80% ${l}%)`;
}

const sub = (a: Vec, b: Vec): Vec => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a: Vec, b: Vec) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec, b: Vec): Vec => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function norm(a: Vec): Vec {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
}
function span(pts: Vec[]): number {
  let max = 0;
  for (let i = 0; i < pts.length; i++) {
    for (let j = i + 1; j < pts.length; j++) max = Math.max(max, Math.hypot(...sub(pts[i], pts[j])));
  }
  return max;
}
