/**
 * Settings tab drawings: magnetometer calibration plot, heading dial, bubble
 * level. The fit math mirrors the firmware's MagCalibrator (lib/robot_core
 * heading.cpp): hard iron = middle of the min/max box, soft iron = scale the
 * x and y spans to their mean.
 */
import type { Vec3Tuple } from './protocol.ts';

export type P2 = [number, number];

export interface Fit {
  cx: number;
  cy: number;
  rx: number;
  ry: number;
}

/** What the firmware would save from these readings; null if they don't span a turn (like MagCalibrator). */
export function minMaxFit(pts: P2[]): Fit | null {
  if (pts.length < 10) return null;
  let [x0, y0] = pts[0];
  let [x1, y1] = pts[0];
  for (const [x, y] of pts) {
    x0 = Math.min(x0, x);
    x1 = Math.max(x1, x);
    y0 = Math.min(y0, y);
    y1 = Math.max(y1, y);
  }
  const rx = (x1 - x0) / 2;
  const ry = (y1 - y0) / 2;
  if (rx < 5 || ry < 5) return null;
  return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, rx, ry };
}

/** A raw reading with the calibration applied (HeadingEstimator::update). */
export const correct = ([x, y]: P2, off: Vec3Tuple, sc: Vec3Tuple): P2 => [(x - off[0]) * sc[0], (y - off[1]) * sc[1]];

/** Mean radius of corrected points and their spread (std / mean): a good calibration is a thin ring round 0. */
export function roundness(pts: P2[]): { r: number; spread: number } | null {
  if (pts.length < 10) return null;
  const rs = pts.map(([x, y]) => Math.hypot(x, y));
  const r = rs.reduce((a, b) => a + b, 0) / rs.length;
  if (r <= 0) return null;
  const sd = Math.sqrt(rs.reduce((a, b) => a + (b - r) ** 2, 0) / rs.length);
  return { r, spread: sd / r };
}

/** Compass degrees (clockwise from up) of a point relative to the centre. */
export const dialAngle = (dx: number, dy: number) => ((Math.atan2(dx, -dy) * 180) / Math.PI + 360) % 360;

const css = (name: string) => getComputedStyle(document.body).getPropertyValue(name).trim() || '#888';

/** Size the canvas to its CSS box (sharp on phones); returns the context and its size in CSS px. */
function prep(c: HTMLCanvasElement): [CanvasRenderingContext2D, number, number] {
  const dpr = window.devicePixelRatio || 1;
  const w = c.clientWidth || 280;
  const h = c.clientHeight || w;
  if (c.width !== Math.round(w * dpr)) c.width = Math.round(w * dpr);
  if (c.height !== Math.round(h * dpr)) c.height = Math.round(h * dpr);
  const g = c.getContext('2d')!;
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  g.font = '11px system-ui, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  return [g, w, h];
}

function ring(g: CanvasRenderingContext2D, x: number, y: number, rx: number, ry = rx): void {
  g.beginPath();
  g.ellipse(x, y, Math.max(0, rx), Math.max(0, ry), 0, 0, Math.PI * 2);
  g.stroke();
}

export interface MagPlot {
  points: P2[];
  corrected: boolean;
  offset: Vec3Tuple;
  scale: Vec3Tuple;
  calibrated: boolean;
  /** 0..1 of the two calibration turns, null when not calibrating */
  progress: number | null;
}

/**
 * Magnetometer readings seen from above (x right, y forward, uT). Raw: the
 * cloud sits off centre (hard iron) and is squashed (soft iron); the dashed
 * box/ellipse is what a calibration from these points would save, the solid
 * one the calibration in use. Corrected: should be a thin ring round 0.
 */
export function drawMagPlot(c: HTMLCanvasElement, p: MagPlot): void {
  const [g, w, h] = prep(c);
  const fg = css('--fg');
  const dim = css('--dim');
  const line = css('--line');
  const accent = css('--accent');
  const warn = css('--warn');
  const pts = p.corrected ? p.points.map((q) => correct(q, p.offset, p.scale)) : p.points;
  // scale: everything (and the origin) fits with a margin
  let span = 30;
  for (const [x, y] of pts) span = Math.max(span, Math.abs(x), Math.abs(y));
  if (!p.corrected && p.calibrated) span = Math.max(span, Math.abs(p.offset[0]) + 10, Math.abs(p.offset[1]) + 10);
  const k = (Math.min(w, h) / 2 - 22) / (span * 1.1);
  const cx = w / 2;
  const cy = h / 2;
  const X = (x: number) => cx + x * k;
  const Y = (y: number) => cy - y * k;

  g.strokeStyle = line;
  g.lineWidth = 1;
  g.beginPath();
  g.moveTo(8, cy);
  g.lineTo(w - 8, cy);
  g.moveTo(cx, 8);
  g.lineTo(cx, h - 8);
  g.stroke();
  g.fillStyle = dim;
  g.fillText(`${Math.round(span * 1.1)} µT`, w - 26, cy + 10);
  g.fillText('fwd', cx + 14, 14);

  pts.forEach(([x, y], i) => {
    g.globalAlpha = 0.25 + (0.75 * i) / pts.length;
    g.fillStyle = accent;
    g.fillRect(X(x) - 1.5, Y(y) - 1.5, 3, 3);
  });
  g.globalAlpha = 1;
  const last = pts[pts.length - 1];
  if (last) {
    g.fillStyle = fg;
    g.beginPath();
    g.arc(X(last[0]), Y(last[1]), 4, 0, Math.PI * 2);
    g.fill();
  }

  g.lineWidth = 1.5;
  if (p.corrected) {
    const rd = roundness(pts);
    if (rd) {
      g.strokeStyle = rd.spread < 0.08 ? accent : warn;
      ring(g, cx, cy, rd.r * k);
    }
  } else {
    // live fit from these points (dashed) and the saved calibration (solid)
    const f = minMaxFit(pts);
    if (f) {
      g.setLineDash([5, 4]);
      g.strokeStyle = dim;
      g.strokeRect(X(f.cx - f.rx), Y(f.cy + f.ry), f.rx * 2 * k, f.ry * 2 * k);
      g.setLineDash([]);
    }
    if (p.calibrated) {
      const r = roundness(p.points.map((q) => correct(q, p.offset, p.scale)))?.r ?? (f ? (f.rx + f.ry) / 2 : 20);
      g.strokeStyle = warn;
      ring(g, X(p.offset[0]), Y(p.offset[1]), (r / p.scale[0]) * k, (r / p.scale[1]) * k);
      g.beginPath();
      g.moveTo(X(p.offset[0]) - 6, Y(p.offset[1]));
      g.lineTo(X(p.offset[0]) + 6, Y(p.offset[1]));
      g.moveTo(X(p.offset[0]), Y(p.offset[1]) - 6);
      g.lineTo(X(p.offset[0]), Y(p.offset[1]) + 6);
      g.stroke();
    }
  }

  if (p.progress !== null) {
    // two turns: the ring round the plot fills up
    const r = Math.min(w, h) / 2 - 6;
    g.lineWidth = 5;
    g.strokeStyle = line;
    ring(g, cx, cy, r);
    g.strokeStyle = accent;
    g.beginPath();
    g.arc(cx, cy, r, -Math.PI / 2, -Math.PI / 2 + Math.PI * 2 * Math.min(1, p.progress));
    g.stroke();
  }
  if (!pts.length) {
    g.fillStyle = dim;
    g.fillText('no magnetometer readings yet', cx, cy + 30);
  }
}

/** Heading dial, north up: needle = heading, triangle = hold target. Tap to turn. */
export function drawDial(c: HTMLCanvasElement, heading: number | null, target: number | null, compass: boolean): void {
  const [g, w, h] = prep(c);
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) / 2 - 16;
  const dim = css('--dim');
  const at = (deg: number, rr: number): P2 => [cx + Math.sin((deg * Math.PI) / 180) * rr, cy - Math.cos((deg * Math.PI) / 180) * rr];
  g.strokeStyle = css('--line');
  g.lineWidth = 2;
  ring(g, cx, cy, r);
  g.lineWidth = 1;
  for (let d = 0; d < 360; d += 15) {
    const [x0, y0] = at(d, r - (d % 45 ? 4 : 9));
    const [x1, y1] = at(d, r);
    g.beginPath();
    g.moveTo(x0, y0);
    g.lineTo(x1, y1);
    g.stroke();
  }
  g.font = '600 13px system-ui, sans-serif';
  (compass ? ['N', 'E', 'S', 'W'] : ['0', '90', '180', '270']).forEach((t, i) => {
    const [x, y] = at(i * 90, r + 10);
    g.fillStyle = i === 0 && compass ? css('--warn') : dim;
    g.fillText(t, x, y);
  });
  if (target !== null) {
    const [x, y] = at(target, r - 2);
    const [a, b] = at(target - 5, r - 16);
    const [e, f] = at(target + 5, r - 16);
    g.fillStyle = css('--warn');
    g.beginPath();
    g.moveTo(x, y);
    g.lineTo(a, b);
    g.lineTo(e, f);
    g.fill();
  }
  if (heading !== null) {
    const [x, y] = at(heading, r - 18);
    const [bx, by] = at(heading + 180, 14);
    g.strokeStyle = css('--accent');
    g.lineWidth = 4;
    g.lineCap = 'round';
    g.beginPath();
    g.moveTo(bx, by);
    g.lineTo(x, y);
    g.stroke();
    g.lineCap = 'butt';
  }
  g.fillStyle = css('--fg');
  g.font = '600 18px system-ui, sans-serif';
  g.fillText(heading === null ? '—' : `${Math.round(heading)}°`, cx, cy + r / 2.2);
}

/** Body tilt from the IMU: pitch = sideways lean, roll = nose up (deg), ±15° to the rim; green within 1°. */
export function drawLevel(c: HTMLCanvasElement, pitch: number | null, roll: number | null): void {
  const [g, w, h] = prep(c);
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) / 2 - 4;
  g.strokeStyle = css('--line');
  g.lineWidth = 1.5;
  ring(g, cx, cy, r);
  ring(g, cx, cy, r / 3);
  g.beginPath();
  g.moveTo(cx - r, cy);
  g.lineTo(cx + r, cy);
  g.moveTo(cx, cy - r);
  g.lineTo(cx, cy + r);
  g.stroke();
  if (pitch === null || roll === null) return;
  const k = (r - 10) / 15;
  const clamp = (v: number) => Math.max(-15, Math.min(15, v));
  const off = Math.hypot(pitch, roll) > 1;
  g.fillStyle = css(off ? '--warn' : '--accent');
  g.beginPath();
  // nose up moves the dot up the screen; sideways lean (pitch) moves it sideways
  g.arc(cx + clamp(pitch) * k, cy - clamp(roll) * k, 9, 0, Math.PI * 2);
  g.fill();
}
