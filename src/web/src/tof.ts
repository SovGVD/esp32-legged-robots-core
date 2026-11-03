import { TOF_ZONES, type TofSensorFrame } from './protocol.ts';

const CELL = 22; // css px per zone
const SIDE = 8;
const STALE_MS = 1500;

/** Zone colour: red close, through yellow and green, to blue at the range limit. */
export function zoneColor(mm: number, range: number): string | null {
  if (mm <= 0) return null;
  const f = Math.min(1, mm / range);
  return `hsl(${Math.round(f * 230)} 85% ${f >= 1 ? 30 : 50}%)`;
}

interface View {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  info: HTMLElement;
  frame: number;
  changed: number;
}

/**
 * ToF view: one 8x8 heat map per sensor, as the sensor looks (row 0 at the
 * top, left on the left), with the distance in each zone in cm.
 */
export class TofPanel {
  private readonly views: View[] = [];
  private readonly range: number;

  constructor(root: HTMLElement, count: number, range: number) {
    this.range = range;
    const ratio = Math.min(3, window.devicePixelRatio || 1);
    for (let i = 0; i < count; i++) {
      const box = document.createElement('figure');
      const canvas = document.createElement('canvas');
      canvas.width = canvas.height = CELL * SIDE * ratio;
      canvas.style.width = canvas.style.height = `${CELL * SIDE}px`;
      const ctx = canvas.getContext('2d')!;
      ctx.scale(ratio, ratio);
      const info = document.createElement('figcaption');
      info.className = 'small mono';
      box.append(canvas, info);
      root.append(box);
      this.views.push({ canvas, ctx, info, frame: -1, changed: 0 });
      this.draw(i, null);
    }
  }

  update(sensors: TofSensorFrame[], now = performance.now()): void {
    this.views.forEach((v, i) => {
      const s = sensors[i];
      if (s && s.frame !== v.frame) {
        v.frame = s.frame;
        v.changed = now;
      }
      this.draw(i, s ?? null, s && s.running && now - v.changed > STALE_MS);
    });
  }

  /** No data (offline, view closed). */
  clear(): void {
    this.views.forEach((v, i) => {
      v.frame = -1;
      this.draw(i, null);
    });
  }

  private draw(i: number, s: TofSensorFrame | null, stale = false): void {
    const { ctx, info } = this.views[i];
    const name = this.views.length === 2 ? ['left', 'right'][i] : 'ToF';
    ctx.clearRect(0, 0, CELL * SIDE, CELL * SIDE);
    ctx.font = `9px ui-monospace, Menlo, monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    let min = 0;
    for (let z = 0; z < TOF_ZONES; z++) {
      const x = (z % SIDE) * CELL;
      const y = Math.floor(z / SIDE) * CELL;
      const mm = s && s.running ? s.mm[z] : 0;
      const color = zoneColor(mm, this.range);
      ctx.fillStyle = color ?? 'rgba(128,128,128,0.25)';
      ctx.fillRect(x + 1, y + 1, CELL - 2, CELL - 2);
      if (mm > 0) {
        if (!min || mm < min) min = mm;
        ctx.fillStyle = '#000';
        ctx.fillText(String(Math.round(mm / 10)), x + CELL / 2, y + CELL / 2);
      }
    }
    let text = name;
    if (!s) text += ': no data';
    else if (!s.running) text += ': not running';
    else if (stale) text += ': stalled';
    else text += min ? `: closest ${Math.round(min / 10)} cm` : ': clear';
    info.textContent = text;
    info.classList.toggle('warn', !!s && (!s.running || stale));
  }
}
