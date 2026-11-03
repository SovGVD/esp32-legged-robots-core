import type { TofMount } from './config.ts';
import { TOF_ZONES, type TofSensorFrame } from './protocol.ts';
import { TofSurface } from './tof3d.ts';
import { TofFirstPerson } from './toffpv.ts';

const CELL = 22; // css px per zone
const SIDE = 8;
const STALE_MS = 1500;
const MODE_KEY = 'tofMode';

/** Zone colour: red close, through yellow and green, to blue at the range limit. */
export function zoneColor(mm: number, range: number): string | null {
  if (mm <= 0) return null;
  const f = Math.min(1, mm / range);
  return `hsl(${Math.round(f * 230)} 85% ${f >= 1 ? 30 : 50}%)`;
}

interface Grid {
  ctx: CanvasRenderingContext2D;
  frame: number;
  changed: number;
}

type Mode = 'fpv' | '3d' | 'grid';
const MODES: Mode[] = ['fpv', '3d', 'grid'];
const MODE_TITLE: Record<Mode, string> = { fpv: 'first person', '3d': '3D', grid: 'grid' };

function loadMode(): Mode {
  try {
    const m = localStorage.getItem(MODE_KEY) as Mode;
    return MODES.includes(m) ? m : 'fpv';
  } catch {
    return 'fpv';
  }
}

/**
 * ToF view, three modes: first person (TofFirstPerson, full size behind the
 * sticks), a 3D surface around the robot to orbit (TofSurface), or one 8x8 heat map per sensor as the sensor looks (row 0 at
 * the top, left on the left) with the distance in each zone in cm.
 */
export class TofPanel {
  private readonly grids: Grid[] = [];
  private readonly range: number;
  private readonly surface: TofSurface | null;
  private readonly fpv: TofFirstPerson | null;
  private readonly host: HTMLElement;
  private readonly gridBox: HTMLElement;
  private readonly info: HTMLElement;
  private readonly button: HTMLButtonElement;
  private mode: Mode = loadMode();
  private last: TofSensorFrame[] = [];

  /** root: the floating panel; host: the drive area the first-person view fills */
  constructor(root: HTMLElement, host: HTMLElement, count: number, range: number, mounts: TofMount[]) {
    this.range = range;
    this.host = host;
    // the 3D views need to know where each sensor is
    const has3d = mounts.length >= count && count > 0;
    this.surface = has3d ? new TofSurface(mounts.slice(0, count), range) : null;
    this.fpv = has3d ? new TofFirstPerson(mounts.slice(0, count)) : null;
    if (this.fpv) host.prepend(this.fpv.canvas);
    this.button = document.createElement('button');
    this.button.type = 'button';
    this.button.className = 'small';
    this.button.addEventListener('click', () => this.setMode(MODES[(MODES.indexOf(this.mode) + 1) % MODES.length]));
    this.gridBox = document.createElement('div');
    this.gridBox.className = 'tof-grids';
    this.info = document.createElement('div');
    this.info.className = 'small mono tof-info';
    const ratio = Math.min(3, window.devicePixelRatio || 1);
    for (let i = 0; i < count; i++) {
      const canvas = document.createElement('canvas');
      canvas.width = canvas.height = CELL * SIDE * ratio;
      canvas.style.width = canvas.style.height = `${CELL * SIDE}px`;
      const ctx = canvas.getContext('2d')!;
      ctx.scale(ratio, ratio);
      this.gridBox.append(canvas);
      this.grids.push({ ctx, frame: -1, changed: 0 });
    }
    if (this.surface) root.append(this.surface.canvas);
    root.append(this.gridBox, this.info);
    if (this.surface) root.append(this.button);
    this.setMode(this.surface ? this.mode : 'grid');
  }

  /** The panel is open or closed (ToF toggle). */
  show(on: boolean): void {
    this.host.classList.toggle('fpv', on && this.mode === 'fpv');
    this.clear();
  }

  private setMode(mode: Mode): void {
    this.mode = mode;
    try {
      localStorage.setItem(MODE_KEY, mode);
    } catch {
      /* private mode: not remembered */
    }
    this.button.textContent = MODE_TITLE[MODES[(MODES.indexOf(mode) + 1) % MODES.length]];
    this.gridBox.hidden = mode !== 'grid';
    if (this.surface) this.surface.canvas.hidden = mode !== '3d';
    this.host.classList.toggle('fpv', mode === 'fpv' && !this.info.closest('[hidden]'));
    this.render();
  }

  update(sensors: TofSensorFrame[], now = performance.now()): void {
    this.grids.forEach((g, i) => {
      const s = sensors[i];
      if (s && s.frame !== g.frame) {
        g.frame = s.frame;
        g.changed = now;
      }
    });
    this.last = sensors;
    this.render(now);
  }

  /** No data (offline, view closed). */
  clear(): void {
    this.grids.forEach((g) => (g.frame = -1));
    this.last = [];
    this.render();
  }

  private render(now = performance.now()): void {
    const texts: string[] = [];
    let warn = false;
    this.grids.forEach((g, i) => {
      const s = this.last[i] ?? null;
      const stale = !!s && s.running && now - g.changed > STALE_MS;
      const min = this.mode === 'grid' ? this.drawGrid(i, s) : closest(s);
      let text = this.grids.length === 2 ? ['left', 'right'][i] : 'ToF';
      if (!s) text += ': no data';
      else if (!s.running) text += ': not running';
      else if (stale) text += ': stalled';
      else text += min ? `: closest ${Math.round(min / 10)} cm` : ': clear';
      texts.push(text);
      warn ||= !!s && (!s.running || stale);
    });
    this.info.textContent = texts.join(' · ');
    this.info.classList.toggle('warn', warn);
    if (this.mode === '3d') this.surface?.update(this.last);
    if (this.mode === 'fpv') this.fpv?.update(this.last, now);
  }

  /** Draws one heat map, returns the closest distance (0: none). */
  private drawGrid(i: number, s: TofSensorFrame | null): number {
    const { ctx } = this.grids[i];
    ctx.clearRect(0, 0, CELL * SIDE, CELL * SIDE);
    ctx.font = `9px ui-monospace, Menlo, monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let z = 0; z < TOF_ZONES; z++) {
      const x = (z % SIDE) * CELL;
      const y = Math.floor(z / SIDE) * CELL;
      const mm = s && s.running ? s.mm[z] : 0;
      ctx.fillStyle = zoneColor(mm, this.range) ?? 'rgba(128,128,128,0.25)';
      ctx.fillRect(x + 1, y + 1, CELL - 2, CELL - 2);
      if (mm > 0) {
        ctx.fillStyle = '#000';
        ctx.fillText(String(Math.round(mm / 10)), x + CELL / 2, y + CELL / 2);
      }
    }
    return closest(s);
  }
}

function closest(s: TofSensorFrame | null): number {
  let min = 0;
  if (s && s.running) for (const mm of s.mm) if (mm > 0 && (!min || mm < min)) min = mm;
  return min;
}
