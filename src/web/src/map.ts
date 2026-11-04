import { NAV_STATES, type LidarMap, type NavInfo } from './protocol.ts';

// cell colours: unknown, free, seen once, occupied (RGBA)
const COLORS = [
  [0, 0, 0, 0],
  [58, 66, 82, 255],
  [140, 150, 168, 255],
  [236, 240, 245, 255],
];
const MATCH = ['', '', 'new area', 'lost? odometry only', 'few points'];
const FLAG_SCANNING = 16;
const FLAG_TOF = 32;

/**
 * Map view (lidar or ToF room scans): the firmware's occupancy map around the robot (north =
 * the direction the robot faced when the map started, up), the last scan,
 * the planned path and the goal. Tap the map to send the robot there.
 */
export class MapPanel {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly info: HTMLElement;
  private readonly image = document.createElement('canvas');
  private map: LidarMap | null = null;
  private nav: NavInfo | null = null;
  private pending: [number, number] | null = null;
  private scale = 0.08; // css px per mm (5 m across ~400 px)
  private down: [number, number] | null = null;
  private readonly scanButton: HTMLButtonElement;

  /**
   * Map view (lidar or ToF room scans).
   * @param onGoto walk to a map point (mm), no arguments: stop
   * @param onScan scan the room once (ToF room scans only)
   */
  constructor(host: HTMLElement, onGoto: (x?: number, y?: number) => void, onScan: () => void) {
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'map';
    this.ctx = this.canvas.getContext('2d')!;
    this.info = document.createElement('div');
    this.info.className = 'small';
    const bar = document.createElement('div');
    const button = (text: string, fn: () => void) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.textContent = text;
      b.addEventListener('click', fn);
      bar.append(b);
      return b;
    };
    button('−', () => this.zoom(1 / 1.5));
    button('+', () => this.zoom(1.5));
    button('Stop', () => {
      this.pending = null;
      onGoto();
    });
    this.scanButton = button('Scan', onScan);
    this.scanButton.title = 'Scan the room: the robot stops, lifts its nose and turns once around';
    this.scanButton.hidden = true;
    host.append(this.canvas, this.info, bar);
    this.canvas.addEventListener('pointerdown', (e) => (this.down = [e.offsetX, e.offsetY]));
    this.canvas.addEventListener('pointerup', (e) => {
      const d = this.down;
      this.down = null;
      if (!d || Math.hypot(e.offsetX - d[0], e.offsetY - d[1]) > 10 || !this.nav) return;
      const [cx, cy] = this.centre();
      const x = this.nav.x + (e.offsetX - cx) / this.scale;
      const y = this.nav.y - (e.offsetY - cy) / this.scale;
      this.pending = [x, y];
      onGoto(x, y);
      this.draw();
    });
  }

  clear(): void {
    this.map = null;
    this.nav = null;
    this.draw();
  }

  setMap(map: LidarMap): void {
    this.map = map;
    if (map.running) {
      this.image.width = this.image.height = map.n;
      const ictx = this.image.getContext('2d')!;
      const img = ictx.createImageData(map.n, map.n);
      for (let r = 0; r < map.n; r++) {
        for (let c = 0; c < map.n; c++) {
          img.data.set(COLORS[map.cells[r * map.n + c]], ((map.n - 1 - r) * map.n + c) * 4);
        }
      }
      ictx.putImageData(img, 0, 0);
    }
    this.draw();
  }

  setNav(nav: NavInfo): void {
    this.nav = nav;
    this.scanButton.hidden = !(nav.flags & FLAG_TOF);
    if (nav.state !== 0) this.pending = null;
    this.draw();
  }

  private zoom(f: number): void {
    this.scale = Math.min(0.5, Math.max(0.02, this.scale * f));
    this.draw();
  }

  private centre(): [number, number] {
    return [this.canvas.clientWidth / 2, this.canvas.clientHeight / 2];
  }

  draw(): void {
    const { canvas, ctx, map, nav } = this;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#1b2029';
    ctx.fillRect(0, 0, w, h);
    if (!map?.running || !nav) {
      const sensor = nav && nav.flags & FLAG_TOF ? 'ToF sensors' : 'Lidar';
      this.info.textContent = map && !map.running ? `${sensor}: no data yet (wiring?)` : 'Waiting for the map…';
      return;
    }
    const s = this.scale;
    const [cx, cy] = this.centre();
    const X = (x: number) => cx + (x - nav.x) * s;
    const Y = (y: number) => cy - (y - nav.y) * s;
    const half = (map.n / 2) * map.cell;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.image, X(-half), Y(half), 2 * half * s, 2 * half * s);

    if (nav.path.length > 1) {
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      nav.path.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))));
      ctx.stroke();
    }
    const goal = this.pending ?? (nav.state === 1 || nav.state === 3 ? nav.goal : null);
    if (goal) {
      ctx.strokeStyle = '#e879f9';
      ctx.lineWidth = 3;
      const [gx, gy] = [X(goal[0]), Y(goal[1])];
      ctx.beginPath();
      ctx.moveTo(gx - 7, gy - 7);
      ctx.lineTo(gx + 7, gy + 7);
      ctx.moveTo(gx - 7, gy + 7);
      ctx.lineTo(gx + 7, gy - 7);
      ctx.stroke();
    }
    // scan points: robot frame -> map
    const sn = Math.sin(nav.heading);
    const cs = Math.cos(nav.heading);
    ctx.fillStyle = '#ef4444';
    for (const [bx, by] of nav.scan) {
      ctx.fillRect(X(nav.x + bx * cs - by * sn) - 1.5, Y(nav.y + bx * sn + by * cs) - 1.5, 3, 3);
    }
    // robot: a triangle pointing where it faces
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-nav.heading);
    ctx.fillStyle = '#38bdf8';
    ctx.beginPath();
    ctx.moveTo(0, -12);
    ctx.lineTo(8, 9);
    ctx.lineTo(-8, 9);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    const parts = [`${(nav.x / 1000).toFixed(2)}, ${(nav.y / 1000).toFixed(2)} m`, NAV_STATES[nav.state] ?? '?'];
    if (nav.flags & FLAG_SCANNING) parts.push('scanning the room');
    if (nav.flags & 4) parts.push('blocked ahead');
    if (nav.flags & 2) parts.push('map frozen');
    if (!(nav.flags & 1)) parts.push(nav.flags & FLAG_TOF ? 'NO TOF DATA' : 'NO LIDAR DATA');
    if (MATCH[nav.match]) parts.push(MATCH[nav.match]);
    this.info.textContent = parts.join(' · ') + ' — tap to walk there';
  }
}
