export interface Stick {
  x: number; // -1 (left) .. 1 (right)
  y: number; // -1 (down) .. 1 (up)
}

/**
 * Round on-screen joystick, like a game's thumb stick: the element is the
 * ring, the knob follows the finger and the value is the knob's offset from
 * the centre over the ring's radius, clamped to the circle. Pointer events,
 * so mouse, pen and several fingers (one per stick) all work. The stick
 * returns to the centre when released or when the pointer is lost.
 */
export class Joystick {
  private pointerId: number | null = null;
  private readonly knob: HTMLElement;

  private readonly el: HTMLElement;
  private readonly onChange: (v: Stick) => void;
  private readonly deadband: number;

  constructor(el: HTMLElement, onChange: (v: Stick) => void, deadband = 0.08) {
    this.el = el;
    this.onChange = onChange;
    this.deadband = deadband;
    this.knob = document.createElement('div');
    this.knob.className = 'knob';
    el.append(this.knob);

    el.addEventListener('pointerdown', (e) => {
      if (this.pointerId !== null) return;
      e.preventDefault();
      this.pointerId = e.pointerId;
      el.setPointerCapture(e.pointerId);
      el.classList.add('active');
      this.move(e);
    });
    el.addEventListener('pointermove', (e) => {
      if (e.pointerId === this.pointerId) this.move(e);
    });
    const end = (e: PointerEvent) => {
      if (e.pointerId === this.pointerId) this.release();
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
    el.addEventListener('lostpointercapture', end);
  }

  release(): void {
    this.pointerId = null;
    this.el.classList.remove('active');
    this.show(0, 0);
    this.onChange({ x: 0, y: 0 });
  }

  private move(e: PointerEvent): void {
    const r = this.el.getBoundingClientRect();
    const radius = r.width / 2;
    let x = (e.clientX - r.left - radius) / radius;
    let y = (e.clientY - r.top - r.height / 2) / radius;
    const len = Math.hypot(x, y);
    if (len > 1) {
      x /= len;
      y /= len;
    }
    // the knob follows the finger, the value has a dead zone around the centre
    this.show(x, y);
    const k = len <= this.deadband ? 0 : Math.min(1, (Math.min(1, len) - this.deadband) / (1 - this.deadband)) / Math.min(1, len);
    this.onChange({ x: round(x * k), y: round(-y * k) });
  }

  private show(x: number, y: number): void {
    this.knob.style.transform = `translate(${x * 75}%, ${y * 75}%)`;
  }
}

const round = (v: number) => Math.round(v * 1000) / 1000;
