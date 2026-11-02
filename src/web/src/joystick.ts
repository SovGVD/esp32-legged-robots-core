export interface Stick {
  x: number; // -1 (left) .. 1 (right)
  y: number; // -1 (down) .. 1 (up)
}

/**
 * On-screen joystick driven by pointer events, so mouse, pen and several
 * fingers (one per stick) all work. The stick returns to the centre when
 * released or when the pointer is lost.
 */
export class Joystick {
  private pointerId: number | null = null;
  private readonly knob: HTMLElement;

  private readonly el: HTMLElement;
  private readonly onChange: (v: Stick) => void;
  private readonly deadband: number;

  constructor(el: HTMLElement, onChange: (v: Stick) => void, deadband = 0.05) {
    this.el = el;
    this.onChange = onChange;
    this.deadband = deadband;
    this.knob = document.createElement('div');
    this.knob.className = 'knob';
    el.append(this.knob);

    el.addEventListener('pointerdown', (e) => {
      if (this.pointerId !== null) return;
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
    const x = this.axis(((e.clientX - r.left) / r.width) * 2 - 1);
    const y = this.axis(((e.clientY - r.top) / r.height) * 2 - 1);
    this.show(x, y);
    this.onChange({ x, y: -y });
  }

  private axis(v: number): number {
    const c = Math.min(1, Math.max(-1, v));
    return Math.abs(c) <= this.deadband ? 0 : c;
  }

  private show(x: number, y: number): void {
    this.knob.style.left = `${(x + 1) * 50}%`;
    this.knob.style.top = `${(y + 1) * 50}%`;
  }
}
