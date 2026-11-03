/**
 * A checkbox that switches a robot setting. The robot's telemetry is the
 * truth: the box shows what telemetry reports, except right after a click,
 * while the robot hasn't confirmed it yet (the box is then marked pending).
 * If the robot never confirms (e.g. leveling without an IMU), the box falls
 * back to the reported state after `timeoutMs`.
 */
export interface ToggleInput {
  checked: boolean;
  disabled: boolean;
  classList: { toggle(token: string, force?: boolean): unknown };
  addEventListener(type: 'change', listener: () => void): void;
}

export class RemoteToggle {
  private pending: { value: boolean; at: number } | null = null;
  private reported: boolean | null = null;

  private readonly input: ToggleInput;
  private readonly now: () => number;
  private readonly timeoutMs: number;

  constructor(input: ToggleInput, send: (on: boolean) => void, timeoutMs: number, now: () => number = Date.now) {
    this.input = input;
    this.timeoutMs = timeoutMs;
    this.now = now;
    input.addEventListener('change', () => {
      this.pending = { value: input.checked, at: this.now() };
      send(input.checked);
      this.render();
    });
    this.sync(null);
  }

  /** `actual` from telemetry; null = offline / unknown. `enabled` = can be switched now. */
  sync(actual: boolean | null, enabled = true): void {
    this.reported = actual;
    if (actual === null) this.pending = null;
    else if (this.pending && (this.pending.value === actual || this.now() - this.pending.at > this.timeoutMs)) {
      this.pending = null;
    }
    this.input.disabled = actual === null || !enabled;
    this.render();
  }

  private render(): void {
    this.input.checked = this.pending ? this.pending.value : !!this.reported;
    this.input.classList.toggle('pending', this.pending !== null);
  }
}
