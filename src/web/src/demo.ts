/**
 * Demo build only (npm run build:demo): a stand-in for the robot's websocket
 * that talks to the mock robot inside the page, so the UI can be tried
 * without a robot or a dev server. #dog (1 ToF) or #hexapod (2 ToF, default).
 */
import { MockRobot, cliPackage, mockConfig } from '../mock/sim.ts';

const robotKind = () => (location.hash === '#dog' ? 'dog' : 'hexapod');

// a new robot needs a fresh page (config, ToF views)
addEventListener('hashchange', () => location.reload());

/** The config the firmware would serve as /c.js. */
export function installDemoConfig(): void {
  (globalThis as { c?: unknown }).c = { ...mockConfig(robotKind() === 'dog' ? 1 : 2, 'demo'), v: 'demo' };
}

/** Just enough of WebSocket for Connection. */
export class DemoSocket {
  static readonly OPEN = 1;
  readyState = 0;
  binaryType = 'arraybuffer';
  onopen: (() => void) | null = null;
  onmessage: ((e: { data: unknown }) => void) | null = null;
  onclose: ((e: { reason: string }) => void) | null = null;
  private readonly robot = new MockRobot({ tofCount: robotKind() === 'dog' ? 1 : 2, tofNav: true });

  constructor() {
    setTimeout(() => {
      this.readyState = 1;
      this.onopen?.();
      this.deliver(cliPackage(`Demo: a mock ${robotKind()} runs in this page. Try "help".\n`));
    }, 300);
  }

  send(data: Uint8Array): void {
    for (const reply of this.robot.receive(data)) this.deliver(reply);
  }

  close(): void {
    this.readyState = 3;
  }

  private deliver(b: Uint8Array): void {
    // like a network: answer on a later task, as an ArrayBuffer copy
    const copy = b.slice().buffer;
    setTimeout(() => this.readyState === 1 && this.onmessage?.({ data: copy }), 5);
  }
}
