import { decode, type Incoming } from './protocol.ts';

export type ConnectionState = 'connecting' | 'online' | 'offline' | 'replaced';

export interface ConnectionEvents {
  onState(state: ConnectionState, detail?: string): void;
  onMessage(msg: Incoming): void;
}

const RECONNECT_MIN_MS = 1000;
const RECONNECT_MAX_MS = 5000;
/** Close reason the firmware uses when a newer client takes over. */
const REPLACED_REASON = 'Another client took control';

/** WebSocket to the robot with automatic reconnect. */
export class Connection {
  private ws: WebSocket | null = null;
  private retryMs = RECONNECT_MIN_MS;
  private retryTimer: ReturnType<typeof setTimeout> | undefined;
  state: ConnectionState = 'offline';

  private readonly url: string;
  private readonly events: ConnectionEvents;

  constructor(url: string, events: ConnectionEvents) {
    this.url = url;
    this.events = events;
  }

  get online(): boolean {
    return this.state === 'online';
  }

  connect(): void {
    clearTimeout(this.retryTimer);
    this.ws?.close();
    this.setState('connecting');

    let ws: WebSocket;
    try {
      ws = new WebSocket(this.url);
    } catch (e) {
      this.setState('offline', String(e));
      return;
    }
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    ws.onopen = () => {
      this.retryMs = RECONNECT_MIN_MS;
      this.setState('online');
    };
    ws.onmessage = (e: MessageEvent) => {
      // the firmware greets with a text "Ok"; everything else is binary
      if (!(e.data instanceof ArrayBuffer)) return;
      const msg = decode(new Uint8Array(e.data));
      if (msg) this.events.onMessage(msg);
    };
    ws.onclose = (e: CloseEvent) => {
      if (this.ws !== ws) return; // superseded by a newer connect()
      this.ws = null;
      if (e.reason === REPLACED_REASON) {
        // Reconnecting would steal control back and the two clients would
        // fight over the robot. Wait for the user instead.
        this.setState('replaced', e.reason);
        return;
      }
      this.setState('offline');
      this.retryTimer = setTimeout(() => this.connect(), this.retryMs);
      this.retryMs = Math.min(this.retryMs * 2, RECONNECT_MAX_MS);
    };
  }

  send(data: Uint8Array): boolean {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
    this.ws.send(data);
    return true;
  }

  private setState(state: ConnectionState, detail?: string): void {
    this.state = state;
    this.events.onState(state, detail);
  }
}
