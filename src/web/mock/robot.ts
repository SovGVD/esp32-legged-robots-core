/**
 * Fake robot for `npm run dev`: the mock (mock/sim.ts) behind a real
 * websocket on /ws of the Vite dev server, so the UI can be developed
 * without an ESP32.
 */
import type { Server as HttpServer } from 'node:http';
import { WebSocketServer, type WebSocket } from 'ws';
import { MockRobot, cliPackage } from './sim.ts';

export function attachMockRobot(httpServer: HttpServer, log = console.log): void {
  const wss = new WebSocketServer({ noServer: true });
  let controller: WebSocket | null = null;

  httpServer.on('upgrade', (req, socket, head) => {
    if (new URL(req.url ?? '/', 'http://x').pathname !== '/ws') return; // leave Vite's HMR socket alone
    wss.handleUpgrade(req, socket, head, (ws) => wss.emit('connection', ws));
  });

  wss.on('connection', (ws) => {
    // like the firmware: the newest client takes over
    controller?.close(1000, 'Another client took control');
    controller = ws;
    log('[mock robot] client connected');
    ws.send('Ok');
    ws.send(cliPackage('Mock robot ready. Try "help".\n'));
    const robot = new MockRobot({ tofCount: Number(process.env.TOF ?? 2), tofNav: process.env.TOF_NAV !== '0', log });

    ws.on('message', (raw: Buffer, isBinary: boolean) => {
      if (!isBinary) return;
      for (const reply of robot.receive(new Uint8Array(raw.buffer, raw.byteOffset, raw.byteLength))) ws.send(reply);
    });
    ws.on('close', () => {
      if (controller === ws) controller = null;
      log('[mock robot] client disconnected');
    });
  });
}
