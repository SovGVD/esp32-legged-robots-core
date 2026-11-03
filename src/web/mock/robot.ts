/**
 * Fake robot for `npm run dev`: speaks the firmware's websocket protocol on
 * /ws of the Vite dev server, so the UI can be developed without an ESP32.
 */
import type { Server as HttpServer } from 'node:http';
import { WebSocketServer, type WebSocket } from 'ws';
import { P_CLI, P_GAIT, P_LEVEL, P_MOVE, P_TELEMETRY, TS, decodeAxis } from '../src/protocol.ts';

const FAILSAFE_MS = 500;

function cliPackage(text: string): Buffer {
  return Buffer.concat([Buffer.of(P_CLI), Buffer.from(text, 'utf8')]);
}

function telemetryPackage(voltage: number, current: number, loopTime: number, extra = 0): Buffer {
  const b = Buffer.alloc(26);
  b[0] = P_TELEMETRY;
  b[1] = 3; // version
  b.writeUInt16BE(TS.hal | TS.imu | TS.power | (1 << 7) | extra, 2); // battery ok
  b.writeUInt16BE(Math.round(voltage * 1000), 4);
  b.writeInt16BE(Math.round(current * 1000), 6);
  b.writeUInt16BE(loopTime, 8);
  b.writeInt16BE(Math.round((Math.random() - 0.5) * 200), 10);
  b.writeInt16BE(Math.round((Math.random() - 0.5) * 200), 12);
  b.writeUInt16BE(Math.round(Math.random() * 3599), 22); // heading
  b.writeUInt16BE(0xffff, 24); // no hold target
  return b;
}

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

    let lastMove = 0;
    let walk = false;
    let level = false;
    let moving = false;
    const watchdog = setInterval(() => {
      if (moving && Date.now() - lastMove > FAILSAFE_MS) {
        moving = false;
        log('[mock robot] failsafe: no move packages');
      }
    }, 100);

    ws.on('message', (raw: Buffer, isBinary: boolean) => {
      if (!isBinary || raw.length === 0) return;
      switch (raw[0]) {
        case P_MOVE: {
          if (raw.length < 14) return log('[mock robot] short move package', raw.length);
          lastMove = Date.now();
          const axes = [2, 4, 6, 8, 10, 12].map((o) => decodeAxis(raw.readUInt16BE(o)).toFixed(2));
          const active = raw[1] === 1 && axes.some((a) => Number(a) !== 0);
          if (raw[1] === 0) log('[mock robot] client sent failsafe');
          else if (active) log(`[mock robot] move x=${axes[0]} y=${axes[1]} z=${axes[2]} pitch=${axes[3]} roll=${axes[4]} yaw=${axes[5]}`);
          moving = raw[1] === 1;
          break;
        }
        case P_TELEMETRY:
          ws.send(
            telemetryPackage(
              7.4 + Math.random() * 0.1,
              1.2 + Math.random() * 0.3,
              2000 + Math.round(Math.random() * 300),
              (walk ? TS.gaitStatic : 0) | (level ? TS.leveling : 0),
            ),
          );
          break;
        case P_GAIT:
          walk = raw[1] === 1;
          log(`[mock robot] gait ${walk ? 'walk' : 'trot'}`);
          break;
        case P_LEVEL:
          level = raw[1] === 1;
          log(`[mock robot] leveling ${level ? 'on' : 'off'}`);
          break;
        case P_CLI: {
          const end = raw.indexOf(0);
          const command = raw.subarray(1, end < 0 ? raw.length : end).toString('utf8').trim();
          const reply = command === 'help' ? 'Commands: help, ping\n' : command === 'ping' ? 'pong\n' : `Unknown command: ${command}\n`;
          ws.send(cliPackage(`> ${command}\n${reply}`));
          break;
        }
        default:
          log('[mock robot] unknown package', raw[0]);
      }
    });
    ws.on('close', () => {
      clearInterval(watchdog);
      if (controller === ws) controller = null;
      log('[mock robot] client disconnected');
    });
  });
}
