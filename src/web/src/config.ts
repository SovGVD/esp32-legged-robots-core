/**
 * Robot configuration. The firmware serves it as /c.js, a classic script
 * that defines a global `c` before this module runs:
 *   const c={v:'0.0.1',w:'ws://192.168.4.1/ws',c:0};
 */
interface FirmwareConfig {
  v?: string; // web server version
  w?: string; // websocket url
  c?: number; // camera frame size, 0 = no camera
}

declare const c: FirmwareConfig | undefined;

export interface Config {
  version: string;
  wsUrl: string;
  camera: number;
}

export function loadConfig(): Config {
  const fw: FirmwareConfig = typeof c === 'object' && c ? c : {};
  // ?ws=ws://host/ws overrides the robot address (handy during development)
  const override = new URLSearchParams(location.search).get('ws');
  return {
    version: fw.v ?? '?',
    wsUrl: override || fw.w || `ws://${location.host}/ws`,
    camera: fw.c ?? 0,
  };
}
