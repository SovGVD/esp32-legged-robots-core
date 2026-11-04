/**
 * Robot configuration. The firmware serves it as /c.js, a classic script
 * that defines a global `c` before this module runs:
 *   const c={v:'0.0.1',w:'ws://192.168.4.1/ws',c:0,g:1,l:1,t:1,r:400,m:[[0,88,0,0,0.7,0.785]]};
 */
interface FirmwareConfig {
  v?: string; // web server version
  w?: string; // websocket url
  c?: number; // camera frame size, 0 = no camera
  g?: number; // 1 = gait can be switched (trot/walk)
  l?: number; // 1 = IMU leveling available
  t?: number; // number of ToF sensors, 0 = none
  r?: number; // ToF range, mm (colour scale)
  n?: number; // 1 = map and navigation (lidar or ToF room scans)
  m?: number[][]; // ToF mounting per sensor: [x, y, z (mm, body frame), yaw, tilt, fov (rad)]
}

/** Where a ToF sensor sits on the body (firmware TofSensor). x right, y forward, z up. */
export interface TofMount {
  x: number;
  y: number;
  z: number;
  yaw: number; // rad, + = left
  tilt: number; // rad, + = down
  fov: number; // rad, full field of view
}

declare const c: FirmwareConfig | undefined;

export interface Config {
  version: string;
  wsUrl: string;
  camera: number;
  gaitToggle: boolean;
  levelToggle: boolean;
  tofSensors: number;
  tofRange: number;
  tofMounts: TofMount[];
  lidar: boolean;
}

export function loadConfig(): Config {
  const fw: FirmwareConfig = typeof c === 'object' && c ? c : {};
  // ?ws=ws://host/ws overrides the robot address (handy during development)
  const override = new URLSearchParams(location.search).get('ws');
  return {
    version: fw.v ?? '?',
    wsUrl: override || fw.w || `ws://${location.host}/ws`,
    camera: fw.c ?? 0,
    gaitToggle: fw.g === 1,
    levelToggle: fw.l === 1,
    tofSensors: fw.t ?? 0,
    tofRange: fw.r || 400,
    lidar: fw.n === 1,
    tofMounts: (fw.m ?? []).map(([x, y, z, yaw, tilt, fov]) => ({ x, y, z, yaw, tilt, fov })),
  };
}
