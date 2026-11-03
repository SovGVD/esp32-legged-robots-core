import type { TofMount } from './config.ts';

export type Vec = [number, number, number];

/**
 * Unit ray of one zone of an 8x8 sensor in the body frame (x right,
 * y forward, z up). Same math as the firmware's TerrainMap::zoneDirection:
 * row 0 looks highest, col 0 leftmost; tilt down about X, then yaw about Z.
 */
export function zoneDirection(m: TofMount, zone: number, side = 8): Vec {
  const row = Math.floor(zone / side);
  const col = zone % side;
  const h = ((col + 0.5) / side - 0.5) * m.fov;
  const v = (0.5 - (row + 0.5) / side) * m.fov;
  let x = Math.tan(h);
  let y = 1;
  let z = Math.tan(v);
  const len = Math.hypot(x, y, z);
  x /= len;
  y /= len;
  z /= len;
  const st = Math.sin(m.tilt);
  const ct = Math.cos(m.tilt);
  const ty = y * ct + z * st;
  const tz = -y * st + z * ct;
  const sy = Math.sin(m.yaw);
  const cy = Math.cos(m.yaw);
  return [x * cy - ty * sy, x * sy + ty * cy, tz];
}

/** Where a zone's target is, body frame, mm; null for no target. */
export function zonePoint(m: TofMount, zone: number, mm: number): Vec | null {
  if (mm <= 0) return null;
  const d = zoneDirection(m, zone);
  return [m.x + d[0] * mm, m.y + d[1] * mm, m.z + d[2] * mm];
}
