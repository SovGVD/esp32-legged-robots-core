#include "robot.h"

static int bytesToInt16(uint8_t byte1, uint8_t byte2);

static int bytesToInt16(uint8_t byte1, uint8_t byte2) {
  return (byte1 << 8) | byte2;
}

void pMove(uint8_t* data, size_t len) {
  if (len < 14) {
    return;   // malformed package
  }
  if (data[1] == 0) {
    // client says "disabled": stop, and ignore the rest of the package
    setFailsafe();
    return;
  }

  LR_moveVector command;
  command.move.x       = constrain(float(bytesToInt16( data[2],  data[3]))/10000-1, -1.0f, 1.0f);
  command.move.y       = constrain(float(bytesToInt16( data[4],  data[5]))/10000-1, -1.0f, 1.0f);
  command.move.z       = constrain(float(bytesToInt16( data[6],  data[7]))/10000-1, -1.0f, 1.0f);
  command.rotate.pitch = constrain(float(bytesToInt16( data[8],  data[9]))/10000-1, -1.0f, 1.0f);
  command.rotate.roll  = constrain(float(bytesToInt16(data[10], data[11]))/10000-1, -1.0f, 1.0f);
  command.rotate.yaw   = constrain(float(bytesToInt16(data[12], data[13]))/10000-1, -1.0f, 1.0f);
  setMoveCommand(command);
  failsafeFeed();
}

static uint16_t sat16(uint32_t v)
{
	return v > 0xFFFF ? 0xFFFF : (uint16_t)v;
}

void pTelemetry() {
  /**
   * Telemetry package map (big-endian), version 3
   * offset | type | length | description
   * -------|------|--------|-------------
   *      0 | char |      1 | 'T' (P_TELEMETRY)
   *      1 | u8   |      1 | telemetry version (P_TELEMETRY_VERSION)
   *      2 | bin  |      2 | status bits, TS_* in def.h
   *      4 | u16  |      2 | voltage*1000
   *      6 | i16  |      2 | current*1000
   *      8 | u16  |      2 | motion loop time in microseconds
   *     10 | i16  |      2 | IMU pitch (sideways lean), deg*100
   *     12 | i16  |      2 | IMU roll (nose up/down), deg*100
   *     14 | u16  |      2 | missed motion ticks since boot
   *     16 | u16  |      2 | IK: foot targets out of reach since boot
   *     18 | u16  |      2 | IK: joint limit clamps since boot
   *     20 | u16  |      2 | IK: servo range clamps since boot
   *     22 | u16  |      2 | heading, deg*10 clockwise from magnetic north (0xFFFF: none)
   *     24 | u16  |      2 | heading hold target, deg*10 (0xFFFF: none)
   * Counters stop at 65535. Version 3 added 22-25 and the TS_TERRAIN.. bits.
   */
  static uint32_t lastOverruns = 0;
  uint32_t overruns = motionOverrunCount();
  MotionStatus m = motionStatus();

  uint16_t st = 0;
  if (isHALEnabled())       st |= TS_HAL;
  if (FS_FAIL)              st |= TS_FAILSAFE;
  if (m.gaitActive)         st |= TS_GAIT_ACTIVE;
  if (m.staticGait)         st |= TS_GAIT_STATIC;
  if (levelingEnabled())    st |= TS_LEVELING;
  if (imu().ready())        st |= TS_IMU;
  if (powerSensor().ready()) st |= TS_POWER;
  st |= ((uint16_t)batteryState() & 0x3) << TS_BATTERY_SHIFT;
  if (m.sitting)            st |= TS_SITTING;
  if (overruns != lastOverruns) st |= TS_OVERRUN;   // since the previous telemetry request (1 s)
  if (m.terrain)            st |= TS_TERRAIN;
  if (m.gapBlocked)         st |= TS_GAP_BLOCKED;
  if (m.headingHold)        st |= TS_HEADING_HOLD;
  if (imu().headingAbsolute()) st |= TS_HEADING_ABS;
  if (tofReady(0))          st |= TS_TOF;
  float heading = 0;
  bool  haveHeading = imu().heading(heading);
  lastOverruns = overruns;

  telemetryPackage[0] = P_TELEMETRY;
  telemetryPackage[1] = P_TELEMETRY_VERSION;
  int16ToBytes(st, 2);
  int16ToBytes(getPowerSensorVoltage()*1000, 4);
  int16ToBytes(getPowerSensorCurrent()*1000, 6);
  int16ToBytes(sat16(loopTime), 8);
  int16ToBytes((int)lroundf(radToDeg(IMU_DATA.pitch) * 100), 10);
  int16ToBytes((int)lroundf(radToDeg(IMU_DATA.roll)  * 100), 12);
  int16ToBytes(sat16(overruns), 14);
  int16ToBytes(sat16(m.unreachable), 16);
  int16ToBytes(sat16(m.jointLimit), 18);
  int16ToBytes(sat16(m.servoRange), 20);
  int16ToBytes(haveHeading ? (int)lroundf(heading * RADTODEG * 10) % 3600 : 0xFFFF, 22);
  int16ToBytes(m.headingHold && m.headingTarget ? (int)lroundf(m.target * RADTODEG * 10) % 3600 : 0xFFFF, 24);
}

void pCli(uint8_t* data, size_t len) {
  for (size_t idx = 1; idx < len; idx++) {
    if (data[idx] == 0) {
      return;
    }
    cliSerial->input(data[idx]);
  }
}

/**
 * Gait toggle from the web client: [P_GAIT, walk]. Applied once the robot
 * stands still (requestGait); telemetry TS_GAIT_STATIC shows when it did.
 */
void pGait(uint8_t* data, size_t len) {
  if (len < 2 || MODEL_WALK_GAIT == nullptr) {
    return;
  }
  requestGait(data[1] ? *MODEL_WALK_GAIT : GAIT_PARAMS);
}

/**
 * Leveling toggle from the web client: [P_LEVEL, on]. Stays off without an
 * IMU, same as the `level on` CLI command; telemetry TS_LEVELING shows it.
 */
void pLevel(uint8_t* data, size_t len) {
  if (len < 2) {
    return;
  }
  if (data[1] && !imu().ready()) {
    return;
  }
  setLeveling(data[1] != 0);
}

/**
 * ToF zone distances for the web client, answer to [P_TOF]. The client asks a
 * few times a second only while its ToF view is open, so this costs nothing
 * otherwise; it runs in the web server task and only copies the newest frame
 * (tof.cpp), the motion task is not involved.
 *
 * offset | type   | length | description
 * -------|--------|--------|-------------
 *      0 | char   |      1 | 'D' (P_TOF)
 *      1 | u8     |      1 | sensor count n
 *      2 | per sensor, 130 bytes each:
 *        | u8     |      1 |   1 = running, 0 = starting / not found (zones then 0)
 *        | u8     |      1 |   frame number (low byte), changes with every new frame
 *        | u16 x64|    128 |   mm per zone, row 0 = top of the view, col 0 = left, 0 = no target
 * Big-endian, like telemetry.
 */
size_t pTof(uint8_t *out)
{
	uint8_t n = tofSensorCount();
	if (n > P_TOF_MAX_SENSORS) n = P_TOF_MAX_SENSORS;
	if (n == 0) {
		return 0;
	}
	out[0] = P_TOF;
	out[1] = n;
	uint8_t *p = out + 2;
	for (uint8_t i = 0; i < n; i++) {
		int16_t  mm[P_TOF_ZONES];
		uint32_t seq = 0;
		bool ok = tofPeekFrame(i, seq, mm);
		*p++ = ok ? 1 : 0;
		*p++ = seq & 0xFF;
		for (uint8_t z = 0; z < P_TOF_ZONES; z++) {
			uint16_t d = ok && mm[z] > 0 ? (uint16_t)mm[z] : 0;
			*p++ = d >> 8;
			*p++ = d & 0xFF;
		}
	}
	return p - out;
}

/**
 * Walk to a map point from the web map view: [P_GOTO, x hi, x lo,
 * y hi, y lo] (i16 mm, map frame, big-endian); [P_GOTO] alone stops;
 * [P_GOTO, 'S'] scans the room once (ToF room scans).
 */
void pGoto(uint8_t *data, size_t len)
{
  if (len == 2 && data[1] == 'S') {
    lidarScan();
    return;
  }
  if (len < 5) {
    lidarStop();
    return;
  }
  int16_t x = (int16_t)((data[1] << 8) | data[2]);
  int16_t y = (int16_t)((data[3] << 8) | data[4]);
  lidarGoto(x, y);
}

static uint8_t *putU16(uint8_t *p, int v)
{
  p[0] = (v >> 8) & 0xFF;
  p[1] = v & 0xFF;
  return p + 2;
}

/**
 * Settings tab state, answer to [P_SETUP]. The web client asks a few times a
 * second only while its Settings tab is open; changes go back as CLI commands
 * (`terrain on`, `heading cal`, `trim LF alpha 1.5`, ...), so the CLI stays
 * the one place that changes settings.
 *
 * offset | type   | length | description
 * -------|--------|--------|-------------
 *      0 | char   |      1 | 'S' (P_SETUP)
 *      1 | u8     |      1 | version (P_SETUP_VERSION)
 *      2 | u8     |      1 | has: 1 terrain mode, 2 foot switches, 4 magnetometer, 8 walk gait,
 *        |        |        |      16 IMU, 32 HAL on, 64 HIL build, 128 raw magnetometer reading below
 *      3 | u8     |      1 | on: 1 terrain, 2 terrain switch pending, 4 feet, 8 gait uses the feet,
 *        |        |        |     16 heading hold, 32 leveling, 64 walk gait, 128 last mag reading trusted
 *      4 | u8     |      1 | magnetometer calibration: 0 not run, 1 turning, 2 by hand, 3 saved, 4 failed, 5 stopped
 *      5 | u8     |      1 | calibration: turned so far, deg/4 (180 = two turns)
 *      6 | u8     |      1 | calibration: seconds since start
 *      7 | u8     |      1 | ToF running, bit per sensor
 *      8 | u16    |      2 | feet pressed, bit per leg
 *     10 | i16 x3 |      6 | raw magnetometer, body frame (x right, y forward, z up), uT*10
 *     16 | i16 x3 |      6 | magnetometer calibration offset, uT*10
 *     22 | u16 x3 |      6 | magnetometer calibration scale*1000
 *     28 | u16    |      2 | calibrated field strength, uT*10 (0: not calibrated)
 *     30 | u16    |      2 | field strength of the last reading, uT*10
 *     32 | u8     |      1 | legs n
 *     33 | u8     |      1 | joints per leg d (0: no servo trims, tracks)
 *     34 | per leg: u8 title length, title chars, d x i16 servo trim deg*100 (alpha, beta, gamma, ...)
 * Big-endian, like telemetry.
 */
size_t pSetup(uint8_t *out, size_t room)
{
  MotionStatus m = motionStatus();
  const HeadingEstimator *he = imu().headingEstimator();
  float raw[3] = { 0, 0, 0 }, offset[3] = { 0, 0, 0 }, scale[3] = { 1, 1, 1 }, calField = 0;
  bool haveRaw = he != nullptr && he->magRaw(raw);
  if (he != nullptr) {
    he->magCalibration(offset, scale, calField);
  }
  bool trusted = false;
  float field = imu().magField(trusted);

  uint8_t has = 0, on = 0;
  if (MODEL_TERRAIN.terrain != nullptr) has |= 1;
  if (footSwitchesFitted())             has |= 2;
  if (imu().hasMagnetometer())          has |= 4;
  if (MODEL_WALK_GAIT != nullptr)       has |= 8;
  if (imu().ready())                    has |= 16;
  if (isHALEnabled())                   has |= 32;
  #ifdef HIL
    has |= 64;
  #endif
  if (haveRaw)                          has |= 128;
  if (m.terrain)                        on |= 1;
  if (m.terrainPending)                 on |= 2;
  if (footSwitchesInUse())              on |= 4;
  if (motionUsesContacts())             on |= 8;
  if (m.headingHold)                    on |= 16;
  if (levelingEnabled())                on |= 32;
  if (m.staticGait)                     on |= 64;
  if (trusted)                          on |= 128;
  uint8_t tof = 0;
  for (uint8_t i = 0; i < tofSensorCount() && i < 8; i++) {
    if (tofReady(i)) tof |= 1 << i;
  }
  float turned = fabsf(m.calTurned) * RADTODEG / 4;
  auto u8 = [](float v) { return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v); };
  auto uT = [](float v) { return (int)constrain(lroundf(v * 10), -32767L, 32767L); };

  out[0] = P_SETUP;
  out[1] = P_SETUP_VERSION;
  out[2] = has;
  out[3] = on;
  out[4] = m.calState;
  out[5] = u8(turned);
  out[6] = u8(m.calTime);
  out[7] = tof;
  uint8_t *p = putU16(out + 8, footPressedBits() & 0xFFFF);
  for (int i = 0; i < 3; i++) p = putU16(p, uT(raw[i]));
  for (int i = 0; i < 3; i++) p = putU16(p, uT(offset[i]));
  for (int i = 0; i < 3; i++) p = putU16(p, (int)constrain(lroundf(scale[i] * 1000), 0L, 65535L));
  p = putU16(p, (int)constrain(lroundf(calField * 10), 0L, 65535L));
  p = putU16(p, (int)constrain(lroundf(field * 10), 0L, 65535L));

  #if PWM_CONTROLLER_TYPE == DCMOTOR
    const uint8_t dof = 0;   // tracks: no servos to trim
  #else
    const uint8_t dof = LEG_DOF;
  #endif
  *p++ = LEG_NUM;
  *p++ = dof;
  for (uint8_t i = 0; i < LEG_NUM; i++) {
    size_t len = strnlen(legs[i].id.title, 8);
    if ((size_t)(p - out) + 1 + len + dof * 2 > room) {
      break;
    }
    *p++ = len;
    memcpy(p, legs[i].id.title, len);
    p += len;
    if (dof == 0) {
      continue;
    }
    const LR_legAngle &t = legs[i].hal.trim;
    const float trims[6] = { t.alpha, t.beta, t.gamma, t.delta, t.epsilon, t.zeta };
    for (uint8_t j = 0; j < dof; j++) {
      p = putU16(p, (int)constrain(lroundf(radToDeg(trims[j]) * 100), -32767L, 32767L));
    }
  }
  return p - out;
}

void int16ToBytes(int value, uint8_t _offset)
{
  telemetryPackage[_offset]   = (value >> 8) & 0xFF;
  telemetryPackage[_offset+1] =       value  & 0xFF;
}
