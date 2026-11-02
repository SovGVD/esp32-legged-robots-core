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
   * Telemetry package map (big-endian), version 2
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
   * Counters stop at 65535.
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
}

void pCli(uint8_t* data, size_t len) {
  for (size_t idx = 1; idx < len; idx++) {
    if (data[idx] == 0) {
      return;
    }
    cliSerial->input(data[idx]);
  }
}

void int16ToBytes(int value, uint8_t _offset)
{
  telemetryPackage[_offset]   = (value >> 8) & 0xFF;
  telemetryPackage[_offset+1] =       value  & 0xFF;
}
