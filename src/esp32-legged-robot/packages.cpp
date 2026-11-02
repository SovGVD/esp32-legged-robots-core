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

void pTelemetry() {
  /**
   * Telemetry package map
   * offset | type | length | description
   * -------|------|--------|-------------
   *      0 | char |      1 | 'T' (P_TELEMETRY)
   *      1 |      |      1 | add something
   *      2 | bin  |      2 | 16 statuses
   *      4 | int  |      2 | voltage*1000
   *      6 | int  |      2 | current*1000
   *      8 | int  |      2 | loop time in microseconds
   */
  telemetryPackage[0] = P_TELEMETRY;
  // TODO statuses
  int16ToBytes(getPowerSensorVoltage()*1000, 4);
  int16ToBytes(getPowerSensorCurrent()*1000, 6);
  int16ToBytes(loopTime, 8);
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
