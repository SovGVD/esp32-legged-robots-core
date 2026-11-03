/**
 * Multizone ToF sensors (VL53L5CX, 8x8 zones) for the terrain map.
 *
 * A low priority task on core 0 starts the sensors (each one needs ~90 KB of
 * firmware uploaded over I2C at power on, a second or two) and reads their
 * frames at 15 Hz. The motion task takes the newest frame of each sensor
 * (tofTakeFrame) and puts it into the terrain map with the body pose of that
 * tick, so the map is only ever touched by the motion task.
 *
 * Mounting and zone order: MODEL_TERRAIN.tof (model_data.h TOF_SENSORS) says
 * where each sensor sits and where it looks. Check the zone order once with
 * `tof print`: hold a hand in front of the left half / the top half of a
 * sensor and see which zones get short. Fix it with TOF_ZONE_MIRROR_X /
 * TOF_ZONE_MIRROR_Y in config.h.
 *
 * I2C: by default the sensors share the main bus with the IMU (and the
 * PCA9685s on the hexapod); reads are split into 32 byte transfers so a
 * frame never holds the bus for more than ~1 ms. TOF_I2C_SDA / TOF_I2C_SCL
 * put them on the second I2C controller (Wire1, 1 MHz) instead.
 *
 * Two sensors (hexapod): both start at the default address 0x29. Sensor 0's
 * LPn is tied high, sensor 1's LPn goes to TOF_LPN_PIN: the firmware holds
 * sensor 1 off, moves sensor 0 to 0x2A, then lets sensor 1 start at 0x29.
 */
#include "robot.h"

#if defined(TOF_TYPE) && TOF_TYPE == VL53L5CX

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#ifndef TOF_ZONE_MIRROR_X
	#define TOF_ZONE_MIRROR_X 1   // the ST driver lists the zones mirrored left/right (SparkFun example 1)
#endif
#ifndef TOF_ZONE_MIRROR_Y
	#define TOF_ZONE_MIRROR_Y 0
#endif
#ifndef TOF_DISTANCE_PERPENDICULAR
	// 1 if the sensor reports the distance along its axis, not along each zone's
	// ray: facing a flat wall, the corner zones then read the same as the middle
	#define TOF_DISTANCE_PERPENDICULAR 0
#endif
#ifndef TOF_RANGING_HZ
	#define TOF_RANGING_HZ 15     // 8x8 runs at most at 15 Hz
#endif
#define TOF_MAX_SENSORS 2
#define TOF_ZONES       64

struct TofFrame {
	int16_t  mm[TOF_ZONES];   // terrain map zone order (TofSensor), 0 = no target
	uint32_t seq;
};

static SparkFun_VL53L5CX     sensors[TOF_MAX_SENSORS];
static VL53L5CX_ResultsData  results;
static TofFrame              frames[TOF_MAX_SENSORS];
static bool                  sensorOk[TOF_MAX_SENSORS] = { false, false };
static volatile bool         started = false;
static portMUX_TYPE          frameLock = portMUX_INITIALIZER_UNLOCKED;
static float                 rayScale[TOF_ZONES];   // perpendicular -> along the ray

static uint8_t sensorCount()
{
	return MODEL_TERRAIN.tofCount < TOF_MAX_SENSORS ? MODEL_TERRAIN.tofCount : TOF_MAX_SENSORS;
}

static TwoWire &tofBus()
{
#if defined(TOF_I2C_SDA) && defined(TOF_I2C_SCL)
	return Wire1;
#else
	return Wire;
#endif
}

// sensor zone (driver order) -> terrain map zone (row 0 at the top, col 0 on the left)
static uint8_t mapZone(uint8_t z)
{
	uint8_t row = z / 8, col = z % 8;
	if (TOF_ZONE_MIRROR_X) col = 7 - col;
	if (TOF_ZONE_MIRROR_Y) row = 7 - row;
	return row * 8 + col;
}

static bool startSensor(uint8_t i, uint8_t address)
{
	SparkFun_VL53L5CX &s = sensors[i];
	Serial.printf("ToF %u: ", i);
	if (!s.begin(address, tofBus())) {
		Serial.println("not found");
		return false;
	}
	s.setWireMaxPacketSize(32);   // short transfers: the motion task's IMU read waits at most ~1 ms
	if (!s.setResolution(TOF_ZONES) || !s.setRangingFrequency(TOF_RANGING_HZ) || !s.startRanging()) {
		Serial.println("setup failed");
		return false;
	}
	Serial.printf("VL53L5CX at 0x%02X, 8x8 at %u Hz\n", s.getAddress(), TOF_RANGING_HZ);
	return true;
}

static void startSensors()
{
	const uint8_t defaultAddress = 0x29;
#if defined(TOF_I2C_SDA) && defined(TOF_I2C_SCL)
	Wire1.begin(TOF_I2C_SDA, TOF_I2C_SCL, 1000000);
	Wire1.setTimeOut(20);
#endif
	if (sensorCount() > 1) {
	#ifdef TOF_LPN_PIN
		pinMode(TOF_LPN_PIN, OUTPUT);
		digitalWrite(TOF_LPN_PIN, LOW);   // sensor 1 off while sensor 0 moves to 0x2A
		vTaskDelay(pdMS_TO_TICKS(10));
		// after a reset without a power cycle sensor 0 may still be at 0x2A
		tofBus().beginTransmission(0x2A);
		bool moved = tofBus().endTransmission() == 0;
		sensorOk[0] = moved ? startSensor(0, 0x2A)
		                    : (startSensor(0, defaultAddress) && sensors[0].setAddress(0x2A));
		digitalWrite(TOF_LPN_PIN, HIGH);
		vTaskDelay(pdMS_TO_TICKS(10));
		sensorOk[1] = startSensor(1, defaultAddress);
	#else
		Serial.println("ToF: two sensors need TOF_LPN_PIN (config.h), using sensor 0 only");
		sensorOk[0] = startSensor(0, defaultAddress);
	#endif
	} else if (sensorCount() == 1) {
		sensorOk[0] = startSensor(0, defaultAddress);
	}
}

static void tofTask(void *)
{
	// zone ray scale for sensors that report perpendicular distances
	if (sensorCount() > 0) {
		TofSensor axis = MODEL_TERRAIN.tof[0];
		axis.yaw = 0;
		axis.tilt = 0;
		for (uint8_t z = 0; z < TOF_ZONES; z++) {
			LR_point d = TerrainMap::zoneDirection(axis, z);
			rayScale[z] = TOF_DISTANCE_PERPENDICULAR && d.y > 0.1f ? 1.0f / d.y : 1.0f;
		}
	}
	startSensors();
	started = true;
	for (;;) {
		for (uint8_t i = 0; i < sensorCount(); i++) {
			if (!sensorOk[i] || !sensors[i].isDataReady() || !sensors[i].getRangingData(&results)) {
				continue;
			}
			TofFrame f;
			for (uint8_t z = 0; z < TOF_ZONES; z++) {
				uint8_t st = results.target_status[z];
				bool valid = st == 5 || st == 9;   // 100 % valid, or valid with a large pulse (ST UM2884)
				uint8_t m = mapZone(z);
				f.mm[m] = valid ? (int16_t)(results.distance_mm[z] * rayScale[m]) : 0;
			}
			portENTER_CRITICAL(&frameLock);
			f.seq = frames[i].seq + 1;
			frames[i] = f;
			portEXIT_CRITICAL(&frameLock);
		}
		vTaskDelay(pdMS_TO_TICKS(8));
	}
}

void initTof()
{
	if (sensorCount() == 0) {
		return;
	}
	// core 0, below the services loop: the firmware upload takes a while
	xTaskCreatePinnedToCore(tofTask, "tof", 6144, nullptr, 0, nullptr, 0);
}

uint8_t tofSensorCount()
{
	return sensorCount();
}

bool tofReady(uint8_t i)
{
	return started && i < sensorCount() && sensorOk[i];
}

bool tofTakeFrame(uint8_t i, uint32_t &seq, float *mm)
{
	if (i >= sensorCount()) {
		return false;
	}
	portENTER_CRITICAL(&frameLock);
	bool fresh = frames[i].seq != seq;
	if (fresh) {
		seq = frames[i].seq;
		for (uint8_t z = 0; z < TOF_ZONES; z++) {
			mm[z] = frames[i].mm[z];
		}
	}
	portEXIT_CRITICAL(&frameLock);
	return fresh;
}

void cliTof()
{
	char *mode = CLI_readWord();
	if (sensorCount() == 0) {
		cliSerial->println("This model has no ToF sensors.");
		return;
	}
	for (uint8_t i = 0; i < sensorCount(); i++) {
		cliSerial->printf("ToF %u: %s\n", i, !started ? "starting" : (sensorOk[i] ? "ok" : "not found"));
		if (strcmp(mode, "print") != 0 || !sensorOk[i]) {
			continue;
		}
		uint32_t seq = 0xFFFFFFFF;   // any frame, even one already taken
		float mm[TOF_ZONES] = {};
		tofTakeFrame(i, seq, mm);
		cliSerial->println("  mm, as the sensor looks (row 0 = top, left = left), - = no target");
		for (uint8_t r = 0; r < 8; r++) {
			cliSerial->print("  ");
			for (uint8_t c = 0; c < 8; c++) {
				float d = mm[r * 8 + c];
				if (d > 0) {
					cliSerial->printf("%5.0f", d);
				} else {
					cliSerial->print("    -");
				}
			}
			cliSerial->println();
		}
	}
}

#else

void initTof() {}
uint8_t tofSensorCount() { return 0; }
bool tofReady(uint8_t i) { (void)i; return false; }
bool tofTakeFrame(uint8_t i, uint32_t &seq, float *mm) { (void)i; (void)seq; (void)mm; return false; }
void cliTof() { cliSerial->println("No ToF sensors (TOF_TYPE in config.h)."); }

#endif
