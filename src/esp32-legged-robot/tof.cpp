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

static TofFrame     frames[TOF_MAX_SENSORS];
static portMUX_TYPE frameLock = portMUX_INITIALIZER_UNLOCKED;

static uint8_t sensorCount()
{
	return MODEL_TERRAIN.tofCount < TOF_MAX_SENSORS ? MODEL_TERRAIN.tofCount : TOF_MAX_SENSORS;
}

// TofFrameSink: a new frame from the driver (its task, or the HIL link)
static void publishFrame(uint8_t i, const int16_t *mm)
{
	if (i >= sensorCount()) {
		return;
	}
	portENTER_CRITICAL(&frameLock);
	memcpy(frames[i].mm, mm, sizeof(frames[i].mm));
	frames[i].seq++;
	portEXIT_CRITICAL(&frameLock);
}

#ifndef HIL   // the HIL build gets its frames from the simulator (hil.cpp)

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

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

class Vl53l5cxArray : public TofArray {
	public:
		const char *name() const override { return "VL53L5CX"; }

		void begin(uint8_t count, TofFrameSink sink) override
		{
			_count = count;
			_sink  = sink;
			if (count == 0) {
				return;
			}
			// core 0, below the services loop: the firmware upload takes a while
			xTaskCreatePinnedToCore(task, "tof", 6144, this, 0, nullptr, 0);
		}

		bool started() const override { return _started; }
		bool ok(uint8_t i) const override { return i < _count && _ok[i]; }

	private:
		static void task(void *arg) { static_cast<Vl53l5cxArray *>(arg)->run(); }

		bool startSensor(uint8_t i, uint8_t address)
		{
			SparkFun_VL53L5CX &s = _sensors[i];
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

		void startSensors()
		{
			const uint8_t defaultAddress = 0x29;
		#if defined(TOF_I2C_SDA) && defined(TOF_I2C_SCL)
			Wire1.begin(TOF_I2C_SDA, TOF_I2C_SCL, 1000000);
			Wire1.setTimeOut(20);
		#endif
			if (_count > 1) {
			#ifdef TOF_LPN_PIN
				pinMode(TOF_LPN_PIN, OUTPUT);
				digitalWrite(TOF_LPN_PIN, LOW);   // sensor 1 off while sensor 0 moves to 0x2A
				vTaskDelay(pdMS_TO_TICKS(10));
				// after a reset without a power cycle sensor 0 may still be at 0x2A
				tofBus().beginTransmission(0x2A);
				bool moved = tofBus().endTransmission() == 0;
				_ok[0] = moved ? startSensor(0, 0x2A)
				               : (startSensor(0, defaultAddress) && _sensors[0].setAddress(0x2A));
				digitalWrite(TOF_LPN_PIN, HIGH);
				vTaskDelay(pdMS_TO_TICKS(10));
				_ok[1] = startSensor(1, defaultAddress);
			#else
				Serial.println("ToF: two sensors need TOF_LPN_PIN (config.h), using sensor 0 only");
				_ok[0] = startSensor(0, defaultAddress);
			#endif
			} else if (_count == 1) {
				_ok[0] = startSensor(0, defaultAddress);
			}
		}

		void run()
		{
			// zone ray scale for sensors that report perpendicular distances
			TofSensor axis = MODEL_TERRAIN.tof[0];
			axis.yaw = 0;
			axis.tilt = 0;
			for (uint8_t z = 0; z < TOF_ZONES; z++) {
				LR_point d = TerrainMap::zoneDirection(axis, z);
				_rayScale[z] = TOF_DISTANCE_PERPENDICULAR && d.y > 0.1f ? 1.0f / d.y : 1.0f;
			}
			startSensors();
			_started = true;
			for (;;) {
				for (uint8_t i = 0; i < _count; i++) {
					if (!_ok[i] || !_sensors[i].isDataReady() || !_sensors[i].getRangingData(&_results)) {
						continue;
					}
					int16_t mm[TOF_ZONES];
					for (uint8_t z = 0; z < TOF_ZONES; z++) {
						uint8_t st = _results.target_status[z];
						bool valid = st == 5 || st == 9;   // 100 % valid, or valid with a large pulse (ST UM2884)
						uint8_t m = mapZone(z);
						mm[m] = valid ? (int16_t)(_results.distance_mm[z] * _rayScale[m]) : 0;
					}
					_sink(i, mm);
				}
				vTaskDelay(pdMS_TO_TICKS(8));
			}
		}

		SparkFun_VL53L5CX    _sensors[TOF_MAX_SENSORS];
		VL53L5CX_ResultsData _results;
		float                _rayScale[TOF_ZONES];   // perpendicular -> along the ray
		uint8_t              _count = 0;
		TofFrameSink         _sink = nullptr;
		bool                 _ok[TOF_MAX_SENSORS] = { false, false };
		volatile bool        _started = false;
};

TofArray &tofArray()
{
	static Vl53l5cxArray dev;
	return dev;
}

#endif   // HIL

void initTof()
{
	if (sensorCount() == 0) {
		return;
	}
	Serial.printf("ToF %s x%u\n", tofArray().name(), sensorCount());
	tofArray().begin(sensorCount(), publishFrame);
}

uint8_t tofSensorCount()
{
	return sensorCount();
}

bool tofReady(uint8_t i)
{
	return tofArray().started() && i < sensorCount() && tofArray().ok(i);
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

bool tofPeekFrame(uint8_t i, uint32_t &seq, int16_t *mm)
{
	if (!tofReady(i)) {
		return false;
	}
	portENTER_CRITICAL(&frameLock);
	seq = frames[i].seq;
	memcpy(mm, frames[i].mm, sizeof(frames[i].mm));
	portEXIT_CRITICAL(&frameLock);
	return true;
}

uint16_t tofMaxRange()
{
	return sensorCount() > 0 ? (uint16_t)MODEL_TERRAIN.tof[0].maxRange : 0;
}

void cliTof()
{
	char *mode = CLI_readWord();
	if (sensorCount() == 0) {
		cliSerial->println("This model has no ToF sensors.");
		return;
	}
	for (uint8_t i = 0; i < sensorCount(); i++) {
		bool ok = tofArray().ok(i);
		cliSerial->printf("ToF %u (%s): %s\n", i, tofArray().name(),
		                  !tofArray().started() ? "starting" : (ok ? "ok" : "not found"));
		if (strcmp(mode, "print") != 0 || !ok) {
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
bool tofPeekFrame(uint8_t i, uint32_t &seq, int16_t *mm) { (void)i; (void)seq; (void)mm; return false; }
uint16_t tofMaxRange() { return 0; }
void cliTof() { cliSerial->println("No ToF sensors (TOF_TYPE in config.h)."); }

#endif
