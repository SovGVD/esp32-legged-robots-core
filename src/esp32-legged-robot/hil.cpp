/**
 * Hardware in the loop (build with -DHIL: `pio run -e robot-dog-hil` or
 * `robot-hexapod-hil`). The ESP32 runs the whole firmware: the motion task,
 * WiFi AP, web UI, CLI, failsafe, battery protection. Only the devices are
 * replaced: the servos, IMU, power sensor, ToF sensors and foot switches
 * live in the MuJoCo simulator on a PC (sim/examples/hil.py), connected
 * over the USB serial port (HIL_BAUD, 921600 by default). The protocol is
 * lib/robot_core/src/hil_link.h.
 *
 * Every motion tick the ESP32 takes the newest sensor frame the PC sent,
 * runs the tick and sends the servo pulses with the time since the previous
 * tick; the PC moves the simulation on by that time and answers with the
 * sensors. So the ESP32's own timer is the clock, like on the robot, and the
 * sensors are one tick old, like a real I2C read.
 *
 * The PC can drive with its keyboard / gamepad too ('M' frames, they feed
 * the failsafe like the web UI); the phone on the robot's WiFi works as usual.
 * Serial text (boot log, warnings) still goes to the PC between the frames.
 * CLI `hil` prints the link state.
 */
#include "robot.h"

#ifdef HIL

#define HIL_RX_CHUNK     256
#define HIL_HELLO_MS     1000   // hello this often until the PC answers
#define HIL_LINK_LOST_MS 500    // no sensors for this long: the PC is gone

static HilParser  parser;
static HilSensors latest;
static bool       haveSensors = false;
static bool       magFresh    = false;
static uint32_t   lastSensorMs = 0;
static uint32_t   lastHelloMs  = 0;
static uint32_t   seq = 0;

static volatile uint32_t sensorFrames = 0;
static volatile uint32_t tofFrames    = 0;
static volatile uint32_t moveFrames   = 0;
static volatile uint32_t badFrames    = 0;   // right CRC, wrong length or type
static volatile uint32_t txDropped    = 0;   // servo frames not sent: the serial buffer was full
static volatile uint32_t linkUps      = 0;

static TofFrameSink tofSink = nullptr;
static uint8_t      tofCount = 0;
static bool         tofSeen[2] = { false, false };

/**
 * Servo bus: keeps the pulses, hilSend() sends them once per tick.
 */
class HilServoBus : public ServoBus {
	public:
		const char *name() const override { return "HIL (simulated)"; }

		bool begin(const uint8_t *channels, uint8_t count) override
		{
			(void)channels;
			_count = count > HIL_MAX_SERVOS ? HIL_MAX_SERVOS : count;
			return true;
		}

		void write(uint8_t index, uint16_t value) override
		{
			if (index < _count) {
				_us[index] = value;
			}
		}

		void setEnabled(bool enabled) override { _enabled = enabled; }
		uint8_t channelLimit() const override { return 255; }   // the model's pins are not used

		uint8_t  count() const { return _count; }
		bool     enabled() const { return _enabled; }
		uint16_t value(uint8_t i) const { return _us[i]; }

	private:
		uint8_t           _count = 0;
		volatile bool     _enabled = true;
		volatile uint16_t _us[HIL_MAX_SERVOS] = {};
};

static HilServoBus bus;

ServoBus &servoBus()
{
	return bus;
}

/**
 * ToF frames arrive from the simulator ('T'), at the sensor's rate.
 */
class HilTofArray : public TofArray {
	public:
		const char *name() const override { return "HIL (simulated)"; }
		void begin(uint8_t count, TofFrameSink sink) override
		{
			tofCount = count > 2 ? 2 : count;
			tofSink  = sink;
		}
		bool started() const override { return true; }
		bool ok(uint8_t sensor) const override { return sensor < tofCount && tofSeen[sensor]; }
};

TofArray &tofArray()
{
	static HilTofArray dev;
	return dev;
}

/**
 * A simulated switch under every foot ('I' frames, bit i = leg i).
 */
class HilFootInput : public FootInput {
	public:
		const char *name() const override { return "sim"; }
		uint8_t count() const override { return LEG_NUM; }
		void begin() override {}
		uint32_t read() override { return haveSensors ? latest.feet : 0; }
};

FootInput &footInput()
{
	static HilFootInput dev;
	return dev;
}

bool hilSensors(HilSensors &out, bool &newMag, bool takeMag)
{
	if (!haveSensors) {
		return false;
	}
	out    = latest;
	newMag = magFresh;
	if (takeMag) {
		magFresh = false;
	}
	return true;
}

static void sendHello()
{
	HilHello h;
	h.version    = HIL_VERSION;
	h.model      = ROBOT_MODEL;
	h.legs       = LEG_NUM;
	h.dof        = LEG_DOF;
	h.loopTimeUs = LOOP_TIME;
	h.tofCount   = MODEL_TERRAIN.tofCount;
	uint8_t buf[9 + HIL_OVERHEAD];
	size_t n = hilEncodeHello(h, buf);
	Serial.write(buf, n);
}

void initHil()
{
	Serial.printf("HIL: devices simulated on the PC over this port, %u baud (sim/examples/hil.py)\n",
	              (unsigned)SERIAL_BAUD);
	sendHello();
	lastHelloMs = millis();
}

static void handleFrame()
{
	const uint8_t *p = parser.payload();
	uint8_t len = parser.length();
	switch (parser.type()) {
		case HIL_SENSORS: {
			HilSensors s;
			if (!hilDecodeSensors(p, len, s)) {
				badFrames++;
				return;
			}
			bool mag = (s.flags & HIL_FLAG_MAG) != 0;
			if (!mag && haveSensors) {
				// keep the last field: a reading counts once (magFresh)
				for (int i = 0; i < 3; i++) s.mag[i] = latest.mag[i];
			}
			latest = s;
			magFresh = magFresh || mag;
			if (!haveSensors || (uint32_t)(millis() - lastSensorMs) > HIL_LINK_LOST_MS) {
				linkUps++;
			}
			haveSensors  = true;
			lastSensorMs = millis();
			sensorFrames++;
			return;
		}
		case HIL_TOF: {
			HilTof t;
			if (!hilDecodeTof(p, len, t) || t.sensor >= tofCount) {
				badFrames++;
				return;
			}
			tofSeen[t.sensor] = true;
			if (tofSink != nullptr) {
				tofSink(t.sensor, t.mm);
			}
			tofFrames++;
			return;
		}
		case HIL_MOVE: {
			HilMove m;
			if (!hilDecodeMove(p, len, m)) {
				badFrames++;
				return;
			}
			LR_moveVector c;
			c.move.x       = constrain(m.v[0], -1.0f, 1.0f);
			c.move.y       = constrain(m.v[1], -1.0f, 1.0f);
			c.move.z       = constrain(m.v[2], -1.0f, 1.0f);
			c.rotate.pitch = constrain(m.v[3], -1.0f, 1.0f);
			c.rotate.roll  = constrain(m.v[4], -1.0f, 1.0f);
			c.rotate.yaw   = constrain(m.v[5], -1.0f, 1.0f);
			setMoveCommand(c);
			failsafeFeed();
			moveFrames++;
			return;
		}
		default:
			badFrames++;
	}
}

void hilReceive()
{
	uint8_t buf[HIL_RX_CHUNK];
	// a ToF frame is 135 bytes: read everything that is there, a few chunks at most
	for (int chunk = 0; chunk < 8; chunk++) {
		int avail = Serial.available();
		if (avail <= 0) {
			break;
		}
		size_t n = Serial.read(buf, avail < HIL_RX_CHUNK ? avail : HIL_RX_CHUNK);
		for (size_t i = 0; i < n; i++) {
			if (parser.feed(buf[i])) {
				handleFrame();
			}
		}
	}
}

void hilSend(uint32_t dtUs)
{
	uint32_t now = millis();
	bool linkUp = haveSensors && (uint32_t)(now - lastSensorMs) <= HIL_LINK_LOST_MS;
	if (!linkUp && (uint32_t)(now - lastHelloMs) >= HIL_HELLO_MS) {
		lastHelloMs = now;
		sendHello();
	}

	HilServos s;
	s.seq     = ++seq;
	s.dtUs    = dtUs;
	s.enabled = bus.enabled() ? 1 : 0;
	s.count   = bus.count();
	for (uint8_t i = 0; i < s.count; i++) {
		s.us[i] = bus.value(i);
	}
	uint8_t frame[10 + 2 * HIL_MAX_SERVOS + HIL_OVERHEAD];
	size_t n = hilEncodeServos(s, frame);
	// never block the motion task: drop the frame if the PC doesn't read
	if ((size_t)Serial.availableForWrite() < n) {
		txDropped++;
		return;
	}
	Serial.write(frame, n);
}

void cliHil()
{
	uint32_t age = millis() - lastSensorMs;
	bool up = haveSensors && age <= HIL_LINK_LOST_MS;
	cliSerial->printf("HIL link: %s (%u baud), last sensors %lu ms ago, connected %lu times\n",
	                  up ? "up" : "waiting for sim/examples/hil.py", (unsigned)SERIAL_BAUD,
	                  haveSensors ? (unsigned long)age : 0UL, (unsigned long)linkUps);
	cliSerial->printf("  received: %lu sensor, %lu ToF, %lu move frames; %lu bad, %lu CRC errors\n",
	                  (unsigned long)sensorFrames, (unsigned long)tofFrames, (unsigned long)moveFrames,
	                  (unsigned long)badFrames, (unsigned long)parser.crcErrors());
	cliSerial->printf("  servo frames: %lu ticks, %lu not sent (serial buffer full)\n",
	                  (unsigned long)seq, (unsigned long)txDropped);
	if (haveSensors) {
		cliSerial->printf("  battery %.2f V %.2f A, feet 0x%02X\n", latest.voltage, latest.current, latest.feet);
	}
}

#else

void cliHil() { cliSerial->println("Not a HIL build (pio run -e robot-dog-hil / robot-hexapod-hil)."); }

#endif
