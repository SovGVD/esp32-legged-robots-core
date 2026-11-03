#include "robot.h"

#if PWM_CONTROLLER_TYPE == PCA9685
#include <Adafruit_PWMServoDriver.h>

/**
 * One or two PCA9685 boards. A servo "pin" is the channel number: 0-15 is the
 * first board (PWM_CONTROLLER_ADDRESS), 16-31 the second one
 * (PWM_CONTROLLER_ADDRESS_2, only when defined).
 *
 * The servo task writes every servo every tick (18 on the hexapod), but the
 * boards only output a new pulse every 20 ms. So write() skips a pulse that
 * hasn't changed, and converts microseconds to PCA9685 ticks itself:
 * Adafruit's writeMicroseconds() reads the prescale register back over I2C on
 * every call, which doubled the bus traffic.
 */
#ifdef PWM_CONTROLLER_ADDRESS_2
	#define PCA9685_BOARDS 2
#else
	#define PCA9685_BOARDS 1
#endif

class Pca9685ServoBus : public ServoBus {
	public:
		const char *name() const override { return "PCA9685 "; }

		bool begin(const uint8_t *channels, uint8_t count) override
		{
			Serial.printf("x%u ", PCA9685_BOARDS);
			_count = count;
			for (uint8_t i = 0; i < count; i++) {
				_channels[i] = channels[i];
			}
			for (uint8_t i = 0; i < PCA9685_BOARDS; i++) {
				_boards[i].begin();
				_boards[i].setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
				_boards[i].setPWMFreq(SERVO_FREQ);
				_ticksPerUs[i] = 27000000.0f / (1000000.0f * (_boards[i].readPrescale() + 1));
			}
			setEnabled(true);
			return true;
		}

		void write(uint8_t index, uint16_t us) override
		{
			if (index >= _count) {
				return;
			}
			uint8_t channel = _channels[index];
			uint8_t board   = channel / 16;
			if (board >= PCA9685_BOARDS) {
				return;   // reported by the pin check at boot
			}
			if (us == _last[index]) {
				return;
			}
			_last[index] = us;
			_boards[board].setPWM(channel % 16, 0, (uint16_t)(us * _ticksPerUs[board]));
		}

		void setEnabled(bool enabled) override
		{
			for (uint8_t i = 0; i < _count; i++) {
				_last[i] = 0;   // write everything again after a sleep
			}
			for (uint8_t i = 0; i < PCA9685_BOARDS; i++) {
				if (enabled) {
					_boards[i].wakeup();
				} else {
					_boards[i].sleep();
				}
			}
		}

		uint8_t channelLimit() const override { return PCA9685_BOARDS * 16; }

	private:
		Adafruit_PWMServoDriver _boards[PCA9685_BOARDS] = {
			Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS, Wire),
			#ifdef PWM_CONTROLLER_ADDRESS_2
				Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS_2, Wire),
			#endif
		};
		uint8_t _channels[LEG_NUM * LEG_DOF] = {};
		uint8_t _count = 0;
		uint16_t _last[LEG_NUM * LEG_DOF] = {};   // last pulse written per servo, us
		float _ticksPerUs[PCA9685_BOARDS] = {};
};

ServoBus &servoBus()
{
	static Pca9685ServoBus bus;
	return bus;
}
#endif
