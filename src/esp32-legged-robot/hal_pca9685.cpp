#include "robot.h"

#if PWM_CONTROLLER_TYPE == PCA9685
#include <Adafruit_PWMServoDriver.h>

/**
 * One or two PCA9685 boards. A servo "pin" is the channel number: 0-15 is the
 * first board (PWM_CONTROLLER_ADDRESS), 16-31 the second one
 * (PWM_CONTROLLER_ADDRESS_2, only when defined).
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
			_boards[board].writeMicroseconds(channel % 16, us);
		}

		void setEnabled(bool enabled) override
		{
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
};

ServoBus &servoBus()
{
	static Pca9685ServoBus bus;
	return bus;
}
#endif
