#include "robot.h"

#if PWM_CONTROLLER_TYPE == DCMOTOR

#define ESP32_DCMOTOR_FREQ    30000
#define ESP32_DCMOTOR_BIT     8

#if LEG_DOF > 2
	#error "Not supported or never tested"
#endif

/**
 * DC motors on a two-input H-bridge (L298N, DRV8833, ...). Each output is one
 * bridge input driven by LEDC; values are duties 0..255 from the
 * differential drive.
 */
class DcMotorBus : public ServoBus {
	public:
		const char *name() const override { return "DC motor LEDC "; }

		bool begin(const uint8_t *channels, uint8_t count) override
		{
			bool ok = true;
			_count = count;
			for (uint8_t i = 0; i < count; i++) {
				_pins[i] = channels[i];
				if (!ledcAttach(_pins[i], ESP32_DCMOTOR_FREQ, ESP32_DCMOTOR_BIT)) {
					Serial.printf("LEDC attach failed for pin %u\n", _pins[i]);
					ok = false;
					continue;
				}
				ledcWrite(_pins[i], 0);
			}
			return ok;
		}

		void write(uint8_t index, uint16_t duty) override
		{
			if (index >= _count) {
				return;
			}
			ledcWrite(_pins[index], _enabled ? duty : 0);
		}

		// Disabled: both inputs low, the motors coast
		void setEnabled(bool enabled) override
		{
			_enabled = enabled;
			if (!enabled) {
				for (uint8_t i = 0; i < _count; i++) {
					ledcWrite(_pins[i], 0);
				}
			}
		}

		uint8_t channelLimit() const override { return 40; }

	private:
		uint8_t _pins[LEG_NUM * LEG_DOF] = {};
		uint8_t _count   = 0;
		bool    _enabled = true;
};

ServoBus &servoBus()
{
	static DcMotorBus bus;
	return bus;
}
#endif
