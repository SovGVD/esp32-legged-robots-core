void settingsPrintTrim()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		#if LEG_DOF == 6
			cliSerial->printf(
				"[%s] trim {%.2f, %.2f, %.2f, %.2f, %.2f, %.2f}\n",
				legs[i].id.title,
				radToDeg(legs[i].hal.trim.alpha),
				radToDeg(legs[i].hal.trim.beta),
				radToDeg(legs[i].hal.trim.gamma),
				radToDeg(legs[i].hal.trim.delta),
				radToDeg(legs[i].hal.trim.epsilon),
				radToDeg(legs[i].hal.trim.zeta)
			);
		#elif LEG_DOF == 3
			cliSerial->printf(
				"[%s] trim {%.2f, %.2f, %.2f}\n",
				legs[i].id.title,
				radToDeg(legs[i].hal.trim.alpha),
				radToDeg(legs[i].hal.trim.beta),
				radToDeg(legs[i].hal.trim.gamma)
			);
		#elif LEG_DOF == 1
			cliSerial->printf(
				"[%s] trim {%.2f}\n",
				legs[i].id.title,
				radToDeg(legs[i].hal.trim.alpha)
			);
		#endif
	}
}
