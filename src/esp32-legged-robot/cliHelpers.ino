void settingsPrintTrim()
{
	Serial.println();
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		Serial.print(legs[i].id.title);
		Serial.print(" leg trim {");
		Serial.print(radToDeg(legs[i].hal.trim.alpha), CLI_DP);
		Serial.print(", ");
		Serial.print(radToDeg(legs[i].hal.trim.beta),  CLI_DP);
		Serial.print(", ");
		Serial.print(radToDeg(legs[i].hal.trim.gamma), CLI_DP);
		#if LEG_DOF == 6
			Serial.print(", ");
			Serial.print(radToDeg(legs[i].hal.trim.delta), CLI_DP);
			Serial.print(", ");
			Serial.print(radToDeg(legs[i].hal.trim.epsilon),  CLI_DP);
			Serial.print(", ");
			Serial.print(radToDeg(legs[i].hal.trim.zeta), CLI_DP);
		#endif
		Serial.println("}");
	}
}
