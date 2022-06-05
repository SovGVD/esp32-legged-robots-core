LR_legAngle modelSettingsLoadTrimLeg(modelSettingsAngleTrim t)
{
	return {
		t.alpha,
		t.beta,
		t.gamma
	};
}

void modelSettingsPrintTrim()
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
    Serial.println("}");
  }
}

void modelSettingsLoadTrim()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		legs[i].hal.trim = modelSettingsLoadTrimLeg(settings.trim[i]);
	}
}

void modelSettingsSaveTrim()
{
	int value = 0;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		settings.trim[i].alpha = legs[i].hal.trim.alpha;
		settings.trim[i].beta  = legs[i].hal.trim.beta;
		settings.trim[i].gamma = legs[i].hal.trim.gamma;
	}
}

void modelSettingsPrint()
{
	modelSettingsPrintTrim();
}

void modelSettingsLoad()
{
	modelSettingsLoadTrim();
}

void modelSettingsSave()
{
	modelSettingsSaveTrim();
}
