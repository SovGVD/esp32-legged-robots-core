void initWiFi()
{
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  WiFi.softAP(wifiSsid, wifiPass);
}

void WiFiEvent(WiFiEvent_t event)
{
	switch(event) {
		case ARDUINO_EVENT_WIFI_AP_START:
			WiFi.softAPsetHostname(wifiSsid);
			WiFi.softAPConfig(apIP, apGateway, apSubnet);
			WiFiIP = WiFi.softAPIP();
			Serial.print("WiFi AP IP: ");
			Serial.println(WiFiIP);
			//dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
			//dnsServer.start(DNS_PORT, "*", WiFiIP);
			break;
		default:
			break;
	}
}

void updateWiFi()
{
  
}

void WiFiInfo()
{
	cliSerial->print("IP:   ");
	cliSerial->println(WiFiIP);
	cliSerial->print("SSID: ");
	cliSerial->println(wifiSsid);
	cliSerial->print("Pass: ");
	cliSerial->println(wifiPass);
}
