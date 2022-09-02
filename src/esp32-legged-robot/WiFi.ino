void initWiFi()
{
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  WiFi.softAP(wifiSsid, wifiPass);
}

void WiFiEvent(WiFiEvent_t event)
{
	switch(event) {
		case SYSTEM_EVENT_AP_START:
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
	cliSerial->printf("IP:   %d.%d.%d.%d\nSSID: %s\nPass: %s\n", WiFiIP[0], WiFiIP[1], WiFiIP[2], WiFiIP[3], wifiSsid, wifiPass);
}
