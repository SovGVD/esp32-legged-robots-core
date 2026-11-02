void initWiFi()
{
	WiFi.persistent(false);
	WiFi.mode(WIFI_AP);
	// Configure the AP synchronously: the web server reads WiFiIP right after
	// this returns to build the websocket URL for the web client.
	WiFi.softAPConfig(apIP, apGateway, apSubnet);
	if (!WiFi.softAP(wifiSsid, wifiPass)) {
		Serial.println("WiFi AP start failed");
	}
	WiFi.softAPsetHostname(wifiSsid);
	WiFiIP = WiFi.softAPIP();
	Serial.print("WiFi AP IP: ");
	Serial.println(WiFiIP);
	//dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
	//dnsServer.start(DNS_PORT, "*", WiFiIP);
}

void updateWiFi()
{

}

void WiFiInfo()
{
	cliSerial->printf("IP:   %d.%d.%d.%d\nSSID: %s\nPass: %s\n", WiFiIP[0], WiFiIP[1], WiFiIP[2], WiFiIP[3], wifiSsid, wifiPass);
}
