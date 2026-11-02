#include "robot.h"

static void updateWiFi();
#include <WiFi.h>

// config_wifi.h is not tracked by git; fall back to the example credentials.
#if __has_include("config_wifi.h")
	#include "config_wifi.h"
#else
	#warning "config_wifi.h not found, using config_wifi.example.h (copy it to config_wifi.h and set your own SSID/password)"
	#include "config_wifi.example.h"
#endif

static IPAddress apIP(192,168,1,1);
static IPAddress apGateway(192,168,1,1);
static IPAddress apSubnet(255,255,255,0);
IPAddress WiFiIP;

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

static void updateWiFi()
{

}

void WiFiInfo()
{
	cliSerial->printf("IP:   %d.%d.%d.%d\nSSID: %s\nPass: %s\n", WiFiIP[0], WiFiIP[1], WiFiIP[2], WiFiIP[3], wifiSsid, wifiPass);
}
