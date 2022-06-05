String webConfig;

void initWebServer() {
	Serial.print("WebServer ");
	initWebServerConfig();
	initWebServerRoutes();
	Serial.println("WebServer ready");
}

/**
 * This is opening caprive portal as soon as client connect to WiFi
 * it is working, but my phone switching back to WiFi with internet
 */
class CaptiveRequestHandler : public AsyncWebHandler {
	public:
		CaptiveRequestHandler() {}
		virtual ~CaptiveRequestHandler() {}

	bool canHandle(AsyncWebServerRequest *request){
		return true;
	}

	void handleRequest(AsyncWebServerRequest *request) {
		// index.html page
		AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html",  index_html_gz, index_html_gz_len);
		response->addHeader("Content-Encoding", "gzip");
		request->send(response);
	}
};

void initWebServerConfig()
{
	// Web server version
	webConfig = "const c={v:" + WEB_SERVER_VERSION;

	// IP address of robot
	addStringToWebServerConfig("w", "ws://" + WiFiIP.toString() + "/ws");
	
	// Camera framerate
	#ifdef ESP32CAMERA
		addIntToWebServerConfig("c", ESP32CAMERA_FRAMESIZE);
	#else
		addIntToWebServerConfig("c", 0);	// no camera available
	#endif

	webConfig += "};";
}

void addStringToWebServerConfig(String key, String value)
{
	webConfig += "," + key + ":'" + value + "'";
}

void addIntToWebServerConfig(String key, int value)
{
	webConfig += "," + key + ":" + value;
}

void initWebServerRoutes()
{
	// Web server config and available features
	server.on("/c.js", HTTP_GET, [](AsyncWebServerRequest *request){
		request->send(200, "application/x-javascript", webConfig);
	});

	ws.onEvent(onWsEvent);
	server.addHandler(&ws);
	server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);

	server.begin();
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{
	if(type == WS_EVT_CONNECT) {
		wsclient = client;
		client->text("Ok");
		clientOnline = true;
	} else if (clientOnline && type == WS_EVT_DATA) {
		FS_WS_count = 0;  // zero FS counter

		switch(data[0]) {
			case P_MOVE:
				pMove(data);
				break;
			case P_TELEMETRY:
				pTelemetry();
				client->binary(telemetryPackage, P_TELEMETRY_LEN);
				break;
			#ifdef CAMERAENABLED
			case P_START_VIDEO:
				mainCameraStream = true;
				break;
			case P_STOP_VIDEO:
				mainCameraStream = false;
				break;
			#endif
			default:
				cliSerial->print("Unknown package ID: ");
				cliSerial->println(data[0], DEC);
		}

	} else if(type == WS_EVT_DISCONNECT) {
		clientOnline = false;
	}
}
