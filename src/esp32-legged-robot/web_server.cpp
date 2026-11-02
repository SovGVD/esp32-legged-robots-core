#include "robot.h"
#include <ESPAsyncWebServer.h>
#include "web/index.html.gz.h"

#ifdef ESP32CAMERA
	#define CAMERAENABLED
	#include <esp32camera.h>
	esp32camera mainCamera;
	bool mainCameraStream = false;
#endif

bool clientOnline = false;
AsyncWebSocketClient *wsclient = nullptr;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
uint8_t telemetryPackage[P_TELEMETRY_LEN];

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
static void initWebServerConfig();
static void initWebServerRoutes();
static void addStringToWebServerConfig(String key, String value);
static void addIntToWebServerConfig(String key, int value);

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

	// Must match the base signature exactly (const), otherwise it does not
	// override and the handler silently never matches any request.
	bool canHandle(AsyncWebServerRequest *request) const override {
		return true;
	}

	void handleRequest(AsyncWebServerRequest *request) override {
		// index.html page
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", index_html_gz, index_html_gz_len);
		response->addHeader("Content-Encoding", "gzip");
		request->send(response);
	}
};

static void initWebServerConfig()
{
	// Web server version
	webConfig = "const c={v:'" + String(WEB_SERVER_VERSION) + "'";

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

static void addStringToWebServerConfig(String key, String value)
{
	webConfig += "," + key + ":'" + value + "'";
}

static void addIntToWebServerConfig(String key, int value)
{
	webConfig += "," + key + ":" + value;
}

static void initWebServerRoutes()
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

static void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{
	if(type == WS_EVT_CONNECT) {
		wsclient = client;
		client->text("Ok");
		clientOnline = true;
		cliSerial->setClientReady(client);
		#ifdef CAMERAENABLED
			mainCamera.setClientReady(client);
		#endif
		cliInitHelp();
	} else if (clientOnline && type == WS_EVT_DATA) {
		FS_WS_count = 0;  // zero FS counter

		switch(data[0]) {
			case P_MOVE:
				pMove(data, len);
				break;
			case P_TELEMETRY:
				pTelemetry();
				client->binary(telemetryPackage, P_TELEMETRY_LEN);
				break;
			case P_CLI:
				pCli(data, len);
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
				Serial.print("Unknown package ID: ");
				Serial.println(data[0], DEC);
				cliSerial->print("Unknown package ID: ");
				cliSerial->println(data[0], DEC);
		}

	} else if(type == WS_EVT_DISCONNECT) {
		cliSerial->setClientDisconnected();
		#ifdef CAMERAENABLED
			mainCamera.setClientDisconnected();
		#endif

		clientOnline = false;
	}
}
