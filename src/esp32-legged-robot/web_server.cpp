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
static volatile uint32_t controllingClientId = 0;
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

	// On-screen toggles: g = gait (trot/walk) can be switched, l = IMU leveling
	addIntToWebServerConfig("g", MODEL_WALK_GAIT != nullptr ? 1 : 0);
	#if PWM_CONTROLLER_TYPE == DCMOTOR
		addIntToWebServerConfig("l", 0);	// tracks don't level
	#else
		addIntToWebServerConfig("l", 1);
	#endif

	// ToF view: t = number of sensors (0 = none), r = range of the colour scale, mm
	addIntToWebServerConfig("t", tofSensorCount());
	addIntToWebServerConfig("r", tofMaxRange());
	// m = ToF mounting for the 3D view, per sensor [x, y, z (mm, body frame), yaw, tilt, fov (rad)]
	webConfig += ",m:[";
	for (uint8_t i = 0; i < tofSensorCount(); i++) {
		const TofSensor &s = MODEL_TERRAIN.tof[i];
		webConfig += String(i ? ",[" : "[") + String(s.position.x, 0) + "," + String(s.position.y, 0) + "," +
		             String(s.position.z, 0) + "," + String(s.yaw, 3) + "," + String(s.tilt, 3) + "," + String(s.fov, 3) + "]";
	}
	webConfig += "]";

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
	cliSerial->setSocket(&ws);
	server.addHandler(&ws);
	server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);

	server.begin();
}

#ifdef CAMERAENABLED
/**
 * Camera task (esp32camera::addTask): each JPEG frame goes to the controlling
 * client as a binary message. A JPEG starts with 0xFF 0xD8, which is no
 * package id, so the client can tell frames from other packages. Frames are
 * dropped while the client's send queue is full instead of piling up.
 */
static void sendCameraFrame(camera_fb_t *fb)
{
	uint32_t id = controllingClientId;
	if (id == 0 || fb == nullptr || !ws.availableForWrite(id)) {
		return;
	}
	ws.binary(id, fb->buf, fb->len);
}

void initCameraStream()
{
	// LEDC channel/timer for the camera clock away from the ones ledcAttach()
	// hands out first (servos, DC motors)
	if (mainCamera.setup(ESP32CAMERA_FRAMESIZE, ESP32CAMERA_FPS, ESP32CAMERA_JPEG_QUALITY,
	                     LEDC_CHANNEL_7, LEDC_TIMER_3)) {
		mainCamera.addTask(sendCameraFrame);
	}
}
#endif

/**
 * One client controls the robot at a time: the newest connection takes over
 * (a phone that reconnects often leaves a stale connection behind) and the
 * previous one is closed. Clients are tracked by id, never by pointer, so a
 * client freed by AsyncTCP is never used afterwards.
 */
static void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{
	if (type == WS_EVT_CONNECT) {
		uint32_t previous = controllingClientId;
		controllingClientId = client->id();
		if (previous != 0 && previous != client->id()) {
			ws.close(previous, 1000, "Another client took control");
		}
		client->text("Ok");
		clientOnline = true;
		cliSerial->setClientReady(client->id());
		cliInitHelp();
	} else if (type == WS_EVT_DATA) {
		if (client->id() != controllingClientId || len == 0) {
			return;
		}
		AwsFrameInfo *info = (AwsFrameInfo *)arg;
		if (!info->final || info->index != 0 || info->len != len) {
			return;   // fragmented frames are not used by the web client
		}

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
			case P_GAIT:
				pGait(data, len);
				break;
			case P_LEVEL:
				pLevel(data, len);
				break;
			case P_TOF: {
				// skip while the send queue is full (slow WiFi): the client asks again soon
				static uint8_t tofPackage[P_TOF_LEN(P_TOF_MAX_SENSORS)];
				size_t tofLen = pTof(tofPackage);
				if (tofLen > 0 && client->canSend()) {
					client->binary(tofPackage, tofLen);
				}
				break;
			}
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

	} else if (type == WS_EVT_DISCONNECT) {
		if (client->id() != controllingClientId) {
			return;   // an old, replaced client
		}
		controllingClientId = 0;
		setFailsafe();
		cliSerial->setClientDisconnected();
		#ifdef CAMERAENABLED
			mainCameraStream = false;
		#endif

		clientOnline = false;
	}
}

/**
 * Called from the services loop: drops clients over the library limit and
 * frees closed ones.
 */
void updateWebServer()
{
	ws.cleanupClients(2);
}
