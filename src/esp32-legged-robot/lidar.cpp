/**
 * 2D lidar (LDROBOT LD06 / LD19) for indoor maps and walking to a point.
 *
 * Wiring: the lidar's TX to LIDAR_RX_PIN (config.h), 5 V and GND; its PWM
 * pin to GND (the lidar then runs its own 10 rev/s speed control). It
 * streams 230400 baud on its own, nothing is sent to it.
 *
 * A task on core 0 reads the UART, parses the packets (Ld06Parser) and
 * collects revolutions (LidarScanBuilder). Each finished revolution goes
 * into LidarSlam with the odometry from the motion task (motionOdometry:
 * gait velocity turned by the gyro), which corrects the pose against the
 * map and adds the scan to it; then the Navigator (if a goal is set) plans
 * and writes its move command, which the motion task picks up
 * (lidarNavCommand). The motion task only copies a few numbers: the map,
 * matching and planning never run on the motion core.
 *
 * Memory: the map (40 KB) is allocated when the first lidar packet arrives,
 * the planner (30 KB) with the first goal, so a robot without a lidar loses
 * nothing.
 *
 * The same lib/robot_core code runs in the simulator (sim/examples/lidar_nav.py),
 * which feeds it LD06 bytes from a simulated lidar.
 */
#include "robot.h"

#if defined(LIDAR_TYPE) && LIDAR_TYPE == LD06 && defined(LIDAR_SUPPORTED)

#ifndef LIDAR_UART
	#define LIDAR_UART 2
#endif
#ifndef LIDAR_RX_PIN
	#error "LIDAR_TYPE needs LIDAR_RX_PIN (config.h): the GPIO the lidar's TX wire goes to"
#endif

static HardwareSerial lidarSerial(LIDAR_UART);
static Ld06Parser        parser;
static LidarScanBuilder  scans;
static LidarSlam         slam;
static Navigator         nav;
static SemaphoreHandle_t lock = nullptr;     // slam, nav, scans: lidar task vs. CLI / web server
static bool              mapOk = false;      // map allocated
static bool              noMemory = false;
static LR_pose2          scanStartOdom = { 0, 0, 0 };
static uint32_t          lastScanMs = 0;
static uint32_t          lastPacketMs = 0;

// navigator -> motion task
static portMUX_TYPE      navLock = portMUX_INITIALIZER_UNLOCKED;
static LR_moveVector     navCmd = {};
static volatile bool     navActive = false;
static volatile uint32_t navCmdMs = 0;
static volatile bool     cancelRequest = false;

static void scanDone()
{
	MotionOdometry odo = motionOdometry();
	xSemaphoreTake(lock, portMAX_DELAY);
	slam.addScan(scans.scan(), scanStartOdom, odo.pose);
	scanStartOdom = odo.pose;
	uint32_t now = millis();
	float dt = lastScanMs ? (now - lastScanMs) / 1000.0f : 0.1f;
	lastScanMs = now;
	if (cancelRequest) {
		nav.stop();
		cancelRequest = false;
	}
	LR_moveVector cmd = nav.update(dt, slam, slam.pose(odo.pose), odo.maxYawRate);
	bool active = nav.active();
	xSemaphoreGive(lock);

	portENTER_CRITICAL(&navLock);
	navCmd   = cmd;
	navCmdMs = now;
	portEXIT_CRITICAL(&navLock);
	navActive = active;
}

static void lidarTask(void *)
{
	uint8_t buf[128];
	for (;;) {
		size_t n = lidarSerial.read(buf, sizeof(buf));
		for (size_t k = 0; k < n; k++) {
			if (!parser.feed(buf[k])) {
				continue;
			}
			lastPacketMs = millis();
			if (!mapOk) {
				if (noMemory) {
					continue;
				}
				xSemaphoreTake(lock, portMAX_DELAY);
				mapOk = slam.begin(*MODEL_LIDAR.lidar, *MODEL_LIDAR.map);
				xSemaphoreGive(lock);
				if (!mapOk) {
					noMemory = true;
					Serial.println("Lidar: no memory for the map (40 KB)");
					continue;
				}
				scanStartOdom = motionOdometry().pose;
				Serial.println("Lidar: data, map started");
			}
			for (uint8_t i = 0; i < LD06_POINTS; i++) {
				if (scans.add(parser.angle(i), parser.distance(i), parser.intensity(i))) {
					scanDone();
				}
			}
		}
		if (n < sizeof(buf)) {
			vTaskDelay(pdMS_TO_TICKS(5));   // ~120 bytes arrive in 5 ms at 230400 baud
		}
	}
}

void initLidar()
{
	if (MODEL_LIDAR.lidar == nullptr) {
		return;
	}
	lock = xSemaphoreCreateMutex();
	scans.begin(*MODEL_LIDAR.lidar);
	nav.begin(*MODEL_LIDAR.nav);
	lidarSerial.setRxBufferSize(2048);
	lidarSerial.begin(LD06_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);
	// core 0 with WiFi and the web server, above the services loop: a scan is ~10 ms of work every 100 ms
	xTaskCreatePinnedToCore(lidarTask, "lidar", 6144, nullptr, 1, nullptr, 0);
	Serial.printf("Lidar: LD06 on UART%u, RX GPIO %u\n", (unsigned)LIDAR_UART, (unsigned)LIDAR_RX_PIN);
}

bool lidarNavCommand(LR_moveVector &cmd)
{
	if (!navActive) {
		return false;
	}
	portENTER_CRITICAL(&navLock);
	cmd = navCmd;
	uint32_t at = navCmdMs;
	portEXIT_CRITICAL(&navLock);
	if (millis() - at > 300) {
		// no scans: the lidar stopped (or the task is stuck), stand still
		LR_moveVector stop = { { 0, 0, 0 }, { 0, 0, 0 } };
		cmd = stop;
	}
	return true;
}

void lidarNavCancel()
{
	navActive = false;
	cancelRequest = true;
}

bool lidarGoto(float x, float y)
{
	if (!mapOk) {
		return false;
	}
	xSemaphoreTake(lock, portMAX_DELAY);
	bool ok = nav.setGoal(x, y);
	xSemaphoreGive(lock);
	cancelRequest = false;
	return ok;
}

void lidarStop()
{
	lidarNavCancel();
}

/**
 * Map for the web client, answer to [P_MAP]: 2 bits per cell, 4 cells per
 * byte (first cell in the low bits), rows from map y = min (bottom) up,
 * cells from x = min: 0 unknown, 1 free, 2 seen once, 3 occupied.
 *
 * offset | type | length | description
 * -------|------|--------|-------------
 *      0 | char |      1 | 'P' (P_MAP)
 *      1 | u8   |      1 | 1 = map running, 0 = no lidar data yet (nothing follows)
 *      2 | u16  |      2 | cells per side n
 *      4 | u16  |      2 | cell size, mm
 *      6 | bin  | n*n/4  | cells
 */
size_t pMap(uint8_t *out, size_t room, uint8_t part)
{
	(void)part;
	if (MODEL_LIDAR.lidar == nullptr || room < 2) {
		return 0;
	}
	out[0] = P_MAP;
	out[1] = mapOk ? 1 : 0;
	if (!mapOk) {
		return 2;
	}
	const int n = LR_LIDAR_GRID;
	size_t len = 6 + (size_t)n * n / 4;
	if (room < len) {
		return 0;
	}
	uint16_t cell = (uint16_t)MODEL_LIDAR.map->cellSize;
	int8_t strong = 2 * MODEL_LIDAR.map->hit;
	out[2] = n >> 8; out[3] = n & 0xFF;
	out[4] = cell >> 8; out[5] = cell & 0xFF;
	xSemaphoreTake(lock, portMAX_DELAY);
	const int8_t *c = slam.grid().cells();
	uint8_t *p = out + 6;
	for (int i = 0; i < n * n; i += 4) {
		uint8_t b = 0;
		for (int k = 0; k < 4; k++) {
			int8_t v = c[i + k];
			uint8_t code = v == 0 ? 0 : (v < 0 ? 1 : (v < strong ? 2 : 3));
			b |= code << (2 * k);
		}
		*p++ = b;
	}
	xSemaphoreGive(lock);
	return len;
}

static void put16(uint8_t *&p, int v)
{
	*p++ = (v >> 8) & 0xFF;
	*p++ = v & 0xFF;
}

/**
 * Robot pose, goal, path and the last scan for the web client, answer to [P_NAV].
 *
 * offset | type | length | description
 * -------|------|--------|-------------
 *      0 | char |      1 | 'N' (P_NAV)
 *      1 | u8   |      1 | flags: 1 lidar data, 2 map frozen, 4 blocked ahead, 8 no memory
 *      2 | u8   |      1 | navigator state (NavState: 0 idle, 1 walking, 2 arrived, 3 no path, 4 no memory)
 *      3 | u8   |      1 | last scan match (LidarMatch: 0 none, 1 ok, 2 new area, 3 rejected, 4 few points)
 *      4 | i16  |      2 | robot x, mm (map frame: x right, y forward at the start)
 *      6 | i16  |      2 | robot y, mm
 *      8 | i16  |      2 | robot heading, deg*100, counter-clockwise, 0 = facing +y
 *     10 | i16  |      2 | goal x, mm
 *     12 | i16  |      2 | goal y, mm
 *     14 | u16  |      2 | scans since the map started
 *     16 | u8   |      1 | path points p
 *     17 | i16x2| 4p     | path x, y, mm
 *      . | u16  |      2 | scan points s
 *      . | i16x2| 4s     | scan points x, y, mm, robot frame (x right, y forward)
 * Big-endian.
 */
size_t pNav(uint8_t *out, size_t room)
{
	if (MODEL_LIDAR.lidar == nullptr || room < 19 + 4 * LR_NAV_PATH + 4 * LR_LIDAR_BINS / 2) {
		return 0;
	}
	MotionOdometry odo = motionOdometry();
	uint8_t *p = out;
	*p++ = P_NAV;
	if (!mapOk) {
		*p++ = noMemory ? 8 : 0;
		memset(p, 0, 17);   // state .. scan count, all zero
		return 19;
	}
	xSemaphoreTake(lock, portMAX_DELAY);
	LR_pose2 pose = slam.pose(odo.pose);
	*p++ = (millis() - lastPacketMs < 1000 ? 1 : 0) | (slam.frozen() ? 2 : 0) | (nav.blocked() ? 4 : 0);
	*p++ = nav.state();
	*p++ = slam.lastMatch();
	put16(p, (int)pose.x);
	put16(p, (int)pose.y);
	put16(p, (int)lroundf(pose.theta * RADTODEG * 100));
	put16(p, (int)nav.goalX());
	put16(p, (int)nav.goalY());
	put16(p, (int)(slam.scans() & 0xFFFF));
	const float *x, *y;
	uint8_t pn = nav.active() ? nav.path(x, y) : 0;
	*p++ = pn;
	for (uint8_t i = 0; i < pn; i++) {
		put16(p, (int)x[i]);
		put16(p, (int)y[i]);
	}
	uint16_t sn = slam.points(x, y);
	put16(p, (sn + 1) / 2);
	for (uint16_t i = 0; i < sn; i += 2) {   // every second point is plenty for the view
		put16(p, (int)x[i]);
		put16(p, (int)y[i]);
	}
	xSemaphoreGive(lock);
	return p - out;
}

static const char *const navStateNames[] = { "idle", "walking to goal", "arrived", "no path", "no memory" };
static const char *const matchNames[] = { "none", "ok", "new area", "rejected (odometry kept)", "too few points" };

static void printDistances()
{
	// closest reading around each of 8 directions (body frame), +-15 deg
	static const char *const names[] = { "front", "front-left", "left", "back-left", "back", "back-right", "right", "front-right" };
	uint16_t best[8] = { 0 };
	xSemaphoreTake(lock, portMAX_DELAY);
	LidarScan s = scans.scan();
	xSemaphoreGive(lock);
	for (uint16_t b = 0; b < LR_LIDAR_BINS; b++) {
		if (s.mm[b] == 0) continue;
		float dx, dy;
		lrLidarBinDirection(*MODEL_LIDAR.lidar, b, dx, dy);
		float a = atan2f(-dx, dy) * RADTODEG;   // counter-clockwise from forward
		if (a < 0) a += 360;
		int sector = (int)lroundf(a / 45) % 8;
		float off = fabsf(a - sector * 45);
		if (off > 180) off = 360 - off;
		if (off <= 15 && (best[sector] == 0 || s.mm[b] < best[sector])) {
			best[sector] = s.mm[b];
		}
	}
	cliSerial->println("Closest reading per direction (robot frame), - = nothing:");
	for (uint8_t i = 0; i < 8; i++) {
		if (best[i]) {
			cliSerial->printf("  %-12s %5u mm\n", names[i], best[i]);
		} else {
			cliSerial->printf("  %-12s     -\n", names[i]);
		}
	}
	cliSerial->println("A box 30 cm in front must show as front; else fix LIDAR_PARAMS yaw (model_data.h).");
}

void cliLidar()
{
	char *mode = CLI_readWord();
	if (strcmp(mode, "print") == 0) {
		printDistances();
		return;
	}
	if (strcmp(mode, "reset") == 0 && mapOk) {
		lidarNavCancel();
		xSemaphoreTake(lock, portMAX_DELAY);
		slam.reset();
		scanStartOdom = motionOdometry().pose;
		xSemaphoreGive(lock);
		cliSerial->println("Map cleared: the robot is at 0, 0 facing +y.");
	} else if (strcmp(mode, "freeze") == 0 && mapOk) {
		xSemaphoreTake(lock, portMAX_DELAY);
		slam.setFrozen(!slam.frozen());
		xSemaphoreGive(lock);
	} else if (strcmp(mode, "stop") == 0) {
		lidarStop();
		cliSerial->println("Stopped walking to the goal.");
	}
	MotionOdometry odo = motionOdometry();
	cliSerial->printf("Lidar: %s, %.1f rev/s, %u packets, %u CRC errors\n",
	                  millis() - lastPacketMs < 1000 ? "running" : "no data (wiring? LIDAR_RX_PIN)",
	                  parser.speed() / 360.0f, (unsigned)parser.packets(), (unsigned)parser.crcErrors());
	if (!mapOk) {
		cliSerial->println(noMemory ? "Map: no memory" : "Map: starts with the first lidar data");
		return;
	}
	xSemaphoreTake(lock, portMAX_DELAY);
	LR_pose2 pose = slam.pose(odo.pose);
	cliSerial->printf("Map: %u occupied cells%s, %u scans, %u rejected; last match %s, score %.2f\n",
	                  (unsigned)slam.grid().occupiedCells(), slam.frozen() ? " (frozen)" : "",
	                  (unsigned)slam.scans(), (unsigned)slam.rejected(), matchNames[slam.lastMatch()], slam.lastScore());
	cliSerial->printf("Pose: x %.2f m, y %.2f m, heading %.0f deg (counter-clockwise from the start)\n",
	                  pose.x / 1000, pose.y / 1000, pose.theta * RADTODEG);
	cliSerial->printf("Goal: %s", navStateNames[nav.state()]);
	if (nav.state() != NAV_IDLE) {
		cliSerial->printf(", %.2f / %.2f m, %.2f m away%s", nav.goalX() / 1000, nav.goalY() / 1000,
		                  nav.distance() / 1000, nav.blocked() ? ", blocked ahead" : "");
	}
	cliSerial->println(FS_FAIL && nav.active() ? " (no client: standing)" : "");
	xSemaphoreGive(lock);
}

void cliGoto()
{
	float x = CLI_readFloat();
	float y = CLI_readFloat();
	if (!mapOk) {
		cliSerial->println("No lidar map yet (`lidar state`).");
		return;
	}
	if (!lidarGoto(x * 1000, y * 1000)) {
		cliSerial->println("No memory for the planner (30 KB).");
		return;
	}
	cliSerial->printf("Walking to %.2f, %.2f m (map frame: x right, y forward at the start). Stick or `lidar stop` stops it.\n", x, y);
}

#else

void initLidar() {}
bool lidarNavCommand(LR_moveVector &cmd) { (void)cmd; return false; }
void lidarNavCancel() {}
size_t pMap(uint8_t *out, size_t room, uint8_t part) { (void)out; (void)room; (void)part; return 0; }
size_t pNav(uint8_t *out, size_t room) { (void)out; (void)room; return 0; }
bool lidarGoto(float x, float y) { (void)x; (void)y; return false; }
void lidarStop() {}
void cliLidar() { cliSerial->println("No lidar (LIDAR_TYPE in config.h)."); }
void cliGoto() { cliSerial->println("No lidar (LIDAR_TYPE in config.h)."); }

#endif
