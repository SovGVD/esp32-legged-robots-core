/**
 * Indoor map, localization and walking to a point: with a 2D lidar (LDROBOT
 * LD06 / LD19, LIDAR_TYPE in config.h) or, without one, with room scans of
 * the terrain ToF sensors (VL53L5CX, TOF_SCAN_PARAMS in model_data.h).
 *
 * Lidar wiring: the lidar's TX to LIDAR_RX_PIN (config.h), 5 V and GND; its
 * PWM pin to GND (the lidar then runs its own 10 rev/s speed control). It
 * streams 230400 baud on its own, nothing is sent to it. A task on core 0
 * reads the UART, parses the packets (Ld06Parser) and collects revolutions
 * (LidarScanBuilder). Each finished revolution goes into LidarSlam with the
 * odometry from the motion task (motionOdometry: gait velocity turned by the
 * gyro), which corrects the pose against the map and adds the scan to it;
 * then the Navigator (if a goal is set) plans and writes its move command.
 *
 * ToF room scans (tof_scan.h): a task on core 0 runs TofNavigator 50 times a
 * second. A scan stops the robot, tilts its nose up (lidarNavBody, applied by
 * the motion task), turns it once around and puts what the sensors saw near
 * level into the same map; walking to a goal it scans every scanEvery mm.
 * The ToF task keeps reading the sensors as usual (tofPeekFrame).
 *
 * Either way the motion task only copies a few numbers (lidarNavCommand):
 * the map, matching and planning never run on the motion core.
 *
 * Memory: the map (40 KB) is allocated when the first data arrives, the
 * planner (30 KB) with the first goal, so a robot without the sensor loses
 * nothing.
 *
 * The same lib/robot_core code runs in the simulator (sim/examples/lidar_nav.py,
 * --tof for the room scans).
 */
#include "robot.h"

#if defined(NAV_LIDAR) || defined(NAV_TOF)

#ifdef NAV_LIDAR
	#ifndef LIDAR_UART
		#define LIDAR_UART 2
	#endif
	#ifndef LIDAR_RX_PIN
		#error "LIDAR_TYPE needs LIDAR_RX_PIN (config.h): the GPIO the lidar's TX wire goes to"
	#endif
	static HardwareSerial    lidarSerial(LIDAR_UART);
	static Ld06Parser        parser;
	static LidarScanBuilder  scans;
	static Navigator         nav;
	#define NAV_STATE() nav
	#define NAV_BLOCKED() nav.blocked()
#else
	static TofNavigator      tofNav;
	static uint32_t          tofSeq[2] = { 0, 0 };
	static volatile float    navTilt = 0;     // nose up, rad (motion task applies it)
	static volatile float    navCrouch = 0;   // body lower, mm
	static volatile bool     scanRequest = false;
	static const LidarParams TOF_ORIGIN = { { 0, 0, 0 }, 0, false, 0, 0, 0 };   // beams start at the body origin
	#define NAV_STATE() tofNav.nav()
	#define NAV_BLOCKED() tofNav.blocked()
#endif

static LidarSlam         slam;
static SemaphoreHandle_t lock = nullptr;     // slam, nav, scans: lidar / ToF nav task vs. CLI / web server
static bool              mapOk = false;      // map allocated
static bool              noMemory = false;
static uint32_t          lastPacketMs = 0;   // last lidar packet / ToF frame
#ifdef NAV_LIDAR
static LR_pose2          scanStartOdom = { 0, 0, 0 };
static uint32_t          lastScanMs = 0;
#endif

// navigator -> motion task
static portMUX_TYPE      navLock = portMUX_INITIALIZER_UNLOCKED;
static LR_moveVector     navCmd = {};
static volatile bool     navActive = false;
static volatile uint32_t navCmdMs = 0;
static volatile bool     cancelRequest = false;

static bool navSupported()
{
#ifdef NAV_LIDAR
	return MODEL_LIDAR.lidar != nullptr;
#else
	return MODEL_LIDAR.tofScan != nullptr && tofSensorCount() > 0;
#endif
}

#ifdef NAV_LIDAR
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
	if (!navSupported()) {
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

bool lidarNavBody(float &tilt, float &crouch)
{
	tilt = 0;
	crouch = 0;
	return false;
}

bool lidarScan()
{
	return false;
}

#else   // NAV_TOF

static void tofNavTask(void *)
{
	TickType_t wake = xTaskGetTickCount();
	uint32_t last = millis();
	float mm[64];
	int16_t raw[64];
	for (;;) {
		vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
		uint32_t now = millis();
		float dt = (now - last) / 1000.0f;
		last = now;
		if (!mapOk) {
			if (noMemory || !tofReady(0)) {
				continue;
			}
			xSemaphoreTake(lock, portMAX_DELAY);
			mapOk = slam.begin(TOF_ORIGIN, *MODEL_LIDAR.tofMap);
			xSemaphoreGive(lock);
			if (!mapOk) {
				noMemory = true;
				Serial.println("ToF map: no memory for the map (40 KB)");
				continue;
			}
			Serial.println("ToF map: sensors running, map started (`lidar scan` or a goal scans the room)");
		}
		MotionOdometry odo = motionOdometry();
		xSemaphoreTake(lock, portMAX_DELAY);
		for (uint8_t i = 0; i < tofSensorCount() && i < 2; i++) {
			uint32_t seq = tofSeq[i];
			if (tofPeekFrame(i, seq, raw) && seq != tofSeq[i]) {
				tofSeq[i] = seq;
				lastPacketMs = now;
				for (uint8_t z = 0; z < 64; z++) {
					mm[z] = raw[z];
				}
				tofNav.addFrame(MODEL_TERRAIN.tof[i], mm, odo.pose, odo.tilt, odo.floorZ);
			}
		}
		if (cancelRequest) {
			tofNav.stop();
			cancelRequest = false;
		}
		if (scanRequest) {
			tofNav.scan();
			scanRequest = false;
		}
		if (odo.gapBlocked) {
			tofNav.gapAhead(tofNav.nav().params().stopDistance);   // terrain mode stopped at a gap: around it
		}
		LR_moveVector cmd = tofNav.update(dt, slam, odo.pose, odo.maxYawRate, odo.gaitActive);
		bool active = tofNav.active();
		float tilt = tofNav.tilt(), crouch = tofNav.crouch();
		xSemaphoreGive(lock);

		portENTER_CRITICAL(&navLock);
		navCmd    = cmd;
		navCmdMs  = now;
		navTilt   = tilt;
		navCrouch = crouch;
		portEXIT_CRITICAL(&navLock);
		navActive = active;
	}
}

void initLidar()
{
	if (!navSupported()) {
		return;
	}
	lock = xSemaphoreCreateMutex();
	tofNav.begin(*MODEL_LIDAR.tofScan, *MODEL_LIDAR.nav);
	// core 0 with WiFi and the web server: a scan's match is ~10 ms once in a while, planning a few ms a second
	xTaskCreatePinnedToCore(tofNavTask, "tofnav", 6144, nullptr, 1, nullptr, 0);
	Serial.println("Map: ToF room scans (no lidar)");
}

bool lidarNavBody(float &tilt, float &crouch)
{
	portENTER_CRITICAL(&navLock);
	tilt = navTilt;
	crouch = navCrouch;
	portEXIT_CRITICAL(&navLock);
	return navSupported();
}

bool lidarScan()
{
	if (!mapOk) {
		return false;
	}
	scanRequest = true;
	return true;
}

#endif

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
		// no updates: the lidar stopped (or the task is stuck), stand still
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
	cancelRequest = false;
	xSemaphoreTake(lock, portMAX_DELAY);
#ifdef NAV_LIDAR
	bool ok = nav.setGoal(x, y);
#else
	bool ok = tofNav.setGoal(x, y);
#endif
	xSemaphoreGive(lock);
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
	if (!navSupported() || room < 2) {
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
	uint16_t cell = (uint16_t)slam.grid().cellSize();
#ifdef NAV_LIDAR
	int8_t strong = 2 * MODEL_LIDAR.map->hit;
#else
	int8_t strong = 2 * MODEL_LIDAR.tofMap->hit;
#endif
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
 *      1 | u8   |      1 | flags: 1 sensor data (lidar / ToF), 2 map frozen, 4 blocked ahead, 8 no memory,
 *        |      |        |        16 ToF scan running, 32 ToF room scans (no lidar)
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
	if (!navSupported() || room < 19 + 4 * LR_NAV_PATH + 4 * LR_LIDAR_BINS / 2) {
		return 0;
	}
	MotionOdometry odo = motionOdometry();
	uint8_t *p = out;
	*p++ = P_NAV;
	if (!mapOk) {
#ifdef NAV_TOF
		*p++ = (noMemory ? 8 : 0) | 32;
#else
		*p++ = noMemory ? 8 : 0;
#endif
		memset(p, 0, 17);   // state .. scan count, all zero
		return 19;
	}
	xSemaphoreTake(lock, portMAX_DELAY);
	LR_pose2 pose = slam.pose(odo.pose);
	const Navigator &nv = NAV_STATE();
	uint8_t flags = (millis() - lastPacketMs < 1000 ? 1 : 0) | (slam.frozen() ? 2 : 0) | (NAV_BLOCKED() ? 4 : 0);
#ifdef NAV_TOF
	flags |= 32 | (tofNav.phase() != TOF_SCAN_IDLE ? 16 : 0);
#endif
	*p++ = flags;
	*p++ = nv.state();
	*p++ = slam.lastMatch();
	put16(p, (int)pose.x);
	put16(p, (int)pose.y);
	put16(p, (int)lroundf(pose.theta * RADTODEG * 100));
	put16(p, (int)nv.goalX());
	put16(p, (int)nv.goalY());
	put16(p, (int)(slam.scans() & 0xFFFF));
	const float *x, *y;
	uint8_t pn = nv.active() ? nv.path(x, y) : 0;
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

#ifdef NAV_LIDAR
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
#else
static const char *const phaseNames[] = { "", "stopping", "nose up", "turning around", "nose down" };
#endif

void cliLidar()
{
	char *mode = CLI_readWord();
	if (!navSupported()) {
		cliSerial->println("This model has no map support.");
		return;
	}
#ifdef NAV_LIDAR
	if (strcmp(mode, "print") == 0) {
		printDistances();
		return;
	}
#else
	if (strcmp(mode, "scan") == 0) {
		cliSerial->println(lidarScan() ? "Scanning the room: the robot stops, lifts its nose and turns once around."
		                               : "No map yet (ToF sensors not running? `tof state`).");
		return;
	}
#endif
	if (strcmp(mode, "reset") == 0 && mapOk) {
		lidarNavCancel();
		xSemaphoreTake(lock, portMAX_DELAY);
		slam.reset();
#ifdef NAV_LIDAR
		scanStartOdom = motionOdometry().pose;
#endif
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
#ifdef NAV_LIDAR
	cliSerial->printf("Lidar: %s, %.1f rev/s, %u packets, %u CRC errors\n",
	                  millis() - lastPacketMs < 1000 ? "running" : "no data (wiring? LIDAR_RX_PIN)",
	                  parser.speed() / 360.0f, (unsigned)parser.packets(), (unsigned)parser.crcErrors());
#else
	cliSerial->printf("ToF room scans: sensors %s\n", millis() - lastPacketMs < 1000 ? "running" : "no data (`tof state`)");
#endif
	if (!mapOk) {
		cliSerial->println(noMemory ? "Map: no memory" : "Map: starts with the first sensor data");
		return;
	}
	xSemaphoreTake(lock, portMAX_DELAY);
	LR_pose2 pose = slam.pose(odo.pose);
	const Navigator &nv = NAV_STATE();
	cliSerial->printf("Map: %u occupied cells%s, %u scans, %u rejected; last match %s, score %.2f\n",
	                  (unsigned)slam.grid().occupiedCells(), slam.frozen() ? " (frozen)" : "",
	                  (unsigned)slam.scans(), (unsigned)slam.rejected(), matchNames[slam.lastMatch()], slam.lastScore());
#ifdef NAV_TOF
	cliSerial->printf("Scan: %s, last scan %u points; gyro bias %.2f deg/s\n",
	                  tofNav.phase() == TOF_SCAN_IDLE ? "idle" : phaseNames[tofNav.phase()],
	                  (unsigned)tofNav.lastPoints(), motionOdometryBias() * RADTODEG);
#endif
	cliSerial->printf("Pose: x %.2f m, y %.2f m, heading %.0f deg (counter-clockwise from the start)\n",
	                  pose.x / 1000, pose.y / 1000, pose.theta * RADTODEG);
	cliSerial->printf("Goal: %s", navStateNames[nv.state()]);
	if (nv.state() != NAV_IDLE) {
		cliSerial->printf(", %.2f / %.2f m, %.2f m away%s", nv.goalX() / 1000, nv.goalY() / 1000,
		                  nv.distance() / 1000, NAV_BLOCKED() ? ", blocked ahead" : "");
	}
	cliSerial->println(FS_FAIL && nv.active() ? " (no client: standing)" : "");
	xSemaphoreGive(lock);
}

void cliGoto()
{
	float x = CLI_readFloat();
	float y = CLI_readFloat();
	if (!mapOk) {
		cliSerial->println("No map yet (`lidar state`).");
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
bool lidarNavBody(float &tilt, float &crouch) { tilt = 0; crouch = 0; return false; }
bool lidarScan() { return false; }
void lidarNavCancel() {}
size_t pMap(uint8_t *out, size_t room, uint8_t part) { (void)out; (void)room; (void)part; return 0; }
size_t pNav(uint8_t *out, size_t room) { (void)out; (void)room; return 0; }
bool lidarGoto(float x, float y) { (void)x; (void)y; return false; }
void lidarStop() {}
void cliLidar() { cliSerial->println("No map: needs LIDAR_TYPE or a VL53L5CX (config.h)."); }
void cliGoto() { cliSerial->println("No map: needs LIDAR_TYPE or a VL53L5CX (config.h)."); }

#endif
