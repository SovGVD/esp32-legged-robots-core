/**
 * 2D lidar (LDROBOT LD06 / LD19, and other 360 degree scanners): poses in the
 * plane, the LD06 serial protocol and one revolution as a scan.
 *
 * Map frame (LidarSlam, Navigator): x right, y forward at the place the map
 * was started, mm; theta counter-clockwise seen from above, rad. A robot at
 * theta faces (-sin theta, cos theta), the same convention as the gait's yaw
 * (GaitEngine, lrRotateZ). Body frame: x right, y forward.
 */
#ifndef LR_LIDAR_H
#define LR_LIDAR_H

#include "lr_compat.h"
#include "geometry.h"
#include <stddef.h>

/** Pose in the plane: mm, rad (counter-clockwise, 0 = facing +y). */
typedef struct {
	float x;
	float y;
	float theta;
} LR_pose2;

/** a then b: b given in a's frame, result in a's parent frame. */
LR_pose2 lrPoseCompose(const LR_pose2 &a, const LR_pose2 &b);
/** The pose that undoes a: compose(a, inverse(a)) = 0. */
LR_pose2 lrPoseInverse(const LR_pose2 &a);
/** Point (x, y) of a's frame in a's parent frame. */
void lrPoseApply(const LR_pose2 &a, float x, float y, float &ox, float &oy);
/** Pose between a (f = 0) and b (f = 1), the angle the short way round. */
LR_pose2 lrPoseLerp(const LR_pose2 &a, const LR_pose2 &b, float f);

/**
 * Where the lidar sits and how it counts angles.
 *
 * LD06 / LD19: angles grow clockwise seen from above (clockwise = true).
 * Where 0 degrees points depends on how the sensor is turned on the robot:
 * check it with `lidar print` (a box 30 cm in front of the robot must show
 * up as "front") and set yaw.
 */
typedef struct {
	LR_point position;     // mm, body frame (z: height of the scan plane above the body origin)
	float    yaw;          // rad, body direction of the sensor's 0 degrees (counter-clockwise from forward)
	bool     clockwise;    // sensor angles grow clockwise seen from above
	float    minRange;     // mm, closer readings are the robot itself (legs, wires)
	float    maxRange;     // mm, further readings are ignored
	uint8_t  minIntensity; // readings weaker than this are noise (LD06: ~200 on white walls, 0 = none)
} LidarParams;

/* --------------------------------------------------------------- LD06 */

#define LD06_HEADER      0x54
#define LD06_VER_LEN     0x2C   // 12 points per packet
#define LD06_POINTS      12
#define LD06_PACKET_LEN  47
#define LD06_BAUD        230400

/** CRC-8 of the LD06 packets (polynomial 0x4D). */
uint8_t lrLd06Crc8(const uint8_t *data, size_t len);

/**
 * LD06 / LD19 serial packets. Feed every byte from the UART; feed() returns
 * true when a packet with a good CRC is complete, then read its 12 points.
 *
 * Packet (little-endian): 0x54, 0x2C, speed (deg/s, u16), start angle
 * (0.01 deg, u16), 12 x (distance mm u16, intensity u8), end angle
 * (0.01 deg, u16), timestamp (ms, u16), CRC-8.
 */
class Ld06Parser {
	public:
		void reset() { _len = 0; }
		bool feed(uint8_t b);
		float    speed() const;              // deg/s (3600 = 10 rev/s)
		float    angle(uint8_t i) const;     // deg [0, 360) of point i
		uint16_t distance(uint8_t i) const;  // mm, 0 = no target
		uint8_t  intensity(uint8_t i) const;
		uint16_t timestamp() const;          // ms, wraps at 30000
		uint32_t packets() const { return _packets; }
		uint32_t crcErrors() const { return _crcErrors; }

	private:
		uint8_t  _buf[LD06_PACKET_LEN];
		uint8_t  _len = 0;
		uint32_t _packets = 0;
		uint32_t _crcErrors = 0;
		uint16_t u16(uint8_t at) const { return (uint16_t)(_buf[at] | (_buf[at + 1] << 8)); }
};

/** Build an LD06 packet (tests and the simulator): 12 points from start to end angle (0.01 deg). */
void lrLd06Encode(uint8_t out[LD06_PACKET_LEN], uint16_t speed, uint16_t startAngle, uint16_t endAngle,
                  const uint16_t *mm, const uint8_t *intensity, uint16_t timestamp);

/* --------------------------------------------------------------- scans */

#define LR_LIDAR_BINS 360   // one bin per degree of the sensor's own angle

/**
 * One revolution: the closest surface in each degree of the sensor's angle,
 * mm, 0 = nothing there (or filtered out). Bins are in the sensor's order,
 * so bin i was measured i/360 of the way through the revolution: the
 * revolution starts at sensor angle 0.
 */
typedef struct {
	uint16_t mm[LR_LIDAR_BINS];
	uint16_t points;   // bins with a reading
} LidarScan;

/** Body frame direction of scan bin `bin`: unit (dx, dy). */
void lrLidarBinDirection(const LidarParams &p, uint16_t bin, float &dx, float &dy);

/**
 * Collects points into revolutions. add() returns true when the point starts
 * a new revolution: the previous one is complete, read it with scan().
 */
class LidarScanBuilder {
	public:
		void begin(const LidarParams &params) { _params = params; reset(); }
		void reset();
		bool add(float angleDeg, uint16_t mm, uint8_t intensity);
		const LidarScan &scan() const { return _done; }
		uint32_t scans() const { return _scans; }

	private:
		LidarParams _params = { { 0, 0, 0 }, 0, true, 50, 8000, 0 };
		LidarScan   _current;
		LidarScan   _done;
		float       _last = -1;
		uint32_t    _scans = 0;
};

#endif
