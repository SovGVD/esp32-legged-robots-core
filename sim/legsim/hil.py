"""
Hardware in the loop: a real ESP32 runs the whole firmware (built with -DHIL,
`pio run -e robot-dog-hil`), the simulator plays its servos, IMU, power
sensor, ToF sensors and foot switches over the USB serial port.

The protocol is lib/robot_core/src/hil_link.h (this file is its Python
side): frames `A5 5A type length payload crc16`, little endian.

  ESP32 -> PC  'H' hello (model, legs, dof, loop time), 'S' servo pulses + dt, every motion tick
  PC -> ESP32  'I' sensors after every 'S', 'T' ToF frames (15 Hz), 'M' move command

`HilSim` is the PC side without the port: give it the bytes that came in,
it steps the MuJoCo robot by the ESP32's own tick time and returns the bytes
to send back. `FakeEsp` stands in for the board (the firmware motion code
built for the PC, FirmwareCore) for tests and `examples/hil.py --fake`.
"""

import math
import struct

import numpy as np

SYNC = b"\xa5\x5a"
VERSION = 1
HELLO, SERVOS, SENSORS, TOF, MOVE = (ord(c) for c in "HSITM")
FLAG_MAG = 0x01
TOF_ZONES = 64
MODELS = {100: "dog", 103: "hexapod"}     # ROBOT_MODEL (def/def.h)
G = 9.81


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (hilCrc16)."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(kind, payload):
    if len(payload) > 255:
        raise ValueError("payload too long")
    body = bytes([kind, len(payload)]) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


# ------------------------------------------------------------------ encoders
def encode_hello(model=100, legs=4, dof=3, loop_time_us=4300, tof_count=1, version=VERSION):
    return frame(HELLO, struct.pack("<BBBBIB", version, model, legs, dof, loop_time_us, tof_count))


def encode_servos(seq, dt_us, enabled, pulses):
    pulses = [int(p) for p in pulses]
    return frame(SERVOS, struct.pack(f"<IIBB{len(pulses)}H", seq, dt_us, 1 if enabled else 0, len(pulses), *pulses))


def encode_sensors(accel, gyro, mag=None, feet=0, voltage=0.0, current=0.0):
    """accel g, gyro rad/s, mag uT (None: no new reading), body frame x right, y forward, z up.
    feet: bit i = leg i pressed."""
    flags = FLAG_MAG if mag is not None else 0
    m = mag if mag is not None else (0.0, 0.0, 0.0)
    return frame(SENSORS, struct.pack("<9fBBff", *accel, *gyro, *m, flags, int(feet) & 0xFF, voltage, current))


def encode_tof(sensor, mm):
    mm = np.clip(np.round(np.asarray(mm, dtype=float)), 0, 32767).astype(int)
    if len(mm) != TOF_ZONES:
        raise ValueError("a ToF frame has 64 zones")
    return frame(TOF, struct.pack(f"<B{TOF_ZONES}h", sensor, *mm))


def encode_move(command):
    """command: x, y, z, pitch, roll, yaw in -1..1 (FirmwareCore.update order)."""
    return frame(MOVE, struct.pack("<6f", *[float(c) for c in command]))


# ------------------------------------------------------------------ decoders
def decode(kind, p):
    """A frame's payload as a dict (None if its length is wrong for the type)."""
    try:
        if kind == HELLO:
            v, model, legs, dof, loop, tof = struct.unpack("<BBBBIB", p)
            return {"version": v, "model": model, "legs": legs, "dof": dof, "loop_time_us": loop, "tof_count": tof}
        if kind == SERVOS:
            seq, dt, en, n = struct.unpack_from("<IIBB", p)
            if len(p) != 10 + 2 * n:
                return None
            return {"seq": seq, "dt_us": dt, "enabled": bool(en), "pulses": np.array(struct.unpack_from(f"<{n}H", p, 10))}
        if kind == SENSORS:
            v = struct.unpack("<9fBBff", p)
            return {"accel": v[0:3], "gyro": v[3:6], "mag": v[6:9] if v[9] & FLAG_MAG else None,
                    "feet": v[10], "voltage": v[11], "current": v[12]}
        if kind == TOF:
            v = struct.unpack(f"<B{TOF_ZONES}h", p)
            return {"sensor": v[0], "mm": np.array(v[1:])}
        if kind == MOVE:
            return {"command": struct.unpack("<6f", p)}
    except struct.error:
        return None
    return None


class Parser:
    """Bytes in, (kind, payload) frames out. Bytes outside frames are the
    firmware's text (Serial.print): they come back as ("text", line) once a
    line is complete."""

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()
        self.crc_errors = 0
        self.frames = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # keep a trailing A5: it may be the start of a sync
                keep = 1 if self.buf.endswith(SYNC[:1]) else 0
                self._text(self.buf[:len(self.buf) - keep], out)
                del self.buf[:len(self.buf) - keep]
                break
            if i > 0:
                self._text(self.buf[:i], out)
                del self.buf[:i]
            if len(self.buf) < 4:
                break
            n = self.buf[3]
            if len(self.buf) < n + 6:
                break
            body = bytes(self.buf[2:4 + n])
            crc = struct.unpack_from("<H", self.buf, 4 + n)[0]
            if crc != crc16(body):
                # not a frame after all (or a damaged one): skip the sync, look again
                self.crc_errors += 1
                del self.buf[:1]
                continue
            out.append((body[0], body[2:]))
            self.frames += 1
            del self.buf[:n + 6]
        return out

    def _text(self, data, out):
        for b in data:
            if b == 10:   # \n
                out.append(("text", self.text.decode("utf-8", "replace").rstrip("\r")))
                self.text.clear()
            elif len(self.text) < 400:
                self.text.append(b)


# --------------------------------------------------------------- PC side
class HilSim:
    """The robot's body for an ESP32 running the HIL firmware.

    feed(data) takes the bytes from the port and returns the bytes to send:
    for every 'S' frame the servo pulses go to the robot and the physics moves
    on by the ESP32's dt; after the last one the sensors (and a ToF frame when
    the sensors have one) go back. Firmware text lines collect in `log` and `new_lines`.
    """

    MAX_DT = 0.05   # s, a stalled ESP32 tick (the firmware clamps the same: MOTION_MAX_DT)

    def __init__(self, robot, tof_sensors=(), battery=8.0, gyro_bias_dps=0.5, magnetometer=True,
                 mag_noise=0.5, field=(0.0, 20.0, -40.0), switch_threshold=0.3, tof_args=None, seed=0,
                 on_hello=None):
        """robot: SimDog or SimHexapod. tof_sensors: FirmwareCore.tof_sensors() (mounts and zone rays).
        battery: V reported to the firmware's battery protection. gyro_bias_dps: z gyro drift.
        field: earth field in the world frame (uT, x east, y north). on_hello(hello): the ESP32 (re)booted."""
        from .tof import ToFArray
        self.robot = robot
        self.battery = battery
        self.current = 0.0
        self.bias = math.radians(gyro_bias_dps)
        self.magnetometer = magnetometer
        self.mag_noise = mag_noise
        self.field = np.array(field, dtype=float)
        self.switch_threshold = switch_threshold
        self.rng = np.random.default_rng(seed)
        self.tof = ToFArray(list(tof_sensors), seed=seed, **(tof_args or {})) if tof_sensors else None
        self.parser = Parser()
        self.on_hello = on_hello
        self.hello = None
        self.log = []           # the board's last text lines
        self.new_lines = []     # text lines nobody has printed yet (the caller clears it)
        self.ticks = 0
        self.sim_time = 0.0
        self.last_seq = None
        self.lost = 0           # ESP32 ticks whose servo frame never arrived
        self.bad = 0
        self.enabled = True
        self.last_pulses = None
        self.feet = 0
        self._R = None
        self._mag_tick = 0
        self.dt_sum = 0.0

    # sensors ------------------------------------------------------------
    def _gyro(self, dt):
        """The body's mean turn rate over the tick (like a gyro behind its low-pass filter), rad/s."""
        r = self.robot
        R = r.data.xmat[r._torso].reshape(3, 3).copy()
        prev, self._R = self._R, R
        if prev is None or dt <= 0:
            rate = np.array(r.read_imu()["gyro"], dtype=float)
        else:
            rel = prev.T @ R
            angle = math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1) / 2)))
            axis = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
            n = np.linalg.norm(axis)
            rate = axis / n * angle / dt if n > 1e-12 else np.zeros(3)
        rate[2] += self.bias
        return rate, R

    def sensors(self, dt):
        """The 'I' frame for now (and advances the magnetometer's 100 Hz clock)."""
        gyro, R = self._gyro(dt)
        accel = np.array(self.robot.read_imu()["accel"], dtype=float) / G
        mag = None
        self._mag_tick += 1
        if self.magnetometer and self._mag_tick >= 4:
            self._mag_tick = 0
            mag = R.T @ self.field + self.rng.normal(0, self.mag_noise, 3)
        sw = self.robot.foot_switches(self.switch_threshold)
        self.feet = sum(1 << i for i, c in enumerate(sw) if c)
        return encode_sensors(accel, gyro, mag, self.feet, self.battery, self.current)

    # stepping -----------------------------------------------------------
    def reset(self):
        self._R = None
        self._mag_tick = 0
        self.last_seq = None
        if self.tof is not None:
            self.tof.reset()

    def apply_servos(self, msg):
        if self.last_seq is not None and msg["seq"] > self.last_seq + 1:
            self.lost += msg["seq"] - self.last_seq - 1
        self.last_seq = msg["seq"]
        self.enabled = msg["enabled"]
        pulses = msg["pulses"] if msg["enabled"] else np.zeros_like(msg["pulses"])
        self.last_pulses = msg["pulses"]
        self.robot.write_pulses(pulses)
        dt = min(max(msg["dt_us"], 0) / 1e6, self.MAX_DT)
        self.robot.step(dt)
        self.sim_time += dt
        self.dt_sum += dt
        self.ticks += 1
        return dt

    def feed(self, data):
        """Bytes from the ESP32 in, bytes to send back out."""
        dt = 0.0
        stepped = False
        for kind, payload in self.parser.feed(data):
            if kind == "text":
                self.log.append(payload)
                del self.log[:-200]
                self.new_lines.append(payload)
                continue
            msg = decode(kind, payload)
            if msg is None:
                self.bad += 1
                continue
            if kind == HELLO:
                self.hello = msg
                self.reset()
                if self.on_hello:
                    self.on_hello(msg)
            elif kind == SERVOS:
                dt += self.apply_servos(msg)
                stepped = True
        if not stepped:
            return b""
        out = self.sensors(dt)
        if self.tof is not None:
            frames = self.tof.poll(self.robot, dt)
            if frames is not None:
                for i, f in enumerate(frames):
                    out += encode_tof(i, f)
        return out


# ------------------------------------------------------------ stand-in board
class FakeEsp:
    """The ESP32 side without a board, for tests and `hil.py --fake`: the firmware
    motion code built for the PC (FirmwareCore) fed from 'I' / 'T' / 'M' frames
    through the same TiltEstimator, answering with 'S' frames. No WiFi, CLI or
    failsafe: those need the real board."""

    def __init__(self, fw, model_id=100):
        self.fw = fw
        self.parser = Parser()
        self.seq = 0
        self.command = (0.0,) * 6
        self.imu = None
        self.model_id = model_id
        self.sensors = None

    def hello(self):
        return encode_hello(self.model_id, self.fw.legs, self.fw.dof, int(round(self.fw.loop_dt * 1e6)),
                            len(self.fw.tof_sensors()))

    def feed(self, data):
        for kind, payload in self.parser.feed(data):
            if kind == "text":
                continue
            msg = decode(kind, payload)
            if msg is None:
                continue
            if kind == SENSORS:
                self.sensors = msg
            elif kind == MOVE:
                self.command = msg["command"]
            elif kind == TOF and self.fw.tof is None:
                pass

    def tick(self, dt=None):
        """One motion tick: the newest sensors in, an 'S' frame out."""
        import ctypes
        dt = self.fw.loop_dt if dt is None else dt
        imu = None
        if self.sensors is not None:
            f3 = ctypes.c_float * 3
            out = (ctypes.c_float * 2)()
            self.fw.lib.rc_tilt_estimate(dt, f3(*self.sensors["accel"]), f3(*self.sensors["gyro"]), out)
            imu = (out[0], out[1], 0.0)
        pulses = self.fw.update(self.command, dt=dt, imu=imu)
        self.seq += 1
        return encode_servos(self.seq, int(round(dt * 1e6)), True, pulses)
