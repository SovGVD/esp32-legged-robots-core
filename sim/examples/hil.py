"""
Hardware in the loop: a real ESP32 is the robot's brain, the robot's body
(servos, IMU, battery, ToF sensors, foot switches) is this simulation.

  pio run -e robot-dog-hil -t upload                  # any ESP32 board, nothing connected to it
  python examples/hil.py --port /dev/ttyUSB0          # Windows: --port COM5
  python examples/hil.py --port /dev/ttyUSB0 --rooms  # two rooms: try the map / walk to a point
  python examples/hil.py --port /dev/ttyUSB0 --terrain bridge
  python examples/hil.py --fake                       # no board: the firmware motion code on the PC

The ESP32 runs everything it runs on the robot: the motion task at its own
rate, WiFi AP, web UI, CLI, failsafe, battery protection, ToF map, room
scans and navigation. Join its WiFi (SmallDogV1 / 1234567890 by default) with
a phone or a laptop and open http://192.168.1.1 to drive it from the web UI:
the sticks, the map tab (tap to walk there), the CLI tab (`hil` shows the
link). Or drive from this PC: keyboard / Xbox gamepad in the "robot control"
window (WASD, Q/E turn, IJKL tilt) sends move commands over the cable; let
go and the web UI has it again.

The model (dog or hexapod) comes from the board's hello. Every motion tick
the board sends its servo pulses and how long the tick took; the sim moves on
by that much and answers with the sensors, so the board's clock runs the
simulation (if the PC can't keep up, the sim runs slower than real time:
"sim/wall" in the window). The board's serial text is printed here.

R puts the robot back on its feet where it started (the board keeps its
state: map, heading). Opening the port resets most boards (DTR), so the
firmware boots fresh each start.

Needs `pip install pyserial pygame-ce`, a display and a C++ compiler (the
firmware geometry is built for the PC on the first run, like drive.py).
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legsim import display

display.prepare()   # X11 or Wayland, before the viewer and pygame load (see legsim/display.py)

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
from legsim import SimDog, FirmwareCore, hil  # noqa: E402
from legsim import scenario, terrain as terrains  # noqa: E402
from legsim.control import ManualControl  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
ap.add_argument("--port", help="the ESP32's serial port (/dev/ttyUSB0, /dev/ttyACM0, COM5)")
ap.add_argument("--baud", type=int, default=921600, help="HIL_BAUD of the firmware (default 921600)")
ap.add_argument("--fake", action="store_true", help="no board: the firmware motion code built for the PC")
ap.add_argument("--hexapod", action="store_true", help="with --fake: the hexapod (a board says which robot it is)")
ap.add_argument("--terrain", default="flat", choices=sorted(terrains.SCENES))
ap.add_argument("--height", type=float, help="obstacle / deck height, mm (default: per robot)")
ap.add_argument("--rooms", action="store_true", help="two rooms with furniture: the ToF map and `goto` / tap to walk")
ap.add_argument("--floor", type=float, metavar="MM", help="with --rooms: things on the floor up to MM high (default 0)")
ap.add_argument("--battery", type=float, default=8.0, help="battery voltage the board sees, V (2S LiPo: 6.4 critical)")
ap.add_argument("--gyro-bias", type=float, default=0.5, help="gyro drift, deg/s")
ap.add_argument("--no-mag", action="store_true", help="no magnetometer (an MPU6500 GY-91)")
ap.add_argument("--mass", type=float, metavar="G", help="hexapod: total robot mass, g")
args = ap.parse_args()
if not args.port and not args.fake:
    ap.error("give the board's --port, or --fake to try without one")


# ---------------------------------------------------------------- the link
class SerialLink:
    def __init__(self, port, baud):
        import serial
        self.port = serial.Serial(port, baud, timeout=0, write_timeout=0.05)
        self.name = f"{port} @ {baud}"

    def read(self):
        n = self.port.in_waiting
        return self.port.read(n) if n else b""

    def write(self, data):
        if data:
            try:
                self.port.write(data)
            except Exception as e:   # write timeout: the board is gone or resetting
                print("serial write:", e)


class FakeLink:
    """The board's side on the PC (hil.FakeEsp), ticking with the wall clock."""

    def __init__(self, model):
        self.fw_board = FirmwareCore(model=model)
        self.board = hil.FakeEsp(self.fw_board, 103 if model == "hexapod" else 100)
        self.pending = self.board.hello()
        self.start = None
        self.ticks = 0
        self.name = "fake board (PC)"

    def read(self):
        now = time.perf_counter()
        if self.start is None:
            self.start = now
        out, self.pending = self.pending, b""
        n = 0
        while self.ticks * self.fw_board.loop_dt < now - self.start and n < 10:
            out += self.board.tick()
            self.ticks += 1
            n += 1
        if n == 10:   # the PC is behind: don't try to catch up
            self.start = now - self.ticks * self.fw_board.loop_dt
        return out

    def write(self, data):
        self.board.feed(data)


link = FakeLink("hexapod" if args.hexapod else "dog") if args.fake else SerialLink(args.port, args.baud)

# wait for the board's hello: it says which robot it is
print(f"HIL: {link.name}, waiting for the board's hello (press its reset button if nothing comes)")
parser = hil.Parser()
hello = None
deadline = time.time() + 30
while hello is None:
    data = link.read()
    for kind, payload in parser.feed(data):
        if kind == "text":
            print("esp32>", payload)
        elif kind == hil.HELLO:
            hello = hil.decode(kind, payload)
    if hello is None:
        if time.time() > deadline:
            sys.exit("No hello from the board in 30 s: is it flashed with a *-hil env, and is the port / baud right?")
        time.sleep(0.01)
if hello["version"] != hil.VERSION:
    sys.exit(f"The board speaks HIL protocol {hello['version']}, this sim {hil.VERSION}: flash the firmware "
             "from the same checkout.")
model = hil.MODELS.get(hello["model"])
if model is None:
    sys.exit(f"The board's robot model {hello['model']} has no simulation (dog and hexapod only).")
print(f"HIL: {model}, {hello['legs']} legs x {hello['dof']} joints, motion tick {hello['loop_time_us']} us, "
      f"{hello['tof_count']} ToF")

# the firmware geometry (ToF mounts, standing pose) from the same sources, built for the PC
# (--fake: the board's own copy; one per process, the C++ side is global state)
fw = link.fw_board if args.fake else FirmwareCore(model=model)
if args.rooms:
    k = 1.0 if model == "dog" else 1.3   # lidar_nav.SCALE
    ground = terrains.rooms(scale=k, floor=args.floor or 0.0)
else:
    ground = scenario.scene(model, args.terrain, args.height)
if model == "hexapod":
    from legsim.hexapod import SimHexapod, HexapodParams
    from legsim.hexapod.params import with_mass
    robot = SimHexapod(params=with_mass(HexapodParams(), args.mass) if args.mass else HexapodParams(), terrain=ground)
    view_distance = 0.9
else:
    robot = SimDog(terrain=ground)
    view_distance = 0.7
if not args.fake:
    fw.reset()
fw.place(robot)

# the real VL53L5CX sees far (the firmware drops what is beyond its own ranges)
reach = fw.tof_scan_params()["max_range"] + 500
tof_sensors = [dict(s, max_range=reach) for s in fw.tof_sensors()][:hello["tof_count"]]


def rebooted(h):
    print("HIL: the board restarted")
    fw.place(robot)


sim = hil.HilSim(robot, tof_sensors, battery=args.battery, gyro_bias_dps=args.gyro_bias,
                 magnetometer=not args.no_mag, tof_args={"subrays": 2, "latency": 0.04}, on_hello=rebooted)
sim.hello = hello
ui = ManualControl(title="robot control (HIL)")

MOVE_EVERY = 0.03     # s between move frames while the PC drives (the failsafe wants one per 500 ms)
RELEASE_FOR = 0.3     # s of zero commands after letting go, then the web UI has it again
last_move = 0.0
driving_until = 0.0
rate = {"t": time.perf_counter(), "ticks": 0, "sim": 0.0, "tps": 0.0, "ratio": 0.0}

with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
    viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = view_distance, -20, 135
    next_draw = next_ui = 0.0
    while viewer.is_running():
        data = link.read()
        if data:
            link.write(sim.feed(data))
            for line in sim.new_lines:
                print("esp32>", line)
            sim.new_lines.clear()

        now = time.perf_counter()
        if now >= next_ui:
            next_ui = now + 1 / 60
            cmd, events = ui.poll()
            if "quit" in events:
                break
            if "reset" in events:
                fw.place(robot)
                sim.reset()
            c = cmd.as_tuple()
            if any(abs(v) > 1e-3 for v in c):
                driving_until = now + RELEASE_FOR
            if now < driving_until and now - last_move >= MOVE_EVERY:
                last_move = now
                link.write(hil.encode_move(c))

        if now >= next_draw:
            next_draw = now + 1 / 30
            if now - rate["t"] >= 1.0:
                span = now - rate["t"]
                rate.update(tps=(sim.ticks - rate["ticks"]) / span, ratio=(sim.sim_time - rate["sim"]) / span,
                            t=now, ticks=sim.ticks, sim=sim.sim_time)
            viewer.cam.lookat[:] = robot.data.xpos[robot._torso]
            if sim.tof is not None:
                viewer.user_scn.ngeom = 0
                sim.tof.draw(robot, viewer.user_scn)
            viewer.sync()
            o = robot.body_orientation()
            p = robot.body_position_mm()
            feet = "".join("x" if sim.feet >> i & 1 else "." for i in range(hello["legs"]))
            ui.draw(help_lines=["Phone / laptop: robot WiFi, http://192.168.1.1", "R: robot back on its feet"],
                    status_lines=[
                        f"{model} on {link.name}",
                        f"board ticks {rate['tps']:5.0f}/s   sim/wall {rate['ratio']:4.2f}   lost {sim.lost}"
                        f"   crc err {sim.parser.crc_errors}",
                        ("servos ON" if sim.enabled else "servos OFF") + f"   feet {feet}   battery {sim.battery:.2f} V"
                        f"   {ground.name}",
                        f"body x {p[0]:7.0f}  y {p[1]:7.0f}  z {p[2]:5.0f} mm",
                        f"pitch {math.degrees(o['pitch']):+5.1f}  roll {math.degrees(o['roll']):+5.1f}  "
                        f"yaw {math.degrees(o['yaw']):+6.1f} deg",
                        *[f"> {line[:60]}" for line in sim.log[-3:]],
                    ])
        if not data:
            time.sleep(0.0005)

# SDL (pygame) and GLFW (MuJoCo viewer) share the X11 connection, and their
# atexit teardown crashes or hangs on Linux (see drive.py)
sys.stdout.flush()
os._exit(0)
