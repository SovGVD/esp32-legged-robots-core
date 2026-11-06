"""Hardware in the loop link: the Python side speaks the firmware's bytes, and
the sim walks when a (stand-in) board drives it over the link."""

import ctypes

import numpy as np
import pytest

from legsim import FirmwareCore, SimDog
from legsim import hil


@pytest.fixture(scope="module")
def fw():
    return FirmwareCore()


def _cpp(fw):
    L = fw.lib
    L.rc_hil_encode_servos.restype = ctypes.c_int
    L.rc_hil_encode_servos.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int, ctypes.c_int,
                                       ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint8)]
    L.rc_hil_encode_hello.restype = ctypes.c_int
    L.rc_hil_encode_hello.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
    L.rc_hil_parse.restype = ctypes.c_int
    L.rc_hil_parse.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float),
                               ctypes.POINTER(ctypes.c_uint32)]
    return L


def _cpp_parse(L, data):
    out = (ctypes.c_float * 70)()
    err = (ctypes.c_uint32 * 2)()
    kind = L.rc_hil_parse(bytes(data), len(data), out, err)
    return kind, np.array(out[:]), (err[0], err[1])


def test_crc_known_value():
    assert hil.crc16(b"123456789") == 0x29B1   # CRC-16/CCITT-FALSE check value


def test_firmware_servo_frame_decodes_in_python(fw):
    L = _cpp(fw)
    us = (ctypes.c_uint16 * 12)(*range(1000, 1012))
    buf = (ctypes.c_uint8 * 64)()
    n = L.rc_hil_encode_servos(77, 4321, 1, 12, us, buf)
    data = bytes(buf[:n])
    assert data == hil.encode_servos(77, 4321, True, range(1000, 1012))
    (kind, payload), = hil.Parser().feed(data)
    msg = hil.decode(kind, payload)
    assert kind == hil.SERVOS and msg["seq"] == 77 and msg["dt_us"] == 4321 and msg["enabled"]
    assert list(msg["pulses"]) == list(range(1000, 1012))


def test_firmware_hello(fw):
    L = _cpp(fw)
    buf = (ctypes.c_uint8 * 32)()
    n = L.rc_hil_encode_hello(buf)
    (kind, payload), = hil.Parser().feed(bytes(buf[:n]))
    msg = hil.decode(kind, payload)
    assert msg["version"] == hil.VERSION and hil.MODELS[msg["model"]] == "dog"
    assert (msg["legs"], msg["dof"]) == (4, 3)
    assert abs(msg["loop_time_us"] / 1e6 - fw.loop_dt) < 1e-6
    assert msg["tof_count"] == len(fw.tof_sensors())


def test_python_frames_decode_in_firmware(fw):
    L = _cpp(fw)
    kind, v, _ = _cpp_parse(L, hil.encode_sensors((0.1, -0.2, 1.0), (0.01, 0.02, -0.3), (12, -5, 40), 0b1010, 7.9, 1.25))
    assert kind == hil.SENSORS
    np.testing.assert_allclose(v[:13], [0.1, -0.2, 1.0, 0.01, 0.02, -0.3, 12, -5, 40, hil.FLAG_MAG, 0b1010, 7.9, 1.25],
                               rtol=1e-6)
    kind, v, _ = _cpp_parse(L, hil.encode_sensors((0, 0, 1), (0, 0, 0)))
    assert kind == hil.SENSORS and v[9] == 0   # no new magnetometer reading
    zones = np.arange(64) * 10
    kind, v, _ = _cpp_parse(L, hil.encode_tof(1, zones))
    assert kind == hil.TOF and v[0] == 1 and list(v[1:65]) == list(zones)
    kind, v, _ = _cpp_parse(L, hil.encode_move((0.5, -1, 0, 0.25, 0, 1)))
    assert kind == hil.MOVE and list(v[:6]) == [0.5, -1, 0, 0.25, 0, 1]


def test_firmware_parser_skips_text_and_bad_frames(fw):
    L = _cpp(fw)
    good = hil.encode_move((0, 1, 0, 0, 0, 0))
    bad = bytearray(good)
    bad[6] ^= 0xFF
    kind, v, (crc_errors, skipped) = _cpp_parse(L, b"Boot log\r\n" + bytes(bad) + good)
    assert kind == hil.MOVE and v[1] == 1 and crc_errors == 1 and skipped >= 10


def test_python_parser_splits_text_and_frames():
    p = hil.Parser()
    f = hil.encode_servos(1, 4300, True, [1500] * 12)
    data = b"MainSetup complete\r\n" + f + b"Failsafe: no commands\n" + f
    out = []
    for i in range(0, len(data), 7):   # arrives in pieces
        out += p.feed(data[i:i + 7])
    assert out[0] == ("text", "MainSetup complete")
    assert out[1][0] == hil.SERVOS and out[2] == ("text", "Failsafe: no commands") and out[3][0] == hil.SERVOS
    bad = bytearray(f)
    bad[8] ^= 1
    assert p.feed(bytes(bad) + f)[-1][0] == hil.SERVOS and p.crc_errors == 1


def test_board_walks_the_sim_over_the_link(fw):
    """A stand-in board (the firmware motion code, fed only from the link's sensor
    frames) drives the sim through HilSim: it stands, then walks forward on 'M'."""
    fw.reset()
    robot = fw.place(SimDog())
    sim = hil.HilSim(robot, fw.tof_sensors(), battery=7.9)
    board = hil.FakeEsp(fw)
    to_pc = board.hello()
    for _ in range(int(0.5 / fw.loop_dt)):
        board.feed(sim.feed(to_pc))
        to_pc = board.tick()
    assert sim.hello is not None and sim.ticks > 0
    start = robot.body_position_mm().copy()
    assert start[2] > 50   # standing
    board.feed(hil.encode_move((0, 1, 0, 0, 0, 0)))
    tof_frames = 0
    for _ in range(int(3.0 / fw.loop_dt)):
        reply = sim.feed(to_pc)
        tof_frames += reply.count(hil.SYNC + bytes([hil.TOF]))
        board.feed(reply)
        to_pc = board.tick()
    p = robot.body_position_mm()
    assert p[1] - start[1] > 100, p - start      # walked forward
    assert p[2] > 50                             # still up
    assert abs(sim.sim_time - sim.ticks * fw.loop_dt) < 1e-6
    assert sim.lost == 0 and sim.bad == 0
    assert 30 <= tof_frames <= 60                # 15 Hz for 3 s
    assert board.sensors is not None and abs(board.sensors["voltage"] - 7.9) < 1e-6
    assert board.sensors["feet"] != 0            # feet on the ground


def test_disabled_servos_go_limp(fw):
    fw.reset()
    robot = fw.place(SimDog())
    sim = hil.HilSim(robot)
    sim.feed(hil.encode_servos(1, 4300, False, [1500] * 12))
    assert np.isnan(robot.servos.pending).all()
