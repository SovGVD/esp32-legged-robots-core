"""Command mapping of the keyboard/gamepad control (no window needed)."""

import pytest

from legsim.control import Command, gamepad_command


def test_gamepad_stick_up_is_forward():
    c = gamepad_command({"leftx": 0.0, "lefty": -1.0, "rightx": 1.0}, tilt_mode=False).as_tuple()
    assert c[1] == pytest.approx(1.0)    # move y forward
    assert c[5] == pytest.approx(1.0)    # yaw right
    assert c[3] == c[4] == 0.0


def test_gamepad_deadband():
    c = gamepad_command({"leftx": 0.05, "lefty": -0.1, "rightx": 0.11}, tilt_mode=False)
    assert c.as_tuple() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_gamepad_tilt_mode():
    c = gamepad_command({"rightx": 1.0, "righty": -1.0}, tilt_mode=True).as_tuple()
    assert c[3] == pytest.approx(1.0) and c[4] == pytest.approx(1.0) and c[5] == 0.0


def test_keyboard(monkeypatch):
    pygame = pytest.importorskip("pygame")
    from legsim.control import keyboard_command
    c = keyboard_command({pygame.K_w, pygame.K_e}, pygame).as_tuple()
    assert c[1] == 1.0 and c[5] == 1.0
    c = keyboard_command({pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_LSHIFT}, pygame).as_tuple()
    assert c[1] == 0.0 and c[0] == pytest.approx(-0.4)


def test_sum_is_clamped():
    c = (Command(y=1.0) + Command(y=1.0)).as_tuple()
    assert c[1] == 1.0
