"""
Driving the simulated robot by hand: keyboard and Xbox 360 (or any SDL
compatible) gamepad, through a small pygame window.

The command is the firmware's LR_moveVector, each value in [-1, 1], with the
same meaning as the web client's on-screen joysticks:

  move x   + = strafe right        move y   + = forward
  yaw      + = turn right          pitch/roll  body tilt (BodyKinematics)

Keyboard (focus the "robot control" window; keys act while held):
  W / S or Up / Down      forward / back
  A / D                   strafe left / right
  Q / E or Left / Right   turn left / right
  I / K, J / L            tilt body: pitch, roll
  Shift                   slow (40 %)
  G  switch gait (trot / static walk, while standing)
  R  reset robot    P  pause    Esc  quit

Xbox 360 gamepad:
  left stick              move (forward/back, strafe)
  right stick X           turn
  hold LB + right stick   tilt body instead of turning
  Y  switch gait          Back  reset robot       Start  pause

Keyboard and gamepad add up, so either can be used at any time.

poll() also reports events: "reset", "pause", "quit", "gait".
"""

from dataclasses import dataclass

DEADBAND = 0.12
SLOW = 0.4


def _clamp(v):
    return max(-1.0, min(1.0, v))


def _dead(v, band=DEADBAND):
    if abs(v) < band:
        return 0.0
    # rescale so the output still starts at 0 just outside the deadband
    return (abs(v) - band) / (1.0 - band) * (1 if v > 0 else -1)


@dataclass
class Command:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def as_tuple(self):
        """FirmwareCore.update() order: move x, y, z, rotate pitch, roll, yaw."""
        return (_clamp(self.x), _clamp(self.y), 0.0, _clamp(self.pitch), _clamp(self.roll), _clamp(self.yaw))

    def __add__(self, o):
        return Command(self.x + o.x, self.y + o.y, self.yaw + o.yaw, self.pitch + o.pitch, self.roll + o.roll)


def keyboard_command(held, pygame):
    """Command from the set of held pygame key codes."""
    k = pygame
    def axis(pos, neg):
        return (1.0 if held & pos else 0.0) - (1.0 if held & neg else 0.0)
    c = Command(
        x=axis({k.K_d}, {k.K_a}),
        y=axis({k.K_w, k.K_UP}, {k.K_s, k.K_DOWN}),
        yaw=axis({k.K_e, k.K_RIGHT}, {k.K_q, k.K_LEFT}),
        pitch=axis({k.K_i}, {k.K_k}),
        roll=axis({k.K_l}, {k.K_j}),
    )
    if held & {k.K_LSHIFT, k.K_RSHIFT}:
        c = Command(c.x * SLOW, c.y * SLOW, c.yaw * SLOW, c.pitch * SLOW, c.roll * SLOW)
    return c


def gamepad_command(axes, tilt_mode):
    """Command from SDL GameController axes (dict name -> value in [-1, 1], stick up = -1)."""
    lx, ly = _dead(axes.get("leftx", 0.0)), _dead(axes.get("lefty", 0.0))
    rx, ry = _dead(axes.get("rightx", 0.0)), _dead(axes.get("righty", 0.0))
    if tilt_mode:
        return Command(x=lx, y=-ly, pitch=-ry, roll=rx)
    return Command(x=lx, y=-ly, yaw=rx)


class ManualControl:
    """Keyboard + gamepad input in a small pygame window.

    poll() returns (Command, events) where events is a set of "reset", "pause", "quit".
    """

    def __init__(self, title="robot control", window=True):
        import pygame
        import pygame._sdl2.controller as controller
        self.pg = pygame
        self.ctl = controller
        # only the parts we need: no audio (pygame.init() would open the sound card)
        pygame.display.init()
        pygame.font.init()
        controller.init()
        self.screen = pygame.display.set_mode((460, 300)) if window else None
        if window:
            pygame.display.set_caption(title)
            self.font = pygame.font.SysFont(None, 22)
        self.held = set()
        self.pads = {}
        for i in range(controller.get_count()):
            self._open_pad(i)
        self.command = Command()

    def _open_pad(self, index):
        if self.ctl.is_controller(index):
            pad = self.ctl.Controller(index)
            self.pads[pad.as_joystick().get_instance_id()] = pad

    def gamepad_names(self):
        return [p.name for p in self.pads.values()]

    def poll(self):
        pg = self.pg
        events = set()
        for e in pg.event.get():
            if e.type == pg.QUIT:
                events.add("quit")
            elif e.type == pg.KEYDOWN:
                self.held.add(e.key)
                if e.key == pg.K_ESCAPE:
                    events.add("quit")
                elif e.key == pg.K_r:
                    events.add("reset")
                elif e.key == pg.K_p:
                    events.add("pause")
                elif e.key == pg.K_g:
                    events.add("gait")
            elif e.type == pg.KEYUP:
                self.held.discard(e.key)
            elif e.type == pg.CONTROLLERDEVICEADDED:
                self._open_pad(e.device_index)
            elif e.type == pg.CONTROLLERDEVICEREMOVED:
                self.pads.pop(e.instance_id, None)
            elif e.type == pg.CONTROLLERBUTTONDOWN:
                if e.button == pg.CONTROLLER_BUTTON_BACK:
                    events.add("reset")
                elif e.button == pg.CONTROLLER_BUTTON_START:
                    events.add("pause")
                elif e.button == pg.CONTROLLER_BUTTON_Y:
                    events.add("gait")
            elif e.type == pg.WINDOWFOCUSLOST:
                self.held.clear()   # keys released while another window has focus never arrive

        cmd = keyboard_command(self.held, pg)
        for pad in self.pads.values():
            axes = {
                "leftx": pad.get_axis(pg.CONTROLLER_AXIS_LEFTX) / 32767.0,
                "lefty": pad.get_axis(pg.CONTROLLER_AXIS_LEFTY) / 32767.0,
                "rightx": pad.get_axis(pg.CONTROLLER_AXIS_RIGHTX) / 32767.0,
                "righty": pad.get_axis(pg.CONTROLLER_AXIS_RIGHTY) / 32767.0,
            }
            cmd = cmd + gamepad_command(axes, pad.get_button(pg.CONTROLLER_BUTTON_LEFTSHOULDER))
        self.command = cmd
        return cmd, events

    def draw(self, status_lines=()):
        if self.screen is None:
            return
        pg = self.pg
        self.screen.fill((24, 26, 30))
        c = self.command.as_tuple()
        lines = [
            "Focus this window to drive with the keyboard.",
            "WASD/arrows move+turn, Q/E turn, IJKL tilt, Shift slow",
            "G gait, R reset, P pause, Esc quit. Pad: sticks, LB tilt, Y gait",
            "gamepads: " + (", ".join(self.gamepad_names()) or "none"),
            "",
            f"move x {c[0]:+.2f}   y {c[1]:+.2f}   yaw {c[5]:+.2f}",
            f"tilt pitch {c[3]:+.2f}   roll {c[4]:+.2f}",
            "",
            *status_lines,
        ]
        for i, text in enumerate(lines):
            self.screen.blit(self.font.render(text, True, (220, 220, 220)), (12, 10 + i * 22))
        pg.display.flip()

    def close(self):
        self.pg.quit()
