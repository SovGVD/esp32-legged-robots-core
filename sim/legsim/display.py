"""
Pick a working window system for the live examples (drive.py, trot.py --view) on Linux.

Call prepare() before `import mujoco.viewer` and before pygame opens a window.

Two things go wrong on Linux desktops:

* The X server refuses the connection ("Authorization required, but no
  authorization protocol specified"): the shell has a stale or missing
  XAUTHORITY (an old tmux/screen, ssh, sudo, another user, an IDE sandbox).
  SDL then silently falls back to Wayland.
* On Wayland, SDL and GLFW draw window borders with libdecor, whose GTK plugin
  loads GTK 3 into the process. If GTK and the installed GNOME schemas don't
  match (e.g. "Settings schema 'org.gnome.settings-daemon.plugins.xsettings'
  does not contain a key named 'antialiasing'"), GLib aborts the whole Python
  process.

So: when X works, both windows use X11 (XWayland on a Wayland desktop), which
never loads libdecor. When it doesn't but Wayland does, both use Wayland with
libdecor off. When neither works, stop with a message that says how to fix it.
Variables you set yourself (SDL_VIDEODRIVER, PYGLFW_LIBRARY_VARIANT) win.
"""

import ctypes
import ctypes.util
import os
import sys

X11_HELP = """\
Can't open the X display {display!r} ({why}).
The MuJoCo viewer and the control window need a desktop session. Try, in a
terminal opened from the desktop itself (not ssh, sudo, or an old tmux/screen):
    echo $DISPLAY $XAUTHORITY $WAYLAND_DISPLAY $XDG_SESSION_TYPE
    xhost +SI:localuser:$(whoami)      # let your own user in, if xhost is installed
Inside tmux/screen, run that echo in a fresh desktop terminal and export the
same DISPLAY and XAUTHORITY values in the old shell.
Don't run the sim with sudo. Over ssh, use `ssh -X` (slow) or run it on the desktop."""


def _x11_error():
    """None if the X display opens, else why not."""
    display = os.environ.get("DISPLAY")
    if not display:
        return "DISPLAY is not set"
    path = ctypes.util.find_library("X11")
    if not path:
        return None   # no libX11 to ask with: let SDL/GLFW try
    try:
        x11 = ctypes.CDLL(path)
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        d = x11.XOpenDisplay(display.encode())
    except OSError:
        return None
    if not d:
        return "the X server refused the connection or isn't running"
    x11.XCloseDisplay(d)
    return None


def _wayland_ok():
    name = os.environ.get("WAYLAND_DISPLAY")
    if not name:
        return False
    path = name if os.path.isabs(name) else os.path.join(os.environ.get("XDG_RUNTIME_DIR", ""), name)
    return os.path.exists(path)


def prepare():
    """Choose X11 or Wayland for SDL (pygame) and GLFW (MuJoCo viewer). Returns 'x11', 'wayland' or None."""
    if not sys.platform.startswith("linux"):
        return None
    # if anything does end up on Wayland, keep libdecor (and with it GTK) out of the process
    os.environ.setdefault("SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR", "0")
    why = _x11_error()
    if why is None:
        os.environ.setdefault("SDL_VIDEODRIVER", "x11")
        os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")
        return "x11"
    if _wayland_ok():
        print(f"note: no X display ({why}); using Wayland windows with plain borders.\n"
              "      Fix X access for normal windows: see sim/legsim/display.py.", file=sys.stderr)
        os.environ.setdefault("SDL_VIDEODRIVER", "wayland")
        os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "wayland")
        try:
            import glfw   # the one mujoco.viewer uses; the hint holds until its glfw.init()
            glfw.init_hint(glfw.WAYLAND_LIBDECOR, glfw.WAYLAND_DISABLE_LIBDECOR)
        except (ImportError, AttributeError):
            pass
        return "wayland"
    sys.exit(X11_HELP.format(display=os.environ.get("DISPLAY", ""), why=why))
