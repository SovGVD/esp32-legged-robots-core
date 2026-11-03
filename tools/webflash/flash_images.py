# PlatformIO post script: after firmware.bin is built, write
# $BUILD_DIR/flash_images.json with every image `pio run -t upload` would
# flash (bootloader, partition table, boot_app0, application) and its offset.
# tools/webflash/build_site.py turns these into an ESP Web Tools manifest.
#
# The images are kept separate (no merged factory bin) so the gaps between
# them, the NVS partition with the servo trims among them, are not
# overwritten when the robot is updated from the browser.
import json
import os

Import("env")  # noqa: F821 (SCons global)


def _offset(value):
    value = env.subst(str(value)).strip()  # noqa: F821
    return value if value and value != "None" else None


def write_flash_images(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    images = []
    for offset, path in env.get("FLASH_EXTRA_IMAGES", []):
        images.append({"offset": _offset(offset), "path": env.subst(path)})
    images.append({
        "offset": _offset("$ESP32_APP_OFFSET") or "0x10000",
        "path": env.subst("$BUILD_DIR/${PROGNAME}.bin"),
    })
    data = {
        "env": env.subst("$PIOENV"),
        "mcu": env.BoardConfig().get("build.mcu", "esp32"),
        "images": images,
    }
    with open(os.path.join(build_dir, "flash_images.json"), "w") as f:
        json.dump(data, f, indent=2)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", write_flash_images)  # noqa: F821
