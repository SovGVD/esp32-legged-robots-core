#!/usr/bin/env python3
"""Assemble the browser flasher site (GitHub Pages) from PlatformIO builds.

    build_site.py --out site --version <text> .pio/build/robot-dog [...]

Every build directory must contain flash_images.json, written by
tools/webflash/flash_images.py. Images are taken from that directory by file
name, or else from the absolute path recorded at build time. The result is:

    site/index.html
    site/firmware.json                 list of robots for the page
    site/<env>/manifest.json           ESP Web Tools manifest
    site/<env>/*.bin
"""
import argparse
import json
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))

CHIP_FAMILY = {
    "esp32": "ESP32",
    "esp32s2": "ESP32-S2",
    "esp32s3": "ESP32-S3",
    "esp32c3": "ESP32-C3",
    "esp32c6": "ESP32-C6",
}


def build_env(build_dir, out, version):
    with open(os.path.join(build_dir, "flash_images.json")) as f:
        info = json.load(f)
    env = info["env"]
    env_out = os.path.join(out, env)
    os.makedirs(env_out, exist_ok=True)

    parts = []
    for image in info["images"]:
        name = os.path.basename(image["path"])
        # CI copies the images next to flash_images.json, the absolute
        # paths of the build machine are gone by then
        src = os.path.join(build_dir, name)
        if not os.path.isfile(src):
            src = image["path"]
        if any(p["path"] == name for p in parts):
            raise SystemExit(f"{env}: duplicate image name {name}")
        shutil.copyfile(src, os.path.join(env_out, name))
        parts.append({"path": name, "offset": int(str(image["offset"]), 0)})
    parts.sort(key=lambda p: p["offset"])

    manifest = {
        "name": f"esp32-legged-robots {env}",
        "version": version,
        # the user picks "erase" only for a clean install; an update keeps
        # the NVS partition (servo trims) untouched
        "new_install_prompt_erase": True,
        "builds": [{"chipFamily": CHIP_FAMILY[info["mcu"]], "parts": parts}],
    }
    with open(os.path.join(env_out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    return {"env": env, "manifest": f"{env}/manifest.json",
            "size": sum(os.path.getsize(os.path.join(env_out, p["path"])) for p in parts)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--version", required=True)
    ap.add_argument("--commit", default="")
    ap.add_argument("build_dirs", nargs="+")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    robots = [build_env(d, args.out, args.version) for d in args.build_dirs]
    with open(os.path.join(args.out, "firmware.json"), "w") as f:
        json.dump({"version": args.version, "commit": args.commit,
                   "robots": robots}, f, indent=2)
    shutil.copyfile(os.path.join(HERE, "index.html"),
                    os.path.join(args.out, "index.html"))
    for r in robots:
        print(f"{r['env']}: {r['manifest']} ({r['size']} bytes)")


if __name__ == "__main__":
    main()
