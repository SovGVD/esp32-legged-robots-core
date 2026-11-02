#!/usr/bin/env bash
# Thin wrapper around PlatformIO (see platformio.ini) plus the web client build.
# You can also call PlatformIO directly, e.g. `pio run -e robot-dog -t upload`.
set -e
CWD=$(pwd -P)

WEB_CLIENT_DIR="${CWD}/src/web"

COMMAND="$1"
PARAM1="$2"
PARAM2="$3"

requirePio()
{
	if ! command -v pio >/dev/null 2>&1; then
		echo "PlatformIO CLI (pio) not found. Install it with:"
		echo "  pip install -U platformio"
		echo "or see https://docs.platformio.org/en/latest/core/installation/index.html"
		exit 1
	fi
}

buildWebClientEnv()
{
	# Check node version to fail if node was not install
	# or at least show version for debug
	# @see https://nodejs.org/en/download/package-manager/
	node -v
	npm -v
	cd $WEB_CLIENT_DIR
	npm install
	npm run build
	cd $CWD
}

requireModel() {
	case $PARAM1 in
		robot-dog|robot-biped|robot-tank|robot-hexapod)
			ENV="$PARAM1"
			;;
		*)
			compileHelp
			exit 0
			;;
	esac
}

compile() {
	requirePio
	requireModel
	pio run -e "$ENV"
}

upload() {
	requirePio
	requireModel
	if [[ -n "$PARAM2" ]]; then
		pio run -e "$ENV" -t upload --upload-port "$PARAM2"
	else
		pio run -e "$ENV" -t upload
	fi
}

runTests() {
	requirePio
	pio test -e native
}

monitor() {
	requirePio
	echo "Press Ctrl-C to exit"
	if [[ -n "$PARAM1" ]]; then
		pio device monitor -b 115200 -p "$PARAM1"
	else
		pio device monitor -b 115200
	fi
}

helpHeader() {
	echo "ESP32 legged robot build command line interface (PlatformIO)."
	echo ""
}

help() {
	helpHeader
	echo "Usage:"
	echo "  ./build.sh <command> [flags...]"
	echo ""
	echo "Available Commands:"
	echo "  build-client-env    Build robot web client (src/web -> web/index.html.gz.h)"
	echo "  compile <model>     Compile ESP32 firmware (pio run -e <model>)"
	echo "  upload <model> [port]  Compile and upload"
	echo "  test                Run motion core unit tests on this PC"
	echo "  cli [port]          Serial monitor"
}

compileHelp() {
	helpHeader
	echo "Usage:"
	echo "  ./build.sh compile ROBOT_MODEL"
	echo ""
	echo "Examples:"
	echo "  ./build.sh compile robot-dog"
	echo ""
	echo "Available models:"
	echo "  robot-dog      quadruped dog-like robot"
	echo "  robot-biped    bipedal robot"
	echo "  robot-tank     tank-like robot"
	echo "  robot-hexapod  hexapod robot"
}

case $COMMAND in
	build-client-env)
		buildWebClientEnv
		;;
	compile)
		compile
		;;
	upload)
		upload
		;;
	test)
		runTests
		;;
	cli)
		monitor
		;;
	*)
		help
		;;
esac
