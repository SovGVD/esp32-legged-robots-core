#!/usr/bin/env bash
set -e
CWD=$(pwd -P)

ARDUINO_PATH="${CWD}/arduino-cli-ide"
ARDUINO_BIN_DIR="${ARDUINO_PATH}/bin"
ARDUINO_BIN="${ARDUINO_BIN_DIR}/arduino-cli"
ARDUINO_CONFIG="${ARDUINO_PATH}/arduino-ide-esp32-legged-robot-dog.yaml"
ARDUINO_DATA="${ARDUINO_PATH}/.arduino"
ARDUINO_DOWNLOADS="${ARDUINO_PATH}/.arduino/staging"
ARDUINO_USER="${ARDUINO_PATH}/Arduino"
ARDUINO_LIB="${ARDUINO_USER}/libraries"
ARDUINO_BUILD="${ARDUINO_PATH}/.arduino/build"
ARDUINO_CLI="${ARDUINO_BIN} --no-color --config-file ${ARDUINO_CONFIG}"

SRC="${CWD}/src/esp32-legged-robot"
SRC_CONFIG_MODEL="${SRC}/config_model.h"

WEB_CLIENT_DIR="${CWD}/src/web"

COMMAND="$1"
PARAM1="$2"
PARAM2="$3"

installArduinoCli()
{
	# Create folders for Arduino CLI
	mkdir -p $ARDUINO_BIN_DIR
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$ARDUINO_BIN_DIR sh
}

buildWebClientEnv()
{
	# Check node version to fail if node was not install
	# or at least show version for debug
	# @see https://nodejs.org/en/download/package-manager/
	# Expected latest LTS (currently 16.x)
	node -v
	npm -v
	cd $WEB_CLIENT_DIR
	npm install
	npm run build
	cd $CWD
}

buildEnv() {
	if [ ! -f "$ARDUINO_BIN" ]; then
		installArduinoCli
	fi
	# Create folders for Arduino
	mkdir -p $ARDUINO_PATH
	mkdir -p $ARDUINO_DATA
	mkdir -p $ARDUINO_DOWNLOADS
	mkdir -p $ARDUINO_USER
	mkdir -p $ARDUINO_BUILD

	# Copy empty config file
	cp -r .arduino-ide-default.yaml $ARDUINO_CONFIG

	echo "Build environment in: $CWD"
	echo "Arduino path: $ARDUINO_PATH"

	$ARDUINO_CLI config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
	$ARDUINO_CLI config set directories.data $ARDUINO_DATA
	$ARDUINO_CLI config set directories.downloads $ARDUINO_DOWNLOADS
	$ARDUINO_CLI config set directories.user $ARDUINO_USER
	$ARDUINO_CLI config set library.enable_unsafe_install true
	$ARDUINO_CLI config dump

	# Install ESP32
	if [ ! -d "$ARDUINO_DATA/packages/esp32/hardware/esp32/1.0.6" ]; then
		$ARDUINO_CLI core update-index
		$ARDUINO_CLI core install esp32:esp32@1.0.6
		$ARDUINO_CLI core list
		$ARDUINO_CLI board listall
	fi

	# Install libraries
	$ARDUINO_CLI lib install ESP32_ISR_Servo@1.1.0
	$ARDUINO_CLI lib install MPU9250_WE@1.1.3
	$ARDUINO_CLI lib install INA219_WE@1.3.1
	$ARDUINO_CLI lib install "Adafruit PWM Servo Driver Library"@2.4.0

	# Install other libraries
	cd $ARDUINO_LIB
	if [ ! -d "$ARDUINO_LIB/esp32camera" ]; then
		git clone https://github.com/SovGVD/esp32camera.git
	fi
	cd $ARDUINO_LIB/esp32camera
	git checkout 193ea76a1047cc83b34a8e366241fca0f08e4022

	cd $ARDUINO_LIB
	if [ ! -d "$ARDUINO_LIB/ESPAsyncWebServer" ]; then
		git clone https://github.com/me-no-dev/ESPAsyncWebServer.git
	fi
	cd $ARDUINO_LIB/ESPAsyncWebServer
	git checkout f71e3d427b5be9791a8a2c93cf8079792c3a9a26

	cd $ARDUINO_LIB
	if [ ! -d "$ARDUINO_LIB/AsyncTCP" ]; then
		git clone https://github.com/me-no-dev/AsyncTCP.git
	fi
	cd $ARDUINO_LIB/AsyncTCP
	git checkout ca8ac5f919d02bea07b474531981ddbfd64de97c
	cd $CWD
}

setModelAndBoard() {
	case $PARAM1 in
		robot-dog)
			MODEL="MODEL_DOG"
			BOARD="esp32:esp32:esp32doit-devkit-v1"
			;;
		robot-biped)
			MODEL="MODEL_BIPED"
			BOARD="esp32:esp32:esp32cam"
			;;
		robot-tank)
			MODEL="MODEL_TANK"
			BOARD="esp32:esp32:esp32doit-devkit-v1"
			;;
		robot-hexapod)
			MODEL="MODEL_HEXAPOD"
			BOARD="esp32:esp32:esp32cam"
			;;
		*)
			compileHelp
			exit 0
			;;
	esac
}

compile() {
	setModelAndBoard
	# Compile
	# TODO: exit on error!
	echo "Compile: ${ARDUINO_CLI} compile --fqbn ${BOARD} ${SRC}..."
	echo "#define ROBOT_MODEL ${MODEL}" > $SRC_CONFIG_MODEL
	$ARDUINO_CLI cache clean
	$ARDUINO_CLI compile --fqbn $BOARD $SRC -v --build-path $ARDUINO_BUILD
	#rm $SRC_CONFIG_MODEL
}

upload() {
	[[ -z "$PARAM2" ]] && { echo "Port required" ; exit 1; }
	compile
	echo "Uploading ${MODEL}. Port:${PARAM2}..."
	$ARDUINO_CLI upload -p ${PARAM2} --fqbn $BOARD $SRC -v --input-dir $ARDUINO_BUILD
}

monitor() {
	[[ -z "$PARAM1" ]] && { echo "Port required" ; exit 1; }
	echo "Press Ctrl-C to exit"
	$ARDUINO_CLI monitor -p ${PARAM1} -c baudrate=115200
}

helpHeader() {
	echo "ESP32 legged robot build command line interface."
	echo ""
}

help() {
	helpHeader
	echo "Usage:"
	echo "  ./build.sh [command]"
	echo ""
	echo "Examples:"
	echo "  ./build.sh <command> [flags...]"
	echo ""
	echo "Available Commands:"
	echo "  install-arduino-cli Install latest version of Arduino CLI from Arduno CLI repository"
	echo "  build-env           Build environment for ESP32 and install required libraries"
	echo "  build-client-env    Build environment for robot web client"
	echo "  compile             Compile ESP32 binary"
	echo "  upload              Compile and upload"
	echo "  cli                 Arduino monitor, terminal"
}

compileHelp() {
	helpHeader
	echo "Usage:"
	echo "  ./build.sh compile ROBOT_MODEL"
	echo ""
	echo "Examples:"
	echo "  ./build.sh compile robot-dog"
	echo ""
	echo "Available Commands:"
	echo "  robot-dog      quadruped dog-like robot"
	echo "  robot-biped    bipedal robot"
	echo "  robot-tank     tank-like robot"
	echo "  robot-hexapod  hexapod robot"
}

case $COMMAND in
	install-arduino-cli)
		installArduinoCli
		;;
	build-env)
		buildEnv
		;;
	build-client-env)
		buildWebClientEnv
		;;
	compile)
		compile
		;;
	upload)
		upload
		;;
	cli)
		monitor
		;;
	*)
		help
		;;
esac
