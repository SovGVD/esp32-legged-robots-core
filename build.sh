#!/usr/bin/env bash

CWD=$(pwd -P)

ARDUINO_PATH="${CWD}/arduino-cli-ide"
ARDUINO_BIN_DIR="${ARDUINO_PATH}/bin"
ARDUINO_BIN="${ARDUINO_BIN_DIR}/arduino-cli"
ARDUINO_CONFIG="${ARDUINO_PATH}/arduino-ide-esp32-legged-robot-dog.yaml"
ARDUINO_DATA="${ARDUINO_PATH}/.arduino"
ARDUINO_DOWNLOADS="${ARDUINO_PATH}/.arduino/staging"
ARDUINO_USER="${ARDUINO_PATH}/Arduino"
ARDUINO_BUILD="${ARDUINO_PATH}/.arduino/build"
ARDUINO_CLI="${ARDUINO_BIN} --no-color --config-file ${ARDUINO_CONFIG}"

SRC="${CWD}/src/esp32-legged-robot"
SRC_CONFIG_MODEL="${SRC}/config_model.h"

COMMAND="$1"
PARAM1="$2"
PARAM2="$3"

installArduinoCli()
{
	# Create folders for Arduino CLI
	mkdir -p $ARDUINO_BIN_DIR
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$ARDUINO_BIN_DIR sh
}

buildEnv() {
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
	$ARDUINO_CLI core update-index
	$ARDUINO_CLI core install esp32:esp32@2.0.3
	$ARDUINO_CLI core list
	$ARDUINO_CLI board listall

	# Install libraries
	# TODO specify versions for git
	$ARDUINO_CLI lib install --git-url https://github.com/me-no-dev/ESPAsyncWebServer.git https://github.com/me-no-dev/AsyncTCP.git
	$ARDUINO_CLI lib install ESP32_ISR_Servo@1.3.0
	$ARDUINO_CLI lib install MPU9250_WE@1.1.3
	$ARDUINO_CLI lib install INA219_WE@1.3.1
	$ARDUINO_CLI lib install "Adafruit PWM Servo Driver Library"@2.4.0
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
		*)
			compileHelp
			exit 0
			;;
	esac
}

compile() {
	setModelAndBoard
	# Compile
	echo "Compile: ${ARDUINO_CLI} compile --fqbn ${BOARD} ${SRC}..."
	echo "#define ROBOT_MODEL ${MODEL}" > $SRC_CONFIG_MODEL
	$ARDUINO_CLI cache clean
	$ARDUINO_CLI compile --fqbn $BOARD $SRC -v --build-path $ARDUINO_BUILD
	rm $SRC_CONFIG_MODEL
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
	echo "  robot-dog   quadruped dog-like robot"
	echo "  robot-biped bipedal robot"
}

case $COMMAND in
	install-arduino-cli)
		installArduinoCli
		;;
	build-env)
		buildEnv
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
