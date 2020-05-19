#!/bin/bash
PROJECT=edge-impulse.ino
BOARD=arduino:mbed:nano33ble
COMMAND=$1

INCLUDE="-I ./src"
INCLUDE+=" -I./src/model-parameters"
INCLUDE+=" -I ./src/repl"
INCLUDE+=" -I ./src/ingestion-sdk-c/"
INCLUDE+=" -I ./src/ingestion-sdk-c/inc"
INCLUDE+=" -I ./src/ingestion-sdk-c/inc/signing"
INCLUDE+=" -I ./src/ingestion-sdk-platform/nano-ble33"
INCLUDE+=" -I ./src/sensors"
INCLUDE+=" -I ./src/QCBOR/inc"
INCLUDE+=" -I ./src/QCBOR/src"
INCLUDE+=" -I ./src/mbedtls_hmac_sha256_sw/"

FLAGS="-DARDUINOSTL_M_H"
FLAGS+=" -DMBED_HEAP_STATS_ENABLED=1"
FLAGS+=" -DMBED_STACK_STATS_ENABLED=1"
FLAGS+=" -O3"
FLAGS+=" -DMBED_DEBUG"
FLAGS+=" -g3"
FLAGS+=" -DEI_SENSOR_AQ_STREAM=FILE"
FLAGS+=" -mfpu=fpv4-sp-d16"

if [ "$COMMAND" = "--build" ];
then
	echo "Building $PROJECT"
	arduino-cli compile --fqbn  $BOARD --build-properties build.extra_flags="$INCLUDE $FLAGS" $PROJECT &
	pid=$! # Process Id of the previous running command
	while kill -0 $pid 2>/dev/null
	do
		echo "Still building..."
		sleep 2
	done
	wait $pid
	ret=$?
	if [ $ret -eq 0 ]; then
		echo "Building $PROJECT done"
	else
		exit "Building $PROJECT failed"
	fi
elif [ "$COMMAND" = "--flash" ];
then
	arduino-cli upload -p $(arduino-cli board list | grep Arduino | cut -d ' ' -f1) --fqbn $BOARD -i arduino-test.ino.arduino.mbed.nano33ble.bin
elif [ "$COMMAND" = "--all" ];
then
	arduino-cli compile --fqbn  $BOARD --build-properties build.extra_flags="$INCLUDE $FLAGS" $PROJECT
	status=$?
	[ $status -eq 0 ] && arduino-cli upload -p $(arduino-cli board list | grep Arduino | cut -d ' ' -f1) --fqbn $BOARD -i arduino-test.ino.arduino.mbed.nano33ble.bin
else
	echo "Nothing to do for target"
fi
