#!/bin/bash
set -e

################################ Project ######################################

PROJECT=firmware-arduino-nano-33-ble-sense

# used for grepping
ARDUINO_CORE="arduino:mbed"
ARDUINO_CORE_VERSION="1.1.6"

BOARD="${ARDUINO_CORE}":nano33ble

# declare associative array pre bash 4 style
ARDUINO_LIBS=(
"Arduino_LSM9DS1=1.0.0"   # Inertial sensor library
"Arduino_HTS221=1.0.0"    # Environment sensor library
"Arduino_LPS22HB=1.0.0"   # Pressure sensor library
"Arduino_APDS9960=1.0.3"  # Interaction sensor library
"Arduino_OV767X=0.0.2"    # Camera sensor library
)

###############################################################################

COMMAND=$1
if [ -z "$ARDUINO_CLI" ]; then
    ARDUINO_CLI=$(which arduino-cli || true)
fi
DIRNAME="$(basename "$SCRIPTPATH")"
EXPECTED_CLI_MAJOR=0
EXPECTED_CLI_MINOR=18

CLI_MAJOR=$($ARDUINO_CLI version | cut -d. -f1 | rev | cut -d ' '  -f1)
CLI_MINOR=$($ARDUINO_CLI version | cut -d. -f2)
CLI_REV=$($ARDUINO_CLI version | cut -d. -f3 | cut -d ' '  -f1)

################################ Helper Funcs #################################

# parses Arduino CLI's (core list and lib list) output and returns the installed version.
# Expected format (spaces can vary):
#    <package/core>   <installed version>  <latest version>  <other>
#
parse_installed() {
    echo "${1}" | awk -F " " '{print $2}' || true
}

# finds a Arduino core installed and returns the version
# otherwise it returns empty string
#
find_arduino_core() {
    core=$1
    version=$2
    result=""
    # space intentional
    line="$($ARDUINO_CLI core list | grep "${core} " || true)"
    if [ -n "$line" ]; then
        installed="$(parse_installed "${line}")"
        if [ "$version" = "$installed" ]; then
           result="$installed"
        fi
    fi
    echo $result
}

# finds a Arduino library installed and returns the version
# otherwise it returns empty string
#
find_arduino_lib() {
    lib=$1
    version=$2
    result=""
    # space intentional
    line="$($ARDUINO_CLI lib list | grep "${lib} " || true)"
    if [ -n "$line" ]; then
        installed="$(parse_installed "${line}")"
        if [ "$version" = "$installed" ]; then
           result="$installed"
        fi
    fi
    echo $result
}

array_get_key() {
    echo "${1%%=*}"
}

array_get_value() {
    echo "${1#*=}"
}

############################# Installing Deps #################################

check_dependency()
{
    if [ ! -x "$ARDUINO_CLI" ]; then
        echo "Cannot find 'arduino-cli' in your PATH. Install the Arduino CLI before you continue."
        echo "Installation instructions: https://arduino.github.io/arduino-cli/latest/"
        exit 1
    fi

    if (( CLI_MINOR < EXPECTED_CLI_MINOR)); then
        echo "You need to upgrade your Arduino CLI version (now: $CLI_MAJOR.$CLI_MINOR.$CLI_REV, but required: $EXPECTED_CLI_MAJOR.$EXPECTED_CLI_MINOR.x or higher)"
        echo "See https://arduino.github.io/arduino-cli/installation/ for upgrade instructions"
        exit 1
    fi

    if (( CLI_MAJOR != EXPECTED_CLI_MAJOR || CLI_MINOR != EXPECTED_CLI_MINOR )); then
        echo "You're using an untested version of Arduino CLI, this might cause issues (found: $CLI_MAJOR.$CLI_MINOR.$CLI_REV, expected: $EXPECTED_CLI_MAJOR.$EXPECTED_CLI_MINOR.x)"
    fi

    echo ""
    echo "Checking Core dependencies..."
    echo ""

    has_arduino_core="$(find_arduino_core "${ARDUINO_CORE}" "${ARDUINO_CORE_VERSION}")"
    if [ -z "$has_arduino_core" ]; then
        echo -e "${ARDUINO_CORE}@${ARDUINO_CORE_VERSION}\tNot Found!"
        echo -e "\tInstalling ${ARDUINO_CORE}@${ARDUINO_CORE_VERSION}..."
        $ARDUINO_CLI core update-index
        $ARDUINO_CLI core install "${ARDUINO_CORE}@${ARDUINO_CORE_VERSION}"
        echo -e "\tInstalling ${ARDUINO_CORE}@${ARDUINO_CORE_VERSION} OK"
    else
        echo -e "${ARDUINO_CORE}@${ARDUINO_CORE_VERSION}\tFound!"
    fi

    echo ""
    echo "Checking Core dependencies OK"
    echo ""
    echo "Checking Library dependencies..."
    echo ""

    lib_update=""
    for lib in ${ARDUINO_LIBS[@]}; do
        key=$(array_get_key "${lib}")
        value=$(array_get_value "$lib")
        has_arduino_lib="$(find_arduino_lib "${key}" "${value}")"
        if [ -n "$has_arduino_lib" ]; then
            echo -e "${key}@${value}\tFound!"
        else
            echo -e "${key}@${value}\tNot Found!"
            echo -e "\tInstalling ${key}@${value} library..."

        if [ -z  "$lib_update" ]; then
            $ARDUINO_CLI lib update-index
            lib_update="once"
        fi
            $ARDUINO_CLI lib install "${key}"@"${value}"
            echo -e "\tInstalling ${key}@${value} library OK"
        fi
    done

    echo ""
    echo "Checking Library dependencies OK"
    echo ""
}

############################### Build Deps #####################################

# CLI v0.14 updates the name of this to --build-property
if ((CLI_MAJOR >= 0 && CLI_MINOR >= 14)); then
    BUILD_PROPERTIES_FLAG=--build-property
else
    BUILD_PROPERTIES_FLAG=--build-properties
fi

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
INCLUDE+=" -I ./src/firmware-sdk/"

FLAGS="-DARDUINOSTL_M_H"
FLAGS+=" -DMBED_HEAP_STATS_ENABLED=1"
FLAGS+=" -DMBED_STACK_STATS_ENABLED=1"
FLAGS+=" -O3"
FLAGS+=" -g3"
FLAGS+=" -DEI_SENSOR_AQ_STREAM=FILE"
FLAGS+=" -DEIDSP_QUANTIZE_FILTERBANK=0"
FLAGS+=" -DEI_CLASSIFIER_SLICES_PER_MODEL_WINDOW=3"
FLAGS+=" -mfpu=fpv4-sp-d16"

if [ "$COMMAND" = "--build" ];
then
    echo "Building $PROJECT"
    check_dependency
    $ARDUINO_CLI compile --fqbn  $BOARD $BUILD_PROPERTIES_FLAG build.extra_flags="$INCLUDE $FLAGS" --output-dir . &
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
    $ARDUINO_CLI upload -p $($ARDUINO_CLI board list | grep Arduino | cut -d ' ' -f1) --fqbn $BOARD --input-dir .
elif [ "$COMMAND" = "--all" ];
then
    $ARDUINO_CLI compile --fqbn  $BOARD $BUILD_PROPERTIES_FLAG build.extra_flags="$INCLUDE $FLAGS"
    status=$?
    [ $status -eq 0 ] && $ARDUINO_CLI upload -p $($ARDUINO_CLI board list | grep Arduino | cut -d ' ' -f1) --fqbn $BOARD --input-dir .
else
    echo "Nothing to do for target"
fi
