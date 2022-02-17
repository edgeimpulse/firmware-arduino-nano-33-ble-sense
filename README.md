# Edge Impulse firmware for Arduino Nano 33 BLE Sense

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Arduino Nano 33 BLE Sense development board. This device supports all Edge Impulse device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board with Edge Impulse? No need to build this firmware. See the instructions [here](https://docs.edgeimpulse.com/docs/arduino-nano-33-ble-sense) for a prebuilt image and instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.

## Requirements

### Hardware

* [Arduino Nano 33 BLE Sense](https://store.arduino.cc/usa/nano-33-ble-sense) or [Arduino Nano 33 BLE](https://store.arduino.cc/usa/nano-33-ble) development board.
* (Optional) [Arduino Tiny Machine Learning Kit](https://store.arduino.cc/tiny-machine-learning-kit) - to add sight to your Arduino Nano 33 BLE.

### Tools

The arduino-cli tool is used to build and upload the Edge Impulse firmware to the Arduino Nano 33 BLE Sense board. Use following link for download and installation procedure:

* [Arduino CLI](https://arduino.github.io/arduino-cli/installation/).

The Edge Impulse firmware depends on some libraries and the Mbed core for Arduino. These will be automatically installed if you don't have them yet.

* Arduino IDE (required for Windows users)

_Installing Arduino IDE is a requirement only for Windows users. macOS and Linux users can use either the Arduino CLI or IDE to build the application._

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software) for your Operating System.
1. In Tools -> Board -> Boards Manager, search for `nano 33` and install the (deprecated) **Arduino Mbed OS Boards v1.1.6**.

## Building the application

1. Build the application:

    ```
    ./arduino-build.sh --build
    ```

1. Flash the application:

    ```
    ./arduino-build.sh --flash
    ```

### Arduino IDE

1. In Tools -> Board -> Boards Manager, search for `nano 33` and install the (deprecated) ****Arduino Mbed OS Boards v1.1.6****.
1. In Arduino Menu -> Preferences, check the location of the **preferences.txt** file (ie: /Users/aureleq/Library/Arduino15/).
1. Copy the `boards.local.txt` file into the Arduino Mbed Nano 33 BLE Sense directory, for instance:
`/Users/aureleq/Library/Arduino15/packages/arduino/hardware/mbed/1.1.6`.
1. Open the `firmware-arduino-nano-33-ble-sense.ino`, select the **Arduino nRF528x Boards (Mbed OS)** > **Arduino Nano 33 BLE** board.
1. Build and flash the application using the **Upload** button. :warning: **It can take up to an hour depending on your computer resources**

## Troubleshooting

* Not flashing? You can double tap the button on the board to put it in bootloader mode.
