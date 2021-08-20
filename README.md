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

## Building the application

1. Build the application:

    ```
    ./arduino-build.sh --build
    ```

1. Flash the application:

    ```
    ./arduino-build.sh --flash
    ```

## Troubleshooting

* Not flashing? You can double tap the button on the board to put it in bootloader mode.
