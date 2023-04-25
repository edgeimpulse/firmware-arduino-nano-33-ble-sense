# Edge Impulse firmware for Arduino Nano 33 BLE Sense

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Arduino Nano 33 BLE Sense development board. This device supports all Edge Impulse device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board with Edge Impulse? No need to build this firmware. See the instructions [here](https://docs.edgeimpulse.com/docs/arduino-nano-33-ble-sense) for a prebuilt image and instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.


## Arduino Nano BLE 33 Sense REV 2 Preliminary release

This version of the firmware is tailored for the REV 2 hardware. Make sure you have installed the latest Arduino Mbed Nano core library (version 4.0.2) and all the sensor libraries needed for the new sensors.

> **Note:** You can also run the `arduino-build.sh` script which will install all of these for you.

There are some changes to the sensors in comparison with the Rev 1 hardware:
- Magnetometer is not supported for now. We've detected some issues which need looking into
- Barometer is absent from the Rev 2 hardware

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
1. In Tools -> Board -> Boards Manager, search for `nano 33` and install the (deprecated) **Arduino Mbed OS Boards v4.0.2**.

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

1. In Tools -> Board -> Boards Manager, search for `nano 33` and install the (deprecated) ****Arduino Mbed OS Boards v4.0.2****.
1. In Arduino Menu -> Preferences, check the location of the **preferences.txt** file (ie: /Users/aureleq/Library/Arduino15/).
1. Copy the `boards.local.txt` file into the Arduino Mbed Nano 33 BLE Sense directory, for instance:
`/Users/aureleq/Library/Arduino15/packages/arduino/hardware/mbed/4.0.2`.
1. Open the `firmware-arduino-nano-33-ble-sense.ino`, select the **Arduino nRF528x Boards (Mbed OS)** > **Arduino Nano 33 BLE** board.
1. Build and flash the application using the **Upload** button. :warning: **It can take up to an hour depending on your computer resources**

## Troubleshooting

* Not flashing? You can double tap the button on the board to put it in bootloader mode.

* #include "UsefulBuh.h" error?

    ```
    #include "UsefulBuf.h"
          ^~~~~~~~~~~~~
    compilation terminated.
    exit status 1
    Error compiling for board Arduino Nano 33 BLE.
    ```

    Add the boards.local.txt in your Arduino IDE application folder


* Failed to allocate TFLite arena (error code 1) / Failed to run impulse (-6)

```

Inferencing settings:
	Image resolution: 96x96
	Frame size: 9216
	No. of classes: 1
Taking photo...

Failed to allocate TFLite arena (error code 1)
Failed to run impulse (-6)
```

You get the above error when there's not enough (contiguous) memory to allocate TFLite arena. This can be caused by different reasons

1. Heap fragmentation
2. Not enough RAM/heap.

In the case of (1) you may want to allocate the tensor arena statically by defining ` "-DEI_CLASSIFIER_ALLOCATION_STATIC"` in `arduino-build.sh` or `boards.local.txt` . If the problem still persists, then it may be that there's not enough RAM/heap for your model and this application. Currently the heap is placed in a `512`k RAM segment.

* Failed to encode frame as JPEG (4)

```

Inferencing settings:
        Image resolution: 96x96
        Frame size: 9216
        No. of classes: 1
Taking photo...
Begin output
Failed to encode frame as JPEG (4)
```

There's not enough (contiguous) memory to allocate the jpeg buffer. Try increasing the `jpeg_buffer_size`. If the problem still persists this may be due
to heap fragmentation. Try statically allocating `jpeg_buffer`.
