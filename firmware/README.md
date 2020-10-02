# Installation via binary
The easiest way to install the firmware is to flash the precompiled .hex code directly onto the microcontroller.

1. Connect VersaVIS both to j-link and USB (for power)
2. Start AtmelStudio->Tools->Device Programming
3. Connect to controller by setting device and pressing Apply
4. Erase flash and program bootloader
5. Do not erase flash and program firmware
6. Reset Arduino. It should now be recognized by the system.

## Known limitations
At the moment it is not possible to upload the binary with bootloader as one file https://github.com/arduino/arduino-builder/issues/359.
