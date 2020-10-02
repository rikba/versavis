# Installation via binary
The easiest way to install the firmware is to flash the precompiled [.hex](https://github.com/rikba/versavis/tree/feature/gnss_sync/firmware/build) code directly onto the microcontroller.

1. Connect VersaVIS both to j-link and USB (for power)
2. Start AtmelStudio->Tools->Device Programming
3. Connect to controller by setting device and pressing Apply
4. Erase flash and program [bootloader](https://github.com/rikba/versavis/blob/feature/gnss_sync/firmware/build/samd21_sam_ba_versavis.hex)
5. Do not erase flash and program [firmware](https://github.com/rikba/versavis/blob/feature/gnss_sync/firmware/build/versavis_rtc.ino.hex)
6. Reset Arduino. It should now be recognized by the system.

## Known limitations
At the moment it is not possible to upload the binary with bootloader as one file https://github.com/arduino/arduino-builder/issues/359.
