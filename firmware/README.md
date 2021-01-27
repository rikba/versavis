# Installation via Atmel Studio and .hex
The easiest way to install the firmware is to flash the precompiled [.hex](build) code directly onto the microcontroller.

1. Connect VersaVIS both to j-link and USB (for power)<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3decGAq1C-H6sobutmEmsX56sbpv_QsCljCY8-OLf8roBtur7XhYzavrZLWN_5PGTu4u6wfp3xma5onPxT7z-XFU7COgftvl_dIlkS1djvktARqx3cBWXmvwrvGRirG0jYcswKyxGXN7RrjDnRMEIMJ=w1310-h982-no?authuser=0" width="480">
2. Start AtmelStudio->Tools->Device Programming
3. Connect to controller by setting device and pressing Apply<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3ct4yqvjgeYgjDEAIB54owkFV8UGA7rZppGb_1rlTTuoj97Dknn3vRUbfQ7P7jdTTmujqgZU0nEGSzZWRXVpVLR8Pr-fIUJrHp94yAA6_6lTmUFIy7rhyIlcvd2MG5iwvjOoABg6ZnrHyPkODZW3b05=w1535-h772-no?authuser=0" width="480">
4. Erase flash and program [bootloader](https://github.com/rikba/versavis/blob/feature/gnss_sync/firmware/build/samd21_sam_ba_versavis.hex)<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3c14hC-yApYa9c4MiOtqt1DvxkUqBIXg_mHwPzyuxuvujSwnJYTWhdcb699UdUOS-gBWFO6FZS53xku1YarGqAG01bYqcs24I5zsO_Fj0NKLxAHt36Sabo4kc7NhsM0KuaDgrCxA49ELMMKzgywGP3s=w1532-h779-no?authuser=0" width="480">
5. Program [firmware](https://github.com/rikba/versavis/blob/feature/gnss_sync/firmware/build/versavis_rtc.ino.hex). **Do not erase flash.**<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3eT89OY5lqtHjehr4w7I9veWCVw9UyBbXR9k9N-OsImobqUZxiwXccaGnPK4FZ2JwOwC5PXJQjJK2Cac7BKrHrKPejn2pnofym8hTOZIyddU_f66g-lfn2Z0bAO3iWTIQdrgaTiqk876UMVQC3m78rW=w1537-h778-no?authuser=0" width="480">
6. Reset Arduino. It should now be recognized by the system.

# Installation via CLI
The firmware can also be built via Arduino CLI.
For this purpose execute the [setup script](setup.sh).
```
roscd versavis
cd ../firmware
./setup.sh
```

After building the rosserial libraries, you will be asked to install Arduino CLI.
Do this once on your system.

Then after compilation the terminal will show <br>
```
Uploading...
```
Quickly double tap the reset button on the VersaVIS.
If you are successful the Upload will conclude with<br>
```
Atmel SMART device 0x10010005 found
Device       : ATSAMD21G18A
Chip ID      : 10010005
Version      : v2.0 [Arduino:XYZ] Oct  1 2020 14:45:50
Address      : 8192
Pages        : 3968
Page Size    : 64 bytes
Total Size   : 248KB
Planes       : 1
Lock Regions : 16
Locked       : none
Security     : false
Boot Flash   : true
BOD          : true
BOR          : true
Arduino      : FAST_CHIP_ERASE
Arduino      : FAST_MULTI_PAGE_WRITE
Arduino      : CAN_CHECKSUM_MEMORY_BUFFER
Erase flash
done in 0.938 seconds

Write 62068 bytes to flash (970 pages)
[==============================] 100% (970/970 pages)
done in 0.416 seconds

Verify 62068 bytes of flash with checksum.
Verify successful
done in 0.040 seconds
CPU reset.
```
On error repeat the installation.

Power cycle your system.
The firmware should start properly.

## Known limitations
At the moment it is not possible to upload the binary with bootloader as one file https://github.com/arduino/arduino-builder/issues/359.

# Hardware clock extension
The 10 MHz oscillator is connected to 5V and the DAC output on the back of the VersaVIS.

<img src="https://lh3.googleusercontent.com/pw/ACtC-3ckeMMcqSuujXA3ojo0O4R_krmXlQ3emo41agas0tnLb6KrBFlrnpiEbgN6wQBfvRRk8fYoTQxKyxTMulHwP8pcRJR8ML7D0hgZUEnXI98rnLNagDnC9VIUnKycZykYSMEcYyIpxFx98VlmIg4RXWp4=w718-h957-no?authuser=0" width="240"> <img src="https://lh3.googleusercontent.com/pw/ACtC-3e5h4-QpysZocPvySDVaXmMf5J6JJLyFmZx2vJyzP-HbynUZ9b-epJqZNRCUD9CuzZwGMxW927-YqoN6UMMmihTQkl_RpZ2ucfs8pr9PCAPsAOFTZx-KdAhQqP48PSIyELZKJCaUwpeIHjOFivFnCzf=w718-h957-no?authuser=0" width="240">
