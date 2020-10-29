# Installation via Atmel Studio and .hex
The easiest way to install the firmware is to flash the precompiled [.hex](https://github.com/rikba/versavis/tree/feature/gnss_sync/firmware/build) code directly onto the microcontroller.

1. Connect VersaVIS both to j-link and USB (for power)<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3decGAq1C-H6sobutmEmsX56sbpv_QsCljCY8-OLf8roBtur7XhYzavrZLWN_5PGTu4u6wfp3xma5onPxT7z-XFU7COgftvl_dIlkS1djvktARqx3cBWXmvwrvGRirG0jYcswKyxGXN7RrjDnRMEIMJ=w1310-h982-no?authuser=0" width="480">
2. Start AtmelStudio->Tools->Device Programming
3. Connect to controller by setting device and pressing Apply<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3ct4yqvjgeYgjDEAIB54owkFV8UGA7rZppGb_1rlTTuoj97Dknn3vRUbfQ7P7jdTTmujqgZU0nEGSzZWRXVpVLR8Pr-fIUJrHp94yAA6_6lTmUFIy7rhyIlcvd2MG5iwvjOoABg6ZnrHyPkODZW3b05=w1535-h772-no?authuser=0" width="480">
4. Erase flash and program [bootloader](https://github.com/rikba/versavis/blob/feature/gnss_sync/firmware/build/samd21_sam_ba_versavis.hex)<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3c14hC-yApYa9c4MiOtqt1DvxkUqBIXg_mHwPzyuxuvujSwnJYTWhdcb699UdUOS-gBWFO6FZS53xku1YarGqAG01bYqcs24I5zsO_Fj0NKLxAHt36Sabo4kc7NhsM0KuaDgrCxA49ELMMKzgywGP3s=w1532-h779-no?authuser=0" width="480">
5. Program [firmware](https://github.com/rikba/versavis/blob/feature/gnss_sync/firmware/build/versavis_rtc.ino.hex). **Do not erase flash.**<br/><img src="https://lh3.googleusercontent.com/pw/ACtC-3eT89OY5lqtHjehr4w7I9veWCVw9UyBbXR9k9N-OsImobqUZxiwXccaGnPK4FZ2JwOwC5PXJQjJK2Cac7BKrHrKPejn2pnofym8hTOZIyddU_f66g-lfn2Z0bAO3iWTIQdrgaTiqk876UMVQC3m78rW=w1537-h778-no?authuser=0" width="480">
6. Reset Arduino. It should now be recognized by the system.

## Known limitations
At the moment it is not possible to upload the binary with bootloader as one file https://github.com/arduino/arduino-builder/issues/359.

# Hardware clock extension
The 10 MHz oscillator is connected to 5V and the DAC output on the back of the VersaVIS.

<img src="https://lh3.googleusercontent.com/pw/ACtC-3ckeMMcqSuujXA3ojo0O4R_krmXlQ3emo41agas0tnLb6KrBFlrnpiEbgN6wQBfvRRk8fYoTQxKyxTMulHwP8pcRJR8ML7D0hgZUEnXI98rnLNagDnC9VIUnKycZykYSMEcYyIpxFx98VlmIg4RXWp4=w718-h957-no?authuser=0" width="240"> <img src="https://lh3.googleusercontent.com/pw/ACtC-3e5h4-QpysZocPvySDVaXmMf5J6JJLyFmZx2vJyzP-HbynUZ9b-epJqZNRCUD9CuzZwGMxW927-YqoN6UMMmihTQkl_RpZ2ucfs8pr9PCAPsAOFTZx-KdAhQqP48PSIyELZKJCaUwpeIHjOFivFnCzf=w718-h957-no?authuser=0" width="240">
