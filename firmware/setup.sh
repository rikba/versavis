#!/bin/bash
source ../../../devel/setup.bash
rm -rf libraries/ros_lib
rosrun rosserial_arduino make_libraries.py libraries

# Maybe hardcode compiler and bossac path and compiler.arm.cmsis.c.flags in ~/.arduino15/packages/VersaVIS/hardware/samd/1.0.0/platform.txt
# https://github.com/Open-Bionics/Beetroot/issues/6

echo "Do you wish to install the arduino CLI? [y or Y to accept]"
read install_arduino
if [[ $install_boards == "Y" || $install_boards == "y" ]]; then
  wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
  tar -xf arduino-1.8.13-linux64.tar.xz
fi

./arduino-1.8.13/arduino --pref "sketchbook.path=/home/$HOSTNAME/catkin_ws/src/versavis/firmware"
./arduino-1.8.13/arduino --pref "boardsmanager.additional.urls=https://github.com/ethz-asl/versavis_hw/raw/master/package_VersaVIS_index.json"
./arduino-1.8.13/arduino --pref "build.path=/home/$HOSTNAME/catkin_ws/src/versavis/firmware/build"

echo "Do you wish to install additional arduino boards? [y or Y to accept]"
read install_boards
if [[ $install_boards == "Y" || $install_boards == "y" ]]; then
  ./arduino-1.8.13/arduino --install-boards arduino:samd
  ./arduino-1.8.13/arduino --install-boards arduino:samd_beta
  ./arduino-1.8.13/arduino --install-boards VersaVIS:samd
fi
./arduino-1.8.13/arduino --upload --board VersaVIS:samd:VersaVIS --port /dev/versavis versavis_rtc/versavis_rtc.ino
