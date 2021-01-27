#!/bin/bash
source ../../../devel/setup.bash
rm -rf libraries/ros_lib
rosrun rosserial_arduino make_libraries.py libraries

# Maybe hardcode compiler and bossac path and compiler.arm.cmsis.c.flags in ~/.arduino15/packages/VersaVIS/hardware/samd/1.0.0/platform.txt
# https://github.com/Open-Bionics/Beetroot/issues/6

echo "Do you wish to install the arduino CLI? [y or Y to accept]"
read install_arduino
if [[ $install_arduino == "Y" || $install_arduino == "y" ]]; then
  CURRENT_DIR=$(pwd)
  cd ~
  wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
  tar -xf arduino-1.8.13-linux64.tar.xz
  rm arduino-1.8.13-linux64.tar.xz
  sudo ./arduino-1.8.13/install.sh
  cd $CURRENT_DIR
fi

arduino --pref "sketchbook.path=/home/$HOSTNAME/catkin_ws/src/versavis/firmware"
arduino --pref "boardsmanager.additional.urls=https://github.com/ethz-asl/versavis_hw/raw/master/package_VersaVIS_index.json"
arduino --pref "build.path=/home/$HOSTNAME/catkin_ws/src/versavis/firmware/build"

echo "Do you wish to install additional arduino boards? [y or Y to accept]"
read install_boards
if [[ $install_boards == "Y" || $install_boards == "y" ]]; then
  arduino --install-boards arduino:samd
  arduino --install-boards arduino:samd_beta
  arduino --install-boards VersaVIS:samd
fi

echo "Start building firmware."
arduino --upload --board VersaVIS:samd:VersaVIS --port /dev/versavis versavis_rtc/versavis_rtc.ino
