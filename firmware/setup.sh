source ../../../devel/setup.bash
rm -rf libraries/ros_lib
rosrun rosserial_arduino make_libraries.py libraries

# Maybe hardcode compiler and bossac path and compiler.arm.cmsis.c.flags in ~/.arduino15/packages/VersaVIS/hardware/samd/1.0.0/platform.txt
# https://github.com/Open-Bionics/Beetroot/issues/6
arduino --pref "sketchbook.path=/home/$HOSTNAME/catkin_ws/src/versavis/firmware"
arduino --pref "boardsmanager.additional.urls=https://github.com/ethz-asl/versavis_hw/raw/master/package_VersaVIS_index.json"
#arduino --install-boards arduino:samd
#arduino --install-boards arduino:samd_beta
#arduino --install-boards VersaVIS:samd
arduino --upload --board VersaVIS:samd:VersaVIS --port /dev/versavis versavis_rtc/versavis_rtc.ino
