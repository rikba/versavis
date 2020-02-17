# This is a super-minimized message to hold data from a baro.
#
# Raw data that has to be scaled!
#
# To minimize the amount of data transfered by this message (e.g. form an Arduino through rosserial)
# the amount of data is truncated to its absolute minimum.

std_msgs/Time time
uint32 number

int16 pressure
