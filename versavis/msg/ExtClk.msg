# This message carries information about the external clock synchronization.

time receive_time    # The PPS arrival time of the remote time captured on the micro controller.
time remote_time     # The remote device time at the moment of the PPS signal.
float32[2] x         # [offset_s, skew_ppm].
float32[4] P         # [P11, P12, P21, P22]

time last_update     # The system time at the last update.
uint32 pps_cnt       # The sequential number of the PPS signal.
