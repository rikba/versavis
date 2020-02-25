# This message carries information about the external clock synchronization.

time receive_time # The PPS arrival time of the remote time captured on the micro controller.
time remote_time  # The remote device time at the moment of the PPS signal.
uint32 pps_cnt    # The sequential number of the PPS signal.
