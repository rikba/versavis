# This message carries information about the external clock synchronization.

time receive_time    # The PPS arrival time of the remote time captured on the micro controller.
time remote_time     # The remote device time at the moment of the PPS signal.
float32[3] x         # [offset_us, skew_ppm, nominal_voltage_V, voltage_to_ppm].
float32[9] P         # Covariance w.r.t. x
float32 dt           # The time since the last filter update.
float32 ppm          # PPM based on current state
uint16 dac           # DAC command

uint32 pps_cnt       # The sequential number of the PPS signal.

bool sync            # True if clock offset remains within RTC_CTRL_CONVERGENCE
