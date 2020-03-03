# This message carries information about the external clock synchronization.

time receive_time    # The PPS arrival time of the remote time captured on the micro controller.
time remote_time     # The remote device time at the moment of the PPS signal.
float32[2] x         # [offset_us, skew_ppm].
float32[4] P         # [P11, P12, P21, P22]
float32 dt           # The time since the last filter update.

uint32 pps_cnt       # The sequential number of the PPS signal.

float32 u            # The clock control voltage.
float32 e            # The clock control error.
float32 i            # The clock control integrator.
float32 d            # The clock control differential error (skew).
uint16 dac           # DAC command
bool sync            # True if clock skew is within RTC_CTRL_CONVERGENCE
