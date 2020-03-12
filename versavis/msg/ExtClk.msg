# This message carries information about the external clock synchronization.

time receive_time    # The PPS arrival time of the remote time captured on the micro controller.
time remote_time     # The remote device time at the moment of the PPS signal.
float32[3] x         # [offset_us, skew_ppm, nominal_voltage_V, voltage_to_ppm].
float32[9] P         # Covariance w.r.t. x
float32 dt           # The time since the last filter update.
float32 ppm          # PPM based on current state

uint32 pps_cnt       # The sequential number of the PPS signal.

float32 u            # The clock control voltage.
float32 e            # The clock control error.
float32 i            # The clock control integrator.
float32 d            # The clock control differential error (skew).
uint16 dac           # DAC command
bool sync            # True if clock skew is within RTC_CTRL_CONVERGENCE
