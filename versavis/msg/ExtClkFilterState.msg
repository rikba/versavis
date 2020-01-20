std_msgs/Time stamp   # The update filter time (t_nmea + pps_cnt)

float32 x           # Estimated state [ppm] offset from nominal frequency.
float32 P           # Estimated posterior covariance [ppm^2]
float32 R           # Measurement noise covariance [Ticks^2/second^2]
float32 Q           # Process noise covariance [ppm^2]
uint32 z            # Measurement [Ticks/second]
uint32 pps_cnt      # The number of seconds since start
uint32 pps_cnt_prev # The pps count at the last update
uint32 t_nmea       # The absolute time at pps_cnt == 0
float32 x_nspt      # Estimated inverse state [nano seconds / Tick]
uint32 nominal_freq # The nominal frequency of the clock [Ticks / second]
