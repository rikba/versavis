std_msgs/Time stamp   # The update filter time (t_nmea + pps_cnt)

float64 x           # Estimated state [Ticks/second]
float64 P           # Estimated posterior covariance [Ticks^2/second^2]
float64 R           # Measurement noise covariance [Ticks^2/second^2]
float64 Q           # Process noise covariance [Ticks^2/second^2]
uint32 z            # Measurement [Ticks/second]
uint32 pps_cnt      # The number of seconds since start
uint32 pps_cnt_prev # The pps count at the last update
uint32 t_nmea       # The absolute time at pps_cnt == 0
float64 x_nspt      # Estimated inverse state [nano seconds / Tick]
