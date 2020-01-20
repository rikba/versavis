std_msgs/Time stamp   # The update filter time (t_nmea + pps_cnt)

float32 x           # Estimated state [ppm] offset from nominal frequency.
float32 P           # Estimated posterior covariance [ppm^2]
float32 R           # Measurement noise covariance [Ticks^2/second^2]
float32 Q           # Process noise covariance [ppm^2]
int16 z             # Measurement: Difference between actual and nominal ticks per second [Ticks/second]
uint32 pps_cnt      # The number of seconds since start
uint32 pps_cnt_prev # The pps count at the last update
uint32 t_nmea       # The absolute time at pps_cnt == 0
