#ifndef versavis_configuration_h
#define versavis_configuration_h

/* ----- General configuration -----*/
// Activate USB serial interface for ARM processor types.
#define USE_USBCON

// Specify the CPU frequency of the controller.
#define CPU_FREQ_HZ 48e6

// Specify the trigger pulse width;
#define TRIGGER_PULSE_US 10

/* ----- Camera configuration ----*/
// Camera 0
#define CAM0_TOPIC "/versavis/cam0/"
#define CAM0_RATE 20
#define CAM0_TYPE trigger_type::NON_INVERTED
#define CAM0_TRIGGER_PIN 14
#define CAM0_EXPOSURE_PIN 5

// Camera 1
#define CAM1_TOPIC ""
#define CAM1_RATE 20
#define CAM1_TYPE trigger_type::NON_INVERTED
#define CAM1_TRIGGER_PIN 15
#define CAM1_EXPOSURE_PIN 6

// Camera 2
#define CAM2_TOPIC ""
#define CAM2_RATE 5
#define CAM2_TYPE trigger_type::NON_INVERTED
#define CAM2_TRIGGER_PIN 16
#define CAM2_EXPOSURE_PIN 7

/* ----- IMU -----*/
// Possible values: USE_ADIS16445, USE_ADIS16448AMLZ, USE_ADIS16448BMLZ and
// USE_ADIS16460
#define USE_ADIS16460
#define IMU_TOPIC "/versavis/imu_micro"
#define IMU_RATE 500
#define IMU_CALIBRATION_SAMPLES 9 // Power of 2**9 = 512 samples

/* ----- ExternalEvent -----*/
#define EXT_EVENT
#ifdef EXT_EVENT
#define EXT_EVENT_TOPIC "/versavis/ext_event"
#define EXT_EVENT_PIN 2
#define EXT_EVENT_LOGIC RISING
#endif

/* ----- Additional triggers ----- */
// Define whether additional test outputs should be used.
// #define ADD_TRIGGERS
#ifdef ADD_TRIGGERS
#define ADDITIONAL_TEST_PIN 2
#endif

/* ----- Illumination module ----- */
// Activation of the illumination module.
// #define ILLUMINATION_MODULE
#ifdef ILLUMINATION_MODULE
#define ILLUMINATION_PWM_PIN 2
#define ILLUMINATION_PIN 26
#endif

/* ----- RTC control ----- */
#define RTC_GCLKIN_10MHZ
#ifdef RTC_GCLKIN_10MHZ
#define RTC_FREQ 10e6
#define RTC_CLK_SYNC // Activate RTC synchronization
// https://docs.google.com/spreadsheets/d/1pf0QecrXWwRHv6BMs1xoJcgUBYCmLpuZk2bO-ykS4-I/edit#gid=0
#define RTC_CLK_SYNC_X1 1.5          // The nominal control offset [V]
#define RTC_CLK_SYNC_X2 6.0          // The nominal control scale [ppm/V]
#define RTC_CLK_SYNC_X0_OFFSET 2.0   // Hotstart initial clock offset [us].
#define RTC_CLK_SYNC_X1_OFFSET 0.1   // Initial offset of control offset [V]
#define RTC_CLK_SYNC_X2_OFFSET 0.3   // Initial offset of control offset [ppm/V]
#define RTC_CLK_SYNC_U_REF 3.3       // Reference voltage [V]
#define RTC_CLK_SYNC_DAC_RANGE 0x3FF // DAC range [1]
#define RTC_CLK_SYNC_DAC_MIN 155.0   // Max. allowed DAC
#define RTC_CLK_SYNC_DAC_MAX 775.0   // Min. allowed DAC
#define RTC_CLK_SYNC_LQR_GAIN 3.08266065
#define RTC_CLK_SYNC_CONV_CRIT 1.0  // Offset convergence criterion [us]
#define RTC_CLK_SYNC_CONV_WINDOW 60 // Window of converged values
#else
#define RTC_FREQ 32768
#define RTC_FREQ_STABILITY 0.04 // PPM / delta deg C
#define RTC_MAX_SKEW 20         // PPM
#endif

/* ----- GNSS time synchronization ----- */
// Activation of GNSS time synchronization.
#define GNSS_SYNC
#ifdef GNSS_SYNC
#define GNSS_SYNC_UART Serial
#define GNSS_SYNC_BAUD 115200
#define GNSS_PPS_ACCURACY 60.0e-9 // [s]
#endif

/* ----- Debug prints. ----- */
// Define whether debug mode should be used. This provides output on the
// standard console but invalidates ROS communication.
// #define DEBUG

#endif // versavis_configuration_h
