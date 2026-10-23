#ifndef __HARDWARE_DEFS__
#define __HARDWARE_DEFS__

#define BATTERY_PIN         A6
#define LED_PIN             9
#define STBY_PIN            8

#define LEFT_ENCODER_PIN    2
#define LEFT_PWM_PIN        5
#define LEFT_DIR_PIN        7
#define LEFT_DIR_INV_PIN    4

#define RIGHT_ENCODER_PIN   3
#define RIGHT_PWM_PIN       6
#define RIGHT_DIR_PIN       11
#define RIGHT_DIR_INV_PIN   12

#define MAIN_LOOP_PERIOD    100 // ms, 10ms = 100Hz
#define MAIN_LOOP_MAX       150 // ms

//#define MAX_LINEAR_VEL      (0.08)  // m/s
#define MAX_LINEAR_VEL      (0.3)  // m/s
#define MAX_ANGULAR_VEL     (0.785) // rad/s

#define WHEEL_DIAMETER      (0.047) // m
#define WHEEL_TRACK         (0.225) // m
//#define ENCODER_RESOLUTION  1040
#define ENCODER_RESOLUTION  260
#define MOTOR_MAX_PWM       128

// Battery voltage reading (18650x2):
#define REF_VOLTAGE         5.0
#define ADC_R1              100000.0
#define ADC_R2              10000.0

// Manually tuning parameters:
#define MOTOR_KP            (200.0)
#define MOTOR_KI            (0.0)
//#define MOTOR_KD            (16.0)
#define MOTOR_KD            (0.0)
#define RAMP_LINEAR_ACCEL   (0.15)

#define LED_PWM_VALUE       10
#define LED_ON_PERIOD       100 // ms

#define ROS_SERIAL_RATE     115200

#define NSE_GPS_W           (0.0)
#define NSE_ODOM_W          (0.5)
#define NSE_COMPASS_W       (0.0)
#define NSE_PRED_W          (0.5)

#endif
