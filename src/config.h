#include <Arduino.h>

#define STEP_AZM_PIN  28
#define DIR_AZM_PIN   29
#define STEP_ELV_PIN  24
#define DIR_ELV_PIN   25

#define VRX_PIN  A4
#define VRY_PIN  A5
#define BTN_PIN  A1
#define BTN_PRESSED 0

#ifdef ANTENNA_TRACKER

    #define MIN_PULSE_WIDTH 5 // For the driver, in microseconds

    #define AZM_SPR 3200 // Consider microstep
    #define ELV_SPR 3200 // Consider microstep

    #define AZM_RATIO 30 // Gear ratio between stepper and tracker head
    #define ELV_RATIO 30 // Gear ratio between stepper and tracker head

    #define AZM_MAX_ANGLE 180.0
    #define AZM_MIN_ANGLE -180.0

    #define ELV_MAX_ANGLE 100.0
    #define ELV_MIN_ANGLE -10.0

    #define AZM_MAX_SPEED_DEG 360.0
    #define ELV_MAX_SPEED_DEG 360.0

    #define AZM_MAX_ACCEL_DEG 280.0
    #define ELV_MAX_ACCEL_DEG 280.0

    #define AZM_ACC_LIMIT true
    #define ELV_ACC_LIMIT true

    #define AZM_INVERT_DIR true
    #define ELV_INVERT_DIR false
#endif 

#ifdef CAMERA_TRACKER

    #define MIN_PULSE_WIDTH 5 // For the driver, in microseconds

    #define AZM_SPR 800 // Consider microstep
    #define ELV_SPR 51200 // Consider microstep

    #define AZM_RATIO 100 // Gear ratio between stepper and tracker head
    #define ELV_RATIO 1 // Gear ratio between stepper and tracker head

    #define AZM_MAX_ANGLE 180.0
    #define AZM_MIN_ANGLE -180.0

    #define ELV_MAX_ANGLE 90.0
    #define ELV_MIN_ANGLE -15.0

    #define AZM_MAX_SPEED_DEG 300.0
    #define ELV_MAX_SPEED_DEG 300.0

    #define AZM_MAX_ACCEL_DEG 500.0
    #define ELV_MAX_ACCEL_DEG 500.0

    #define AZM_ACC_LIMIT true
    #define ELV_ACC_LIMIT false

    #define AZM_INVERT_DIR true
    #define ELV_INVERT_DIR true
#endif 

#ifdef DUMBO

    #define MIN_PULSE_WIDTH 5 // For the driver, in microseconds

    #define AZM_SPR 3200 // Consider microstep
    #define ELV_SPR 3200 // Consider microstep

    #define AZM_RATIO 26.6703125 // Gear ratio between stepper and tracker head
    #define ELV_RATIO 50.09375 // Gear ratio between stepper and tracker head

    #define AZM_MAX_ANGLE 180.0
    #define AZM_MIN_ANGLE -180.0

    #define ELV_MAX_ANGLE 90.0
    #define ELV_MIN_ANGLE -20.0

    #define AZM_MAX_SPEED_DEG 500.0
    #define ELV_MAX_SPEED_DEG 500.0

    #define AZM_MAX_ACCEL_DEG 1000.0
    #define ELV_MAX_ACCEL_DEG 1000.0

    #define AZM_ACC_LIMIT true
    #define ELV_ACC_LIMIT true

    #define AZM_INVERT_DIR true
    #define ELV_INVERT_DIR false
#endif 

#ifdef SCOPE_TRACKER

    #define MIN_PULSE_WIDTH 5 // For the driver, in microseconds

    #define AZM_SPR 3200 // Consider microstep
    #define ELV_SPR 3200 // Consider microstep

    #define AZM_RATIO 54 // Gear ratio between stepper and tracker head
    #define ELV_RATIO 100 // Gear ratio between stepper and tracker head

    #define AZM_MAX_ANGLE 160.0
    #define AZM_MIN_ANGLE -175.0

    #define ELV_MAX_ANGLE 90.0
    #define ELV_MIN_ANGLE -30.0

    #define AZM_MAX_SPEED_DEG 50.0
    #define ELV_MAX_SPEED_DEG 50.0

    #define AZM_MAX_ACCEL_DEG 75.0
    #define ELV_MAX_ACCEL_DEG 75.0

    #define AZM_ACC_LIMIT true
    #define ELV_ACC_LIMIT true

    #define AZM_USE_ENDSTOP
    //#define ELV_USE_ENDSTOP

    #define AZM_ENDSTOP_PIN 0
    #define AZM_ENDSTOP_ACTIVE_LEVEL LOW
    #define AZM_ENDSTOP_POSITION -180.0
    #define AZM_HOMING_DIRECTION_CCW
    #define AZM_HOMING_SPEED 5.0

    #define ELV_ENDSTOP_PIN 3
    #define ELV_ENDSTOP_ACTIVE_LEVEL LOW
    #define ELV_ENDSTOP_POSITION 90.0
    #define ELV_HOMING_DIRECTION_CW
    #define ELV_HOMING_SPEED 1.5

    #define USE_ENABLE 
    #define ENABLE_PIN 3

    #define AZM_INVERT_DIR true
    #define ELV_INVERT_DIR false
#endif 

#define AZM_MAX_ACCEL AZM_MAX_ACCEL_DEG * (AZM_SPR * AZM_RATIO) / 360.00 // In steps per second per second
#define ELV_MAX_ACCEL ELV_MAX_ACCEL_DEG * (ELV_SPR * ELV_RATIO) / 360.00 // In steps per second per second

#define AZM_MAX_SPEED AZM_MAX_SPEED_DEG * (AZM_SPR * AZM_RATIO) / 360.00 // In steps per second
#define ELV_MAX_SPEED ELV_MAX_SPEED_DEG * (ELV_SPR * ELV_RATIO) / 360.00 // In steps per second
 
#define ESTIMATION_RATE 100 // In Hz
#define BINOC_DATA_RATE 20  // In Hz

#define CMD_PORT Serial5
#define CMD_BAUD 115200

#define SERIAL_TO_PC Serial