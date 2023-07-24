#include <Arduino.h>

#define STEP_AZM_PIN  28
#define DIR_AZM_PIN   29
#define STEP_ELV_PIN  24
#define DIR_ELV_PIN   25

#define VRX_PIN  A0
#define VRY_PIN  A1
#define BTN_PIN   A4
#define BTN_PRESSED 0

#define AZM_SPR 3200 // Consider microstep
#define ELV_SPR 3200 // Consider microstep

#define AZM_RATIO 30 // Gear ratio between stepper and tracker head
#define ELV_RATIO 30 // Gear ratio between stepper and tracker head

#define ELV_MAX_ANGLE 100.0
#define ELV_MIN_ANGLE -10.0

#define AZM_MAX_SPEED 5000 // In steps per second 75000
#define ELV_MAX_SPEED 5000 // In steps per second 100000

#define AZM_MAX_ACCEL 25000 // In steps per second per second 25000 
#define ELV_MAX_ACCEL 25000 // In steps per second per second 25000

#define AZM_MAX_ACCEL_DEG AZM_MAX_ACCEL * 360.00 / (AZM_SPR * AZM_RATIO) // In deg per second per second
#define ELV_MAX_ACCEL_DEG ELV_MAX_ACCEL * 360.00 / (ELV_SPR * ELV_RATIO) // In deg per second per second

#define AZM_MAX_SPEED_DEG AZM_MAX_SPEED * 360.00 / (AZM_SPR * AZM_RATIO) // In deg per second
#define ELV_MAX_SPEED_DEG ELV_MAX_SPEED * 360.00 / (ELV_SPR * ELV_RATIO) // In deg per second
 
#define MIN_PULSE_WIDTH 20 // For the driver, in microseconds
#define BINOC_DATA_RATE 100 // In Hz

#define CMD_PORT Serial5
#define CMD_BAUD 115200

#define SERIAL_TO_PC Serial