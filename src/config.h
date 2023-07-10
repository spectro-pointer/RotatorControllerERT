#include <Arduino.h>

#define STEP_AZM_PIN  2
#define DIR_AZM_PIN   3
#define STEP_ELV_PIN  4
#define DIR_ELV_PIN   5

#define AZM_SPR 200 // Consider microstep
#define ELV_SPR 200 // Consider microstep

#define AZM_RATIO 1 // Gear ratio between stepper and tracker head
#define ELV_RATIO 1 // Gear ratio between stepper and tracker head

#define AZM_MAX_SPEED 1000 // In steps per second
#define ELV_MAX_SPEED 1000 // In steps per second

#define AZM_ACCEL 1000 // In steps per second per second
#define ELV_ACCEL 1000 // In steps per second per second
 
#define MIN_PULSE_WIDTH 20 // For the driver, in microseconds

#define CMD_PORT Serial