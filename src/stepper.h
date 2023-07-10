#include <AccelStepper.h>
#include <config.h>

int32_t degToStepAzm(float deg);
int32_t degToStepElv(float deg);

float stepToDegAzm(int32_t step);
float stepToDegElv(int32_t step);

AccelStepper stepperAzm(AccelStepper::DRIVER, STEP_AZM_PIN, DIR_AZM_PIN);
AccelStepper stepperElv(AccelStepper::DRIVER, STEP_ELV_PIN, DIR_ELV_PIN);

void azmSetup();
void elvSetup();