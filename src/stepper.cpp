#include <stepper.h>

int32_t degToStepAzm(float deg) {
    return (AZM_RATIO * AZM_SPR * deg / 360);
}

float stepToDegAzm(int32_t step) {
    return (360.00 * step / (AZM_SPR * AZM_RATIO));
}

int32_t degToStepElv(float deg) {
    return (ELV_RATIO * ELV_SPR * deg / 360);
}

float stepToDegElv(int32_t step) {
    return (360.00 * step / (ELV_SPR * ELV_RATIO));
}

void azmSetup() {
    stepperAzm.setMaxSpeed(AZM_MAX_SPEED);
    stepperAzm.setAcceleration(AZM_ACCEL);
    stepperAzm.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperAzm.setPinsInverted(true, false, true);
}

void elvSetup() {
    stepperElv.setMaxSpeed(ELV_MAX_SPEED);
    stepperElv.setAcceleration(ELV_ACCEL);
    stepperElv.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperElv.setPinsInverted(false, false, true);
}
