#include <stepper.h>

solverClass::solverClass() 
: alpha(0), alphaPrev(0), alpha1(0), beta(0), betaPrev(0), beta1(0), beta1Prev(0), beta2(0), betaPlusOne(0), betaPlusOne1(0), lambda(0), speedOutput(0), timeLastUpdated(0), timeStep(1.0/BINOC_DATA_RATE)
{

}

void solverClass::update(float alphaIn, float beta) {
    alpha = alphaIn;
    alpha1 = (alphaPrev-alpha)/timeStep;
    alphaPrev = alpha;

    beta1 = (beta-betaPrev)/timeStep;
    beta2 = (beta1-beta1Prev)/timeStep;
    beta1Prev = beta1;
    betaPrev = beta;

    betaPlusOne = beta+timeStep*beta1+((timeStep*timeStep)/2)*beta2;
    betaPlusOne1 = beta1+timeStep*beta2;

    lambda = 2*(betaPlusOne/timeStep)-2*(alpha/timeStep)-2*alpha1+(alpha1/2.0)+(betaPlusOne1/2.0);
}

float solverClass::computeSpeed() {
    double relativeTime = (millis()-timeLastUpdated)/1000.0;
    if (relativeTime<(timeStep/2)) {
        speedOutput = alpha1+(relativeTime/(timeStep/2))*(lambda-alpha1);
    } else {
        relativeTime=relativeTime-(timeStep/2);
        speedOutput = lambda+(relativeTime/(timeStep/2))*(betaPlusOne1-lambda);
    }
    return speedOutput;
}

float solverClass::getSpeed() {
    return speedOutput;
}

void solverClass::setTimeStep(double timeStepIn) {
    timeStep = timeStepIn;
}

controlOutput conClass::computeOutput() {
    output.azmSpeed = azm.computeSpeed();
    output.elvSpeed = elv.computeSpeed();
    return output;
}

conClass::conClass()
: stepperAzm(AccelStepper::DRIVER, STEP_AZM_PIN, DIR_AZM_PIN), 
  stepperElv(AccelStepper::DRIVER, STEP_ELV_PIN, DIR_ELV_PIN)
{
    stepperAzm.setMaxSpeed(AZM_MAX_SPEED);
    stepperAzm.setAcceleration(AZM_ACCEL);
    stepperAzm.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperAzm.setPinsInverted(true, false, true);

    stepperElv.setMaxSpeed(ELV_MAX_SPEED);
    stepperElv.setAcceleration(ELV_ACCEL);
    stepperElv.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperElv.setPinsInverted(false, false, true);
}

void conClass::update(PacketTrackerCmd cmd) {
    lastCmd = cmd;
    azm.update(stepToDegAzm(stepperAzm.currentPosition()), cmd.azm);
    elv.update(stepToDegElv(stepperElv.currentPosition()), cmd.elv);
}

controlOutput conClass::getOutput() {
    return output;
}

PacketTrackerCmd conClass::getLastCmd() {
    return lastCmd;
}

TRACKING_MODE conClass::getMode() {
    TRACKING_MODE modeOut;
    modeOut = (TRACKING_MODE)mode;
    return modeOut;
}

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
