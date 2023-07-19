#include <stepper.h>
#include <config.h>

solverClass::solverClass() 
: alpha(0), alpha1(0), beta(0), beta1(0), beta2(0), lambda(0), speedOutput(0)
{

}

void solverClass::update(float alphaIn, float speedIn, float lastPoint, float maxSpeed, float maxAccel) {

    // Serial.print("Updating solver: ");

    addNewCmdInBuffer(lastPoint);
    timeLastUpdated = micros();

    double sampleTime = 1.0/BINOC_DATA_RATE;

    alpha = alphaIn;
    alpha1 = speedIn;

    beta1 = (cmdBuffer[2]-cmdBuffer[0])/(2*sampleTime);
    beta2 = (cmdBuffer[0]-2*cmdBuffer[1]+cmdBuffer[2])/(sampleTime*sampleTime);

    beta = cmdBuffer[1];

    // // Condition one, the absolute value of the speed delta must be lower than the maximum acceleration * the sample time

    // bool conditionOne = (abs(beta1-alpha1) < maxAccel*sampleTime);

    // // Condition two, the absolute value of the position delta must be lower than the distance achievable with the 
    // // optimal trajectory

    // double distanceToDo = abs(beta-alpha);

    // double tfOptimal = sampleTime;
    // double tmOptimal = 0.5*(((beta1-alpha1)/maxAccel)+tfOptimal);
    // tmOptimal = constrain(tmOptimal, sampleTime/1000.0, tfOptimal);

    // double lambdaOptimal1;
    // double lambdaOptimal2;

    // if (abs(alpha1)>abs(beta1)) {
    //     lambdaOptimal1 = alpha1+tmOptimal*maxAccel;
    //     lambdaOptimal2 = alpha1-tmOptimal*maxAccel;
    // }
    // else {
    //     lambdaOptimal1 = beta1+(tfOptimal-tmOptimal)*maxAccel;
    //     lambdaOptimal2 = beta1-(tfOptimal-tmOptimal)*maxAccel;
    // }

    // double distanceOptimal1 = (tmOptimal/2.0)*(alpha1+lambdaOptimal1)+((tfOptimal-tmOptimal)/2.0)*(beta1+lambdaOptimal1);
    // double distanceOptimal2 = (tmOptimal/2.0)*(alpha1+lambdaOptimal2)+((tfOptimal-tmOptimal)/2.0)*(beta1+lambdaOptimal2);

    // double distanceOptimal = max(abs(distanceOptimal1), abs(distanceOptimal2));
    // bool conditionTwo = (distanceToDo < distanceOptimal);

    //tf = sampleTime;

    //double goalSpeed;

    // if (conditionOne and conditionTwo) {
    //     solverMode = 0;
    //     pointIsReachable = true;
    //     tm = 0.5*((beta1-alpha1)/maxAccel+tf);
    //     tm = constrain(tm, sampleTime/1000.0, tf);
    //     lambda = 2.0*((beta-alpha)/tf)+(tm/tf)*((beta1-alpha1)/2.0)-(beta1/2.0);
    // }
    // else {
        //pointIsReachable = false;
        // int speedSign = ((beta-alpha)>0)-((beta-alpha)<0);
        // double goalSpeed = speedSign*sqrt(maxAccel*abs(beta-alpha));
        // double speedError = goalSpeed-alpha1;

        // solverMode = 1;
        // accelGoal = constrain((speedError/sampleTime),-maxAccel,maxAccel);
    // }
    // Serial.print(map(alpha,0,360,0,1000));
    // Serial.print(" ");
    // Serial.print(map(beta,0,360,0,1000));
    // Serial.print(" ");
    // Serial.print(map(lastPoint,0,360,0,1000));
    // Serial.print(" ");
    // Serial.print(printMode*100.0);
    // Serial.print(" ");
    // Serial.print(goalSpeed*2.0);
    // Serial.print(" ");
    // Serial.println(alpha1*2.0);
}

float solverClass::computeSpeed(double position, double goal, double maxSpeed, double maxAccel) {
    // double relativeTime = (micros()-timeLastUpdated)/1000000.0;

    // if (solverMode == 0) {
    //     if (relativeTime<tm) {
    //         speedOutput = alpha1+(relativeTime/tm)*(lambda-alpha1);
    //     } else if (relativeTime<tf) {
    //         relativeTime=relativeTime-tm;
    //         if ((tf-tm)!=0) {
    //         speedOutput = lambda+(relativeTime/(tf-tm))*(beta1-lambda);
    //         }
    //         else {
    //             speedOutput = lambda;
    //         }
    //     }
    //     else {
    //         speedOutput = beta1;
    //     }
    // }
    // else {
    //     speedOutput = alpha1+accelGoal*relativeTime;
    // }

    static long long lastTimeSpeedComputed = micros();

    int speedSign = ((beta-position)>0)-((beta-position)<0);
    double goalSpeed = speedSign*sqrt(maxAccel*abs(beta-position));
    double speedError = goalSpeed-speedOutput;

    //solverMode = 1;
    accelGoal = constrain((speedError/sampleTime),-maxAccel,maxAccel);

    speedOutput = speedOutput + accelGoal*(micros()-lastTimeSpeedComputed)/1000000.0;

    speedOutput = constrain(speedOutput, -maxSpeed, maxSpeed);
    return speedOutput;
}

float solverClass::getSpeed() {
    return speedOutput;
}

void solverClass::addNewCmdInBuffer(float cmdIn) {
    cmdBuffer[0] = cmdBuffer[1];
    cmdBuffer[1] = cmdBuffer[2];
    cmdBuffer[2] = cmdIn;
}

controlOutput conClass::computeOutput() {
    output.azmSpeed = azm.computeSpeed(stepToDegAzm(stepperAzm.currentPosition()),stepToDegAzm(stepperAzm.get),AZM_MAX_SPEED_DEG);
    output.elvSpeed = elv.computeSpeed(ELV_MAX_SPEED_DEG);
    return output;
}

conClass::conClass()
: stepperAzm(AccelStepper::DRIVER, STEP_AZM_PIN, DIR_AZM_PIN), 
  stepperElv(AccelStepper::DRIVER, STEP_ELV_PIN, DIR_ELV_PIN)
{
    stepperAzm.setMaxSpeed(AZM_MAX_SPEED);
    stepperAzm.setAcceleration(AZM_MAX_ACCEL);
    stepperAzm.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperAzm.setPinsInverted(true, false, true);

    stepperElv.setMaxSpeed(ELV_MAX_SPEED);
    stepperElv.setAcceleration(ELV_MAX_ACCEL);
    stepperElv.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperElv.setPinsInverted(false, false, true);
}

void conClass::update(PacketTrackerCmd cmd) {
    azm.update(stepToDegAzm(stepperAzm.currentPosition()), stepToDegAzm(stepperAzm.speed()), cmd.azm, AZM_MAX_SPEED_DEG, AZM_MAX_ACCEL_DEG);
    elv.update(stepToDegElv(stepperElv.currentPosition()), stepToDegElv(stepperElv.speed()), cmd.elv, ELV_MAX_SPEED_DEG, ELV_MAX_ACCEL_DEG);
}

controlOutput conClass::getOutput() {
    return output;
}

TRACKING_MODE conClass::getMode() {
    return mode;
}

void conClass::setMode(TRACKING_MODE modeIn) {
    mode = modeIn;
}

float degToStepAzm(float deg) {
    return (AZM_RATIO * AZM_SPR * deg / 360);
}

float stepToDegAzm(long step) {
    return (360.00 * step / (AZM_SPR * AZM_RATIO));
}

float degToStepElv(float deg) {
    return (ELV_RATIO * ELV_SPR * deg / 360);
}

float stepToDegElv(long step) {
    return (360.00 * step / (ELV_SPR * ELV_RATIO));
}
