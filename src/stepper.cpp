#include <stepper.h>
#include <config.h>

solverClass::solverClass(bool accLimitInput) 
: alpha(0), alpha1(0), beta(0), beta1(0), beta2(0), lambda(0), speedOutput(0), accLimit(accLimitInput)
{

}

void solverClass::update(float alphaIn, float speedIn, float lastPoint, float maxSpeed, float maxAccel) {

    // Serial.print("Updating solver: ");
    addNewCmdInBuffer(lastPoint);

    double sampleTime = 1.0/BINOC_DATA_RATE;

    alpha = alphaIn;
    alpha1 = speedIn;

    beta1 = (cmdBuffer[2]-cmdBuffer[0])/(2*sampleTime);
    beta2 = (cmdBuffer[0]-2*cmdBuffer[1]+cmdBuffer[2])/(sampleTime*sampleTime);

    beta = cmdBuffer[1];

    // Condition one, the absolute value of the speed delta must be lower than the maximum acceleration * the sample time

    bool conditionOne = (abs(beta1-alpha1) < maxAccel*sampleTime);

    // Condition two, the absolute value of the position delta must be lower than the distance achievable with the 
    // optimal trajectory

    double distanceToDo = abs(beta-alpha);

    double tfOptimal = sampleTime;
    double tmOptimal = 0.5*(((beta1-alpha1)/maxAccel)+tfOptimal);
    tmOptimal = constrain(tmOptimal, sampleTime/1000.0, tfOptimal);

    double lambdaOptimal1;
    double lambdaOptimal2;

    if (abs(alpha1)>abs(beta1)) {
        lambdaOptimal1 = alpha1+tmOptimal*maxAccel;
        lambdaOptimal2 = alpha1-tmOptimal*maxAccel;
    }
    else {
        lambdaOptimal1 = beta1+(tfOptimal-tmOptimal)*maxAccel;
        lambdaOptimal2 = beta1-(tfOptimal-tmOptimal)*maxAccel;
    }

    double distanceOptimal1 = (tmOptimal/2.0)*(alpha1+lambdaOptimal1)+((tfOptimal-tmOptimal)/2.0)*(beta1+lambdaOptimal1);
    double distanceOptimal2 = (tmOptimal/2.0)*(alpha1+lambdaOptimal2)+((tfOptimal-tmOptimal)/2.0)*(beta1+lambdaOptimal2);

    double distanceOptimal = max(abs(distanceOptimal1), abs(distanceOptimal2));
    bool conditionTwo = (distanceToDo < distanceOptimal);

    tf = sampleTime;

    double goalSpeed;

    if ((conditionOne and conditionTwo) or !accLimit) {
        solverMode = 0;
        pointIsReachable = true;
        tm = 0.5*((beta1-alpha1)/maxAccel+tf);
        tm = constrain(tm, sampleTime/1000.0, tf);
        lambda = 2.0*((beta-alpha)/tf)+(tm/tf)*((beta1-alpha1)/2.0)-(beta1/2.0);
    }
    else if (accLimit) {
        pointIsReachable = false;
        int speedSign = ((beta-alpha)>0)-((beta-alpha)<0);
        goalSpeed = speedSign*sqrt(maxAccel*abs(beta-alpha));
        double speedError = goalSpeed-alpha1;

        solverMode = 1;
        accelGoal = constrain((speedError/sampleTime),-maxAccel,maxAccel);
    }
    // Serial.print(map(alpha,0,360,0,1000));
    // Serial.print(" ");
    // Serial.print(map(beta,0,360,0,1000));
    // Serial.print(" ");
    // Serial.print(map(lastPoint,0,360,0,1000));
    // Serial.print(" ");
    // Serial.print(solverMode*100.0);
    // Serial.print(" ");
    // Serial.print(goalSpeed*2.0);
    // Serial.print(" ");
    // Serial.println(alpha1*2.0);
    //Serial.print(" ");
    //Serial.println((micros()-timeLastUpdated)/20.0);

    timeLastUpdated = micros();
}

float solverClass::computeSpeed(double maxSpeed) {
    double relativeTime = (micros()-timeLastUpdated)/1000000.0;

    if (solverMode == 0) {
        if (relativeTime<tm) {
            speedOutput = alpha1+(relativeTime/tm)*(lambda-alpha1);
        } else if (relativeTime<tf) {
            relativeTime=relativeTime-tm;
            if ((tf-tm)!=0) {
            speedOutput = lambda+(relativeTime/(tf-tm))*(beta1-lambda);
            }
            else {
                speedOutput = lambda;
            }
        }
        else {
            speedOutput = beta1;
        }
    }
    else {
        speedOutput = alpha1+accelGoal*relativeTime;
    }
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

controlOutput conClass::computeOutputSpeedPosition() {
    outputSpeedPosition.azmSpeed = azm.computeSpeed(AZM_MAX_SPEED_DEG);
    outputSpeedPosition.elvSpeed = elv.computeSpeed(ELV_MAX_SPEED_DEG);
    return outputSpeedPosition;
}

conClass::conClass()
:   azmSpr(AZM_SPR), elvSpr(ELV_SPR), 
    azmGear(AZM_RATIO), elvGear(ELV_RATIO),
    stepperAzm(AccelStepper::DRIVER, STEP_AZM_PIN, DIR_AZM_PIN), 
    stepperElv(AccelStepper::DRIVER, STEP_ELV_PIN, DIR_ELV_PIN),
    azm(AZM_ACC_LIMIT), elv(ELV_ACC_LIMIT)
{
    stepperAzm.setMaxSpeed(AZM_MAX_SPEED);
    stepperAzm.setAcceleration(AZM_MAX_ACCEL);
    stepperAzm.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperAzm.setPinsInverted(AZM_INVERT_DIR, false, true);

    stepperElv.setMaxSpeed(ELV_MAX_SPEED);
    stepperElv.setAcceleration(ELV_MAX_ACCEL);
    stepperElv.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepperElv.setPinsInverted(ELV_INVERT_DIR, false, true);
}

void conClass::update(PacketTrackerCmd cmd) {
    azm.update(stepToDegAzm(stepperAzm.currentPosition()), stepToDegAzm(stepperAzm.speed()), cmd.azm, AZM_MAX_SPEED_DEG, AZM_MAX_ACCEL_DEG);
    elv.update(stepToDegElv(stepperElv.currentPosition()), stepToDegElv(stepperElv.speed()), cmd.elv, ELV_MAX_SPEED_DEG, ELV_MAX_ACCEL_DEG);
}

controlOutput conClass::getOutputSpeedPosition() {
    return outputSpeedPosition;
}

TRACKING_MODE conClass::getMode() {
    return mode;
}

void conClass::setMode(TRACKING_MODE modeIn) {
    mode = modeIn;
}

long  conClass::degToStepAzm(double deg) {
    return (azmGear * azmSpr * deg / 360);
}

double conClass::stepToDegAzm(long step) {
    return (360.00 * step / (azmSpr * azmGear));
}

long  conClass::degToStepElv(double deg) {
    return (elvGear * elvSpr * deg / 360);
}

double conClass::stepToDegElv(long step) {
    return (360.00 * step / (elvSpr * elvGear));
}

void conClass::setGroundPosition(PositionPacket positionIn) {
    groundPosition = positionIn;
};

double getAngleStepper(double angle1, double angle2) {
    double angle = angle2 - angle1;
    if (angle > 180) {
        angle -= 360;
    }
    if (angle < -180) {
        angle += 360;
    }
    return angle;
}

