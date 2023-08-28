#include <AccelStepper.h>
#include <config.h>
#include "comm.h"

enum TRACKING_MODE {
    STATIONARY,
    TRACKING_SMOOTH,
    TRACKING_STEP
};

float degToStepAzm(float deg);
float degToStepElv(float deg);

float stepToDegAzm(long step);
float stepToDegElv(long step);

struct controlOutput {
    float azmSpeed;
    float elvSpeed;
};

class solverClass {
    public:
        solverClass(bool accLimitInput);
        void update(float alpha, float speed, float newPoint, float maxSpeed, float maxAccel);
        float computeSpeed(double maxSpeed);
        float getSpeed();
        void addNewCmdInBuffer(float cmd);
    private: 
        float alpha;
        float alpha1;

        float beta;
        float beta1;
        float beta2; 

        float lambda;

        float speedOutput;

        unsigned long long timeLastUpdated;
        double tm;
        double tf;

        bool pointIsReachable;

        float cmdBuffer[3];

        int solverMode;
        double accelGoal;
        bool accLimit;
};

class conClass {
    public:
        conClass();
        void update(PacketTrackerCmd cmd);
        PacketTrackerCmd getLastCmd();
        controlOutput getOutput();
        controlOutput computeOutput();
        TRACKING_MODE getMode();
        void setMode(TRACKING_MODE mode);
        AccelStepper stepperAzm;
        AccelStepper stepperElv;
    private:
        solverClass azm;
        solverClass elv;
        controlOutput output;
        PacketTrackerCmd lastCmd;
        TRACKING_MODE mode; 
};

double getAngleStepper(double angle1, double angle2);
