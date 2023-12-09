#include <AccelStepper.h>
#include <config.h>
#include "comm.h"

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ControlParameter {
  double P;
  double cutOffFreq;
  double maxTimeWindow;
};

struct AxisParemeter {
  double maxAcceleration;
  double maxSpeed;
};

struct SystemParemter {
  IPAddress ip;
  int port;
};

struct TrackerParameter {
  AxisParemeter azmParameter;
  AxisParemeter elvParameter;
  SystemParemter ethernetParameter;
  ControlParameter control;
};

struct CameraErrorPacket {
  int32_t Cx;
  int32_t Cy;
  int32_t Tx;
  int32_t Ty;
  bool isVisible;
}; 

struct PositionPacket {
  double azm;
  double elv;
};

struct PointerPacket {
  double azm;
  double elv;
  bool isPressed;
};

enum TRACKING_MODE {
    STATIONARY,
    TRACKING_POSITION,
    TRACKING_ERROR
};

long degToStepAzm(double deg);
long degToStepElv(double deg);

double stepToDegAzm(long step);
double stepToDegElv(long step);

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
        controlOutput getOutputSpeedPosition();
        controlOutput computeOutputSpeedPosition();
        TRACKING_MODE getMode();
        void setMode(TRACKING_MODE mode);
        AccelStepper stepperAzm;
        AccelStepper stepperElv;

        CameraErrorPacket   lastCameraError;
        PositionPacket      lastPosition;
        PointerPacket       lastPointer;

        controlOutput outputSpeedPosition;
        controlOutput outputSpeedError;
    private:
        solverClass azm;
        solverClass elv;
        TRACKING_MODE mode; 
};

double getAngleStepper(double angle1, double angle2);
