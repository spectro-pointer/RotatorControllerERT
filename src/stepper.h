#include <AccelStepper.h>
#include <config.h>
#include "comm.h"

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

enum TRACKING_MODE {
    TRACKING_ERROR = 1,
    TRACKING_POSITION,
    TRACKING_MANUAL,
    TRACKING_POINTER,
    TRACKING_STOP
};

struct ControlParameter {
  double kp;
  double kd;
  double freq;
  double maxTime;
};

struct AxisParemeter {
  double  maxAcc;
  double  maxSpe;
  double  stepRev;
  double  gearBox;
  double  limExist;
  double  limMin;
  double  limMax;
  double  dir;
  double  back;
};

struct SystemParemter {
  // IPAddress ip;
  double    port;
};

struct TrackerParameter {
  AxisParemeter     azmParameter;
  AxisParemeter     elvParameter;
  SystemParemter    ethernetParameter;
  ControlParameter  control;
  TRACKING_MODE mode;
};

struct CameraErrorPacket {
  char name[4];
  int32_t isVisible;
  int32_t Cx;
  int32_t Cy;
  int32_t age;
  int32_t camID;
  float Kp;
  float maxSpeed;
}; 

struct AnglePacket {
  float azm;
  float elv;
};

struct PositionPacket {
  double lat;
  double lon;
  double alt;
};

struct PointerPacket {
  double azm;
  double elv;
  bool isPressed;
};

struct ManualPacket {
  float joystickX;
  float joystickY;
  bool joystickButton;
  bool switchUp;
  bool switchDown;
  bool switchLeft;
  bool switchRight;
};

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
        void setGroundPosition(PositionPacket positionIn);
        void setMode(TRACKING_MODE mode);
        AccelStepper stepperAzm;
        AccelStepper stepperElv;

        CameraErrorPacket   lastCameraError;
        CameraErrorPacket   lastJoystickError;
        PositionPacket      lastPosition;
        PositionPacket      groundPosition;
        AnglePacket         lastAngle;
        PointerPacket       lastPointer;
        ManualPacket        lastManual;

        unsigned long lastCameraErrorTime;
        unsigned long lastPositionTime;
        unsigned long lastAngleTime;
        unsigned long lastPointerTime;
        unsigned long lastManualTime;

        controlOutput outputSpeedPosition;
        controlOutput outputSpeedError;

        unsigned long maxSpeedAzm;
        unsigned long maxSpeedElv;

        unsigned long maxAccelAzm;
        unsigned long maxAccelElv;

        unsigned azmSpr; // Consider microstep
        unsigned elvSpr; // Consider microstep

        double azmGear; // Gear ratio between stepper and tracker head
        double elvGear; // Gear ratio between stepper and tracker head

        long degToStepAzm(double deg);
        long degToStepElv(double deg);

        double stepToDegAzm(long step);
        double stepToDegElv(long step);
    private:
        solverClass azm;
        solverClass elv;
        TRACKING_MODE mode; 
};

double getAngleStepper(double angle1, double angle2);
