#include <Arduino.h>
#include <capsule.h>
#include <Bounce2.h>
#include "stepper.h"
#include "config.h"
#include "filters.h"

Bounce endstopAzm = Bounce();
Bounce endstopElv = Bounce();

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic command(handleCommand);

void loopAutomatic();
void loopManual();

static conClass control;
PacketTrackerCmd lastCmd;

unsigned long lastCmdTime = 0;

double internalAzmNorthEstimate = 0;
double internalElvHorizonEstimate = 0;

SecondOrderEstimator azmEstimator;
SecondOrderEstimator elvEstimator;

SecondOrderLowPassFilter azmFilter(0.1, 100);
SecondOrderLowPassFilter elvFilter(0.1, 100);

void doHoming();
void printPosition();

void setup() {  
  CMD_PORT.begin(CMD_BAUD);
  SERIAL_TO_PC.begin(115200);

  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);

  doHoming();
}

void loop() {
  loopAutomatic();
  printPosition();
}

void loopAutomatic() {

  while (CMD_PORT.available()) {
    command.decode(CMD_PORT.read());
  }

  static TRACKING_MODE lastMode = TRACKING_MODE::STATIONARY;

  if (lastMode != control.getMode()) {
    lastMode = control.getMode();
    if (control.getMode() == TRACKING_MODE::TRACKING_SMOOTH) {
        azmEstimator.reset(lastCmd.azm);
        elvEstimator.reset(lastCmd.elv);
    }
  }

  switch(control.getMode()) {
    case TRACKING_MODE::STATIONARY:
      loopManual();
      azmEstimator.reset();
      elvEstimator.reset();
    break;
    case TRACKING_MODE::TRACKING_SMOOTH:
    {
      static unsigned long lastEstimationTime = millis();
      if ((millis()-lastEstimationTime) >= 1000.0/ESTIMATION_RATE) {
        //int btn = digitalRead(BTN_PIN);

        //double vrx;
        //double vry;

        // if (btn == BTN_PRESSED) {
          double vrx = map(analogRead(VRX_PIN),0,1024,-1000.0,1000.0);
          double vry = map(analogRead(VRY_PIN),0,1024,1000.0,-1000.0);

          if (abs(vrx)<100) { vrx = 0; }
          if (abs(vry)<100) { vry = 0; } 
        // }
        // else {
        //   vrx = 0;
        //   vry = 0;
        // }

        internalAzmNorthEstimate -= (vrx/500.0)/ESTIMATION_RATE;
        internalElvHorizonEstimate -= (vry/500.0)/ESTIMATION_RATE;

        lastEstimationTime = millis();

        double azmEstimation = azmEstimator.computeAngle(millis());
        double elvEstimation = elvEstimator.computeAngle(millis());

        // From now on all angle values are in the referential of the tracker, not north referenced

        double internalAzmValue = ((int(((azmEstimation-internalAzmNorthEstimate)+900.0)*100000.0)%36000000)/100000.0)-180.0;
        double internalElvValue = elvEstimation-internalElvHorizonEstimate;

        double azmFiltered = azmFilter.process(internalAzmValue);
        double elvFiltered = elvFilter.process(internalElvValue);

        // Serial.print(lastCmd.azm);
        // Serial.print(" ");
        // Serial.println(azmEstimation);
        // Serial.print(" ");
        // Serial.println(azmFiltered);
        // Serial.print(" ");
        // Serial.print(internalAzmNorthEstimate);
        // Serial.print(" ");
        // Serial.println(internalAzmValue);

        PacketTrackerCmd newCmd;

        newCmd.azm = azmFiltered;
        newCmd.elv = elvFiltered;

        newCmd.elv = constrain(newCmd.elv, ELV_MIN_ANGLE, ELV_MAX_ANGLE);
        newCmd.azm = constrain(newCmd.azm, AZM_MIN_ANGLE, AZM_MAX_ANGLE);

        control.update(newCmd);
      }
      controlOutput output = control.computeOutput();
      control.stepperAzm.setSpeed(degToStepAzm(output.azmSpeed));
      control.stepperElv.setSpeed(degToStepElv(output.elvSpeed));
      control.stepperAzm.runSpeed();
      control.stepperElv.runSpeed();
    }
    break;
    case TRACKING_MODE::TRACKING_STEP:
      control.stepperAzm.run();
      control.stepperElv.run();
    break;
  }
}

void loopManual() {

  #ifdef AZM_USE_ENDSTOP
    endstopAzm.update();
    endstopElv.update();
  #endif

  double vrx = map(analogRead(VRX_PIN),0,1024,-1000.0,1000.0);
  double vry = map(analogRead(VRY_PIN),0,1024,1000.0,-1000.0);


  if (abs(vrx)<300) { vrx = 0; }
  if (abs(vry)<300) { vry = 0; }

  int btn = digitalRead(BTN_PIN);
  static long lastButtonPressed = 0;

  // if (btn == BTN_PRESSED) {
  //   lastButtonPressed = millis();
  //   #ifndef SCOPE_TRACKER
  //     //control.stepperAzm.setCurrentPosition(0);
  //     //control.stepperElv.setCurrentPosition(0);
  //   #endif
  // }
  // else if ((millis() - lastButtonPressed) > 100) {
    #ifdef CAMERA_TRACKER
      control.stepperAzm.setSpeed(degToStepAzm(vrx/200.0));
      control.stepperElv.setSpeed(degToStepElv(vry/200.0));
    #endif
    #ifdef DUMBO
      if (abs(vrx)<500) {
        vrx = vrx / 100.0;
      }
      if (abs(vry)<500) {
        vry = vry / 100.0;
      }
      control.stepperAzm.setSpeed(degToStepAzm(vrx/50.0));
      control.stepperElv.setSpeed(degToStepElv(vry/50.0));
    #endif
    #ifdef ANTENNA_TRACKER
      control.stepperAzm.setSpeed(degToStepAzm(vrx/50.0));
      control.stepperElv.setSpeed(degToStepElv(vry/50.0));
    #endif
    #ifdef SCOPE_TRACKER
      if (endstopAzm.read() != AZM_ENDSTOP_ACTIVE_LEVEL) {
        control.stepperAzm.setSpeed(degToStepAzm(vrx/200.0));
      }
      else {
        if (vrx > 0) {
          control.stepperAzm.setSpeed(degToStepAzm(vrx/200.0));
        }
        else {
          control.stepperAzm.setSpeed(0);
        }
      }
      if (endstopElv.read() != ELV_ENDSTOP_ACTIVE_LEVEL) {
        control.stepperElv.setSpeed(degToStepElv(vry/200.0));
      }
      else {
        if (vry < 0) {
          control.stepperElv.setSpeed(degToStepElv(vry/200.0));
        }
        else {
          control.stepperElv.setSpeed(0);
        }
      }
    #endif
    control.stepperAzm.runSpeed();
    control.stepperElv.runSpeed();
  // }
}

void printPosition() {
  static unsigned long lastPrint = 0;

  if (millis()-lastPrint>100) {
    lastPrint = millis();
    SERIAL_TO_PC.print(" AZM: ");
    SERIAL_TO_PC.print(stepToDegAzm(control.stepperAzm.currentPosition()));
    SERIAL_TO_PC.print(" ELV: ");
    SERIAL_TO_PC.println(stepToDegElv(control.stepperElv.currentPosition()));
  }
}

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

  // We are not checking the ID anymore. Any packet that is sent to us is considered
  // to be a tracker packet. Because there is only one possible type of packet
  // sent to the tracker. 

  if (len == packetTrackerCmdSize){

    memcpy(&lastCmd, dataIn, packetTrackerCmdSize);

    static PacketTrackerCmd lastCmdPrev;

    if (abs(getAngleStepper(lastCmdPrev.azm, lastCmd.azm))>20.00 or abs(getAngleStepper(lastCmdPrev.elv, lastCmd.elv))>20.00) {
      control.setMode(TRACKING_MODE::STATIONARY);
      lastCmdPrev = lastCmd;
      return;
    }

    lastCmdPrev = lastCmd;
  // switch(packetId) {
  //   case CAPSULE_ID::TRACKER_CMD:
    lastCmdTime = millis();
    control.setMode((TRACKING_MODE)lastCmd.mode);

    switch (lastCmd.mode) {
      case TRACKING_MODE::STATIONARY: 
      break;
      case TRACKING_MODE::TRACKING_SMOOTH:
        azmEstimator.update(lastCmd.azm,millis()-lastCmd.timeStamp);
        elvEstimator.update(lastCmd.elv,millis()-lastCmd.timeStamp);
        azmEstimator.setMaxTimeWindow(lastCmd.maxTimeWindow);
        elvEstimator.setMaxTimeWindow(lastCmd.maxTimeWindow);
        // static unsigned lastTime = lastCmd.timeStamp;
        // Serial.print("TimeStampDiff: ");
        // Serial.println(lastCmd.timeStamp);
        // lastTime = lastCmd.timeStamp;
        azmFilter.setCutoffFreq(lastCmd.cutoffFreq);
        elvFilter.setCutoffFreq(lastCmd.cutoffFreq);
      break;
      case TRACKING_MODE::TRACKING_STEP:
        control.stepperAzm.moveTo((long)degToStepAzm(lastCmd.azm));
        control.stepperElv.moveTo((long)degToStepElv(lastCmd.elv));
      break;
    }
//   break;
// }
  }
}

void doHoming() {
  #ifdef AZM_USE_ENDSTOP
    endstopAzm.attach(AZM_ENDSTOP_PIN, INPUT_PULLUP);
    endstopAzm.interval(50); // interval in ms
  #endif

  #ifdef ELV_USE_ENDSTOP
    endstopElv.attach(ELV_ENDSTOP_PIN, INPUT_PULLUP);
    endstopElv.interval(50); // interval in ms
  #endif

  //endstopElv.attach(ELV_ENDSTOP_PIN, INPUT_PULLUP);
  //endstopElv.interval(50); // interval in ms

  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
    #ifdef AZM_USE_ENDSTOP
      endstopAzm.update();
    #endif
    #ifdef ELV_USE_ENDSTOP
      endstopElv.update();
    #endif
  }

  #ifdef AZM_USE_ENDSTOP
      while (endstopAzm.read() != AZM_ENDSTOP_ACTIVE_LEVEL) {
        endstopAzm.update();
        #ifdef AZM_HOMING_DIRECTION_CW
          control.stepperAzm.setSpeed(degToStepAzm(AZM_HOMING_SPEED));
          control.stepperAzm.runSpeed();
        #endif 
        #ifdef AZM_HOMING_DIRECTION_CCW
          control.stepperAzm.setSpeed(degToStepAzm(-AZM_HOMING_SPEED));
          control.stepperAzm.runSpeed();
        #endif
      }
      control.stepperAzm.setCurrentPosition(degToStepAzm(AZM_ENDSTOP_POSITION));
      control.stepperAzm.moveTo(degToStepAzm(0));
      control.stepperAzm.runToPosition();
  #endif 

  #ifdef ELV_USE_ENDSTOP
      while (endstopElv.read()!= ELV_ENDSTOP_ACTIVE_LEVEL) {
        endstopElv.update();
        Serial.println(endstopElv.read());
        #ifdef ELV_HOMING_DIRECTION_CW
          control.stepperElv.setSpeed(degToStepElv(ELV_HOMING_SPEED));
          control.stepperElv.runSpeed();
        #endif 
        #ifdef ELV_HOMING_DIRECTION_CCW
          control.stepperElv.setSpeed(degToStepElv(-ELV_HOMING_SPEED));
          control.stepperElv.runSpeed();
        #endif
      }
      control.stepperElv.setCurrentPosition(degToStepElv(ELV_ENDSTOP_POSITION));
  #else
      control.stepperElv.setCurrentPosition(0);
  #endif
}
