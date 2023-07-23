#include <Arduino.h>
#include <capsule.h>

#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include <stepper.h>
#include <config.h>

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic command(handleCommand);

void loopAutomatic();
void loopManual();

static conClass control;
PacketTrackerCmd lastCmd;

unsigned long lastCmdTime = 0;

float binocAzmOffset = 0;
float binocElvOffset = 0;

enum GLOBAL_MODE {
  MANUAL,
  AUTO
};

GLOBAL_MODE globalMode;

void setup() {  
  CMD_PORT.begin(CMD_BAUD);
  SERIAL_TO_PC.begin(115200);
  globalMode = GLOBAL_MODE::MANUAL;

  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);
  pinMode(BTN_PIN, INPUT);

}

void loop() {
  // switch (globalMode) {
  //   case GLOBAL_MODE::AUTO:
      loopAutomatic();
  //   break;
  //   case GLOBAL_MODE::MANUAL:
  //     loopManual();
  //   break;
  // }
}

void loopAutomatic() {
  // while(SERIAL_TO_PC.available()) {
  //   char c = SERIAL_TO_PC.read();
  //   if (c == 'm') {
  //     globalMode = GLOBAL_MODE::MANUAL;
  //     SERIAL_TO_PC.println("Switching to manual mode");
  //   }
  // }

  while (CMD_PORT.available()) {
    command.decode(CMD_PORT.read());
  }

  if (millis()-lastCmdTime > 10*(1000.0/BINOC_DATA_RATE)) {
    control.setMode(TRACKING_MODE::STATIONARY);
  }

  switch(control.getMode()) {
    case TRACKING_MODE::STATIONARY:
      //control.stepperAzm.stop();
      //control.stepperElv.stop();
      //control.stepperAzm.run();
      //control.stepperElv.run();
      //Serial.println("Stationary");
      loopManual();
    break;
    case TRACKING_MODE::TRACKING_FAST:
    {
      controlOutput output = control.computeOutput();
      control.stepperAzm.setSpeed(degToStepAzm(output.azmSpeed));
      control.stepperElv.setSpeed(degToStepElv(output.elvSpeed));
      control.stepperAzm.runSpeed();
      control.stepperElv.runSpeed();
    }
    break;
    case TRACKING_MODE::TRACKING_SLOW:
      //Serial.println("Telem");
      control.stepperAzm.run();
      control.stepperElv.run();
    break;
  }
}

void loopManual() {

  double vrx = map(analogRead(VRX_PIN),0,600,-1000.0,1000.0);
  double vry = map(analogRead(VRY_PIN),0,600,1000.0,-1000.0);

  if (abs(vrx)<100) { vrx = 0; }
  if (abs(vry)<100) { vry = 0; }

  int btn = digitalRead(BTN_PIN);

  static long lastButtonPressed = 0;

  if (btn == BTN_PRESSED) {
    lastButtonPressed = millis();
    control.stepperAzm.setCurrentPosition(0);
    control.stepperElv.setCurrentPosition(0);
  }
  else if ((millis() - lastButtonPressed) > 100) {
    control.stepperAzm.setSpeed(degToStepAzm(vrx/50.0));
    control.stepperElv.setSpeed(degToStepElv(vry/50.0));
    control.stepperAzm.runSpeed();
    control.stepperElv.runSpeed();
  }

  // while (CMD_PORT.available()) {
  //   command.decode(CMD_PORT.read());
  // }

  // while(SERIAL_TO_PC.available()) {
  //   char c = SERIAL_TO_PC.read();
  //   if (c == 'a') {
  //     globalMode = GLOBAL_MODE::AUTO;
  //     SERIAL_TO_PC.println("Switching to auto mode");
  //   }
  // }
}

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

  memcpy(&lastCmd, dataIn, packetTrackerCmdSize);

  // Serial.print("Received command: ");
  // Serial.print(lastCmd.mode);
  // Serial.print(" ");
  // Serial.print(lastCmd.azm);
  // Serial.print(" ");
  // Serial.println(lastCmd.elv);

  // Serial.print(lastCmd.azm);
  // Serial.print(" ");
  // Serial.println(stepToDegAzm(control.stepperAzm.currentPosition()));

  switch(packetId) {
    case CAPSULE_ID::TRACKER_CMD:
      lastCmdTime = millis();
      control.setMode((TRACKING_MODE)lastCmd.mode);

      switch (lastCmd.mode) {
        case TRACKING_MODE::STATIONARY: 
          // SERIAL_TO_PC.print("Received command to go stationary, speed is:");
          // SERIAL_TO_PC.println(control.stepperAzm.speed());
          // SERIAL_TO_PC.println("");
          // SERIAL_TO_PC.println("");
          // SERIAL_TO_PC.println("");
          // SERIAL_TO_PC.println("");
          // SERIAL_TO_PC.println("");
        break;
        case TRACKING_MODE::TRACKING_FAST:
          lastCmd.elv = constrain(lastCmd.elv, ELV_MIN_ANGLE, ELV_MAX_ANGLE);
          control.update(lastCmd);
          // SERIAL_TO_PC.print("Received command to go fast, speed is: ");
          // SERIAL_TO_PC.println(control.stepperAzm.speed());
        break;
        case TRACKING_MODE::TRACKING_SLOW:
          control.stepperAzm.moveTo((long)degToStepAzm(lastCmd.azm));
          control.stepperElv.moveTo((long)degToStepElv(lastCmd.elv));
          // SERIAL_TO_PC.print("Received command to go slow, speed is: ");
          // SERIAL_TO_PC.println(control.stepperAzm.speed());
        break;
      }
    break;

  }
}
