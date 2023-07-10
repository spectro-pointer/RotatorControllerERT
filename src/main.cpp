#include <Arduino.h>
#include <capsule.h>

#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include <stepper.h>
#include <config.h>

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic command(handleCommand);

static conClass control;

void setup() {  
  CMD_PORT.begin(115200);
}

void loop() {
  while (CMD_PORT.available()) {
    command.decode(CMD_PORT.read());
  }
  controlOutput output = control.computeOutput();
  control.stepperAzm.setSpeed(output.azmSpeed);
  control.stepperElv.setSpeed(output.elvSpeed);
  control.stepperAzm.runSpeed();
  control.stepperElv.runSpeed();
}

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch(packetId) {
    case CAPSULE_ID::TRACKER_CMD:
    PacketTrackerCmd lastCommand;
    memcpy(&lastCommand, dataIn, packetTrackerCmdSize);
    control.update(lastCommand);
    break;
  }
}
