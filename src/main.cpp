#include <Arduino.h>
#include <capsule.h>

#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include <stepper.h>
#include <config.h>

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic command(handleCommand);

static PacketTrackerCmd lastCommand;

void setup() {  
  azmSetup();
  elvSetup();
  CMD_PORT.begin(115200);
}

void loop() {
  // Change direction at the limits
  // if (stepperAzm.distanceToGo() == 0) {
  //   stepperAzm.moveTo(-stepperAzm.currentPosition());
  //   stepperAzm.run();
  // }
  while (CMD_PORT.available()) {
    command.decode(CMD_PORT.read());
  }
}

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch(packetId) {
    case CAPSULE_ID::TRACKER_CMD:
    memcpy(&lastCommand, dataIn, packetTrackerCmdSize);
    break;
  }
}

