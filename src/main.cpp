#include <Arduino.h>
#include <capsule.h>

#include <packetInterface.h>
#include <stepper.h>
#include <config.h>

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic command(handleCommand);

static unsigned long lastCommandReceived = 0;
static float commandFrequency = 1;

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
    case CAPSULE_ID::MOTHER_TO_TRACKER:

    commandPacket lastPosition;
    memcpy(&lastPosition, dataIn, commandPacketSize);

    double timeDelta = millis() - lastCommandReceived;
    commandFrequency = 1000.0 / timeDelta;
    lastCommandReceived = millis();

    break;
  }
}

