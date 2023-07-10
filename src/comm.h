#include <Arduino.h>
#include <Capsule.h>

void handleCommandPacket(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic commandPacket(handleCommandPacket);