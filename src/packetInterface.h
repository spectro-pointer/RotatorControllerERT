#include <Arduino.h>

enum CAPSULE_ID {
    AIR_TO_GROUND = 0x00,
    GROUND_TO_AIR,
    MOTHER_TO_DEVICE,
    DEVICE_TO_MOTHER,
    PC_TO_MOTHER,
    MOTHER_TO_TRACKER,
    TRACKER_TO_MOTHER
};

struct commandPacket {
  float  azimuth;
  float  elevation;
};
const uint32_t commandPacketSize = sizeof(commandPacket);