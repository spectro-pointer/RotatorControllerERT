#include <stdint.h>

typedef struct __attribute__((__packed__)) {
	float azm;
	float elv;
	int mode;
	float cutoffFreq;
	unsigned maxTimeWindow;
	unsigned timeStamp;
} PacketTrackerCmd;
const uint32_t packetTrackerCmdSize = sizeof(PacketTrackerCmd);