#ifndef LIGHTS_DATA_H
#define LIGHTS_DATA_H
#include <Arduino.h>

struct LightsData {
    bool lightStatus, turnRightSignalStatus, turnLeftSignalStatus;
    uint16_t headLightIntensity;
};
#endif