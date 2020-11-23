#ifndef SENSORS_DATA_H
#define SENSORS_DATA_H
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>

struct SensorsData {
    long timeOfRead;
    float temperature, humidity, roll, pitch, yaw;
    uint16_t VL53L0X_ranges[7], soundIntensity, pressure;
};

typedef struct {
    bool inited;
    Adafruit_VL53L0X sensor;
    int address;
    uint16_t range;         // range value used in continuous mode stuff.
    uint8_t sensor_status;  // status from last ranging in continous.
    VL53L0X_RangingMeasurementData_t* measurement;
} l0x_t;
#endif