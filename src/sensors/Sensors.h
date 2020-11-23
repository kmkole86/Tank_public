#ifndef SENSOR_H
#define SENSOR_H

#include <Adafruit_BMP280.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>

#include "MPU9250.h"
#include "sensors\Sensors.h"

#define TASK_CREATE_ERR 6

#define QUEUE_READ_OK 15
#define QUEUE_CREATE_ERR 5
#define QUEUE_SEND_ERR 16
#define QUEUE_READ_ERR 17

extern Adafruit_BMP280 BMP280_sensor;
extern MPU9250 myIMU;
extern Adafruit_Si7021 Si7021_sensor;
extern l0x_t rangers[7];

QueueHandle_t dataQueue;
TaskHandle_t readSensorTask;

class SensorController {
   public:
    uint8_t init();
    static void readSensorsTask(void *pvParameters);
};

float normalizeFloatTo2Decimals(float value) {
    return (int)(value * 100 + 0.5) / 100.0;
}

// /**sensors reading (since its can take time) is moved to
// *  to another thread (task) thats going to be executed
// *  on ESPs second core
// */
void SensorController::readSensorsTask(void *pvParameters) {
    BaseType_t xStatus;
    SensorsData cachedData;

    for (;;) {
        //l0x
        for (uint8_t i = 0; i < 7; i++) {
            Wire1.beginTransmission(MUX_ADDR);
            Wire1.write(1 << i);
            Wire1.endTransmission();
            vTaskDelay(pdMS_TO_TICKS(50));

            rangers[i].sensor.rangingTest(rangers[i].measurement);
            cachedData.VL53L0X_ranges[i] = rangers[i].measurement->RangeMilliMeter;

            if (cachedData.VL53L0X_ranges[i] > 500)  //max range value over it might consider range error code
                cachedData.VL53L0X_ranges[i] = 500;
        }
        vTaskDelay(pdMS_TO_TICKS(50));

        cachedData.soundIntensity = (uint8_t)((100 * analogRead(SOUND_SENSOR_PIN)) / 4096);

        cachedData.temperature = Si7021_sensor.readTemperature();  // might return NaN, check and correct
        if (!isnan(cachedData.temperature))
            cachedData.temperature = normalizeFloatTo2Decimals(cachedData.temperature);
        else
            cachedData.temperature = 0;

        cachedData.humidity = Si7021_sensor.readHumidity();
        if (!isnan(cachedData.humidity))
            cachedData.humidity = normalizeFloatTo2Decimals(cachedData.humidity);
        else
            cachedData.humidity = 0;

        cachedData.pressure = BMP280_sensor.readPressure() / 100;  //mBar = Pa/100
        cachedData.pressure = (int)(cachedData.pressure);
        vTaskDelay(pdMS_TO_TICKS(50));

        myIMU.update();
        vTaskDelay(pdMS_TO_TICKS(50));

        cachedData.roll = myIMU.getRoll();
        cachedData.roll = normalizeFloatTo2Decimals(cachedData.roll);  //refactor to normalize fun

        cachedData.pitch = myIMU.getPitch();
        cachedData.pitch = normalizeFloatTo2Decimals(cachedData.pitch);

        cachedData.yaw = myIMU.getYaw();
        cachedData.yaw = normalizeFloatTo2Decimals(cachedData.yaw);

        xStatus = xQueueOverwrite(dataQueue, &cachedData);
        if (xStatus != pdPASS) {
            Serial.println(F("ERROR WRITING QUEUE"));
        }
        vTaskDelay(pdMS_TO_TICKS(400));  // by config in RTOS=> delay needed so that other task can get chance to exec
    }
}

// create DATA QUEUE, create and pin TASK for Core_1
uint8_t SensorController::init() {
    // init dataQueue
    dataQueue = xQueueCreate(1, sizeof(struct SensorsData));
    if (dataQueue != NULL)  // proceed to task create only if queue created successfully
    {
        BaseType_t taskCreateStatus = xTaskCreatePinnedToCore(SensorController::readSensorsTask,
                                                              "readSensorsTask",
                                                              10000,
                                                              NULL,
                                                              1,
                                                              &readSensorTask, 0);

        if (taskCreateStatus == pdFAIL)
            return TASK_CREATE_ERR;
    } else
        return QUEUE_CREATE_ERR;  //queue create error

    return STATUS_OK;
}
#endif