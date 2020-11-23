#include <Adafruit_BMP280.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>
#include <Wire.h>

#include "MPU9250.h"
#include "buzzer\Buzzer.h"
#include "common\Constants.h"
#include "common\HWConstants.h"
#include "display\Display.h"
#include "engine\Engine.h"
#include "light\Lights.h"
#include "sensors\Sensors.h"

#define AHRS false  // Set to false for basic data read
#define SerialDebug true
size_t sizeOfJson;
Adafruit_BMP280 BMP280_sensor(&Wire1);
MPU9250 myIMU;
Adafruit_Si7021 Si7021_sensor = Adafruit_Si7021(&Wire1);
l0x_t rangers[7] = {
    {false, Adafruit_VL53L0X(), 0x51, 0, 0, new VL53L0X_RangingMeasurementData_t()},
    {false, Adafruit_VL53L0X(), 0x52, 0, 0, new VL53L0X_RangingMeasurementData_t()},
    {false, Adafruit_VL53L0X(), 0x53, 0, 0, new VL53L0X_RangingMeasurementData_t()},
    {false, Adafruit_VL53L0X(), 0x54, 0, 0, new VL53L0X_RangingMeasurementData_t()},
    {false, Adafruit_VL53L0X(), 0x55, 0, 0, new VL53L0X_RangingMeasurementData_t()},
    {false, Adafruit_VL53L0X(), 0x56, 0, 0, new VL53L0X_RangingMeasurementData_t()},
    {false, Adafruit_VL53L0X(), 0x57, 0, 0, new VL53L0X_RangingMeasurementData_t()}};

Engine engine;
Buzzer buzzer;
Display display;
LightsController lightsController;
SensorController sensorController;
SensorsData data;

long tickTime = 0;
BluetoothSerial SerialBT;
StaticJsonDocument<2048> sendJSONBuffer;  // refactor, calculate real size
char sendBuffer[400] = {'\0'};            // refactor, calculate real size

uint16_t charIndex = 0;
bool isDataReceived = false;
bool isReceiveInProgress = false;
char receiveBuffer[512] = {'\0'};            // refactor, calculate real size
StaticJsonDocument<2048> receiveJSONBuffer;  // refactor, calculate real size

void populateJsonData(const long id, const LightsData *lightsData, const uint8_t gear, const float rightMotorSpeed, const float leftMotorSpeed) {
    sendJSONBuffer["id"] = id;
    sendJSONBuffer["time"] = millis();

    sendJSONBuffer["data"]["hl"] = lightsData->headLightIntensity;
    sendJSONBuffer["data"]["tsr"] = lightsData->turnRightSignalStatus;
    sendJSONBuffer["data"]["tsl"] = lightsData->turnLeftSignalStatus;
    sendJSONBuffer["data"]["gear"] = gear;
    sendJSONBuffer["data"]["rms"] = rightMotorSpeed;
    sendJSONBuffer["data"]["lms"] = leftMotorSpeed;
    sendJSONBuffer["data"]["t"] = data.temperature;
    sendJSONBuffer["data"]["h"] = data.humidity;
    sendJSONBuffer["data"]["p"] = data.pressure;
    sendJSONBuffer["data"]["s"] = data.soundIntensity;

    sendJSONBuffer["data"]["roll"] = data.roll;
    sendJSONBuffer["data"]["pitch"] = data.pitch;
    sendJSONBuffer["data"]["yaw"] = data.yaw;

    sendJSONBuffer["data"]["radar"].add(data.VL53L0X_ranges[0]);
    sendJSONBuffer["data"]["radar"].add(data.VL53L0X_ranges[1]);
    sendJSONBuffer["data"]["radar"].add(data.VL53L0X_ranges[2]);
    sendJSONBuffer["data"]["radar"].add(data.VL53L0X_ranges[3]);
    sendJSONBuffer["data"]["radar"].add(data.VL53L0X_ranges[4]);
    sendJSONBuffer["data"]["radar"].add(data.VL53L0X_ranges[5]);
    sendJSONBuffer["data"]["radar"].add(data.VL53L0X_ranges[6]);
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_OPEN_EVT:
            //Serial.println("ESP_SPP_OPEN_EVT");
            break;
        case ESP_SPP_CLOSE_EVT:
            //Serial.println("ESP_SPP_CLOSE_EVT");
            break;
        case ESP_SPP_START_EVT:
            //Serial.println("ESP_SPP_START_EVT");
            break;
        case ESP_SPP_CL_INIT_EVT:
            //Serial.println("ESP_SPP_CL_INIT_EVT \n");
            break;
        case ESP_SPP_CONG_EVT:
            //Serial.println("ESP_SPP_CONG_EVT cong");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            //Serial.println("ESP_SPP_SRV_OPEN_EVT");
            break;
        default:
            //Serial.println("ESP_SPP_EVENT OTHER");
            break;
    }
}

void initPins() {
    pinMode(BT_CONN_PIN, INPUT);  //BT connect button
    pinMode(SOUND_SENSOR_PIN, INPUT);
    pinMode(LIGHT_SENSOR_PIN, INPUT);  //yet to be added to board
}

bool initI2C() {
    return Wire.begin(SDA_W, SCL_W, 400000) & Wire1.begin(SDA_W1, SCL_W1, 400000);
}

bool initDisplay() {
    return display.init();
}

bool initBMP280() {
    bool inited = BMP280_sensor.begin();
    if (inited)
        BMP280_sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,
                                  Adafruit_BMP280::SAMPLING_X2,
                                  Adafruit_BMP280::SAMPLING_X16,
                                  Adafruit_BMP280::FILTER_X16,
                                  Adafruit_BMP280::STANDBY_MS_500);
    return inited;
}

bool initMUX() {
    Wire1.beginTransmission(MUX_ADDR);
    return Wire1.endTransmission() == 0;
}

bool initL0xWithRetry() {
    bool allInited;
    uint8_t retryNum = 0;

    do {
        allInited = true;
        for (int i = 0; i < 7 && !rangers[i].inited; i++) {
            Wire1.beginTransmission(MUX_ADDR);
            Wire1.write(1 << i);
            if (Wire1.endTransmission() == 0)
                rangers[i].inited = rangers[i].sensor.begin(rangers[i].address, false, &Wire1, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);

            allInited &= rangers[i].inited;
        }
        retryNum++;
    } while (retryNum < 3 && !allInited);
    return allInited;
}

bool initSi7021() {
    return Si7021_sensor.begin();
}

bool initIMU9250() {
    bool inited = true;  //refactor error checking
    myIMU.setup(Wire1);
    delay(7000);
    myIMU.calibrateAccelGyro();
    myIMU.calibrateMag();
    return inited;
}

bool initBT() {
    SerialBT.register_callback(callback);
    return SerialBT.begin("ESP42");
}

void initialise() {
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.print("|--------------->>>");
    Serial.println(millis());
    initPins();
    buzzer.init();
    if (!initI2C()) {
        Serial.println("wire err.");
        buzzer.buzzErrBlocking();
    } else if (!initDisplay()) {
        Serial.println("display err.");
        buzzer.buzzErrBlocking();
    } else if (!initSi7021()) {
        Serial.println("si7021 err.");
        display.showError("si7021 err.");
        buzzer.buzzErrBlocking();
    } else if (!initBMP280()) {
        Serial.println("bmp280 err.");
        display.showError("bmp280 err.");
        buzzer.buzzErrBlocking();
    } else if (!initMUX()) {
        Serial.println("MUX err.");
        display.showError("MUX err.");
        buzzer.buzzErrBlocking();
    } else if (!initL0xWithRetry()) {
        Serial.println("l0x err.");
        display.showError("l0x err.");
        buzzer.buzzErrBlocking();
    } else if (!initIMU9250()) {
        Serial.println("IMU err.");
        display.showError("IMU err.");
        buzzer.buzzErrBlocking();
    } else if (sensorController.init() != STATUS_OK) {
        Serial.println("sctrl err.");
        display.showError("sctrl err.");
        buzzer.buzzErrBlocking();
    } else if (!initBT()) {
        Serial.println("btinit err.");
        display.showError("btinit err.");
        buzzer.buzzErrBlocking();
    } else {
        JsonObject data = sendJSONBuffer.createNestedObject("data");
        data.createNestedArray("radar");
        sendJSONBuffer["data"]["radar"].add(0);
        sendJSONBuffer["data"]["radar"].add(0);
        sendJSONBuffer["data"]["radar"].add(0);
        sendJSONBuffer["data"]["radar"].add(0);
        sendJSONBuffer["data"]["radar"].add(0);
        sendJSONBuffer["data"]["radar"].add(0);
        sendJSONBuffer["data"]["radar"].add(0);
        lightsController.init();
        engine.init();

        Serial.print("<<<---------------|");
        Serial.println(millis());
        buzzer.buzzOK();  //if you reach this, everything is OK
    }
}

void runI2CScanner() {
    Serial.println("Scanning W");
    byte error, address;
    int nDevices;
    nDevices = 0;

    for (address = 1; address < 127; address++) {
        delay(50);
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("W 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("W err 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    Serial.println("Scanning W1");
    for (address = 1; address < 127; address++) {
        delay(50);
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();

        if (error == 0) {
            Serial.print("W1 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("W1 err 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    Serial.println("Scan end");
}

void update(long timeNow) {
    lightsController.update(timeNow);
    engine.update(timeNow);
}

void handleCommand() {
    JsonVariant lights = receiveJSONBuffer["hl"];
    if (!lights.isNull())
        lightsController.changeLightsStatus();

    JsonVariant highLowBeam = receiveJSONBuffer["hlhb"];
    if (!highLowBeam.isNull())
        lightsController.changeHighBeamStatus();

    JsonVariant gearUp = receiveJSONBuffer["gru"];
    if (!gearUp.isNull() && engine.getGear() < 2) {
        engine.setGear(engine.getGear() + 1);
    }

    JsonVariant gearDown = receiveJSONBuffer["grd"];
    if (!gearDown.isNull()) {
        if (engine.getGear() > 0)
            engine.setGear(engine.getGear() - 1);
    }

    JsonVariant turnSignalLeft = receiveJSONBuffer["tsl"];
    if (!turnSignalLeft.isNull()) {
        Serial.print("tsL: ");
        Serial.println(turnSignalLeft.as<bool>());
        lightsController.changeLeftTurnSignalState();
    }

    JsonVariant turnSignalRight = receiveJSONBuffer["tsr"];
    if (!turnSignalRight.isNull()) {
        Serial.print("tsR: ");
        Serial.println(turnSignalRight.as<bool>());
        lightsController.changeRightTurnSignalState();
    }

    JsonVariant all4 = receiveJSONBuffer["a4"];
    if (!all4.isNull()) {
        Serial.print("all4: ");
        Serial.println(all4.as<bool>());
        lightsController.changeAll4Status();
    }

    JsonVariant motorLeft = receiveJSONBuffer["lm"];
    JsonVariant motorRight = receiveJSONBuffer["rm"];
    if (!motorLeft.isNull() && !motorRight.isNull())
        engine.setSpeed(motorLeft.as<float>(), motorRight.as<float>());
}

void setup() {
    initialise();  //cant be init()
    lightsController.changeLightsStatus();
    tickTime = millis() + 3000;
}

void receiveData() {
    while (SerialBT.available() > 0 && isDataReceived == false) {
        char receivedChar = SerialBT.read();

        if (receivedChar == COMMAND_START) {
            charIndex = 0;
            isReceiveInProgress = true;
        } else if (isReceiveInProgress == true) {
            if (receivedChar != COMMAND_END) {
                receiveBuffer[charIndex++] = receivedChar;
            } else {
                isReceiveInProgress = false;
                int endIndex = charIndex + 1;
                receiveBuffer[endIndex] = '\0';
                Serial.println("received buff");
                Serial.println(receiveBuffer);
                DeserializationError err = deserializeJson(receiveJSONBuffer, receiveBuffer);
                if (err) {
                    Serial.println("de_ERR ");
                    Serial.println(err.c_str());
                }
                isDataReceived = true;
            }
        }
    }
}

void loop() {
    if (isDataReceived) {
        handleCommand();
        isDataReceived = false;
    }
    receiveData();
    update(millis());
    if (millis() > tickTime) {
        Serial.print("tick ");
        tickTime = millis() + 1000;
        Serial.println(tickTime);
    }

    BaseType_t xStatus = xQueueReceive(dataQueue, &data, 0);
    if (xStatus == pdPASS) {
        sendJSONBuffer["time"] = millis();

        sendJSONBuffer["data"]["hl"] = lightsController.getLightsStatus();
        sendJSONBuffer["data"]["hlhb"] = lightsController.getHighBeamStatus();
        sendJSONBuffer["data"]["tsr"] = lightsController.getRightTurnSignalStatus();
        sendJSONBuffer["data"]["tsl"] = lightsController.getLeftTurnSignalStatus();
        sendJSONBuffer["data"]["a4"] = lightsController.getAll4Status();
        sendJSONBuffer["data"]["g"] = engine.getGear();
        sendJSONBuffer["data"]["rms"] = engine.getRightMotorSpeed();
        sendJSONBuffer["data"]["lms"] = engine.getLeftMotorSpeed();
        if (!isnan(data.temperature))
            sendJSONBuffer["data"]["t"] = (int)(data.temperature * 100 + 0.5) / 100.0;
        else
            sendJSONBuffer["data"]["t"] = 0;

        sendJSONBuffer["data"]["h"] = data.humidity;
        sendJSONBuffer["data"]["p"] = data.pressure;
        sendJSONBuffer["data"]["s"] = data.soundIntensity;

        sendJSONBuffer["data"]["roll"] = data.roll;
        sendJSONBuffer["data"]["pitch"] = data.pitch;
        sendJSONBuffer["data"]["yaw"] = data.yaw;

        for (size_t i = 0; i < 7; i++)
            sendJSONBuffer["data"]["radar"][i] = data.VL53L0X_ranges[i];

        sizeOfJson = measureJson(sendJSONBuffer);
        serializeJson(sendJSONBuffer, sendBuffer);

        SerialBT.write('<');
        SerialBT.write((uint8_t *)sendBuffer, sizeOfJson);
        SerialBT.write('|');
        SerialBT.write('m');
        SerialBT.write('d');
        SerialBT.write('5');
        SerialBT.write('>');

        display.showData(true, &data, engine.getGear(), engine.getRightMotorSpeed(), engine.getLeftMotorSpeed(), lightsController.getLightsStatus(), lightsController.getHighBeamStatus(), lightsController.getLeftTurnSignalStatus(), lightsController.getRightTurnSignalStatus());
    }
}
