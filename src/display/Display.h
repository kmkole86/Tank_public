#ifndef DISPLAY_H
#define DISPLAY_H
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

#include "common/HWConstants.h"
#include "models/SensorsData.h"


class Display {
   private:
    const uint8_t DISPLAY_WIDTH = 128;
    const uint8_t DISPLAY_HEIGHT = 64;
    String readErrorMsg = " RE";
    String outOfRange = "OOR";
    Adafruit_SSD1306 display1, display2;
    void clear();
    void show();
    void printRange(uint16_t range);
    void showConnectionStatus(bool connected);
    void showSpeed(uint8_t gear, uint8_t rightMotor, uint8_t leftMotor);
    void showMux();
    void showTemp(float temp);
    void showHumid(float humid);
    void showPressure(uint16_t pressure);
    void showRoll(float roll);
    void showPitch(float pitch);
    void showYaw(float yaw);
    void showRangeFinders(uint16_t ranges[]);
    void showSoundIntensity(uint16_t value);
    void showLightStatus(bool isOn);
    void showHighBeamStatus(bool isOn);
    void showTurnRightStatus(bool isOn);
    void showTurnLeftStatus(bool isOn);

   public:
    Display();
    bool init();
    void showData(bool isConnected, SensorsData* sensorsData, uint8_t gear, uint8_t m1Speed, uint8_t m2Speed, bool isLightOn, bool isHighBeam, bool isTrunSignalLeftOn, bool isTurnSignalRightOn);
    void showBtReadStatus(uint8_t btReadStatus);
    void showParseError();
    void showError(String error);
};
#endif