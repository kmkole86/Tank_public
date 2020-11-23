#ifndef LIGHTS_H
#define LIGHTS_H
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

#include "models/LightsData.h"

#define LIGHT_PWM_FREQ 1000
#define HEADLIGHT_PWM_CHANN 2
#define TURN_SIGNAL_LEFT_PWM_CHANN 3
#define TURN_SIGNAL_RIGHT_PWM_CHANN 4
#define STOPLIGHT_PWM_CHANN 5
#define DUTY_MAX_VALUE 4096
#define TURN_SIGNAL_LIGHT_INTENSITY 3072
#define STOP_LIGHT_INTENSITY 2048
#define LOW_BEAM_LIGHT_INTENSITY 2048
#define HIGH_BEAM_LIGHT_INTENSITY 3072

class LightsController {
   private:
    long updateTime;
    LightsData lightsData;
    bool isLightOn, isLeftTurnSignalOn, isRightTurnSignalOn, isAll4On, blinkStatus;
    uint16_t headLightIntensity = LOW_BEAM_LIGHT_INTENSITY;
    uint16_t stopLightIntensity = STOP_LIGHT_INTENSITY;

    Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver();

   public:
    void init();

    bool getHighBeamStatus();
    void changeHighBeamStatus();
    void changeLightsStatus();
    bool getLightsStatus();

    uint16_t getStopLightsIntensity();
    void setBrakeLightOn(bool value);
    bool getBrakeLightsStatus();

    void changeLeftTurnSignalState();
    bool getLeftTurnSignalStatus();

    void changeRightTurnSignalState();
    bool getRightTurnSignalStatus();

    void changeAll4Status();
    bool getAll4Status();

    void update(long timeNow);

   private:
    void updateHeadLightsIntensity();
    void turnLeftBlinkerOn();
    void turnLeftBlinkerOff();
    void turnRightBlinkerOn();
    void turnRightBlinkerOff();
    void updateStopLightsIntensity(uint16_t value);
};

void LightsController::changeHighBeamStatus() {
    if (headLightIntensity == 3072)
        headLightIntensity = 2048;
    else {
        headLightIntensity = 3072;
    }
    updateHeadLightsIntensity();
}
void LightsController::changeAll4Status() {
    isAll4On = !isAll4On;

    if (isAll4On) {
        isLeftTurnSignalOn = true;
        turnLeftBlinkerOn();
        isRightTurnSignalOn = true;
        turnRightBlinkerOn();
    } else {
        isLeftTurnSignalOn = false;
        turnLeftBlinkerOff();
        isRightTurnSignalOn = false;
        turnRightBlinkerOff();
    }
}
bool LightsController::getAll4Status() {
    return isAll4On;
}

void LightsController::init() {
    pwmDriver.begin();
    pwmDriver.setPWMFreq(1000);
}

void LightsController::changeLightsStatus() {
    isLightOn = !isLightOn;
    if (isLightOn) {
        pwmDriver.setPWM(HEADLIGHT_PWM_CHANN, 0, DUTY_MAX_VALUE < headLightIntensity ? DUTY_MAX_VALUE : headLightIntensity);
        pwmDriver.setPWM(STOPLIGHT_PWM_CHANN, 0, STOP_LIGHT_INTENSITY);
    } else {
        pwmDriver.setPWM(HEADLIGHT_PWM_CHANN, 0, DUTY_MAX_VALUE);
        pwmDriver.setPWM(STOPLIGHT_PWM_CHANN, 0, DUTY_MAX_VALUE);
    }
}

void LightsController::updateHeadLightsIntensity() {
    if (isLightOn)
        pwmDriver.setPWM(HEADLIGHT_PWM_CHANN, 0, DUTY_MAX_VALUE < headLightIntensity ? DUTY_MAX_VALUE : headLightIntensity);
}

void LightsController::setBrakeLightOn(bool value) {
    if (value)
        pwmDriver.setPWM(STOPLIGHT_PWM_CHANN, DUTY_MAX_VALUE, 0);
    else {
        if (isLightOn)
            pwmDriver.setPWM(STOPLIGHT_PWM_CHANN, STOP_LIGHT_INTENSITY, DUTY_MAX_VALUE - STOP_LIGHT_INTENSITY);
        else
            pwmDriver.setPWM(STOPLIGHT_PWM_CHANN, 0, DUTY_MAX_VALUE);
    }
}

void LightsController::updateStopLightsIntensity(uint16_t value) {
    stopLightIntensity = value;
    if (isLightOn)
        pwmDriver.setPWM(STOPLIGHT_PWM_CHANN, 0, DUTY_MAX_VALUE < stopLightIntensity ? DUTY_MAX_VALUE : stopLightIntensity);
}

void LightsController::turnLeftBlinkerOn() {
    pwmDriver.setPWM(TURN_SIGNAL_LEFT_PWM_CHANN, 0, TURN_SIGNAL_LIGHT_INTENSITY);
}

void LightsController::turnLeftBlinkerOff() {
    pwmDriver.setPWM(TURN_SIGNAL_LEFT_PWM_CHANN, 0, DUTY_MAX_VALUE);
}

void LightsController::turnRightBlinkerOn() {
    pwmDriver.setPWM(TURN_SIGNAL_RIGHT_PWM_CHANN, 0, TURN_SIGNAL_LIGHT_INTENSITY);
}

void LightsController::turnRightBlinkerOff() {
    pwmDriver.setPWM(TURN_SIGNAL_RIGHT_PWM_CHANN, 0, DUTY_MAX_VALUE);
}

void LightsController::changeRightTurnSignalState() {
    if (isLeftTurnSignalOn) {
        isLeftTurnSignalOn = false;
        turnLeftBlinkerOff();
    }
    isRightTurnSignalOn = !isRightTurnSignalOn;
    if (isRightTurnSignalOn)
        turnRightBlinkerOn();
    else
        turnRightBlinkerOff();
}

void LightsController::changeLeftTurnSignalState() {
    if (isRightTurnSignalOn) {
        isRightTurnSignalOn = false;
        turnRightBlinkerOff();
    }
    isLeftTurnSignalOn = !isLeftTurnSignalOn;
    if (isLeftTurnSignalOn)
        turnLeftBlinkerOn();
    else
        turnLeftBlinkerOff();
}

void LightsController::update(long timeNow) {
    if ((isRightTurnSignalOn || isLeftTurnSignalOn) && (timeNow > updateTime)) {
        blinkStatus = !blinkStatus;
        if (isRightTurnSignalOn) {
            if (blinkStatus)
                turnRightBlinkerOn();
            else
                turnRightBlinkerOff();
        }

        if (isLeftTurnSignalOn) {
            if (blinkStatus)
                turnLeftBlinkerOn();
            else
                turnLeftBlinkerOff();
        }

        updateTime = timeNow + 500;
    }
}

bool LightsController::getLightsStatus() { return isLightOn; }
bool LightsController::getHighBeamStatus() { return headLightIntensity == HIGH_BEAM_LIGHT_INTENSITY; }
bool LightsController::getRightTurnSignalStatus() { return isRightTurnSignalOn; }
bool LightsController::getLeftTurnSignalStatus() { return isLeftTurnSignalOn; }
#endif