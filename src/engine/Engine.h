#ifndef ENGINE_H
#define ENGINE_H
#include <Arduino.h>

#define MOTOR_R_DIR_1_PIN 27
#define MOTOR_R_DIR_2_PIN 25
#define MOTOR_R_PWR_PIN 16
#define MOTOR_R_PWM_CHANN 1

#define MOTOR_L_DIR_1_PIN 26
#define MOTOR_L_DIR_2_PIN 4
#define MOTOR_L_PWR_PIN 17
#define MOTOR_L_PWM_CHANN 2

#define MOTOR_PWM_FREQ 3000
#define MOTOR_PWM_RESOLUTION 8
#define MOTOR_MAX_SPEED 254
#define MOTOR_MIN_SPEED -254

#define LOW_TRANSMISSION_RATIO 192
#define HIGH_TRANSMISSION_RATIO 254
#define NEUTRAL_TRANSMISSION_RATIO 254

#define MOVE_COMMAND_VALID_TIME 500

class Engine {
   private:
    uint8_t gear;
    uint8_t transmission;
    int16_t rightMotorSpeed, leftMotorSpeed;
    long moveCommandExpirationTime;
    void setRightMotorDirectionForward();
    void setRightMotorDirectionBackward();
    void setLeftMotorDirectionForward();
    void setLeftMotorDirectionBackward();
    void setBothMothorsDirectionNeutral();

   public:
    void init();
    void setGear(uint8_t value);
    uint8_t getGear();
    uint8_t getRightMotorSpeed();
    uint8_t getLeftMotorSpeed();
    void setSpeed(const float value1, const float value2);
    void update(const long timeNow);
    bool isMoving();
    String getGearLiteral();
};

String Engine::getGearLiteral() {
    switch (gear) {
        case 0:
            return "N";
            break;
        case 1:
            return "L";
            break;
        case 2:
            return "H";
            break;
        default:
            return "-";
    }
}

void Engine::update(long timeNow) {
    if (isMoving() && timeNow > moveCommandExpirationTime)  // move comand last certain period
        setSpeed(0, 0);
}

bool Engine::isMoving() {
    return getLeftMotorSpeed() != 0 || getRightMotorSpeed() != 0;
}

void Engine::init() {
    // setup motor right
    ledcSetup(MOTOR_R_PWM_CHANN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(MOTOR_R_PWR_PIN, MOTOR_R_PWM_CHANN);
    ledcWrite(MOTOR_R_PWM_CHANN, 0);
    pinMode(MOTOR_R_DIR_1_PIN, OUTPUT);
    pinMode(MOTOR_R_DIR_2_PIN, OUTPUT);

    // setup motor left
    ledcSetup(MOTOR_L_PWM_CHANN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(MOTOR_L_PWR_PIN, MOTOR_L_PWM_CHANN);
    ledcWrite(MOTOR_L_PWM_CHANN, 0);
    pinMode(MOTOR_L_DIR_1_PIN, OUTPUT);
    pinMode(MOTOR_L_DIR_2_PIN, OUTPUT);

    setGear(0);
}

void Engine::setGear(uint8_t value) {
    // Pin1 L Pin2 H backward, Pin1 H Pin2 L forward

    this->gear = value;
    switch (value) {
        case 1:
            setSpeed(0, 0);
            transmission = LOW_TRANSMISSION_RATIO;
            setLeftMotorDirectionForward();
            setRightMotorDirectionForward();
            break;
        case 2:
            setSpeed(0, 0);
            transmission = HIGH_TRANSMISSION_RATIO;
            setLeftMotorDirectionForward();
            setRightMotorDirectionForward();
            break;
        case 0:
            transmission = NEUTRAL_TRANSMISSION_RATIO;
            setSpeed(0, 0);
            setBothMothorsDirectionNeutral();
            break;
        default:
            transmission = NEUTRAL_TRANSMISSION_RATIO;
            this->gear = 0;
            setBothMothorsDirectionNeutral();
    }
}

uint8_t Engine::getGear() {
    return gear;
}

void Engine::setSpeed(const float lMotorSpeed, const float rMotorSpeed) {
    //is motor direction change required
    if (std::signbit(lMotorSpeed) != std::signbit(leftMotorSpeed)) {
        if (lMotorSpeed >= 0) {
            setLeftMotorDirectionForward();
            Serial.print(" left Fwd ");
        } else {
            setLeftMotorDirectionBackward();
            Serial.print(" left Bck ");
        }
    }

    if (std::signbit(rMotorSpeed) != std::signbit(rightMotorSpeed)) {
        if (rMotorSpeed >= 0) {
            Serial.print(" right Fwd ");
            setRightMotorDirectionForward();
        } else {
            Serial.print(" right Bck ");
            setRightMotorDirectionBackward();
        }
    }

    rightMotorSpeed = static_cast<int16_t>(rMotorSpeed * transmission);
    leftMotorSpeed = static_cast<int16_t>(lMotorSpeed * transmission);

    rightMotorSpeed = constrain(rightMotorSpeed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    leftMotorSpeed = constrain(leftMotorSpeed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);

    ledcWrite(MOTOR_R_PWM_CHANN, abs(rightMotorSpeed));
    ledcWrite(MOTOR_L_PWM_CHANN, abs(leftMotorSpeed));
    if (rightMotorSpeed != 0 || leftMotorSpeed != 0)
        moveCommandExpirationTime = millis() + MOVE_COMMAND_VALID_TIME;  //move command is valid for 300ms
}

uint8_t Engine::getRightMotorSpeed() { return rightMotorSpeed; }
uint8_t Engine::getLeftMotorSpeed() { return leftMotorSpeed; }

void Engine::setRightMotorDirectionForward() {
    digitalWrite(MOTOR_R_DIR_1_PIN, HIGH);
    digitalWrite(MOTOR_R_DIR_2_PIN, LOW);
}
void Engine::setRightMotorDirectionBackward() {
    digitalWrite(MOTOR_R_DIR_1_PIN, LOW);
    digitalWrite(MOTOR_R_DIR_2_PIN, HIGH);
}
void Engine::setLeftMotorDirectionForward() {
    digitalWrite(MOTOR_L_DIR_1_PIN, HIGH);
    digitalWrite(MOTOR_L_DIR_2_PIN, LOW);
}
void Engine::setLeftMotorDirectionBackward() {
    digitalWrite(MOTOR_L_DIR_1_PIN, LOW);
    digitalWrite(MOTOR_L_DIR_2_PIN, HIGH);
}
void Engine::setBothMothorsDirectionNeutral() {
    digitalWrite(MOTOR_L_DIR_1_PIN, LOW);
    digitalWrite(MOTOR_L_DIR_2_PIN, LOW);
    digitalWrite(MOTOR_R_DIR_1_PIN, LOW);
    digitalWrite(MOTOR_R_DIR_2_PIN, LOW);
}
#endif