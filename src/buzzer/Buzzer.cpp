#include "Buzzer.h"

void Buzzer::init() {
    // configure one of 16 PWM generators (ch 0-16, frequency, resolution e.g. 8bits 0-256 (min-max value). you have constants signal of freq you setted, but you control it with resotuion
    ledcSetup(BUZZER_PWM_CHANN, BUZZER_PWM_FREQ, PWM_RESOLUTION);
    // attach prev setted PWM GENERATOR  to PIN
    ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANN);
}

//infinite loop, block the setup func
void Buzzer::buzzBlocking(uint8_t code) {
    while (1) {
        int8_t i = 3;  // move 101 pomeris za 2 i &0x01 uzimas
        while (i != -1) {
            ledcWrite(BUZZER_PWM_CHANN, 255);  //ledcWriteTone(BUZZER_PWM_CHANN, 255);   // duty cycle set 8bit 0-255
            delay(bitRead(code, i) ? 500 : 150);
            ledcWrite(BUZZER_PWM_CHANN, 0);  //ledcWriteTone(BUZZER_PWM_CHANN, 0);
            delay(500);
            i--;
        }
        delay(3000);
    }
}

void Buzzer::buzzOK() {
    ledcWrite(BUZZER_PWM_CHANN, 255);  //ledcWriteTone(BUZZER_PWM_CHANN, BUZZER_PWM_FREQ);
    delay(1000);
    ledcWrite(BUZZER_PWM_CHANN, 0);  //ledcWriteTone(BUZZER_PWM_CHANN, 0);
}

void Buzzer::buzzErrBlocking() {
    buzzBlocking(INIT_ERR);
}

void Buzzer::update() {
    //TODO implement
}