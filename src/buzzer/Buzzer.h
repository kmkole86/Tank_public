#ifndef BUZZER_H
#define BUZZER_H
#include <Arduino.h>

#include "common/HWConstants.h"
//BUZZ the code using binary nums. 1 is 500ms long 0 is 150ms long
class Buzzer {
   public:
    uint8_t GOING_BACKWARD = 1;
    void init();
    void update();
    void buzzOK();
    void buzzErrBlocking();

   private:
    bool isBuzzing;
    const uint8_t INIT_ERR = 10;  // critical
    void buzzBlocking(uint8_t code);
};
#endif