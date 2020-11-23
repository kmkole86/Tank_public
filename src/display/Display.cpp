#include "Display.h"

Display::Display() {
    display1 = Adafruit_SSD1306(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);
    display2 = Adafruit_SSD1306(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);
}

bool Display::init() {
    bool disp1InitedSucc = display1.begin(SSD1306_SWITCHCAPVCC, DISPLAY1_ADDR);
    bool disp2InitedSucc = display2.begin(SSD1306_SWITCHCAPVCC, DISPLAY2_ADDR);

    if (disp1InitedSucc && disp2InitedSucc) {
        display1.setRotation(2);
        display2.setRotation(2);

        display1.clearDisplay();
        display2.clearDisplay();

        display1.setTextSize(1);
        display1.setTextColor(WHITE);
        display1.setCursor(10, 22);
        display1.print("WIN 95...");

        display2.setTextSize(1);
        display2.setTextColor(WHITE);
        display2.setCursor(10, 22);
        display2.print("WIN 95...");

        display1.display();
        display2.display();
    }
    return disp1InitedSucc && disp2InitedSucc;
}

void Display::showConnectionStatus(bool isBtConnected) {
    if (isBtConnected)
        display1.setTextColor(BLACK, WHITE);

    display1.setCursor(0, 0);
    display1.print(F("CONN"));
    display1.setTextColor(WHITE);
}

void Display::showSpeed(uint8_t gear, uint8_t leftMotor, uint8_t rightMotor) {
    display1.setCursor(30, 0);
    display1.print(F("G:"));
    display1.print(gear);
    display1.setCursor(60, 0);
    display1.print(F("L"));
    display1.setCursor(66, 0);
    display1.print(leftMotor, DEC);
    display1.setCursor(90, 0);
    display1.print(F(" R"));
    display1.setCursor(102, 0);
    display1.print(rightMotor, DEC);
}

void Display::showMux() {
    display2.setCursor(48, 11);
    display2.setTextColor(BLACK, WHITE);
    display2.print(F(" MUX "));
    display2.setTextColor(WHITE);
}

void Display::showTemp(float temp) {
    display1.setCursor(0, 11);
    display1.print(F("T:"));
    display1.setCursor(12, 11);
    if (!isnan(temp))
        display1.print(temp, 2);
    else
        display1.print(F("-"));

    display1.setCursor(42, 11);
    display1.print(F("C"));
}

void Display::showPressure(uint16_t pressure) {
    display1.setCursor(60, 11);
    display1.print(F("P:"));
    display1.setCursor(72, 11);
    if (!isnan(pressure))
        display1.print(pressure);
    else
        display1.print(F("-"));
    display1.print(F("mBar"));
}

void Display::showHumid(float humid) {
    display1.setCursor(0, 22);
    display1.print(F("H:"));
    display1.setCursor(12, 22);

    if (!isnan(humid))
        display1.print(humid, 2);
    else
        display1.print(readErrorMsg);

    display1.setCursor(42, 22);
    display1.print(F("%"));
}

void Display::showSoundIntensity(uint16_t intensity) {
    display1.setCursor(60, 22);
    display1.print(F("S:"));
    display1.setCursor(72, 22);
    display1.print(intensity);
    display1.setCursor(90, 22);
    display1.print(F("%"));
}

void Display::printRange(uint16_t range) {
    if (range < 10) {
        display2.print(F("  "));
        display2.print(range);
    } else if (range < 100) {
        display2.print(F(" "));
        display2.print(range);
    } else if (range < 600)
        display2.print(range);
    else
        display2.print(outOfRange);
}

void Display::showRangeFinders(uint16_t ranges[]) {
    // display2
    //first row
    display2.setCursor(0, 0);
    display2.print('[');
    display2.setCursor(6, 0);
    printRange(ranges[0]);
    display2.setCursor(24, 0);
    display2.print(']');
    display2.setCursor(48, 0);
    display2.print('[');
    display2.setCursor(54, 0);
    printRange(ranges[1]);
    display2.setCursor(72, 0);
    display2.print(']');
    display2.setCursor(96, 0);
    display2.print('[');
    display2.setCursor(102, 0);
    printRange(ranges[2]);
    display2.setCursor(120, 0);
    display2.print(']');
    //second row
    display2.setCursor(0, 11);
    display2.print('[');
    display2.setCursor(6, 11);
    printRange(ranges[6]);
    display2.setCursor(24, 11);
    display2.print(']');
    display2.setCursor(96, 11);
    display2.print('[');
    display2.setCursor(102, 11);
    printRange(ranges[3]);
    display2.setCursor(120, 11);
    display2.print(']');
    //third row
    display2.setCursor(0, 22);
    display2.print('[');
    display2.setCursor(6, 22);
    printRange(ranges[5]);
    display2.setCursor(24, 22);
    display2.print(']');
    display2.setCursor(96, 22);
    display2.print('[');
    display2.setCursor(102, 22);
    printRange(ranges[4]);
    display2.setCursor(120, 22);
    display2.print(']');
}

void Display::showRoll(float roll) {
    display1.setCursor(0, 33);
    display1.print(F("R:"));
    display1.setCursor(12, 33);
    display1.print(roll, 2);
}

void Display::showPitch(float pitch) {
    display1.setCursor(60, 33);
    display1.print(F("P:"));
    display1.setCursor(72, 33);
    display1.print(pitch, 2);
}

void Display::showYaw(float yaw) {
    display1.setCursor(0, 44);
    display1.print(F("Y:"));
    display1.setCursor(12, 44);
    display1.print(yaw, 2);
}

void Display::showError(String error) {
    display1.clearDisplay();
    display2.clearDisplay();

    display1.setCursor(10, 22);
    display2.setCursor(10, 22);

    display1.print(error);
    display2.print(error);

    display1.display();
    display2.display();
}

void Display::showBtReadStatus(uint8_t btReadStatus) {
    display2.setCursor(0, 33);
    display2.print(F("btR_STAT:"));
    display2.setCursor(60, 33);
    display2.print(F("RECEIVED"));
    show();
}

void Display::showParseError() {
    display2.setCursor(0, 33);
    display2.print(F("CMD PARSE ERR"));
    show();
}

void Display::showData(bool isConnected, SensorsData *sensorsData, uint8_t gear, uint8_t m1Speed, uint8_t m2Speed, bool isLightOn, bool isHighBeam, bool isTrunSignalLeftOn, bool isTurnSignalRightOn) {
    clear();
    showConnectionStatus(isConnected);
    showSpeed(gear, m1Speed, m2Speed);  //TODO: implement reading from phone FWD/BWD
    showMux();
    showRangeFinders(sensorsData->VL53L0X_ranges);
    showTemp(sensorsData->temperature);
    showHumid(sensorsData->humidity);
    showPressure(sensorsData->pressure);  //lib do not support read err
    showSoundIntensity(sensorsData->soundIntensity);
    showRoll(sensorsData->roll);
    showPitch(sensorsData->pitch);
    showYaw(sensorsData->yaw);
    showLightStatus(isLightOn);
    showHighBeamStatus(isHighBeam);
    showTurnLeftStatus(isTrunSignalLeftOn);
    showTurnRightStatus(isTurnSignalRightOn);
    show();
}

void Display::showLightStatus(bool isOn) {
    if (isOn)
        display2.setTextColor(BLACK, WHITE);
    display2.setCursor(0, 33);
    display2.print(F("LIGHT"));
    display2.setTextColor(WHITE);
}

void Display::showHighBeamStatus(bool isOn) {
    if (isOn)
        display2.setTextColor(BLACK, WHITE);
    display2.setCursor(60, 33);
    display2.print(F("H_BEAM"));
    display2.setTextColor(WHITE);
}

void Display::showTurnRightStatus(bool isOn) {
    if (isOn)
        display2.setTextColor(BLACK, WHITE);
    display2.setCursor(30, 44);
    display2.print(F("TSR"));
    display2.setTextColor(WHITE);
}

void Display::showTurnLeftStatus(bool isOn) {
    if (isOn)
        display2.setTextColor(BLACK, WHITE);
    display2.setCursor(0, 44);
    display2.print(F("TSL"));
    display2.setTextColor(WHITE);
}

void Display::clear() {
    display1.clearDisplay();
    display2.clearDisplay();
}

void Display::show() {
    display1.display();
    display2.display();
}