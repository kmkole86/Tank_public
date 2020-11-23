#ifndef HWConstants_H
#define HWConstants_H

#define DISP1_EN 1
#define DISP2_EN 1

// sensors addresses
#define MUX_ADDR 0x70
#define IMU_9250_ADDR 0x69
#define DISPLAY1_ADDR 0x3D
#define DISPLAY2_ADDR 0x3C
#define EEPROM_ADDR 0x57
#define CLOCK_ADDR 0x68
#define MAGNETOMETER_ADDR 0x0C

// pwm channels
#define BUZZER_PWM_CHANN 0

// pwm freq
#define BUZZER_PWM_FREQ 1500
#define PWM_RESOLUTION 8

//Wire pins
#define SDA_W 23
#define SCL_W 22
#define SDA_W1 32
#define SCL_W1 33

// pins
#define BUZZER_PIN 2
#define LIGHT_SENSOR_PIN 39  // VN PIN
#define SOUND_SENSOR_PIN 35
#define BT_CONN_PIN 34

#endif