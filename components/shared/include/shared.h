#pragma once
#include "driver/gpio.h"
// Motor A
#define PIN_PWM_A GPIO_NUM_2
#define PIN_DIR_A_1 GPIO_NUM_4
#define PIN_DIR_A_2 GPIO_NUM_3

#define PIN_ENCODER_B_C1 GPIO_NUM_44
#define PIN_ENCODER_B_C2 GPIO_NUM_7
// Motor B
#define PIN_PWM_B GPIO_NUM_43
#define PIN_DIR_B_1 GPIO_NUM_5
#define PIN_DIR_B_2 GPIO_NUM_6

#define PIN_ENCODER_A_C1 GPIO_NUM_8
#define PIN_ENCODER_A_C2 GPIO_NUM_9
//Pulsador de reinicio
#define PIN_RESET GPIO_NUM_1
#define PIN_LED GPIO_NUM_21
void func(void);
