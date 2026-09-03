#pragma once

#include "driver/gpio.h"

#define MOTOR_IN1 GPIO_NUM_9
#define MOTOR_IN2 GPIO_NUM_10

// PWM: 2 kHz, 10-bit. Lower freq = more torque at slow speed.
#define MOTOR_PWM_FREQ_HZ 2000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX 1023
