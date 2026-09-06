#pragma once

#include "driver/gpio.h"

#define MOTOR_IN1 GPIO_NUM_9
#define MOTOR_IN2 GPIO_NUM_10

// BNO080 I2C — confirmed by bus scan
#define IMU_SDA_GPIO GPIO_NUM_12
#define IMU_SCL_GPIO GPIO_NUM_11
#define IMU_I2C_ADDR 0x4B

// PWM: 2 kHz, 10-bit. Lower freq = more torque at slow speed.
#define MOTOR_PWM_FREQ_HZ 2000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX 1023
