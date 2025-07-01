#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

#define SERVO_MIN_PULSE_WIDTH 500   // 0.5ms
#define SERVO_MAX_PULSE_WIDTH 2500  // 2.5ms

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 500

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t Channel);

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle);

#endif