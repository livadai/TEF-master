#include "servo.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <math.h>
#include "user_lib.h"
#include "main.h"
#include "stdio.h"

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t Channel) {
	
    HAL_TIM_PWM_Start(htim, Channel);
}

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle) {

    if (angle < SERVO_MIN_ANGLE) {
        angle = SERVO_MIN_ANGLE;
    } else if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }

    float pulse_width = SERVO_MIN_PULSE_WIDTH + (angle * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH)) / SERVO_MAX_ANGLE;

    __HAL_TIM_SET_COMPARE(htim, Channel, pulse_width);
}