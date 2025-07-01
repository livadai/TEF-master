#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "stm32f1xx_hal.h"

// 定义步进电机控制命令
#define CMD_POSITION_CONTROL 0xFD

// 定义步进电机状态
typedef enum {
    MOTOR_IDLE,
    MOTOR_RUNNING,
    MOTOR_ERROR
} MotorState;

// 函数声明
void StepperMotor_Init(UART_HandleTypeDef *huart);
void StepperMotor_SetPosition(uint8_t direction, uint16_t speed, uint32_t pulse_count);
void StepperMotor_Process(void);

#endif // STEPPER_MOTOR_H