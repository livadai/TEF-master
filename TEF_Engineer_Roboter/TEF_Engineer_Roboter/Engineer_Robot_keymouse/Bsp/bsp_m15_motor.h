#ifndef BSP_M15_MOTOR_H
#define BSP_M15_MOTOR_H

#include "stm32f4xx_hal.h"  

// 定义电机ID
#define MOTOR_ID 1

// 定义模式值
#define MODE_SPEED_LOOP 0x02    //速度环
#define MODE_POSITION_LOOP 0x03  //位置环

// 定义全局变量，用于存储电机的反馈位置
extern int16_t g_current_position;

// 函数声明
void CAN_filter_Init(CAN_HandleTypeDef* hcan);
void BSP_M15_Init(void);;
void BSP_M15_SetZero(void);
void BSP_M15_SetMode(uint8_t mode);
void BSP_M15_SetPosition(int16_t position);
void BSP_M15_SetSpeed(int16_t rpm);
int16_t BSP_M15_GetPosition(void);  // 获取电机位置
void BSP_M15_SetPID(uint8_t motorID, uint8_t mode, uint16_t P_dividend, uint8_t P_divisor, uint16_t I_dividend,uint8_t I_divisor);
void BSP_M15_SetPositionLimit(uint8_t motorID, uint8_t mode, int16_t max_limit, int16_t min_limit);
void BSP_M15_SavePID(uint8_t motorID);


#endif // BSP_M15_MOTOR_H

