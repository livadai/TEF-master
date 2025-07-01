#ifndef BSP_M15_MOTOR_H
#define BSP_M15_MOTOR_H

#include "stm32f4xx_hal.h"  

// ������ID
#define MOTOR_ID 1

// ����ģʽֵ
#define MODE_SPEED_LOOP 0x02    //�ٶȻ�
#define MODE_POSITION_LOOP 0x03  //λ�û�

// ����ȫ�ֱ��������ڴ洢����ķ���λ��
extern int16_t g_current_position;

// ��������
void CAN_filter_Init(CAN_HandleTypeDef* hcan);
void BSP_M15_Init(void);;
void BSP_M15_SetZero(void);
void BSP_M15_SetMode(uint8_t mode);
void BSP_M15_SetPosition(int16_t position);
void BSP_M15_SetSpeed(int16_t rpm);
int16_t BSP_M15_GetPosition(void);  // ��ȡ���λ��
void BSP_M15_SetPID(uint8_t motorID, uint8_t mode, uint16_t P_dividend, uint8_t P_divisor, uint16_t I_dividend,uint8_t I_divisor);
void BSP_M15_SetPositionLimit(uint8_t motorID, uint8_t mode, int16_t max_limit, int16_t min_limit);
void BSP_M15_SavePID(uint8_t motorID);


#endif // BSP_M15_MOTOR_H

