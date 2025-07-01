#ifndef __CUSTOM_INTERRUPTS_H
#define __CUSTOM_INTERRUPTS_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

// �������ջ�������С
#define BT_BUFFER_SIZE 64

// С������ָ���
typedef enum {
    BT_CMD_STOP = 0,     // ֹͣ
    BT_CMD_FORWARD = 1,  // ǰ��
    BT_CMD_BACKWARD = 2, // ����
    BT_CMD_LEFT = 3,     // ��ת
    BT_CMD_RIGHT = 4,    // ��ת
    BT_CMD_SPIN_LEFT = 5,// ����
    BT_CMD_SPIN_RIGHT = 6// ����
} BT_CommandType;

// ����ģ��ṹ��
typedef struct {
    UART_HandleTypeDef *huart;   // ���ھ��
    uint8_t rx_buffer[BT_BUFFER_SIZE]; // ���ջ�����
    uint16_t rx_index;           // ��������
    uint8_t cmd_ready;           // ���������־
    BT_CommandType current_cmd;  // ��ǰ����
} Bluetooth_HandleTypeDef;

// ��������
void Custom_Interrupts_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// ����ģ�麯��
void BT_Init(Bluetooth_HandleTypeDef *hbt, UART_HandleTypeDef *huart);
void BT_UART_IRQHandler(Bluetooth_HandleTypeDef *hbt);
void BT_ProcessData(Bluetooth_HandleTypeDef *hbt);
BT_CommandType BT_GetCurrentCommand(Bluetooth_HandleTypeDef *hbt);

// �ⲿ��������
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart6;
extern Bluetooth_HandleTypeDef hbt; // ����ģ��ȫ��ʵ��

#endif /* __CUSTOM_INTERRUPTS_H */

