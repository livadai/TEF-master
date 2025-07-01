#ifndef __CUSTOM_INTERRUPTS_H
#define __CUSTOM_INTERRUPTS_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

// 蓝牙接收缓冲区大小
#define BT_BUFFER_SIZE 64

// 小车控制指令定义
typedef enum {
    BT_CMD_STOP = 0,     // 停止
    BT_CMD_FORWARD = 1,  // 前进
    BT_CMD_BACKWARD = 2, // 后退
    BT_CMD_LEFT = 3,     // 左转
    BT_CMD_RIGHT = 4,    // 右转
    BT_CMD_SPIN_LEFT = 5,// 左旋
    BT_CMD_SPIN_RIGHT = 6// 右旋
} BT_CommandType;

// 蓝牙模块结构体
typedef struct {
    UART_HandleTypeDef *huart;   // 串口句柄
    uint8_t rx_buffer[BT_BUFFER_SIZE]; // 接收缓冲区
    uint16_t rx_index;           // 接收索引
    uint8_t cmd_ready;           // 命令就绪标志
    BT_CommandType current_cmd;  // 当前命令
} Bluetooth_HandleTypeDef;

// 函数声明
void Custom_Interrupts_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// 蓝牙模块函数
void BT_Init(Bluetooth_HandleTypeDef *hbt, UART_HandleTypeDef *huart);
void BT_UART_IRQHandler(Bluetooth_HandleTypeDef *hbt);
void BT_ProcessData(Bluetooth_HandleTypeDef *hbt);
BT_CommandType BT_GetCurrentCommand(Bluetooth_HandleTypeDef *hbt);

// 外部变量声明
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart6;
extern Bluetooth_HandleTypeDef hbt; // 蓝牙模块全局实例

#endif /* __CUSTOM_INTERRUPTS_H */

