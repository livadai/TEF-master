#include "bsp_m15_motor.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "bsp_can.h"
#include "can.h"

// 定义全局变量，用于存储电机的反馈位置
int16_t g_current_position = 0;

// 初始化电机
void BSP_M15_Init(void)
{
    // 设置电机模式为sudu环
    BSP_M15_SetMode(MODE_SPEED_LOOP);
}

// 设置电机模式
void BSP_M15_SetMode(uint8_t mode)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x105;  // 设置模式的标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = mode;  // 设置模式
    TxData[1] = 0;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// ????????
void BSP_M15_SetZero(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x112;  // 设置模式的标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0;  
    TxData[1] = 0;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}


// 设置电机速度
void BSP_M15_SetSpeed(int16_t rpm)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // 将rpm转换为给定值，范围为-21000 ~ 21000
    int16_t speed = rpm*100;  // 直接使用rpm值，确保传入的rpm在-210到210之间

    TxHeader.StdId = 0x32;  // 速度给定值的标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = (speed >> 8) & 0xFF;  // 高位
    TxData[1] = speed & 0xFF;         // 低位
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// 设置电机位置
void BSP_M15_SetPosition(int16_t angle)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // 将角度转换为给定值
    int16_t position = (int16_t)((float)angle / 360.0f * 32767.0f);

    TxHeader.StdId = 0x32;  // 控制报文的标识符，假设电机ID为1
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = (position >> 8) & 0xFF;  // 高位
    TxData[1] = position & 0xFF;         // 低位
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		
	
}

// 获取电机位置
int16_t BSP_M15_GetPosition(void)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    // 检查是否有新的消息
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // 检查标识符是否为电机的反馈报文
        if (RxHeader.StdId == 0x97)  
        {
            // 解析位置数据
            int16_t position = (RxData[0] << 8) | RxData[1];

            // 将给定值转换为角度
            int16_t angle = (int16_t)((float)position / 32767.0f * 360.0f);

            return angle;
        }
    }

    return -1;  
}

void BSP_M15_SetPID(uint8_t motorID, uint8_t mode, uint16_t P_dividend, uint8_t P_divisor, uint16_t I_dividend,uint8_t I_divisor)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x10C;  // PID参数调节的标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = motorID;  
    TxData[1] = mode;     
    TxData[2] = (P_dividend >> 8) & 0xFF;  // P被除数高八位
    TxData[3] = P_dividend & 0xFF;         // P被除数低八位
    TxData[4] = P_divisor;   // P除数
    TxData[5] = (I_dividend >> 8) & 0xFF;          // I被除数高八位
    TxData[6] = I_dividend  & 0xFF;   // I被除数低八位
    TxData[7] = I_divisor;       // I除数

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) ;
}

void BSP_M15_SetPositionLimit(uint8_t motorID, uint8_t mode, int16_t max_limit, int16_t min_limit)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x10D;  // 限幅输出值调节的标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    // 将限幅值放大32倍
    int16_t max_limit_scaled = max_limit * 32;
    int16_t min_limit_scaled = min_limit * 32;

    TxData[0] = motorID;  // 电机ID
    TxData[1] = mode;     // 模式
    TxData[2] = (max_limit_scaled >> 8) & 0xFF;  // 最大输出值高八位
    TxData[3] = max_limit_scaled & 0xFF;         // 最大输出值低八位
    TxData[4] = (min_limit_scaled >> 8) & 0xFF;  // 最小输出值高八位
    TxData[5] = min_limit_scaled & 0xFF;         // 最小输出值低八位
    TxData[6] = 0;  // 保留
    TxData[7] = 0;  // 保留

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void BSP_M15_SavePID(uint8_t motorID)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x10C;  // 参数保存的标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = motorID;  // 电机ID
    TxData[1] = 0xFE;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}




