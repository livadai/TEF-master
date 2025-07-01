#include "bsp_m15_motor.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "bsp_can.h"
#include "can.h"

// ����ȫ�ֱ��������ڴ洢����ķ���λ��
int16_t g_current_position = 0;

// ��ʼ�����
void BSP_M15_Init(void)
{
    // ���õ��ģʽΪsudu��
    BSP_M15_SetMode(MODE_SPEED_LOOP);
}

// ���õ��ģʽ
void BSP_M15_SetMode(uint8_t mode)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x105;  // ����ģʽ�ı�ʶ��
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = mode;  // ����ģʽ
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

    TxHeader.StdId = 0x112;  // ����ģʽ�ı�ʶ��
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


// ���õ���ٶ�
void BSP_M15_SetSpeed(int16_t rpm)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // ��rpmת��Ϊ����ֵ����ΧΪ-21000 ~ 21000
    int16_t speed = rpm*100;  // ֱ��ʹ��rpmֵ��ȷ�������rpm��-210��210֮��

    TxHeader.StdId = 0x32;  // �ٶȸ���ֵ�ı�ʶ��
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = (speed >> 8) & 0xFF;  // ��λ
    TxData[1] = speed & 0xFF;         // ��λ
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// ���õ��λ��
void BSP_M15_SetPosition(int16_t angle)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // ���Ƕ�ת��Ϊ����ֵ
    int16_t position = (int16_t)((float)angle / 360.0f * 32767.0f);

    TxHeader.StdId = 0x32;  // ���Ʊ��ĵı�ʶ����������IDΪ1
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = (position >> 8) & 0xFF;  // ��λ
    TxData[1] = position & 0xFF;         // ��λ
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		
	
}

// ��ȡ���λ��
int16_t BSP_M15_GetPosition(void)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    // ����Ƿ����µ���Ϣ
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // ����ʶ���Ƿ�Ϊ����ķ�������
        if (RxHeader.StdId == 0x97)  
        {
            // ����λ������
            int16_t position = (RxData[0] << 8) | RxData[1];

            // ������ֵת��Ϊ�Ƕ�
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

    TxHeader.StdId = 0x10C;  // PID�������ڵı�ʶ��
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = motorID;  
    TxData[1] = mode;     
    TxData[2] = (P_dividend >> 8) & 0xFF;  // P�������߰�λ
    TxData[3] = P_dividend & 0xFF;         // P�������Ͱ�λ
    TxData[4] = P_divisor;   // P����
    TxData[5] = (I_dividend >> 8) & 0xFF;          // I�������߰�λ
    TxData[6] = I_dividend  & 0xFF;   // I�������Ͱ�λ
    TxData[7] = I_divisor;       // I����

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) ;
}

void BSP_M15_SetPositionLimit(uint8_t motorID, uint8_t mode, int16_t max_limit, int16_t min_limit)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x10D;  // �޷����ֵ���ڵı�ʶ��
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    // ���޷�ֵ�Ŵ�32��
    int16_t max_limit_scaled = max_limit * 32;
    int16_t min_limit_scaled = min_limit * 32;

    TxData[0] = motorID;  // ���ID
    TxData[1] = mode;     // ģʽ
    TxData[2] = (max_limit_scaled >> 8) & 0xFF;  // ������ֵ�߰�λ
    TxData[3] = max_limit_scaled & 0xFF;         // ������ֵ�Ͱ�λ
    TxData[4] = (min_limit_scaled >> 8) & 0xFF;  // ��С���ֵ�߰�λ
    TxData[5] = min_limit_scaled & 0xFF;         // ��С���ֵ�Ͱ�λ
    TxData[6] = 0;  // ����
    TxData[7] = 0;  // ����

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void BSP_M15_SavePID(uint8_t motorID)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x10C;  // ��������ı�ʶ��
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = motorID;  // ���ID
    TxData[1] = 0xFE;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}




