#include "can_receive.h"
#include "can.h"

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
		
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void can1_four( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006)
{
    gimbal_tx_message.StdId = 0x200;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (D2006 >> 8);
    gimbal_can_send_data[1] = D2006;
    gimbal_can_send_data[2] = (L2006>> 8);
    gimbal_can_send_data[3] = L2006;
    gimbal_can_send_data[4] = (R2006 >> 8);
    gimbal_can_send_data[5] = R2006;
    gimbal_can_send_data[6] = (S2006 >> 8);
    gimbal_can_send_data[7] = S2006;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, 0);
}

void can1_two( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006)
{
    gimbal_tx_message.StdId = 0x1FF;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (D2006 >> 8);
    gimbal_can_send_data[1] = D2006;
    gimbal_can_send_data[2] = (L2006>> 8);
    gimbal_can_send_data[3] = L2006;
    gimbal_can_send_data[4] = (R2006 >> 8);
    gimbal_can_send_data[5] = R2006;
    gimbal_can_send_data[6] = (S2006 >> 8);
    gimbal_can_send_data[7] = S2006;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, 0);
}

void can2_four( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006)
{
    gimbal_tx_message.StdId = 0x200;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (D2006 >> 8);
    gimbal_can_send_data[1] = D2006;
    gimbal_can_send_data[2] = (L2006>> 8);
    gimbal_can_send_data[3] = L2006;
    gimbal_can_send_data[4] = (R2006 >> 8);
    gimbal_can_send_data[5] = R2006;
    gimbal_can_send_data[6] = (S2006 >> 8);
    gimbal_can_send_data[7] = S2006;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, 0);
}

void can2_two( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006)
{
    gimbal_tx_message.StdId = 0x1FF;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (D2006 >> 8);
    gimbal_can_send_data[1] = D2006;
    gimbal_can_send_data[2] = (L2006>> 8);
    gimbal_can_send_data[3] = L2006;
    gimbal_can_send_data[4] = (R2006 >> 8);
    gimbal_can_send_data[5] = R2006;
    gimbal_can_send_data[6] = (S2006 >> 8);
    gimbal_can_send_data[7] = S2006;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, 0);
}

// 定义电机数据结构数组（CAN1和CAN2各6个）
motor_measure_t CHASSIS_MOTORS[6]; // CAN1: ID 0x201~0x206
motor_measure_t ARM_MOTORS[6];     // CAN2: ID 0x201~0x206

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    // 计算电机索引（ID - 0x201）
    uint8_t motor_idx = rx_header.StdId - 0x201;

    // 检查ID范围（0x201~0x206）并处理
    if (motor_idx < 6) {
        motor_measure_t* motor = (hcan->Instance == CAN1) 
                               ? &CHASSIS_MOTORS[motor_idx] 
                               : &ARM_MOTORS[motor_idx];
        get_motor_measure(motor, rx_data);
    }
		
}




