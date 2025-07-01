#include "CAN_receive.h"
#include "main.h"
#include "bsp_m15_motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

		
#define get_m15_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_position= (ptr)->position;                                   \
        (ptr)->speed = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->given_current = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->position = (uint16_t)((data)[4] << 8 | (data)[5]);  \
    }
	
m15_measure_t M15_DATA;
motor_measure_t CHASSIS_DATAMOTOR1; //底盘数据
motor_measure_t CHASSIS_DATAMOTOR2;
motor_measure_t CHASSIS_DATAMOTOR3;
motor_measure_t CHASSIS_DATAMOTOR4;

motor_measure_t RIGHT_DATA2006;//末端执行器三个2006数据
motor_measure_t LEFT_DATA2006;
motor_measure_t DOWN_DATA2006;		

motor_measure_t J1_DATA3508;//机械臂第一级3508电机数据
motor_measure_t J2_DATA3508;
		
motor_measure_t STRETCH_DATA2006;//伸缩结构2006电机数据

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
  if (hcan->Instance == CAN1)
  {
    // 处理 CAN1-底盘 的数据
    switch (rx_header.StdId)
    {
      case CAN_3508_M1_ID:
        get_motor_measure(&CHASSIS_DATAMOTOR1, rx_data); 
        break;
      case CAN_3508_M2_ID:
        get_motor_measure(&CHASSIS_DATAMOTOR2, rx_data); 
        break;
      case CAN_3508_M3_ID:
        get_motor_measure(&CHASSIS_DATAMOTOR3, rx_data); 
        break;
      case CAN_3508_M4_ID:
        get_motor_measure(&CHASSIS_DATAMOTOR4, rx_data); 
        break;
			case 0x97:
        get_m15_measure(&M15_DATA, rx_data);; 
        break;
    }
  }
  else if (hcan->Instance == CAN2)
  {
    // 处理 CAN2-机械臂 的数据
    switch (rx_header.StdId)
    {
      case CAN_J1_MOTOR_ID:
        get_motor_measure(&J1_DATA3508, rx_data); // 关节1数据获取
        break;
		 case CAN_J2_MOTOR_ID:
        get_motor_measure(&J2_DATA3508, rx_data); // 关节2数据获取
        break;
      case CAN_LEFT_END_ID:
        get_motor_measure(&LEFT_DATA2006, rx_data); // 末端执行器左2006数据获取
        break;
      case CAN_RIGHT_END_ID:
        get_motor_measure(&RIGHT_DATA2006, rx_data); // 末端执行器右2006数据获取
        break;
      case CAN_STRETCH_ID:
        get_motor_measure(&STRETCH_DATA2006, rx_data); // 伸缩结构2006电机数据获取
        break;
      case CAN_DOWN_END_ID:
        get_motor_measure(&DOWN_DATA2006, rx_data); // 末端执行器下2006数据获取
        break;
    }
  }
}


void CAN_cmd_gimbal1( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID1;
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
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_gimbal2( int16_t J1, int16_t J2, int16_t rev2,int16_t rev3)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID2;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (J1 >> 8);
    gimbal_can_send_data[1] = J1;
    gimbal_can_send_data[2] = (J2>> 8);
    gimbal_can_send_data[3] = J2;
    gimbal_can_send_data[4] = (rev2 >> 8);
    gimbal_can_send_data[5] = rev2;
    gimbal_can_send_data[6] = (rev3 >> 8);
    gimbal_can_send_data[7] = rev3;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//电机的电流值16384对应20A
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

const motor_measure_t *get_j1_gimbal_motor_measure_point(void)
{
	return &J1_DATA3508;
}

const motor_measure_t *get_j2_gimbal_motor_measure_point(void)
{
	return &J2_DATA3508;
}

const motor_measure_t *get_l2006_motor_measure_point(void)
{
    return &LEFT_DATA2006;
}

const motor_measure_t *get_r2006_motor_measure_point(void)
{
    return &RIGHT_DATA2006;
}

const motor_measure_t *get_d2006_motor_measure_point(void)
{
    return &DOWN_DATA2006;
}

const motor_measure_t *get_s2006_motor_measure_point(void)
{
    return &STRETCH_DATA2006;
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    switch (i)
    {
        case 0:
            return &CHASSIS_DATAMOTOR1;
        case 1:
            return &CHASSIS_DATAMOTOR2;
        case 2:
            return &CHASSIS_DATAMOTOR3;
        case 3:
            return &CHASSIS_DATAMOTOR4;
        default:
            return NULL;
    }
}
