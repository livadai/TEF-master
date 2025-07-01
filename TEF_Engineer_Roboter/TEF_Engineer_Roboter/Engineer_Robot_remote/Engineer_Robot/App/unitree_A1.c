#include "unitree_A1.h"
#include "string.h"
#include "usart.h"
#include "bsp_usart.h"

#define PI 3.14159

motor_send_t MotorA1_send; 
motor_recv_t MotorA1_recv;

uint8_t test[34];

// CRC校验位的代码
uint32_t crc32_core_Ver3(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

// 位置模式：将T和W设置为0，针对pos都PD控制
void modfiy_pos_cmd(motor_send_t *MotorA1_send,uint8_t id, float Pos, float KP, float KW)
{

    MotorA1_send->hex_len = 34;
    MotorA1_send->mode = 10;
	  MotorA1_send->id   = id;
    MotorA1_send->Pos  = 2*PI/360*9.1*Pos; //将弧度值转换为角度值，减速比为9.1
    MotorA1_send->W    = 0.0;
    MotorA1_send->T    = 0.0;
    MotorA1_send->K_P  = KP; //比例系数，0<KP<16
    MotorA1_send->K_W  = KW; //微分系数, 0<KW<32
}

// 速度模式：T和KP必须为0，对W的P控制
void modfiy_speed_cmd(motor_send_t *MotorA1_send,uint8_t id, float Omega)
{

    MotorA1_send->hex_len = 34;
    MotorA1_send->mode = 10;
	  MotorA1_send->id   = id;
    MotorA1_send->Pos  = 0;
	  MotorA1_send->W    = Omega * 9.1f;//Omega:rad/s
    MotorA1_send->T    = 0.0;
    MotorA1_send->K_P  = 0.0;
    MotorA1_send->K_W  = 3.0;
}

// 电机发送接收函数
void unitreeA1_rxtx(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        uint8_t A1_send[34]; // 发送数据
        uint8_t RDate[78];  // 接收数据

        MotorA1_send.motor_send_data.start[0] = 0xFE;
        MotorA1_send.motor_send_data.start[1] = 0xEE;
        MotorA1_send.motor_send_data.motorID = MotorA1_ID;
        MotorA1_send.motor_send_data.reserved = 0x00;

        MotorA1_send.motor_send_data.mode = 10; 
        MotorA1_send.motor_send_data.ModifyBit = 0x00;
        MotorA1_send.motor_send_data.ReadBit= 0x00;
        MotorA1_send.motor_send_data.reserved2= 0x00;
        MotorA1_send.motor_send_data.Modify  = 0;
        MotorA1_send.motor_send_data.T= MotorA1_send.T * 256;
        MotorA1_send.motor_send_data.W= MotorA1_send.W * 128;
        MotorA1_send.motor_send_data.Pos= (int)((MotorA1_send.Pos / 6.2832f) * 16384.0f);
        MotorA1_send.motor_send_data.K_P= MotorA1_send.K_P * 2048;
        MotorA1_send.motor_send_data.K_W= MotorA1_send.K_W * 1024;

        MotorA1_send.motor_send_data.LowHzMotorCmdIndex = 0;
        MotorA1_send.motor_send_data.LowHzMotorCmdByte  = 0;
        MotorA1_send.motor_send_data.Res[0] = 0x00;

        MotorA1_send.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&MotorA1_send.motor_send_data), 7); // CRC校验

        memcpy(A1_send, &MotorA1_send.motor_send_data, 34);
				for(int i=0; i <34; i++)test[i] = A1_send[i];
        // DMA 发送数据 + 接收数据
        HAL_UART_Transmit(&huart6, A1_send, 34,0x03);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart6, RDate, 78);

        MotorA1_recv.motor_recv_data.motorID = RDate[2];  
        MotorA1_recv.motor_recv_data.mode= RDate[4];  
        MotorA1_recv.motor_recv_data.Temp= RDate[6];
        MotorA1_recv.motor_recv_data.MError= RDate[7]; 
        MotorA1_recv.motor_recv_data.T= RDate[13] << 8  | RDate[12]; 
        MotorA1_recv.motor_recv_data.W= RDate[15] << 8  | RDate[14]; 
        MotorA1_recv.motor_recv_data.Pos= RDate[33] << 24 | RDate[32] << 16 | RDate[31] << 8 | RDate[30];  

        MotorA1_recv.motor_id = MotorA1_recv.motor_recv_data.motorID;                           
        MotorA1_recv.mode= MotorA1_recv.motor_recv_data.mode;                               
        MotorA1_recv.Temp= MotorA1_recv.motor_recv_data.Temp;                                
        MotorA1_recv.MError= MotorA1_recv.motor_recv_data.MError;                               
        MotorA1_recv.T= (float) MotorA1_recv.motor_recv_data.T / 256;                     
        MotorA1_recv.Pos= (float) (MotorA1_recv.motor_recv_data.Pos / (16384.0f/2/PI));      
        MotorA1_recv.W= (float) MotorA1_recv.motor_recv_data.W / 128;      

    }
}
