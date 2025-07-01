#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include "stm32f4xx_hal.h"  

#define MotorA1_ID 0xBB

// 发送用单个数据数据结构
typedef union
{
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;
	
} COMData32;

//电机发送报文34字节
typedef struct {
	
    // 数据包头
    uint8_t start[2];  // 包头，固定为 0xFE 和 0xEE
    uint8_t motorID;   //可为0，1，2，0xBB（向所有电机广播）
    uint8_t reserved;  

    // 数据体
    uint8_t mode;      // 电机运行模式___0-停，5-开环缓慢转动，10-闭环伺服控制
    uint8_t ModifyBit; 
    uint8_t ReadBit;   
    uint8_t reserved2;
	
    uint32_t Modify;   
	
    int16_t T;         // 电机前馈力矩，×256 倍描述
    int16_t W;         // 电机速度命令，×128 倍描述
	
    int32_t Pos;       // 电机位置命令，× (16384 / 2π) 倍描述
	
    int16_t K_P;       // 电机位置刚度，×2048 倍描述
    int16_t K_W;       // 电机速度刚度，×1024 倍描述
		
    uint8_t LowHzMotorCmdIndex; 
    uint8_t LowHzMotorCmdByte;  
    int32_t Res[1];       

    // CRC 校验
    COMData32 CRCdata;  // CRC 校验
		
} MotorCmd;


// 电机报文反馈数据结构体78字节
typedef struct {
	
	  //数据包头
    uint8_t start[2];  // 包头
    uint8_t motorID;   // 电机ID
    uint8_t reserved;
	
	  //数据体
    uint8_t mode;      // 当前模式
    uint8_t ReadBit;
    int8_t Temp;       // 温度
    uint8_t MError;    // 错误信息
	
    uint8_t Read[4];   
	
    int16_t T;         // 当前力矩*256描述
    int16_t W;         // 当前角速度*128
	
    int32_t LW;        
	
    int16_t W2;
    int32_t LW2;
    int16_t Acc;    //加速度*1
    int16_t OutAcc;
    int32_t Pos;       // 当前角度位置× (16384 / 2π) 倍描述
    int32_t Pos2;
    int16_t gyro[3];   
    int16_t acc[3];    
    uint8_t reserved3[24];
    COMData32 CRCdata;  // CRC校验
} MotorData;


//电机发送数据
typedef struct
{
		    MotorCmd motor_send_data;
        int hex_len;   //34
        unsigned short id;   
        unsigned short mode; 
        float T;             
        float W;             
        float Pos;           
        float K_P;           
        float K_W;           
     
}motor_send_t;


//电机接收数据
typedef struct
{
        MotorData motor_recv_data; 

					int hex_len;       // 78
					unsigned char motor_id; 
					unsigned char mode;     
					int Temp;              
					int MError;            
					float T;                
					float W;                
					float Pos;                         

} motor_recv_t; 

uint32_t crc32_core(uint32_t *ptr, uint32_t len);

extern motor_send_t MotorA1_send; 
extern motor_recv_t MotorA1_recv; 

void modfiy_pos_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW);
void modfiy_speed_cmd(motor_send_t *send,uint8_t id, float Omega);
void unitreeA1_rxtx(UART_HandleTypeDef *huart);

#endif // UNITREE_A1_H


