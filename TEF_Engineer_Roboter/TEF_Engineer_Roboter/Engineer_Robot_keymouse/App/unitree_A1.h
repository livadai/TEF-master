#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include "stm32f4xx_hal.h"  

#define MotorA1_ID 0xBB

// �����õ����������ݽṹ
typedef union
{
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;
	
} COMData32;

//������ͱ���34�ֽ�
typedef struct {
	
    // ���ݰ�ͷ
    uint8_t start[2];  // ��ͷ���̶�Ϊ 0xFE �� 0xEE
    uint8_t motorID;   //��Ϊ0��1��2��0xBB�������е���㲥��
    uint8_t reserved;  

    // ������
    uint8_t mode;      // �������ģʽ___0-ͣ��5-��������ת����10-�ջ��ŷ�����
    uint8_t ModifyBit; 
    uint8_t ReadBit;   
    uint8_t reserved2;
	
    uint32_t Modify;   
	
    int16_t T;         // ���ǰ�����أ���256 ������
    int16_t W;         // ����ٶ������128 ������
	
    int32_t Pos;       // ���λ������� (16384 / 2��) ������
	
    int16_t K_P;       // ���λ�øնȣ���2048 ������
    int16_t K_W;       // ����ٶȸնȣ���1024 ������
		
    uint8_t LowHzMotorCmdIndex; 
    uint8_t LowHzMotorCmdByte;  
    int32_t Res[1];       

    // CRC У��
    COMData32 CRCdata;  // CRC У��
		
} MotorCmd;


// ������ķ������ݽṹ��78�ֽ�
typedef struct {
	
	  //���ݰ�ͷ
    uint8_t start[2];  // ��ͷ
    uint8_t motorID;   // ���ID
    uint8_t reserved;
	
	  //������
    uint8_t mode;      // ��ǰģʽ
    uint8_t ReadBit;
    int8_t Temp;       // �¶�
    uint8_t MError;    // ������Ϣ
	
    uint8_t Read[4];   
	
    int16_t T;         // ��ǰ����*256����
    int16_t W;         // ��ǰ���ٶ�*128
	
    int32_t LW;        
	
    int16_t W2;
    int32_t LW2;
    int16_t Acc;    //���ٶ�*1
    int16_t OutAcc;
    int32_t Pos;       // ��ǰ�Ƕ�λ�á� (16384 / 2��) ������
    int32_t Pos2;
    int16_t gyro[3];   
    int16_t acc[3];    
    uint8_t reserved3[24];
    COMData32 CRCdata;  // CRCУ��
} MotorData;


//�����������
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


//�����������
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


