#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"     
#include "usart.h"    

// SBUSЭ����س�������
#define SBUS_RX_BUF_NUM 50u       // ���ջ�������С������Ϊ����֡���ȷ�ֹ�����
#define RC_FRAME_LENGTH 25u       // SBUSЭ��֡���ȣ��̶�25�ֽڣ�
#define RC_CH_VALUE_OFFSET 1024u  // ͨ��ֵƫ������SBUSԭʼֵΪ0-2048�����ȥ1024�õ�-1024~+1024��Χ��

// ң�����������ݽṹ�壨ʹ��__packedȷ�������ڴ���룬�������������ֽڣ�
typedef __packed struct {
	
    int16_t Start;               // ֡��ʼ��־��0x0F��
    
    // �ĸ���Ҫ����ͨ����ͨ����Ӧң������ҡ�ˣ�
    int16_t Ch1, Ch2, Ch3, Ch4;  
    
    // 8������ͨ��
    int16_t SA, SB, SC, SD,SE, SF, SG, SH,LD, RD, LS, RS;     

} RC_ctrl_t;

// ȫ��ң�������ݽṹ������
extern RC_ctrl_t rc_ctrl;

// ��������
void remote_control_init(void);                     // ң������ʼ������
const RC_ctrl_t *get_remote_control_point(void);    // ��ȡң��������ָ��

#endif