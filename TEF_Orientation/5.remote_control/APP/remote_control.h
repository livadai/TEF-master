#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"     
#include "usart.h"    

// SBUS协议相关常量定义
#define SBUS_RX_BUF_NUM 50u       // 接收缓冲区大小（设置为大于帧长度防止溢出）
#define RC_FRAME_LENGTH 25u       // SBUS协议帧长度（固定25字节）
#define RC_CH_VALUE_OFFSET 1024u  // 通道值偏移量（SBUS原始值为0-2048，需减去1024得到-1024~+1024范围）

// 遥控器控制数据结构体（使用__packed确保紧凑内存对齐，避免编译器填充字节）
typedef __packed struct {
	
    int16_t Start;               // 帧起始标志（0x0F）
    
    // 四个主要控制通道（通常对应遥控器的摇杆）
    int16_t Ch1, Ch2, Ch3, Ch4;  
    
    // 8个开关通道
    int16_t SA, SB, SC, SD,SE, SF, SG, SH,LD, RD, LS, RS;     

} RC_ctrl_t;

// 全局遥控器数据结构体声明
extern RC_ctrl_t rc_ctrl;

// 函数声明
void remote_control_init(void);                     // 遥控器初始化函数
const RC_ctrl_t *get_remote_control_point(void);    // 获取遥控器数据指针

#endif