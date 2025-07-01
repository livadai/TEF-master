#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "main.h"
#include "judge.h"
#include "string.h"
#include "bsp_usart.h"
#include "usart.h"
#include "stdio.h"
#include "judge_task.h"

#define SBUS_RX_BUF_NUM 50u

#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

#define RC_SW_UP                ((uint16_t)-671)
#define RC_SW_MID               ((uint16_t)0)
#define RC_SW_DOWN              ((uint16_t)671)


typedef  __packed struct {
    uint16_t w : 1;      // bit 0: W键
    uint16_t s : 1;      // bit 1: S键
    uint16_t a : 1;      // bit 2: A键
    uint16_t d : 1;      // bit 3: D键
    uint16_t shift : 1;  // bit 4: Shift键
    uint16_t ctrl : 1;   // bit 5: Ctrl键
    uint16_t q : 1;      // bit 6: Q键
    uint16_t e : 1;      // bit 7: E键
    uint16_t r : 1;      // bit 8: R键
    uint16_t f : 1;      // bit 9: F键
    uint16_t g : 1;      // bit 10: G键
    uint16_t z : 1;      // bit 11: Z键
    uint16_t x : 1;      // bit 12: X键
    uint16_t c : 1;      // bit 13: C键
    uint16_t v : 1;      // bit 14: V键
    uint16_t b : 1;      // bit 15: B键
} keyboard_state_t;


typedef __packed struct
{
        __packed struct
        {
            int16_t Start;	
	
						int16_t Ch1;//右横
						int16_t Ch2;//左竖
						int16_t Ch3;//右竖
						int16_t Ch4;//左横

						int16_t SA;
						int16_t SB;
						int16_t SC;
						int16_t SD;
						
						int16_t SE;
						int16_t SF;
						int16_t SG;
						int16_t SH;
		
						int16_t LD;
						int16_t RD;
						int16_t LS;
						int16_t RS;
        } rc;
				
				__packed struct
				{
						int16_t mouse_x; //鼠标x轴移动速度，负值标识向左移动 

						int16_t mouse_y; //鼠标y轴移动速度，负值标识向下移动
						int16_t mouse_z; //鼠标滚轮移动速度，负值标识向后滚动

						int8_t left_button_down; //鼠标左键是否按下：0为未按下；1为按下
						int8_t right_button_down; //鼠标右键是否按下：0为未按下，1为按下 

						keyboard_state_t keyboard; //键盘按键信息，每个bit对应一个按键，0为未按下，1为按下

						uint16_t reserved; //保留位
				} key_mouse;
        

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



extern void remote_control_init(void);
void USART1_IRQHandler123(void);

extern const RC_ctrl_t *get_remote_control_point(void);

#endif
