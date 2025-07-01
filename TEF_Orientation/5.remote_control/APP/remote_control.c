#include "remote_control.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

RC_ctrl_t rc_ctrl;
uint8_t *active_buf;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


// SBUS数据解析
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    if (sbus_buf[0] != 0x0F)
    {
        return; 
    }

    rc_ctrl->Start = sbus_buf[0];  

    rc_ctrl->Ch1 = (((uint16_t)sbus_buf[1]) | ((uint16_t)((sbus_buf[2]))) << 8) & 0x07FF;
    rc_ctrl->Ch2 = ((uint16_t)((sbus_buf[2] & 0xf8) >> 3)) | (((uint16_t)(sbus_buf[3] & 0x3f)) << 5);
    rc_ctrl->Ch3 = ((uint16_t)((sbus_buf[3] & 0xc0) >> 6)) | ((((uint16_t)sbus_buf[4]) << 2)) | (((uint16_t)(sbus_buf[5] & 0x01)) << 10);
    rc_ctrl->Ch4 = ((uint16_t)((sbus_buf[5] & 0xfe) >> 1)) | (((uint16_t)(sbus_buf[6] & 0x0f)) << 7);

    rc_ctrl->SA = ((uint16_t)((sbus_buf[6] & 0xf0) >> 4)) | (((uint16_t)(sbus_buf[7] & 0x7f)) << 4);
    rc_ctrl->SB = ((uint16_t)((sbus_buf[7] & 0x80) >> 7)) | (((uint16_t)sbus_buf[8]) << 1) | (((uint16_t)(sbus_buf[9] & 0x03)) << 9);
    rc_ctrl->SC = ((uint16_t)((sbus_buf[9] & 0xfc) >> 2)) | (((uint16_t)(sbus_buf[10] & 0x1f)) << 6);
    rc_ctrl->SD = ((uint16_t)((sbus_buf[10] & 0xe0) >> 5)) | (((uint16_t)(sbus_buf[11])) << 3);
    rc_ctrl->SE = ((uint16_t)sbus_buf[12]) | (((uint16_t)(sbus_buf[13] & 0x07)) << 8);
    rc_ctrl->SF = ((uint16_t)((sbus_buf[13] & 0xf8) >> 3)) | (((uint16_t)(sbus_buf[14] & 0x3f)) << 5);
    rc_ctrl->SG = ((uint16_t)((sbus_buf[14] & 0xc0) >> 6)) | (((uint16_t)sbus_buf[15]) << 2) | (((uint16_t)(sbus_buf[16] & 0x01)) << 10);
    rc_ctrl->SH = ((uint16_t)((sbus_buf[16] & 0xfe) >> 1)) | (((uint16_t)(sbus_buf[17] & 0x0f)) << 7);
    rc_ctrl->LD = ((uint16_t)((sbus_buf[17] & 0xf0) >> 4)) | (((uint16_t)(sbus_buf[18] & 0x7f)) << 4);
    rc_ctrl->RD = ((uint16_t)((sbus_buf[18] & 0x80) >> 7)) | (((uint16_t)sbus_buf[19]) << 1) | (((uint16_t)(sbus_buf[20] & 0x03)) << 9);
    rc_ctrl->LS = ((uint16_t)((sbus_buf[20] & 0xfc) >> 2)) | (((uint16_t)(sbus_buf[21] & 0x1f)) << 6);
    rc_ctrl->RS = ((uint16_t)((sbus_buf[21] & 0xe0) >> 5)) | (((uint16_t)sbus_buf[22]) << 3);

    rc_ctrl->Ch1 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch2 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch3 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch4 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SA -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SB -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SC -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SE -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SF -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SG -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SH -= RC_CH_VALUE_OFFSET;
    rc_ctrl->LD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->RD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->LS -= RC_CH_VALUE_OFFSET;
    rc_ctrl->RS -= RC_CH_VALUE_OFFSET;//353
}

// 中断处理函数
void USART1_IRQHandler(void) {
    if (huart1.Instance->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        uint16_t rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
        
        if (rx_len == RC_FRAME_LENGTH) {
            active_buf = (hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) ? sbus_rx_buf[1] : sbus_rx_buf[0];
            sbus_to_rc(active_buf, &rc_ctrl);
        }
        
        // 切换缓冲区
        hdma_usart1_rx.Instance->CR ^= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, SBUS_RX_BUF_NUM);
        __HAL_DMA_ENABLE(&hdma_usart1_rx);
    }
}

void remote_control_init(void) {
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // 显式启用IDLE中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, sbus_rx_buf[0], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void) {
    return &rc_ctrl;
}

