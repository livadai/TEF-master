#include "remote_control.h"
#include "main.h"
#include "judge.h"
#include "string.h"
#include "bsp_usart.h"
#include "usart.h"
#include "stdio.h"
#include "judge_task.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t rx_buffer[1];  // ÿ�ν��� 1 ���ֽ�

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//ң�������Ʊ���
RC_ctrl_t rc_ctrl;

//����ԭʼ���ݣ�Ϊ25���ֽڣ�����50���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

int fputc(int ch,FILE *f)
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart6,temp,1,2);
	return ch;
}	



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART3) 
//    {

//    }
//}

//�����ж�
void USART1_IRQHandler123(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //�趨������1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {

            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}


extern void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    if (sbus_buf[0] != 0x0F)
    {
        return; 
    }

    rc_ctrl->rc.Start = sbus_buf[0];  

    rc_ctrl->rc.Ch1 = (((uint16_t)sbus_buf[1]) | ((uint16_t)((sbus_buf[2]))) << 8) & 0x07FF;
    rc_ctrl->rc.Ch2 = ((uint16_t)((sbus_buf[2] & 0xf8) >> 3)) | (((uint16_t)(sbus_buf[3] & 0x3f)) << 5);
    rc_ctrl->rc.Ch3 = ((uint16_t)((sbus_buf[3] & 0xc0) >> 6)) | ((((uint16_t)sbus_buf[4]) << 2)) | (((uint16_t)(sbus_buf[5] & 0x01)) << 10);
    rc_ctrl->rc.Ch4 = ((uint16_t)((sbus_buf[5] & 0xfe) >> 1)) | (((uint16_t)(sbus_buf[6] & 0x0f)) << 7);

    rc_ctrl->rc.SA = ((uint16_t)((sbus_buf[6] & 0xf0) >> 4)) | (((uint16_t)(sbus_buf[7] & 0x7f)) << 4);
    rc_ctrl->rc.SB = ((uint16_t)((sbus_buf[7] & 0x80) >> 7)) | (((uint16_t)sbus_buf[8]) << 1) | (((uint16_t)(sbus_buf[9] & 0x03)) << 9);
    rc_ctrl->rc.SC = ((uint16_t)((sbus_buf[9] & 0xfc) >> 2)) | (((uint16_t)(sbus_buf[10] & 0x1f)) << 6);
    rc_ctrl->rc.SD = ((uint16_t)((sbus_buf[10] & 0xe0) >> 5)) | (((uint16_t)(sbus_buf[11])) << 3);
    rc_ctrl->rc.SE = ((uint16_t)sbus_buf[12]) | (((uint16_t)(sbus_buf[13] & 0x07)) << 8);
    rc_ctrl->rc.SF = ((uint16_t)((sbus_buf[13] & 0xf8) >> 3)) | (((uint16_t)(sbus_buf[14] & 0x3f)) << 5);
    rc_ctrl->rc.SG = ((uint16_t)((sbus_buf[14] & 0xc0) >> 6)) | (((uint16_t)sbus_buf[15]) << 2) | (((uint16_t)(sbus_buf[16] & 0x01)) << 10);
    rc_ctrl->rc.SH = ((uint16_t)((sbus_buf[16] & 0xfe) >> 1)) | (((uint16_t)(sbus_buf[17] & 0x0f)) << 7);
    rc_ctrl->rc.LD = ((uint16_t)((sbus_buf[17] & 0xf0) >> 4)) | (((uint16_t)(sbus_buf[18] & 0x7f)) << 4);
    rc_ctrl->rc.RD = ((uint16_t)((sbus_buf[18] & 0x80) >> 7)) | (((uint16_t)sbus_buf[19]) << 1) | (((uint16_t)(sbus_buf[20] & 0x03)) << 9);
    rc_ctrl->rc.LS = ((uint16_t)((sbus_buf[20] & 0xfc) >> 2)) | (((uint16_t)(sbus_buf[21] & 0x1f)) << 6);
    rc_ctrl->rc.RS = ((uint16_t)((sbus_buf[21] & 0xe0) >> 5)) | (((uint16_t)sbus_buf[22]) << 3);

    // ��ȥƫ����
    rc_ctrl->rc.Ch1 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.Ch2 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.Ch3 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.Ch4 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SA -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SB -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SC -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SE -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SF -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SG -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.SH -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.LD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.RD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.LS -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.RS -= RC_CH_VALUE_OFFSET;//353

}
