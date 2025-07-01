#include "judge_task.h"
#include "judge.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "bsp_usart.h"
#include "usart.h"

uint8_t rx_data[1];

void judge_task(void const *pvParameters)
{   
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, RxBuff, RxBuff_SIZE);
    while (1)
    {   
        // �����߼�
        osDelay(10);  // ��ʱ10ms����������ռ�ù���CPU��Դ
    }
}

