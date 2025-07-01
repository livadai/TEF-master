#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart6_tx_dma_init(void)
{
    //使能串口DMA发送和接受
    SET_BIT(huart6.Instance->CR3,USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3,USART_CR3_DMAT);
    //失能DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart6_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart6_tx.Instance->NDTR = 0;
    
}

void usart2_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
 
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }
 
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF7);
 
    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);
 
    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

 



