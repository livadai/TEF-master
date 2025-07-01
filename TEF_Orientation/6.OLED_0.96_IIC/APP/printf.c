#include <stdio.h>
#include "usart.h"
 
int fputc(int c,FILE* s)
{
	HAL_UART_Transmit(&huart6,(const uint8_t*)&c,1,0xFFFF );
	return c;
}


