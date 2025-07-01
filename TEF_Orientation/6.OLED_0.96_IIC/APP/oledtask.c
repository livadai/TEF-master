#include "oledtask.h"
#include "oled.h"
#include "bmp.h"
#include "main.h"
#include "stdio.h"
#include "i2c.h"

uint8_t t;

void oled_task(void const *pvParameters) {
    OLED_Init();  // 初始化（仅执行一次）
    t=' ';
    while (1) {
						// 保持原有显示逻辑不变
						OLED_Clear(0);
						OLED_ShowCHinese(0,0,0);//全
						OLED_ShowString(6,3,"Hello WORLD ..",16);
						OLED_ShowString(0,6,"ASCII:",16);  
						OLED_ShowString(63,6,"CODE:",16);  
						OLED_ShowChar(48,6,t,16);//显示ASCII字符	   
						t++;
						if(t>'~')t=' ';
						OLED_ShowNum(103,6,t,3,16);//显示ASCII字符的码值 	
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						OLED_Clear(0);
						OLED_DrawBMP(0,0,128,8,BMP1);
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						OLED_Clear(0);
						OLED_DrawBMP(0,0,128,8,BMP2);
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						OLED_Clear(0);
						OLED_DrawBMP(0,0,128,8,BMP3);
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						HAL_Delay(500);
						OLED_Clear(0);
    }
}
