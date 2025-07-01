#include "remote_control.h"
#include "Vacuum_suction_task.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "main.h"
#include "stdio.h"
#include "bsp_m15_motor.h"
#include "servo.h"
#include "tim.h"

const RC_ctrl_t *rc_Va;

uint16_t current_angle = 0;

void Vacuum_task(void const *pvParameters){
	
	  vTaskDelay(VACUUM_SUCTION_TASK_INIT_TIME);
	  
	  Servo_Init(&htim2, TIM_CHANNEL_2);
	 
    while(1){
			
			rc_Va = get_remote_control_point();
			
			if (rc_Va->rc.SF<0) {
				  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);
			}
			else{
					HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET);
			}
			
//			  Servo_SetAngle(&htim2, TIM_CHANNEL_2, 0);
//        HAL_Delay(1000);

//        Servo_SetAngle(&htim2, TIM_CHANNEL_2, 90);
//        HAL_Delay(1000);

//        Servo_SetAngle(&htim2, TIM_CHANNEL_2, 180);
//        HAL_Delay(1000);

//        Servo_SetAngle(&htim2, TIM_CHANNEL_2, 270);
//        HAL_Delay(1000);
			
	    if (rc_Va->rc.SC > 0) {
            
            current_angle = 105;
            Servo_SetAngle(&htim2, TIM_CHANNEL_2, current_angle);
						HAL_Delay(100);
      }
			else if(rc_Va->rc.SC < 0){
				
			      current_angle =75;
            Servo_SetAngle(&htim2, TIM_CHANNEL_2, current_angle);
						HAL_Delay(100);
			}
			else {
				    Servo_SetAngle(&htim2, TIM_CHANNEL_2, 90);
			}		
					
		}
}
