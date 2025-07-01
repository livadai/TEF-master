#include "gimbal_task.h"
#include "can_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <math.h>
#include "user_lib.h"
#include "bsp_m15_motor.h"
#include "main.h"
#include "3508_angle.h"
#include "stdio.h"
#include "bsp_usart.h"
#include "usart.h"
#include "TwoDOF_ArmIK.h"

//栈溢出高水位线
#if INCLUDE_uxTaskGetStackHighWaterMark	
uint32_t gimbal_high_water;
#endif	

static void gimbal_init(gimbal_control_t *gimbal_control_init);
static void gimbal_set_mode(gimbal_control_t *gimbal_control_mode);
static void gimbal_feedback_update(gimbal_control_t *gimbal_control_update);
static void gimbal_set_control(gimbal_control_t *gimbal_control_control);
static void gimbal_control_loop(gimbal_control_t *gimbal_control_control_loop);
//static void M15_control(gimbal_control_t *gimbal_rc);
static void M15_Set(void);

//云台控制数据
gimbal_control_t gimbal_control;

static float angle;
void gimbal_task(void const *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
   
    gimbal_init(&gimbal_control);
	
	  //m3508_init();
	
	  BSP_M15_SetMode(MODE_POSITION_LOOP);
		BSP_M15_SetPID(1, 0x03, 0x01, 0, 0, 0);
    BSP_M15_SavePID(1);
    BSP_M15_SetPositionLimit(1, 0x03, 10, -10);
		BSP_M15_SetPosition(0);


    while (1)
    {    
        //m3508_can_tx();
				
			
			
			  gimbal_set_mode(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
			  //M15_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);

        CAN_cmd_gimbal1(gimbal_control.gimbal_d2006_motor.given_current, 
                        gimbal_control.gimbal_l2006_motor.given_current,
                        gimbal_control.gimbal_r2006_motor.given_current,
                        gimbal_control.gimbal_s2006_motor.given_current);
			
        CAN_cmd_gimbal2(gimbal_control.gimbal_j1_motor.given_current, 
                        gimbal_control.gimbal_j2_motor.given_current,
                        0, 0);
       
        vTaskDelay(10);
         
        //系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}		

const gimbal_motor_t *get_l2006_motor_point(void)
{
    return &gimbal_control.gimbal_l2006_motor;
}

const gimbal_motor_t *get_r2006_motor_point(void)
{
    return &gimbal_control.gimbal_r2006_motor;
}

const gimbal_motor_t *get_d2006_motor_point(void)
{
    return &gimbal_control.gimbal_d2006_motor;
}

const gimbal_motor_t *get_s2006_motor_point(void)
{
    return &gimbal_control.gimbal_s2006_motor;
}

const gimbal_motor_t *get_j1_motor_point(void)
{
    return &gimbal_control.gimbal_j1_motor;
}

const gimbal_motor_t *get_j2_motor_point(void)
{
    return &gimbal_control.gimbal_j2_motor;
}

static void gimbal_init(gimbal_control_t *gimbal_control_init)
{
    if (gimbal_control_init == NULL)
    {
        return;
    }

    //云台速度环pid值
    const static fp32 l2006_speed_pid[3] = {L2006_SPEED_PID_KP, L2006_SPEED_PID_KI, L2006_SPEED_PID_KD};
    const static fp32 r2006_speed_pid[3] = {R2006_SPEED_PID_KP, R2006_SPEED_PID_KI, R2006_SPEED_PID_KD};
    const static fp32 d2006_speed_pid[3] = {D2006_SPEED_PID_KP, D2006_SPEED_PID_KI, D2006_SPEED_PID_KD};
    const static fp32 s2006_speed_pid[3] = {S2006_SPEED_PID_KP, S2006_SPEED_PID_KI, S2006_SPEED_PID_KD};
    const static fp32 j1_speed_pid[3] = {J1_SPEED_PID_KP, J1_SPEED_PID_KI, J1_SPEED_PID_KD};
    const static fp32 j2_speed_pid[3] = {J2_SPEED_PID_KP, J2_SPEED_PID_KI, J2_SPEED_PID_KD};
    
    //云台开机状态为原始
    gimbal_control_init->gimbal_mode = GIMBAL_NORMAL_MODE;
		
    //获取遥控器指针
    gimbal_control_init->gimbal_rc_ctrl = get_remote_control_point();
		
    //电机数据指针获取
    gimbal_control_init->gimbal_j1_motor.gimbal_motor_measure = get_j1_gimbal_motor_measure_point();
    gimbal_control_init->gimbal_j2_motor.gimbal_motor_measure = get_j2_gimbal_motor_measure_point();
    gimbal_control_init->gimbal_l2006_motor.gimbal_motor_measure = get_l2006_motor_measure_point();
    gimbal_control_init->gimbal_r2006_motor.gimbal_motor_measure = get_r2006_motor_measure_point();
    gimbal_control_init->gimbal_d2006_motor.gimbal_motor_measure = get_d2006_motor_measure_point();
    gimbal_control_init->gimbal_s2006_motor.gimbal_motor_measure = get_s2006_motor_measure_point();

    //初始化速度环PID
    PID_init(&gimbal_control_init->motor_j1_speed_pid, PID_POSITION, j1_speed_pid, J1_SPEED_PID_MAX_OUT, J1_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_control_init->motor_j2_speed_pid, PID_POSITION, j2_speed_pid, J2_SPEED_PID_MAX_OUT, J2_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_control_init->motor_l2006_speed_pid, PID_POSITION, l2006_speed_pid, L2006_SPEED_PID_MAX_OUT, L2006_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_control_init->motor_r2006_speed_pid, PID_POSITION, r2006_speed_pid, R2006_SPEED_PID_MAX_OUT, R2006_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_control_init->motor_d2006_speed_pid, PID_POSITION, d2006_speed_pid, D2006_SPEED_PID_MAX_OUT, D2006_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_control_init->motor_s2006_speed_pid, PID_POSITION, s2006_speed_pid, S2006_SPEED_PID_MAX_OUT, S2006_SPEED_PID_MAX_IOUT);

    //更新一下数据
    gimbal_feedback_update(gimbal_control_init);
}

static void gimbal_set_mode(gimbal_control_t *gimbal_control_mode)
{
    if (gimbal_control_mode == NULL)
    {
        return;
    }
    
    //设置云台模式
    if (gimbal_control_mode->gimbal_rc_ctrl->rc.SA == 671)
    {
        gimbal_control_mode->gimbal_mode = GIMBAL_NORMAL_MODE;
    }
    else
    {
        gimbal_control_mode->gimbal_mode = GIMBAL_STOP_MODE;
    }
}

//云台测量数据更新
static void gimbal_feedback_update(gimbal_control_t *gimbal_control_update)
{
    if (gimbal_control_update == NULL)
    {
        return;
    }
    gimbal_control_update->gimbal_j1_motor.speed = gimbal_control_update->gimbal_j1_motor.gimbal_motor_measure->speed_rpm;
    gimbal_control_update->gimbal_j1_motor.accel = gimbal_control_update->motor_j1_speed_pid.Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
		
    gimbal_control_update->gimbal_j2_motor.speed = gimbal_control_update->gimbal_j2_motor.gimbal_motor_measure->speed_rpm;
    gimbal_control_update->gimbal_j2_motor.accel = gimbal_control_update->motor_j2_speed_pid.Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
		
    gimbal_control_update->gimbal_l2006_motor.speed = gimbal_control_update->gimbal_l2006_motor.gimbal_motor_measure->speed_rpm;
    gimbal_control_update->gimbal_l2006_motor.accel = gimbal_control_update->motor_l2006_speed_pid.Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
		
    gimbal_control_update->gimbal_r2006_motor.speed = gimbal_control_update->gimbal_r2006_motor.gimbal_motor_measure->speed_rpm;
    gimbal_control_update->gimbal_r2006_motor.accel = gimbal_control_update->motor_r2006_speed_pid.Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
		
    gimbal_control_update->gimbal_d2006_motor.speed = gimbal_control_update->gimbal_d2006_motor.gimbal_motor_measure->speed_rpm;
    gimbal_control_update->gimbal_d2006_motor.accel = gimbal_control_update->motor_d2006_speed_pid.Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
		
    gimbal_control_update->gimbal_s2006_motor.speed = gimbal_control_update->gimbal_s2006_motor.gimbal_motor_measure->speed_rpm;
    gimbal_control_update->gimbal_s2006_motor.accel = gimbal_control_update->motor_s2006_speed_pid.Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
}

//设置云台控制设置值
static void gimbal_set_control(gimbal_control_t *gimbal_control_control)
{
    if (gimbal_control_control == NULL)
    {
        return;
    }

    //停止模式
    if (gimbal_control_control->gimbal_mode == GIMBAL_STOP_MODE)
    {
        gimbal_control_control->gimbal_l2006_motor.speed_set = 0;
        gimbal_control_control->gimbal_r2006_motor.speed_set = 0;
        gimbal_control_control->gimbal_d2006_motor.speed_set = 0;
        gimbal_control_control->gimbal_s2006_motor.speed_set = 0;
        gimbal_control_control->gimbal_j1_motor.speed_set = 0;
        gimbal_control_control->gimbal_j2_motor.speed_set = 0;
        return;
    }
    else
		{
						//① PITCH end
						if (gimbal_control_control->gimbal_rc_ctrl->rc.Ch3 <-150)
						{
								gimbal_control_control->gimbal_l2006_motor.speed_set = LR2006_SPEED;
								gimbal_control_control->gimbal_r2006_motor.speed_set = LR2006_SPEED;
						}
						else if (gimbal_control_control->gimbal_rc_ctrl->rc.Ch3 >150)
						{
								gimbal_control_control->gimbal_l2006_motor.speed_set = -LR2006_SPEED;
								gimbal_control_control->gimbal_r2006_motor.speed_set = -LR2006_SPEED;
						}
						else 
						{   
							  						//xipan end
										if (gimbal_control_control->gimbal_rc_ctrl->rc.SB == 671)
										{
												gimbal_control_control->gimbal_l2006_motor.speed_set = 1.5*LR2006_SPEED;
												gimbal_control_control->gimbal_r2006_motor.speed_set = -1.5*LR2006_SPEED;
										}
										else if (gimbal_control_control->gimbal_rc_ctrl->rc.SB== -671)
										{
												gimbal_control_control->gimbal_l2006_motor.speed_set = -1.5*LR2006_SPEED;
												gimbal_control_control->gimbal_r2006_motor.speed_set = 1.5*LR2006_SPEED;
										}
										else if(gimbal_control_control->gimbal_rc_ctrl->rc.SB == 0)
										{
												gimbal_control_control->gimbal_l2006_motor.speed_set = 0;
												gimbal_control_control->gimbal_r2006_motor.speed_set = 0;
										}

						}
						

						//② D end66
						if (gimbal_control_control->gimbal_rc_ctrl->rc.Ch1 <-150)
						{
								gimbal_control_control->gimbal_d2006_motor.speed_set = D2006_SPEED;
						}
						else if (gimbal_control_control->gimbal_rc_ctrl->rc.Ch1 >150)
						{
								gimbal_control_control->gimbal_d2006_motor.speed_set = -D2006_SPEED;
						}
						else
						{
								gimbal_control_control->gimbal_d2006_motor.speed_set = 0;
						}


						//S end
						if (gimbal_control_control->gimbal_rc_ctrl->rc.RS < -150)
						{
								gimbal_control_control->gimbal_s2006_motor.speed_set = S2006_SPEED;
						}
						else if (gimbal_control_control->gimbal_rc_ctrl->rc.RS >150)
						{
								gimbal_control_control->gimbal_s2006_motor.speed_set = -S2006_SPEED;
						}
						else
						{
								gimbal_control_control->gimbal_s2006_motor.speed_set = 0;
						}

						
						//J1 J2 end
						if (gimbal_control_control->gimbal_rc_ctrl->rc.Ch2>150)
						{
								gimbal_control_control->gimbal_j2_motor.speed_set =gimbal_control_control->gimbal_rc_ctrl->rc.Ch2*5.962;
							  gimbal_control_control->gimbal_j1_motor.speed_set = -gimbal_control_control->gimbal_rc_ctrl->rc.Ch2*5.962;
						}
						else if (gimbal_control_control->gimbal_rc_ctrl->rc.Ch2 <-150)
						{
								gimbal_control_control->gimbal_j2_motor.speed_set = gimbal_control_control->gimbal_rc_ctrl->rc.Ch2*5.962;
							  gimbal_control_control->gimbal_j1_motor.speed_set = -gimbal_control_control->gimbal_rc_ctrl->rc.Ch2*5.962;
						}
						else
						{
							  gimbal_control_control->gimbal_j1_motor.speed_set = 0;
							  if(gimbal_control_control->gimbal_rc_ctrl->rc.LS <-150)
								{
								  gimbal_control_control->gimbal_j2_motor.speed_set =gimbal_control_control->gimbal_rc_ctrl->rc.LS*5.962; 
								
								}
								else if (gimbal_control_control->gimbal_rc_ctrl->rc.LS >150)
								{
								  gimbal_control_control->gimbal_j2_motor.speed_set =gimbal_control_control->gimbal_rc_ctrl->rc.LS*5.962; 
								
								}
								else
								{
								 gimbal_control_control->gimbal_j2_motor.speed_set = 0;
								}
						}

		}
}



//云台控制PID计算
static void gimbal_control_loop(gimbal_control_t *gimbal_control_control_loop)
{
    if (gimbal_control_control_loop == NULL)
    {
        return;
    }

    //计算L2006, R2006, D2006, S2006速度环PID
    PID_calc(&gimbal_control_control_loop->motor_l2006_speed_pid, 
             gimbal_control_control_loop->gimbal_l2006_motor.speed, 
             gimbal_control_control_loop->gimbal_l2006_motor.speed_set);

    PID_calc(&gimbal_control_control_loop->motor_r2006_speed_pid, 
             gimbal_control_control_loop->gimbal_r2006_motor.speed, 
             gimbal_control_control_loop->gimbal_r2006_motor.speed_set);

    PID_calc(&gimbal_control_control_loop->motor_d2006_speed_pid, 
             gimbal_control_control_loop->gimbal_d2006_motor.speed, 
             gimbal_control_control_loop->gimbal_d2006_motor.speed_set);

    PID_calc(&gimbal_control_control_loop->motor_s2006_speed_pid, 
             gimbal_control_control_loop->gimbal_s2006_motor.speed, 
             gimbal_control_control_loop->gimbal_s2006_motor.speed_set);

 
        //速度环控制
        PID_calc(&gimbal_control_control_loop->motor_j1_speed_pid, 
                 gimbal_control_control_loop->gimbal_j1_motor.speed, 
                 gimbal_control_control_loop->gimbal_j1_motor.speed_set);

        PID_calc(&gimbal_control_control_loop->motor_j2_speed_pid, 
                 gimbal_control_control_loop->gimbal_j2_motor.speed, 
                 gimbal_control_control_loop->gimbal_j2_motor.speed_set);
    

    //赋值电流值
    gimbal_control_control_loop->gimbal_l2006_motor.given_current = (int16_t)(gimbal_control_control_loop->motor_l2006_speed_pid.out);
    gimbal_control_control_loop->gimbal_r2006_motor.given_current = (int16_t)(gimbal_control_control_loop->motor_r2006_speed_pid.out);
    gimbal_control_control_loop->gimbal_d2006_motor.given_current = (int16_t)(gimbal_control_control_loop->motor_d2006_speed_pid.out);
    gimbal_control_control_loop->gimbal_s2006_motor.given_current = (int16_t)(gimbal_control_control_loop->motor_s2006_speed_pid.out);
    gimbal_control_control_loop->gimbal_j1_motor.given_current = (int16_t)(gimbal_control_control_loop->motor_j1_speed_pid.out);
    gimbal_control_control_loop->gimbal_j2_motor.given_current = (int16_t)(gimbal_control_control_loop->motor_j2_speed_pid.out);
}