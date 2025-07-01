#include "chassis_task.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "pid.h"
#include "cmsis_os.h"
#include <stdlib.h>  
#include <math.h>  
#include "chassis_behaviour.h"
#include "bsp_m15_motor.h"
#include "bsp_usart.h"
#include "usart.h"
#include "stdio.h"

#define PI 3.1415926

//��������
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//ջ�����ˮλ��
#if INCLUDE_uxTaskGetStackHighWaterMark	
uint32_t chassis_high_water;
#endif	
		
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);


//�����˶�����
chassis_move_t chassis_move;

//int fputc(int ch,FILE *f)
//{
//	uint8_t temp[1] = {ch};
//	HAL_UART_Transmit(&huart6,temp,1,2);
//	return ch;
//}		
		
void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
   
    chassis_init(&chassis_move);
	
			  BSP_M15_SetMode(MODE_POSITION_LOOP);
		    BSP_M15_SetPID(1, 0x03, 0x01, 0, 0, 0);
        BSP_M15_SavePID(1);
        BSP_M15_SetPositionLimit(1, 0x03, 10, -10);
			  BSP_M15_SetPosition(0);
    

    while (1)
    {    
        
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
       
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
     
        //���̿���������
        chassis_set_contorl(&chassis_move);
			 
       
        //���̿���PID����
        chassis_control_loop(&chassis_move);

        //���Ϳ��Ƶ���
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
        
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}		
		
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;

    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();

    //��ȡ���̵������ָ�룬��ʼ��PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }

    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);


    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //����һ������
    chassis_feedback_update(chassis_move_init);
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    
    chassis_behaviour_mode_set(chassis_move_mode);
}

//����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}
		

//���̲������ݸ���
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

}


//����ң����ͨ��ֵ����������ͺ����ٶ�
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.Ch2, vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.Ch4, vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
    vy_set_channel = -vy_channel * -CHASSIS_VY_RC_SEN;

    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
		

//���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);


    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        
        //��angle_set�� ����ת�ٶȿ���
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        //��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}
		

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}
		
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
  
        //raw����ֱ�ӷ���
        return;
    }

    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //����pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
		