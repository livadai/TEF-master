//遥控器的值判断底盘行为

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "bsp_m15_motor.h"

#define PI 3.1415926

#define rc_deadband_limit1(input, output, dealine)        \
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

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

//底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

//留意，这个底盘行为模式变量
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //遥控器设置模式
    if (chassis_move_mode->chassis_RC->rc.SA==0)
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;//无力
    }
    else if (chassis_move_mode->chassis_RC->rc.SA==671)//下拨
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;//速度值为0，有电流有阻力
    }
    else if (chassis_move_mode->chassis_RC->rc.SA==-671)//上拨
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;//底盘存在电机速度环
			  
    }

    //根据行为模式选择一个底盘控制模式
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }

}


//设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
 
}


//底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

//底盘不移动的行为状态机下，底盘模式是不跟随角度，
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
		
		int16_t vy_value;
		rc_deadband_limit1(chassis_move_rc_to_vector->chassis_RC->rc.Ch4, vy_value, CHASSIS_RC_DEADLINE);
    *vx_set = 0.0f;
    *vy_set = -vy_value* -CHASSIS_VY_RC_SEN;
    *wz_set = 0.0f;
}


//底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.Ch1;
}

