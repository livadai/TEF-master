#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "can_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

#define PI 3.1415926

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//遥控器前进摇杆（max 671）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.003f

//遥控器左右摇杆（max 671）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.003f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 671）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//不跟随云台的时候 遥控器的yaw遥杆（max 671）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f

//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f

//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


typedef enum
{
  
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       // 底盘有旋转速度控制，底盘移动速度和旋转速度由遥控器决定，无角度环控制
  CHASSIS_VECTOR_RAW,                 //底盘电机电流控制值是直接由遥控器通道值计算出来的，将直接发送到CAN总线上

} chassis_mode_e; //

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               
  chassis_mode_e chassis_mode;               
  chassis_mode_e last_chassis_mode;          
  chassis_motor_t motor_chassis[4];          
  pid_type_def motor_speed_pid[4];             
  pid_type_def chassis_angle_pid;             

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值

  fp32 vx;  //单位 m/s
  fp32 vy;                          
  fp32 wz; //底盘旋转角速度，逆时针为正 
  fp32 vx_set;                    
  fp32 vy_set;                      
  fp32 wz_set;                      
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  
  fp32 vx_min_speed;  //后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //左方向最大速度 单位m/s
  fp32 vy_min_speed;  //右方向最大速度 单位m/s

} chassis_move_t;

extern void chassis_task(void const *pvParameters);

//根据遥控器通道值，计算纵向和横移速度
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
