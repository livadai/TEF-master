#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "can_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 500

#define ANGLE_FACTOR 0.003 //旋钮转化为角度值比例

//云台电机速度宏定义
#define M15_SPEED 10
#define J1_SPEED  4000    // J1 3508电机
#define J2_SPEED  4000    // J2 3508电机
#define LR2006_SPEED 10000 // 2006 电机速度末端执行器左右
#define D2006_SPEED 1500  // 2006 末端执行器下
#define S2006_SPEED  6000 // 伸缩2006

#define M15_MAX_SPEED 50  //限制M15最大速度

//L2006 速度环 PID参数以及 PID最大输出，积分输出
#define L2006_SPEED_PID_KP        10.0f
#define L2006_SPEED_PID_KI        0.0f
#define L2006_SPEED_PID_KD        0.0f
#define L2006_SPEED_PID_MAX_OUT   16000.0f
#define L2006_SPEED_PID_MAX_IOUT  2000.0f

//R2006 速度环 PID参数以及 PID最大输出，积分输出
#define R2006_SPEED_PID_KP        10.0f
#define R2006_SPEED_PID_KI        0.0f
#define R2006_SPEED_PID_KD        0.0f
#define R2006_SPEED_PID_MAX_OUT   16000.0f
#define R2006_SPEED_PID_MAX_IOUT  2000.0f

//D2006 速度环 PID参数以及 PID最大输出，积分输出
#define D2006_SPEED_PID_KP        10.0f
#define D2006_SPEED_PID_KI        0.0f
#define D2006_SPEED_PID_KD        0.0f
#define D2006_SPEED_PID_MAX_OUT   16000.0f
#define D2006_SPEED_PID_MAX_IOUT  2000.0f

//S2006 速度环 PID参数以及 PID最大输出，积分输出
#define S2006_SPEED_PID_KP        10.0f
#define S2006_SPEED_PID_KI        0.0f
#define S2006_SPEED_PID_KD        0.0f
#define S2006_SPEED_PID_MAX_OUT   16000.0f
#define S2006_SPEED_PID_MAX_IOUT  2000.0f

//J1 速度环 PID参数以及 PID最大输出，积分输出   
#define J1_SPEED_PID_KP        10.0f     
#define J1_SPEED_PID_KI        0.0f
#define J1_SPEED_PID_KD        0.0f
#define J1_SPEED_PID_MAX_OUT   16000.0f
#define J1_SPEED_PID_MAX_IOUT  2000.0f

//J2 速度环 PID参数以及 PID最大输出，积分输出
#define J2_SPEED_PID_KP        10.0f  
#define J2_SPEED_PID_KI        0.0f
#define J2_SPEED_PID_KD        0.0f
#define J2_SPEED_PID_MAX_OUT   16000.0f
#define J2_SPEED_PID_MAX_IOUT  2000.0f

// 遥控器死区范围
#define RC_DEAD_ZONE 10

//云台任务控制频率
#define GIMBAL_CONTROL_FREQUENCE 500.0f

#define GIMBAL_CONTROL_TIME_MS 2
#define GIMBAL_CONTROL_TIME 0.002f

#define TURN_ONE CH1 //掉头存矿处1开关
#define TURN_TWO CH2 //掉头存矿处2开关
#define TURN_ZERO CH3 //掉头回零开关

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
  
  GIMBAL_NORMAL_MODE,       
  GIMBAL_STOP_MODE,                 

} gimbal_mode_e; 


typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t given_current;
} gimbal_motor_t;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{   
	  gimbal_mode_e gimbal_mode;
	  const RC_ctrl_t *gimbal_rc_ctrl; 
    gimbal_motor_t gimbal_j1_motor;
	  gimbal_motor_t gimbal_j2_motor;
	  gimbal_motor_t gimbal_l2006_motor;
	  gimbal_motor_t gimbal_r2006_motor;
	  gimbal_motor_t gimbal_d2006_motor;
	  gimbal_motor_t gimbal_s2006_motor;
	
	  pid_type_def motor_j1_speed_pid;
	  pid_type_def motor_j2_speed_pid;
	  pid_type_def motor_l2006_speed_pid;
	  pid_type_def motor_r2006_speed_pid;
	  pid_type_def motor_d2006_speed_pid;
	  pid_type_def motor_s2006_speed_pid;
	
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
	
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad

} gimbal_control_t;


extern const gimbal_motor_t *get_l2006_motor_point(void);
extern const gimbal_motor_t *get_r2006_motor_point(void);
extern const gimbal_motor_t *get_d2006_motor_point(void);
extern const gimbal_motor_t *get_s2006_motor_point(void);
extern const gimbal_motor_t *get_j1_motor_point(void);
extern const gimbal_motor_t *get_j2_motor_point(void);

// 云台控制任务
void gimbal_task(void const *pvParameters);

#endif

