#ifndef __3508_ANGLE_H
#define __3508_ANGLE_H

#include "CAN_receive.h"
#include <stdint.h>
#include <math.h>

// 机械参数
#define M3508_GEAR_RATIO      (3591.0f *50/ 187.0f)  // 减速比 3591/187 ≈ 19.2:1
#define M3508_ECD_MAX         8191                // 单圈编码器最大值
#define M3508_ECD_RANGE       8192                // 编码器数值范围（8192个值）
#define MAX_ANGLE_RANGE       270.0f              // 输出轴最大允许转动角度（±270°）
#define MAX_CURRENT           10000               // 最大允许电流（mA）

// J1_PID参数
#define ANGLE1_KP              15.9f               // 位置环比例系数
#define ANGLE1_KI              0.0f                // 位置环积分系数
#define ANGLE1_KD              0.0f                // 位置环微分系数
#define ANGLE1_MAX_OUT         30.0f               // 位置环最大输出（°/s）

#define SPEED1_KP              27.2f             // 速度环比例系数
#define SPEED1_KI              0.0f               // 速度环积分系数
#define SPEED1_KD              0.0f               // 速度环微分系数
#define SPEED1_MAX_OUT         10000               // 速度环最大输出（mA）

// J2_PID参数
#define ANGLE2_KP              15.9f               // 位置环比例系数
#define ANGLE2_KI              0.0f                // 位置环积分系数
#define ANGLE2_KD              0.0f                // 位置环微分系数
#define ANGLE2_MAX_OUT         30.0f               // 位置环最大输出（°/s）

#define SPEED2_KP              27.2f             // 速度环比例系数
#define SPEED2_KI              0.0f               // 速度环积分系数
#define SPEED2_KD              0.0f               // 速度环微分系数
#define SPEED2_MAX_OUT         10000               // 速度环最大输出（mA）

// 串级PID结构体
typedef struct {
    float kp, ki, kd;         // PID参数
    float integral;           // 积分项
    float prev_error;         // 上次误差
    float output;             // 输出值
    float output_limit;       // 输出限幅
    float integral_limit;     // 积分限幅
} cascade_pid_t;

// 电机控制结构体
typedef struct {
    // 电机参数
    const motor_measure_t *motor_measure;
    float gear_ratio_inv;                         // 减速比倒数（1/19.2）
    
    // 编码器累计
    uint16_t last_ecd;                            // 上次编码器值
    int32_t total_ecd_delta;                      // 累计编码器变化量（多圈累计）
    
    // 控制环
    cascade_pid_t angle_pid;                      // 位置环PID
    cascade_pid_t speed_pid;                      // 速度环PID
    
    // 状态变量
    float target_angle;                           // 目标角度（相对零点，±270°）
    float current_angle;                          // 当前角度（相对零点，±270°）
    float target_speed;                           // 目标速度（°/s）
    float current_speed;                          // 当前速度（°/s）
	
		double start_angle ;
    
    // 输出控制
    int16_t given_current;                        // 最终输出电流
} m3508_control_t;

extern m3508_control_t ctrl_J1;
extern m3508_control_t ctrl_J2;

// 函数声明
void m3508_init(void);
void m3508_update_target_angle(void);
void m3508_control_loop_J1(void);
void m3508_control_loop_J2(void);
void m3508_can_tx(void);

#endif