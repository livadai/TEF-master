#ifndef TWODOF_ARMIK_H
#define TWODOF_ARMIK_H

#include <math.h>
#include <stdio.h>
#include "struct_typedef.h"

#define M_PI 3.1415926

// 机械臂参数
#define OA 33.0  // OA 的长度
#define AP 57.0  // AP 的长度

#define MAX_RANGE_X  50
#define MAX_RANGE_Y  50

// 正解算：根据角1和角2计算点P的坐标
void forward_kinematics(double angle1, double angle2, double *Px, double *Py);

// 逆解算：根据点P的坐标计算角1和角2
int inverse_kinematics(double Px, double Py, double *angle1, double *angle2);

// 遥控器控制：根据通道值计算电机角度变化量
int m3508_update_target_position(double *delta_angle1, double *delta_angle2);

void rc_deadband_limit1(int16_t input, int16_t *output, int16_t deadband);

#endif 


