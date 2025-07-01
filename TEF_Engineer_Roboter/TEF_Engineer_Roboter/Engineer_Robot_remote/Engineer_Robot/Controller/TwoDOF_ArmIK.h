#ifndef TWODOF_ARMIK_H
#define TWODOF_ARMIK_H

#include <math.h>
#include <stdio.h>
#include "struct_typedef.h"

#define M_PI 3.1415926

// ��е�۲���
#define OA 33.0  // OA �ĳ���
#define AP 57.0  // AP �ĳ���

#define MAX_RANGE_X  50
#define MAX_RANGE_Y  50

// �����㣺���ݽ�1�ͽ�2�����P������
void forward_kinematics(double angle1, double angle2, double *Px, double *Py);

// ����㣺���ݵ�P����������1�ͽ�2
int inverse_kinematics(double Px, double Py, double *angle1, double *angle2);

// ң�������ƣ�����ͨ��ֵ�������Ƕȱ仯��
int m3508_update_target_position(double *delta_angle1, double *delta_angle2);

void rc_deadband_limit1(int16_t input, int16_t *output, int16_t deadband);

#endif 


