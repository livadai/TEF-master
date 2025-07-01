#ifndef __3508_ANGLE_H
#define __3508_ANGLE_H

#include "CAN_receive.h"
#include <stdint.h>
#include <math.h>

// ��е����
#define M3508_GEAR_RATIO      (3591.0f *50/ 187.0f)  // ���ٱ� 3591/187 �� 19.2:1
#define M3508_ECD_MAX         8191                // ��Ȧ���������ֵ
#define M3508_ECD_RANGE       8192                // ��������ֵ��Χ��8192��ֵ��
#define MAX_ANGLE_RANGE       270.0f              // ������������ת���Ƕȣ���270�㣩
#define MAX_CURRENT           10000               // ������������mA��

// J1_PID����
#define ANGLE1_KP              15.9f               // λ�û�����ϵ��
#define ANGLE1_KI              0.0f                // λ�û�����ϵ��
#define ANGLE1_KD              0.0f                // λ�û�΢��ϵ��
#define ANGLE1_MAX_OUT         30.0f               // λ�û�����������/s��

#define SPEED1_KP              27.2f             // �ٶȻ�����ϵ��
#define SPEED1_KI              0.0f               // �ٶȻ�����ϵ��
#define SPEED1_KD              0.0f               // �ٶȻ�΢��ϵ��
#define SPEED1_MAX_OUT         10000               // �ٶȻ���������mA��

// J2_PID����
#define ANGLE2_KP              15.9f               // λ�û�����ϵ��
#define ANGLE2_KI              0.0f                // λ�û�����ϵ��
#define ANGLE2_KD              0.0f                // λ�û�΢��ϵ��
#define ANGLE2_MAX_OUT         30.0f               // λ�û�����������/s��

#define SPEED2_KP              27.2f             // �ٶȻ�����ϵ��
#define SPEED2_KI              0.0f               // �ٶȻ�����ϵ��
#define SPEED2_KD              0.0f               // �ٶȻ�΢��ϵ��
#define SPEED2_MAX_OUT         10000               // �ٶȻ���������mA��

// ����PID�ṹ��
typedef struct {
    float kp, ki, kd;         // PID����
    float integral;           // ������
    float prev_error;         // �ϴ����
    float output;             // ���ֵ
    float output_limit;       // ����޷�
    float integral_limit;     // �����޷�
} cascade_pid_t;

// ������ƽṹ��
typedef struct {
    // �������
    const motor_measure_t *motor_measure;
    float gear_ratio_inv;                         // ���ٱȵ�����1/19.2��
    
    // �������ۼ�
    uint16_t last_ecd;                            // �ϴα�����ֵ
    int32_t total_ecd_delta;                      // �ۼƱ������仯������Ȧ�ۼƣ�
    
    // ���ƻ�
    cascade_pid_t angle_pid;                      // λ�û�PID
    cascade_pid_t speed_pid;                      // �ٶȻ�PID
    
    // ״̬����
    float target_angle;                           // Ŀ��Ƕȣ������㣬��270�㣩
    float current_angle;                          // ��ǰ�Ƕȣ������㣬��270�㣩
    float target_speed;                           // Ŀ���ٶȣ���/s��
    float current_speed;                          // ��ǰ�ٶȣ���/s��
	
		double start_angle ;
    
    // �������
    int16_t given_current;                        // �����������
} m3508_control_t;

extern m3508_control_t ctrl_J1;
extern m3508_control_t ctrl_J2;

// ��������
void m3508_init(void);
void m3508_update_target_angle(void);
void m3508_control_loop_J1(void);
void m3508_control_loop_J2(void);
void m3508_can_tx(void);

#endif