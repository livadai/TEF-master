#include "3508_angle.h"
#include "can_receive.h"
#include "user_lib.h"
#include <math.h>
#include "remote_control.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "gimbal_task.h"
#include "can_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "3508_angle.h"
#include "stdio.h"
#include "bsp_usart.h"
#include "usart.h"
#include "TwoDOF_ArmIK.h"

 m3508_control_t ctrl_J1;
 m3508_control_t ctrl_J2;

const RC_ctrl_t *rc_ctrl_angle;

static void cascade_pid_init(cascade_pid_t *pid, float kp, float ki, float kd, 
                            float output_limit, float integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
}

static float cascade_pid_calculate(cascade_pid_t *pid, float setpoint, float feedback, float dt) {
    // �������
    float error = setpoint - feedback;
    
    // ��������㣨���޷���
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    // ΢������㣨���ò���ȫ΢�֣�
    float derivative = (error - pid->prev_error) / dt;
    
    // PID���
    pid->output = pid->kp * error + 
                 pid->ki * pid->integral + 
                 pid->kd * derivative;
    
    // ����޷�
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }
    
    // �������
    pid->prev_error = error;
    
    return pid->output;
}

static inline int16_t ecd_delta(uint16_t new_ecd, uint16_t last_ecd) {
    int32_t delta = (int32_t)new_ecd - (int32_t)last_ecd;
    const int32_t half_range = M3508_ECD_RANGE / 2;
    
    if (delta > half_range) {
        delta -= M3508_ECD_RANGE;  // ��ת�������
    } else if (delta < -half_range) {
        delta += M3508_ECD_RANGE;  // ��ת�������
    }
    return (int16_t)delta;
}

//-----------------------------------------------------
// ��ʼ������
//-----------------------------------------------------
void m3508_init(void) {
    // ��ʼ��Ӳ������
    ctrl_J1.motor_measure = get_j1_gimbal_motor_measure_point();
    ctrl_J1.gear_ratio_inv = 1.0f / M3508_GEAR_RATIO;  // ���ٱȵ�����1/19.2��
	
	  ctrl_J2.motor_measure = get_j2_gimbal_motor_measure_point();
    ctrl_J2.gear_ratio_inv = 1.0f / M3508_GEAR_RATIO;  // ���ٱȵ�����1/19.2��
	
	  ctrl_J1.start_angle= 90.0;   // ��ʼ��1
    ctrl_J2.start_angle= 90.0; // ��ʼ��2
    
    // ��ʼ��PID������
    cascade_pid_init(&ctrl_J1.angle_pid, 
                    ANGLE1_KP, ANGLE1_KI, ANGLE1_KD, 
                    ANGLE1_MAX_OUT,   // ����޷���30��/s
                    ANGLE1_MAX_OUT);  // �����޷�
    
    cascade_pid_init(&ctrl_J1.speed_pid, 
                    SPEED1_KP, SPEED1_KI, SPEED1_KD, 
                    SPEED1_MAX_OUT,   // ����޷���10000mA
                    SPEED1_MAX_OUT);  // �����޷�
	
	  cascade_pid_init(&ctrl_J2.angle_pid, 
                    ANGLE2_KP, ANGLE2_KI, ANGLE2_KD, 
                    ANGLE2_MAX_OUT,   // ����޷���30��/s
                    ANGLE2_MAX_OUT);  // �����޷�
    
    cascade_pid_init(&ctrl_J2.speed_pid, 
                    SPEED2_KP, SPEED2_KI, SPEED2_KD, 
                    SPEED2_MAX_OUT,   // ����޷���10000mA
                    SPEED2_MAX_OUT);  // �����޷�
    
    // ��ʼ���������ۼ�
    ctrl_J1.last_ecd = ctrl_J1.motor_measure->ecd;
    ctrl_J1.total_ecd_delta = 0;
    
	 // ��ʼ���������ۼ�
    ctrl_J2.last_ecd = ctrl_J2.motor_measure->ecd;
    ctrl_J2.total_ecd_delta = 0;
		
    // ��ʼ��״̬����
    ctrl_J1.target_angle = 0.0f;
    ctrl_J1.current_angle = 0.0f;
    ctrl_J1.target_speed = 0.0f;
    ctrl_J1.current_speed = 0.0f;
    ctrl_J1.given_current = 0;
		
		// ��ʼ��״̬����
    ctrl_J2.target_angle = 0.0f;
    ctrl_J2.current_angle = 0.0f;
    ctrl_J2.target_speed = 0.0f;
    ctrl_J2.current_speed = 0.0f;
    ctrl_J2.given_current = 0;
		
		
}

//void rc_deadband_limit1(int16_t input, int16_t *output, int16_t deadband) {
//    if (abs(input) < deadband) {
//        *output = 0;  // �������ڣ����Ϊ0
//    } 
//		else if(input < -500){
//		    *output = -500;
//		}
//		else if(input > 500){
//		    *output = 500;
//		}
//		else {
//        *output = input; // ����������ֱ�����ԭʼֵ
//    }
//}

void m3508_update_target_angle(void) {
    static int16_t last_joystick_value_J1 = 0;  // ������һ�ε�ҡ��ֵ
    int16_t joystick_channel_J1 = 0;            // ���ڴ洢ҡ��ͨ��ֵ
	
	  static int16_t last_joystick_value_J2 = 0;  // ������һ�ε�ҡ��ֵ
    int16_t joystick_channel_J2 = 0;            // ���ڴ洢ҡ��ͨ��ֵ

    // ��ȡ��ǰҡ��ֵ
    rc_ctrl_angle = get_remote_control_point();

    // ��ҡ��ֵ������������
    rc_deadband_limit1(rc_ctrl_angle->rc.Ch2, &joystick_channel_J1, 10);
	
	// ��ҡ��ֵ������������
    rc_deadband_limit1(rc_ctrl_angle->rc.Ch1, &joystick_channel_J2, 10);


    // ����ҡ��ֵ�ı仯��
    float delta_joystick_J1 = (joystick_channel_J1 - 0) * 0.0002f;

    // ���ҡ��ֵ�仯����һ����ֵ������΢С������
    if (fabsf(delta_joystick_J1) > 0.05f) {
        // ����ҡ��ֵ�ı仯������Ŀ��Ƕ�
        ctrl_J1.target_angle += delta_joystick_J1 ;  // ANGLE_INCREMENT ��ÿ�α仯�ĽǶ�����
    }
		
		    // ����ҡ��ֵ�ı仯��
    float delta_joystick_J2 = (joystick_channel_J2 - 0) * 0.0002f;

    // ���ҡ��ֵ�仯����һ����ֵ������΢С������
    if (fabsf(delta_joystick_J2) > 0.05f) {
        // ����ҡ��ֵ�ı仯������Ŀ��Ƕ�
        ctrl_J2.target_angle += delta_joystick_J2 ;  // ANGLE_INCREMENT ��ÿ�α仯�ĽǶ�����
    }
		
//		m3508_can_tx();
		
    // ���浱ǰҡ��ֵ
    last_joystick_value_J1 = joystick_channel_J1;
    // ���浱ǰҡ��ֵ
    last_joystick_value_J2 = joystick_channel_J2;

    // ����Ŀ��Ƕ��ںϷ���Χ��
    if (ctrl_J1.target_angle > MAX_ANGLE_RANGE) {
        ctrl_J1.target_angle = MAX_ANGLE_RANGE;
    } else if (ctrl_J1.target_angle < -MAX_ANGLE_RANGE) {
        ctrl_J1.target_angle = -MAX_ANGLE_RANGE;
    }
		    // ����Ŀ��Ƕ��ںϷ���Χ��
    if (ctrl_J2.target_angle > MAX_ANGLE_RANGE) {
        ctrl_J2.target_angle = MAX_ANGLE_RANGE;
    } else if (ctrl_J2.target_angle < -MAX_ANGLE_RANGE) {
        ctrl_J2.target_angle = -MAX_ANGLE_RANGE;
    }
}

void m3508_control_loop_J1(void) {
	
    static uint32_t last_tick = 0;
    float dt = (HAL_GetTick() - last_tick) / 1000.0f;
    last_tick = HAL_GetTick();
	
    double delta_angle1, delta_angle2;
    if (m3508_update_target_position(&delta_angle1, &delta_angle2) == 0) {
        // ���� 3508 �����Ŀ��Ƕ�
        ctrl_J1.target_angle += delta_angle1;
    } else {
        printf("�� P ���ɴ�������ʧ��\n");
    }
    
    // 2. ���㵱ǰ�Ƕȣ�����ᣩ
    uint16_t current_ecd_J1 = ctrl_J1.motor_measure->ecd;
    int16_t delta_J1 = ecd_delta(current_ecd_J1, ctrl_J1.last_ecd);
    ctrl_J1.total_ecd_delta += delta_J1;  // �ۼ��ܱ仯��
    ctrl_J1.last_ecd = current_ecd_J1;
    
    ctrl_J1.current_angle = (ctrl_J1.total_ecd_delta / (float)M3508_ECD_RANGE) * 360.0f * ctrl_J1.gear_ratio_inv;
    
    
    // 3. λ�û����⻷������Ŀ���ٶ�
    ctrl_J1.target_speed = cascade_pid_calculate(
        &ctrl_J1.angle_pid, 
        ctrl_J1.target_angle, 
        ctrl_J1.current_angle, 
        dt
    );
    
    // 4. �ٶȻ����ڻ�������Ŀ�����
    ctrl_J1.current_speed = (ctrl_J1.motor_measure->speed_rpm / 60.0f) * 360.0f * ctrl_J1.gear_ratio_inv; // ת��Ϊ��/s
    float target_current_J1 = cascade_pid_calculate(
        &ctrl_J1.speed_pid, 
        ctrl_J1.target_speed, 
        ctrl_J1.current_speed, 
        dt
    );
    
    // 5. ��������޷�
    if (target_current_J1 > MAX_CURRENT) {
        target_current_J1 = MAX_CURRENT;
    } else if (target_current_J1 < -MAX_CURRENT) {
        target_current_J1 = -MAX_CURRENT;
    }
    ctrl_J1.given_current = (int16_t)target_current_J1;

}

void m3508_control_loop_J2(void) {
    // 1. ����ʱ�����������������Ϊ10ms��
    static uint32_t last_tick = 0;
    float dt = (HAL_GetTick() - last_tick) / 1000.0f;
    last_tick = HAL_GetTick();
    
	
	   double delta_angle1, delta_angle2;
    if (m3508_update_target_position(&delta_angle1, &delta_angle2) == 0) {
        // ���� 3508 �����Ŀ��Ƕ�
        ctrl_J2.target_angle += delta_angle2;
    } else {
        printf("�� P ���ɴ�������ʧ��\n");
    }
    // 2. ���㵱ǰ�Ƕȣ�����ᣩ
    uint16_t current_ecd_J2 = ctrl_J2.motor_measure->ecd;
    int16_t delta_J2 = ecd_delta(current_ecd_J2, ctrl_J2.last_ecd);
    ctrl_J2.total_ecd_delta += delta_J2;  // �ۼ��ܱ仯��
    ctrl_J2.last_ecd = current_ecd_J2;
    
    ctrl_J2.current_angle = (ctrl_J2.total_ecd_delta / (float)M3508_ECD_RANGE) * 360.0f * ctrl_J2.gear_ratio_inv;
    
    
    // 3. λ�û����⻷������Ŀ���ٶ�
    ctrl_J2.target_speed = cascade_pid_calculate(
        &ctrl_J2.angle_pid, 
        ctrl_J2.target_angle, 
        ctrl_J2.current_angle, 
        dt
    );
    
    // 4. �ٶȻ����ڻ�������Ŀ�����
    ctrl_J2.current_speed = (ctrl_J2.motor_measure->speed_rpm / 60.0f) * 360.0f * ctrl_J2.gear_ratio_inv; // ת��Ϊ��/s
    float target_current_J2 = cascade_pid_calculate(
        &ctrl_J2.speed_pid, 
        ctrl_J2.target_speed, 
        ctrl_J2.current_speed, 
        dt
    );
    
    // 5. ��������޷�
    if (target_current_J2 > MAX_CURRENT) {
        target_current_J2 = MAX_CURRENT;
    } else if (target_current_J2 < -MAX_CURRENT) {
        target_current_J2 = -MAX_CURRENT;
    }
    ctrl_J2.given_current = (int16_t)target_current_J2;
}

void m3508_can_tx(void){
	   m3508_control_loop_J2();
	   m3508_control_loop_J1();
		 CAN_cmd_gimbal2( ctrl_J1.given_current,ctrl_J2.given_current, 0,0);
}

float m3508_get_current_angle(void) {
    return ctrl_J1.current_angle;
	  return ctrl_J2.current_angle;
}