#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "can_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

#define PI 3.1415926

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//ң����ǰ��ҡ�ˣ�max 671��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.003f

//ң��������ҡ�ˣ�max 671��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.003f

//�������yawģʽ�£�ң������yawң�ˣ�max 671�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//��������̨��ʱ�� ң������yawң�ˣ�max 671��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//ҡ������
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//�����������Ƶ��
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f

//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f

//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


typedef enum
{
  
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       // ��������ת�ٶȿ��ƣ������ƶ��ٶȺ���ת�ٶ���ң�����������޽ǶȻ�����
  CHASSIS_VECTOR_RAW,                 //���̵����������ֵ��ֱ����ң����ͨ��ֵ��������ģ���ֱ�ӷ��͵�CAN������

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

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx;  //��λ m/s
  fp32 vy;                          
  fp32 wz; //������ת���ٶȣ���ʱ��Ϊ�� 
  fp32 vx_set;                    
  fp32 vy_set;                      
  fp32 wz_set;                      
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  
  fp32 vx_min_speed;  //���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //��������ٶ� ��λm/s
  fp32 vy_min_speed;  //�ҷ�������ٶ� ��λm/s

} chassis_move_t;

extern void chassis_task(void const *pvParameters);

//����ң����ͨ��ֵ����������ͺ����ٶ�
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
