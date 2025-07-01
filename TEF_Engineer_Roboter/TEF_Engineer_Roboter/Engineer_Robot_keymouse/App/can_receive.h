#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "main.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef enum
{
	  //底盘
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	  
	
	  //云台
    CAN_GIMBAL_ALL_ID1 = 0x1FF,
    CAN_DOWN_END_ID = 0x205,  //末端下2006电机
    CAN_LEFT_END_ID = 0x206,  //末端左2006电机
	  CAN_RIGHT_END_ID = 0x207, //末端右2006电机
	  CAN_STRETCH_ID = 0x208, //伸缩电机
	  
	  CAN_GIMBAL_ALL_ID2 = 0x200,
    CAN_J1_MOTOR_ID = 0x201,  //3508电机
	  CAN_J2_MOTOR_ID=0x202,   //3508电机
   

} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    uint16_t last_position;
    int16_t speed;
    int16_t position;
    uint8_t given_current;
} m15_measure_t;

extern m15_measure_t M15_DATA;

extern motor_measure_t CHASSIS_DATAMOTOR1; //底盘数据
extern motor_measure_t CHASSIS_DATAMOTOR2;
extern motor_measure_t CHASSIS_DATAMOTOR3;
extern motor_measure_t CHASSIS_DATAMOTOR4;

extern motor_measure_t RIGHT_DATA2006;//末端执行器三个2006数据
extern motor_measure_t LEFT_DATA2006;
extern motor_measure_t DOWN_DATA2006;

extern motor_measure_t J1_DATA3508;//机械臂第一级3508电机数据
extern motor_measure_t J2_DATA3508;
		
extern motor_measure_t STRETCH_DATA2006;//伸缩结构2006电机数据


extern void CAN_cmd_gimbal1( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006);
extern void CAN_cmd_gimbal2( int16_t J1, int16_t J2, int16_t rev2,int16_t rev3);

extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const motor_measure_t *get_j1_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_j2_gimbal_motor_measure_point(void);


extern const motor_measure_t *get_l2006_motor_measure_point(void);
extern const motor_measure_t *get_r2006_motor_measure_point(void);
extern const motor_measure_t *get_d2006_motor_measure_point(void);
extern const motor_measure_t *get_s2006_motor_measure_point(void);

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
