#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"

typedef enum
{
  CHASSIS_ZERO_FORCE,                   //电流值为0，底盘无力, 跟没上电那样
  CHASSIS_NO_MOVE,                      //有电流但无速度，底盘保持不动
  CHASSIS_NO_FOLLOW_YAW,                //底盘不跟随角度，但轮子是有速度环
} chassis_behaviour_e;

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
