#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);

#endif
