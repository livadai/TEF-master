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
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);

#endif
