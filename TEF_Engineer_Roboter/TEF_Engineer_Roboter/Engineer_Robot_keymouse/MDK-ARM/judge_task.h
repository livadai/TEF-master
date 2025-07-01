#include "remote_control.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "main.h"
#include "stdio.h"

#define JUDGE_TASK_INIT_TIME 5000

void judge_task(void const *pvParameters);
void Parse_KeyMouse_Data(uint8_t *data);

extern uint8_t rx_data[1];