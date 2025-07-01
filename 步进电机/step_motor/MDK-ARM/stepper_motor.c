#include "stepper_motor.h"

// ����ȫ�ֱ���
UART_HandleTypeDef *huart_motor;
MotorState motor_state = MOTOR_IDLE;

// ��ʼ�������������
void StepperMotor_Init(UART_HandleTypeDef *huart) {
    huart_motor = huart;
    motor_state = MOTOR_IDLE;
}

// ����λ��ģʽ
void StepperMotor_SetPosition(uint8_t direction, uint16_t speed, uint32_t pulse_count) {
    uint8_t cmd[13];
    cmd[0] = 0x01;
    cmd[1] = CMD_POSITION_CONTROL;
    cmd[2] = direction;
    cmd[3] = (speed >> 8) & 0xFF;
    cmd[4] = speed & 0xFF;
    cmd[5] = 0x00;
    cmd[6] = (pulse_count >> 24) & 0xFF;
    cmd[7] = (pulse_count >> 16) & 0xFF;
    cmd[8] = (pulse_count >> 8) & 0xFF;
    cmd[9] = pulse_count & 0xFF;
    cmd[10] = 0x01;
    cmd[11] = 0x00;
    cmd[12] = 0x6B; // У���ֽ�

    HAL_UART_Transmit(huart_motor, cmd, sizeof(cmd), HAL_MAX_DELAY);
    motor_state = MOTOR_RUNNING;
}

// ���������״̬
void StepperMotor_Process(void) {
    // ����������״̬���ʹ������߼�
    if (motor_state == MOTOR_RUNNING) {
        // ������Ƿ�����˶�
        // �����ɣ����� motor_state Ϊ MOTOR_IDLE
    }
}