#include "custom_interrupts.h"

// ȫ�ֱ�������
Bluetooth_HandleTypeDef hbt;
volatile uint8_t led_state = 0;

void Custom_Interrupts_Init(void)
{
    // ������ʱ���ж�
    HAL_TIM_Base_Start_IT(&htim2);
    
    // ��ʼ������ģ��
    BT_Init(&hbt, &huart6);
}

// ��ʱ���жϻص�
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim2.Instance)
    {
        // �л�LED״̬
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
    }
}

// �ⲿ�жϻص�
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
        // �л�LED״̬
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
        
    }
}

// ����ģ���ʼ��
void BT_Init(Bluetooth_HandleTypeDef *hbt, UART_HandleTypeDef *huart) {
    hbt->huart = huart;
    hbt->rx_index = 0;
    hbt->cmd_ready = 0;
    hbt->current_cmd = BT_CMD_STOP;
    memset(hbt->rx_buffer, 0, BT_BUFFER_SIZE);
    
    // ���ô��ڽ����ж�
    __HAL_UART_ENABLE_IT(hbt->huart, UART_IT_RXNE);
    HAL_UART_Receive_IT(hbt->huart, hbt->rx_buffer, 1);
}

// �����жϴ���
void BT_UART_IRQHandler(Bluetooth_HandleTypeDef *hbt) {
    uint8_t recv_dat;
    static uint8_t rec_state = 0;
    
    if(__HAL_UART_GET_FLAG(hbt->huart, UART_FLAG_RXNE)) {
        recv_dat = (uint8_t)(hbt->huart->Instance->DR & 0x00FF);
        
        switch(rec_state) {
            case 0: // �ȴ�֡ͷ
                if((recv_dat == '$') && (!hbt->cmd_ready)) {
                    rec_state = 1;
                    hbt->rx_index = 0;
                }
                break;
                
            case 1: // ��������
                if(recv_dat == '#') {
                    hbt->cmd_ready = 1;
                    rec_state = 0;
                    BT_ProcessData(hbt);
                } 
                else if(hbt->rx_index < BT_BUFFER_SIZE - 1) {
                    hbt->rx_buffer[hbt->rx_index++] = recv_dat;
                } 
                else {
                    // ���������������״̬
                    rec_state = 0;
                    hbt->rx_index = 0;
                }
                break;
        }
    }
}

// ������յ�������
void BT_ProcessData(Bluetooth_HandleTypeDef *hbt) {
    if(hbt->cmd_ready && hbt->rx_index > 0) {
        // ����������
        switch(hbt->rx_buffer[0]) {
            case '0':
                hbt->current_cmd = BT_CMD_STOP;
                break;
            case '1':
                hbt->current_cmd = BT_CMD_FORWARD;
                break;
            case '2':
                hbt->current_cmd = BT_CMD_BACKWARD;
                break;
            case '3':
                hbt->current_cmd = BT_CMD_LEFT;
                break;
            case '4':
                hbt->current_cmd = BT_CMD_RIGHT;
                break;
        }
        
        // �����ת����
        if(hbt->rx_index > 2) {
            switch(hbt->rx_buffer[2]) {
                case '1':
                    hbt->current_cmd = BT_CMD_SPIN_LEFT;
                    break;
                case '2':
                    hbt->current_cmd = BT_CMD_SPIN_RIGHT;
                    break;
            }
        }
        
        // ���ý���״̬
        hbt->cmd_ready = 0;
        hbt->rx_index = 0;
        memset(hbt->rx_buffer, 0, BT_BUFFER_SIZE);
    }
}

// ��ȡ��ǰ����
BT_CommandType BT_GetCurrentCommand(Bluetooth_HandleTypeDef *hbt) {
    return hbt->current_cmd;
}

