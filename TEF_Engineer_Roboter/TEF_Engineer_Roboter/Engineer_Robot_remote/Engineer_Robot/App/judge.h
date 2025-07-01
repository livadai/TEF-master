#ifndef JUDGE_H
#define JUDGE_H

#include "stm32f4xx_hal.h"
#include "remote_control.h"

/***************������ID********************/

/*
	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz
	ID: 0x0002  Byte:  1    �����������         		������������
	ID: 0x0003  Byte:  32   ����������Ѫ������   		1Hz����       **
	ID: 0x0004  Byte:  3   	���ڷ���״̬  		?		���ڷ���ʱ����**
	ID: 0x0005  Byte:  3   	�˹�������ս���ӳ���ͷ�����   **

	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı����
	ID: 0X0104  Byte:  2    ���о�������
	ID: 0x0105  Byte:  1    ���ڷ���ڵ���ʱ

	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0x0208  Byte:  2    ����ʣ������  �����л����� �ڱ�
	ID: 0x0209  Byte:  4    ������RFID״̬

	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz
    ID: 0x0304 2025����
*/

#define JUDGE_BUFFER_LEN 200              //����������
#define RxBuff_SIZE 200										//�յ�����ϵͳ������


#define JUDGE_FRAME_HEADER 0xA5           //֡ͷ

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1     //����ϵͳ���Բ�������

#define shoot_speed_limit 20;

#define TRUE 1
#define FALSE 0

#define BLUE 0
#define RED 1

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16

/* RFID������ */
#define    CARD_ATTACK        ((uint8_t)0x00)
#define    CARD_PROTECT       ((uint8_t)0x01)
#define    CARD_BLOOD_RED     ((uint8_t)0x02)
#define    CARD_BLOOD_BLUE    ((uint8_t)0x03)
#define    CARD_HEAL_RED      ((uint8_t)0x04)
#define    CARD_HEAL_BLUE     ((uint8_t)0x05)
#define    CARD_COLD_RED      ((uint8_t)0x06)
#define    CARD_COLD_BLUE     ((uint8_t)0x07)
#define    CARD_FORT          ((uint8_t)0x08)


#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16

//ͨ��Э���ʽ
typedef enum
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,

}JudgedataOffset;  //����Э��ƫ����

// frame_header ��ʽ
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
}	FrameHeaderOffset; //����Э��ƫ����


typedef enum
{
	ID_game_state       = 0x0001,
	ID_game_result      = 0x0002,
	ID_game_robot_survivors       	= 0x0003,//���������˴������
	ID_game_missile_state = 0x0004, //���ڷ���״̬

	ID_event_data  					= 0x0101,//�����¼�����
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_supply_warm 	= 0x0104,//����ϵͳ��������
	ID_missile_shoot_time =0x0105  , //���ڷ���ڵ���ʱ

	ID_game_robot_state    			= 0x0201,//������״̬����
	ID_power_heat_data    			= 0x0202,//ʵʱ������������
	ID_game_robot_pos        		= 0x0203,//������λ������
	ID_buff_musk					= 0x0204,//��������������
	ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
	ID_robot_hurt					= 0x0206,//�˺�״̬����
	ID_shoot_data					= 0x0207,//ʵʱ�������
	ID_shoot_num          = 0x0208,//ʣ�෢����

}CmdID;

typedef struct
{

    int16_t mouse_x;

    int16_t mouse_y;

    int16_t mouse_z;

    int8_t left_button_down;

    int8_t right_button_down;

    uint16_t keyboard_value;

    uint16_t reserved;

} ext_robot_command_t;

typedef enum
{
	LEN_game_state       				=  3,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_survivors       		=  36,	//0x0003  ����������Ѫ������
	LED_game_missile_state      =3  , //0X0004���ڷ���״̬
	LED_game_buff               =3 , //0X0005

	LEN_event_data  					=  4,	//0x0101  �����¼�����
	LEN_supply_projectile_action        =  4,	//0x0102���ز���վ������ʶ����
	LEN_supply_warm        =2, //����ϵͳ���� 0x0104
	LEN_missile_shoot_time =1  , //���ڷ���ڵ���ʱ

	LEN_game_robot_state    			= 26,	//0x0201������״̬����
	LEN_power_heat_data   				= 16,	//0x0202ʵʱ������������
	LEN_game_robot_pos        			= 16,	//0x0203������λ������
	LEN_buff_musk        				=  1,	//0x0204��������������
	LEN_aerial_robot_energy        		=  3,	//0x0205���л���������״̬����
	LEN_robot_hurt        				=  1,	//0x0206�˺�״̬����
	LEN_shoot_data       				=  6,	//0x0207	ʵʱ�������
	LEN_shoot_num          = 2,//ʣ�෢����

}JudgeDataLength;





/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;

} xFrameHeader;

/* ID: 0x0001  Byte:  3    ����״̬���� */
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct
{
	uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    ����������Ѫ������ */
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_6_robot_HP;
	uint16_t red_7_robot_HP;

	uint16_t red_outpost_HP;
  uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_6_robot_HP;
	uint16_t blue_7_robot_HP;

	uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
}  ext_game_robot_HP_t;

/* ID: 0x0004  Byte:  3    ���ڷ���״̬ */
typedef __packed struct
{
	uint8_t dart_belong;
	uint16_t stage_remaining_time;
} ext_dart_status_t;

/* ID: 0x0005  Byte:  3    buff */
typedef __packed struct
{
	uint8_t F1_zone_status:1;
	uint8_t F1_zone_buff_debuff_status:3;

	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;

	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;

	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;

	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;

	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;
} ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct
{
	uint32_t event_type;
} ext_event_data_t;


/* ID: 0x0102  Byte:  3    ���ز���վ������ʶ���� */
typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* ID: 0x0104  Byte: 2   ����ϵͳ������Ϣ */
typedef __packed struct
{
  uint8_t level;
	uint8_t foul_robot_id;
}  ext_referee_warning_t;

/* ID: 0x0105  Byte:1  ���ڷ���ڵ���ʱ */
typedef __packed struct
{
	uint8_t dart_remaining_time;
}  ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 18    ������״̬���� */
typedef __packed struct
{

uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_id1_17mm_cooling_rate;    // ������ 17mm �ӵ���������
uint16_t shooter_id1_17mm_cooling_limit;  //������ 17mm �ӵ�������ȴ�ٶ� ��λ /s
uint16_t shooter_id1_17mm_speed_limit;
uint16_t shooter_id2_17mm_cooling_rate;
uint16_t shooter_id2_17mm_cooling_limit;
uint16_t shooter_id2_17mm_speed_limit;
	uint16_t shooter_id1_42mm_cooling_rate;
uint16_t shooter_id1_42mm_cooling_limit;
uint16_t shooter_id1_42mm_speed_limit;
uint16_t chassis_power_limit;

} ext_game_robot_state_t;


typedef __packed struct
{
uint8_t robot_id;    //������ID��������У�鷢��
uint8_t robot_level; //1һ����2������3����
uint16_t remain_HP;  //������ʣ��Ѫ��
uint16_t max_HP;     //��������Ѫ��
uint16_t shooter_id1_17mm_cooling_rate; //������1 ��17mm ǹ��ÿ����ȴֵ
uint16_t shooter_id1_17mm_cooling_limit; // ������1 ��17mm ǹ����������
uint16_t shooter_id1_17mm_speed_limit;   //������1 ��17mm ǹ�������ٶ� ��λ m/s
uint16_t shooter_id2_17mm_cooling_rate;
uint16_t shooter_id2_17mm_cooling_limit;
uint16_t shooter_id2_17mm_speed_limit;
uint16_t shooter_id1_42mm_cooling_rate;
uint16_t shooter_id1_42mm_cooling_limit;
uint16_t shooter_id1_42mm_speed_limit;
uint16_t chassis_power_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_statust;

/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;   //˲ʱ����
	uint16_t chassis_power_buffer;//60������������
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct
{
uint8_t power_rune_buff;
} ext_buff_t;

/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct
{
uint8_t attack_time;
} aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  6    ʵʱ������� */
typedef __packed struct
{
	uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} ext_shoot_data_t;


/* ID: 0x0208  Byte:  2    �ӵ�ʣ������ */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
}  ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  2    FRID״̬ */
typedef __packed struct
{
	uint32_t rfid_status ;
}  ext_rfid_status_t;

/*

	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��
	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)��
	�ͻ��� ID��
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)��
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)��
*/
/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;


/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz
	�ֽ�ƫ���� 	��С 	˵�� 			��ע
	0 			2 		���ݵ����� ID 	0x0200~0x02FF
										���������� ID ��ѡȡ������ ID �����ɲ������Զ���

	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ�

	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID

	6 			n 		���ݶ� 			n ��ҪС�� 113
*/
typedef __packed struct
{
	uint8_t data[10]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;



//**�����ǲ���ϵͳ����������**//
uint8_t Judgment_data_Get(uint8_t *Judgment_data);  //�ղ���ϵͳ����������



//**CRC8��CRC16У��**//
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);


uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
//**�����ղ���ϵͳ�������ñ�����������**//
extern uint8_t RxBuff[RxBuff_SIZE];
extern volatile uint8_t RxLength;
extern volatile uint8_t RxEndFlag;
extern uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ];  //����ϵͳ�������������ݴ�������

//**����ϵͳ״̬���ݴ���**//
void Heat_limit(void);			//ǹ����������
void Power_limit(void);			//���̹�������
void Judge_shoot_speed_limit(void); //�ӵ���������

#endif

