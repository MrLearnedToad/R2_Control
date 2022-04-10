
/*
	��VESC CANͨ�ŶԽӵĺ�����
	updated:2021-10-28
	ϣ���Ժ���������˲�Ҫ����
*/

#ifndef __VESC_CAN__
#define __VESC_CAN__

#include "fdcan.h"
#include "stdint.h"
#include "string.h"
/*�����ӵ�VESC���idֵ
id��VESC TOOL������
�������õ�Сһ��*/
#define VESC_MAX_ID 8 

#define VESC_SET_DUTY 					0
#define VESC_SET_CURRENT 				1
#define VESC_SET_CURRENT_BRAKE 	2
#define VESC_SET_RPM 						3
#define VESC_SET_POSITION 			4

#define RPM_ERPM(x) 				(x)*7
#define CLAMP(x, lower, upper)	(x >= upper ? upper : (x <= lower ? lower : x))
extern uint8_t TXDATA[4];

typedef struct{
	
	float cur_current; //����
	int cur_rpm; //ת��
	int cur_pos; //λ��
    uint8_t error_flag;
    uint8_t last_call_time;
}Motor_INFO;
extern Motor_INFO VESC_Feedback[VESC_MAX_ID+1];

/*PID���ݸ�ʽ*/
typedef struct
{
	int Err,Err_1,Err_sum;
	float Kp,Ki,Kd;
	int Max,Min;
	float P_Out,I_Out,D_Out;
	float PID_Out;
}VESC_PID;


/*�ⲿ���ú���*/

/*��ʼ������Ҫ��PID����,д��������ʼʱ*/
void VESC_PID_INIT(void);
/*����λ��*/
void VESC_SET_POS(uint8_t ID, int Goal_pos,FDCAN_HandleTypeDef *hfdcan);
/*�ⲿ���������ٶȵĺ���*/
void VESC_SET_SPEED(uint8_t ID,int Goal_rpm,FDCAN_HandleTypeDef *hfdcan);
/*��ȡ��ǰ�ٶ�*/
int VESC_GET_SPEED(uint8_t ID);
/* ɲ�� */
void VESC_SET_CurrentBrake(uint8_t ID, int Current,FDCAN_HandleTypeDef *hfdcan);
/*���͵ĵײ㺯��*/
void VESC_COMMAND_SEND(FDCAN_HandleTypeDef *hfdcan,uint32_t cmd,uint8_t id,float set_value);
/*CAN���պ���,Ӧ����д��ص�������*/
void VESC_CAN_DECODE(uint32_t ExtID,uint8_t pData[]);


/*�ڲ����ú���*/
extern Motor_INFO VESC_Feedback[];
uint32_t VESC_DECODE_RPM(uint8_t pData[]);
float VESC_DECODE_CURRENT(uint8_t pData[]);
int VESC_DECODE_POS(uint8_t pData[]);
#endif
