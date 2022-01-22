#ifndef _JOYHANDLE_H_
#define _JOYHANDLE_H_


#include "nrf.h"
#include "stdbool.h"

//#define ROCKER_LX		0
//#define ROCKER_LY		1
//#define ROCKER_RX		2
//#define ROCKER_RY		3
#define Car_Move_X_Rocker 1
#define Car_Move_Y_Rocker 0
#define Car_Move_Z_Rocker 2

#define Platform_Pitch_Rocker  3
#define Platform_Yaw_Rocker  2

#define Platform_Move_Switch  25
#define Platform_Aim_Button  20
#define Shoot_Button  21



/**********���ý���ģʽinit***********
nrf_mode = true;
Handle_Init(&hspi4);
***********�ж��и����ֱ�cmd**********
instruction_refresh()��
***********�õ�ҡ�ˡ�����ֵ***********
read_rocker(int id)
void read_keys(void)
*/



extern bool nrf_mode;//nrf����ģʽ

extern uint8_t button[26];//����

extern int16_t nrf_trans_cmd[7];   //�������ֱ�ȫ������



/******************************************************************
*@ 	name		: instruction_refresh	
*@	functio	: �ֱ����ݽ���						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
void instruction_refresh(void);



/******************************************************************
*@ 	name		: read_rocker	
*@	functio	: ���ҡ������		���0~3				
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
int16_t Read_Rocker(int id);

uint8_t Read_Button(int id);

/******************************************************************
*@ 	name		: read_keys	
*@	functio	: ��ð������ݣ����� boll button[26]��						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
void read_keys(void);

#endif
