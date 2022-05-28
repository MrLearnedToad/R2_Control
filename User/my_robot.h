#ifndef __MY_ROBOT__
#define __MY_ROBOT__
#include "nrf.h"
#include "stdbool.h"

//#define ROCKER_LX		0
//#define ROCKER_LY		1
//#define ROCKER_RX		2
//#define ROCKER_RY		3

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

extern uint8_t button[29];//����

extern int16_t nrf_trans_cmd[7];   //�������ֱ�ȫ������


void nrf_init(void);

/******************************************************************
*@ 	name		: instruction_refresh	
*@	functio	: �ֱ����ݽ���						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
void instruction_refresh(void);

void Ack_load(uint8_t lenth);

/******************************************************************
*@ 	name		: read_rocker	
*@	functio	: ���ҡ������		���0~3				
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
int16_t Read_Rocker(int id);



/******************************************************************
*@ 	name		: read_keys	
*@	functio	: ��ð������ݣ����� boll button[26]��						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
int Read_Button(int i);

#endif
