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



/**********设置接受模式init***********
nrf_mode = true;
Handle_Init(&hspi4);
***********中断中更新手柄cmd**********
instruction_refresh()；
***********得到摇杆、按键值***********
read_rocker(int id)
void read_keys(void)
*/



extern bool nrf_mode;//nrf工作模式

extern uint8_t button[26];//按键

extern int16_t nrf_trans_cmd[7];   //解析后手柄全部数据



/******************************************************************
*@ 	name		: instruction_refresh	
*@	functio	: 手柄数据接收						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
void instruction_refresh(void);



/******************************************************************
*@ 	name		: read_rocker	
*@	functio	: 获得摇杆数据		编号0~3				
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
int16_t Read_Rocker(int id);

uint8_t Read_Button(int id);

/******************************************************************
*@ 	name		: read_keys	
*@	functio	: 获得按键数据，放在 boll button[26]中						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
void read_keys(void);

#endif
