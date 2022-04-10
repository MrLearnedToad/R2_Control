
/*
	与VESC CAN通信对接的函数库
	updated:2021-10-28
	希望以后调这个库的人不要骂我
*/

#ifndef __VESC_CAN__
#define __VESC_CAN__

#include "fdcan.h"
#include "stdint.h"
#include "string.h"
/*所连接的VESC最大id值
id在VESC TOOL里设置
建议设置的小一点*/
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
	
	float cur_current; //电流
	int cur_rpm; //转速
	int cur_pos; //位置
    uint8_t error_flag;
    uint8_t last_call_time;
}Motor_INFO;
extern Motor_INFO VESC_Feedback[VESC_MAX_ID+1];

/*PID数据格式*/
typedef struct
{
	int Err,Err_1,Err_sum;
	float Kp,Ki,Kd;
	int Max,Min;
	float P_Out,I_Out,D_Out;
	float PID_Out;
}VESC_PID;


/*外部调用函数*/

/*初始化所需要的PID参数,写在主程序开始时*/
void VESC_PID_INIT(void);
/*设置位置*/
void VESC_SET_POS(uint8_t ID, int Goal_pos,FDCAN_HandleTypeDef *hfdcan);
/*外部调用设置速度的函数*/
void VESC_SET_SPEED(uint8_t ID,int Goal_rpm,FDCAN_HandleTypeDef *hfdcan);
/*获取当前速度*/
int VESC_GET_SPEED(uint8_t ID);
/* 刹车 */
void VESC_SET_CurrentBrake(uint8_t ID, int Current,FDCAN_HandleTypeDef *hfdcan);
/*发送的底层函数*/
void VESC_COMMAND_SEND(FDCAN_HandleTypeDef *hfdcan,uint32_t cmd,uint8_t id,float set_value);
/*CAN接收函数,应将其写入回调函数内*/
void VESC_CAN_DECODE(uint32_t ExtID,uint8_t pData[]);


/*内部调用函数*/
extern Motor_INFO VESC_Feedback[];
uint32_t VESC_DECODE_RPM(uint8_t pData[]);
float VESC_DECODE_CURRENT(uint8_t pData[]);
int VESC_DECODE_POS(uint8_t pData[]);
#endif
