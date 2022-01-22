/*********************************************************************************
*@函数功能简介：
*@	RM3508_Set_I：		RM3508电流控制
*@	RM3508_Set_Speed：	RM3508速度闭环，通过电流作为输出
*@	RM3508_Set_Pos：		RM3508位置闭环，通过速度作为输出，串级PID
*@
*@	RM3508_Get_Feedback：RM3508数据反馈获取，CAN中断中使用
*@	M_2006_Get_Torque：	RM3508转矩信息获取
*@	RM3508_Get_Speed：	速度信息获取
*@	RM3508_Get_Pos：		位置信息获取
*@	pos_rec：			位置信息更新，CAN中断中使用，以足够高的采样率解决边界问题
*@
*@	Ang2Cnt：			角度制数据转化为编码器的Cnt值
*@	Cnt2Ang：			编码器Cnt值转化为实际转过的角度
*@
*@	RM3508和3508互通的，区别只是电机减速比不同以及3508多了一个电机温度反馈
*@	因此该文件修改电机的减速比后可以直接用于3508的控制
*@  
*@ @file	:	RM3508.c
*@ @author	:	SHJ
*@ @version	:	v2.1，修复了同时控制多个电机，数据被覆写的Bug；每个电机给了一套独立的PID
*@				调参时可以对每个电机针对性的调整；将位置闭环的入口参数改为cnt值
*@ @date	:	2019-1-30
*********************************************************************************/

#ifndef __RM3508_H
#define __RM3508_H

#include "stm32h7xx.h"


#define RM3508_CNT_PER_ROUND (8192) //编码器线程
#define RM3508_CNT_PER_ROUND_OUT(x) (RM3508_CNT_PER_ROUND * RM3508_Reduction_Ratio[(x - 1)])
#define RM3508_CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))
#define RM3508_ABS(x) (x >= 0 ? x : -x)


extern uint8_t RM3508_Feedback_Buf[8][7]; //电机反馈报文
extern int RM3508_Pos[8];
extern uint8_t RM3508_Sendbuf[8];

uint8_t RM3508_Set_I(int target_i,uint8_t motor_id);

void RM3508_Get_Feedback(uint32_t std_id, uint8_t *data_p);
int RM3508_Get_Torque(uint8_t motor_id);
int RM3508_Get_Speed(uint8_t motor_id);
int RM3508_Get_Pos(uint8_t motor_id);
void RM3508_Pos_Rec(uint8_t motor_id); //CAN中断中实时更新位置信息
/*将3508电机的当前值设置为任意位置*/
void RM3508_Set_NowPos(uint8_t ID,int32_t Pos_Angle);
uint8_t RM3508_Temperature(uint8_t id);
int RM3508_Ang2Cnt(float angle, int ID);
double RM3508_Cnt2Ang(int32_t cnt, int ID);

uint8_t *Get_RM3508_Send(void);

#endif
