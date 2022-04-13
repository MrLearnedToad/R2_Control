#ifndef __RESOLVE_H__
#define __RESOLVE_H__
#include "main.h"
#include "user_task.h"
#include "Ann.h"
#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

/*一维kalman滤波结构体定义,A=1,B=0,H=1*/
typedef struct Kalman_filter
{
	float C_last;				    /*上次预测过程协方差矩阵 C(k|k-1)*/
	float X_last;				    /*系统状态预测矩阵，列矩阵*/
	
	float Q;						/*过程噪声协方差*/
	float R;						/*量测噪声协方差*/
	
	float K;						/*卡尔曼增益，列矩阵*/
	float X;						/*最优估计输出矩阵，列矩阵*/
	float C;						/*最优估计协方差矩阵C(k|k)*/
                            
	float Input;				    /*量测值，即Z(k)*/
} Kal_Filter;

extern int32_t Wheel_Speed[4];//分解到三个轮胎上的速度
extern PID_T pid_pos;
/*内部调用的函数*/

static void send_velocity(float x,float y,float z);



/*外部可调用*/
void Elmo_Run(void);
void Set_Pos(void);
float Kalman_Filter(Kal_Filter* K_Flt, float Input);
//float Pid_Run(PID_T *Pid, float Target, float Feedback);

#endif
