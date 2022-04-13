#ifndef __RESOLVE_H__
#define __RESOLVE_H__
#include "main.h"
#include "user_task.h"
#include "Ann.h"
#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

/*һάkalman�˲��ṹ�嶨��,A=1,B=0,H=1*/
typedef struct Kalman_filter
{
	float C_last;				    /*�ϴ�Ԥ�����Э������� C(k|k-1)*/
	float X_last;				    /*ϵͳ״̬Ԥ������о���*/
	
	float Q;						/*��������Э����*/
	float R;						/*��������Э����*/
	
	float K;						/*���������棬�о���*/
	float X;						/*���Ź�����������о���*/
	float C;						/*���Ź���Э�������C(k|k)*/
                            
	float Input;				    /*����ֵ����Z(k)*/
} Kal_Filter;

extern int32_t Wheel_Speed[4];//�ֽ⵽������̥�ϵ��ٶ�
extern PID_T pid_pos;
/*�ڲ����õĺ���*/

static void send_velocity(float x,float y,float z);



/*�ⲿ�ɵ���*/
void Elmo_Run(void);
void Set_Pos(void);
float Kalman_Filter(Kal_Filter* K_Flt, float Input);
//float Pid_Run(PID_T *Pid, float Target, float Feedback);

#endif
