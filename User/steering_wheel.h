#ifndef __MOVE_H_
#define __MOVE_H_

/* include ------------------------------------------*/
#include "main.h"
#include "stdint.h"
#include "string.h"

#include "VESC_CAN.h"
#include "GM6020.h"

/* Private define ------------------------------------*/

#define Limit(x,max,min)	(((x)>(max)) ? (max) : ( ((x)<(min)) ? (min) : (x)))
#define pi	3.1415926

/* ÿ����ꤵ�Ŀ���ٶ��뷽�� */
typedef struct{
	
	float Reduction_ratio;          //���ٱ�,30��
	float v_x;
	float v_y;
	float v_out;                    //ƽ������+��ת����
	float angle;
	float angle_offset;
}Vel_cc;
	

typedef struct{
						
	float Kxy;           //ƽ������ϵ��  rpm* [r(wheel)/60] 			 = v(car)     Kxy =  [r(wheel)/60]						     
	float Kz;            //��ת����ϵ��  rpm* [r(wheel)/60/R(car)] = ��(car)    Kz  =  [r(wheel)/60/R(car)]   
	
	Vel_cc	Wheel[4];    //�ĸ�����
	
}V_SteeringWheels;

extern V_SteeringWheels Goal_V;


/* function --------------------------------------------*/

/*������ͷ*/
//void Openloop_WorldCoordinate(float Vel_x,float Vel_y,float Vel_z);
/*������ͷ*/
void Openloop_CarCoordinate(float Vel_x,float Vel_y,float Vel_z);
/*�ջ���ͷ*/
void Closeloop_WorldCoordinate(float goal_z);

/*�ڲ���������can�ź�*/
void motor_run(void);

#endif
