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

/* 每个轱辘的目标速度与方向 */
typedef struct{
	
	float Reduction_ratio;          //减速比,30用
	float v_x;
	float v_y;
	float v_out;                    //平移向量+旋转向量
	float angle;
	float angle_offset;
}Vel_cc;
	

typedef struct{
						
	float Kxy;           //平移增益系数  rpm* [r(wheel)/60] 			 = v(car)     Kxy =  [r(wheel)/60]						     
	float Kz;            //旋转增益系数  rpm* [r(wheel)/60/R(car)] = ω(car)    Kz  =  [r(wheel)/60/R(car)]   
	
	Vel_cc	Wheel[4];    //四个轮子
	
}V_SteeringWheels;

extern V_SteeringWheels Goal_V;


/* function --------------------------------------------*/

/*开环无头*/
//void Openloop_WorldCoordinate(float Vel_x,float Vel_y,float Vel_z);
/*开环有头*/
void Openloop_CarCoordinate(float Vel_x,float Vel_y,float Vel_z);
/*闭环无头*/
void Closeloop_WorldCoordinate(float goal_z);

/*内部函数，发can信号*/
void motor_run(void);

#endif
