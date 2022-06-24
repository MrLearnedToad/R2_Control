#include "steering_wheel.h"
#include "fdcan_bsp.h"
#include "math.h"

V_SteeringWheels Goal_V=
{	.Kxy=3932.179,
	.Kz = 70,
    .Wheel[0].angle_offset=-30.0f,
    .Wheel[1].angle_offset=157.5f,
    .Wheel[2].angle_offset=-22.5f,
    .Wheel[3].angle_offset=-22.5f,
};
V_SteeringWheels Goal_Vlast;


	
/*************************************************
*	@breif: 开环有头模式 目标速度、角度解算，↑y→x，逆时针正,弧度制 
*	@input: Vel_y 前后方向速度  
*					Vel_x 右左方向速度       
*					Vel_z 旋转速度      	
*2@------@1	
* |  ↑  |	
*3@------@4	
**************************************************/			
void Openloop_CarCoordinate(float Vel_x,float Vel_y,float Vel_z)
{	

	/* 1轮速度解算 */    
	Goal_V.Wheel[0].Reduction_ratio=1;
	Goal_V.Wheel[0].v_x = 1* Vel_x * Goal_V.Kxy - 0.588f * Vel_z * Goal_V.Kz;
	Goal_V.Wheel[0].v_y = 1* Vel_y * Goal_V.Kxy + 0.809f * Vel_z * Goal_V.Kz;  

	Goal_V.Wheel[0].v_out = sqrt(Goal_V.Wheel[0].v_x*Goal_V.Wheel[0].v_x + Goal_V.Wheel[0].v_y*Goal_V.Wheel[0].v_y);
	if(Goal_V.Wheel[0].v_x == 0 && Goal_V.Wheel[0].v_y==0) Goal_V.Wheel[0].angle = Goal_Vlast.Wheel[0].angle;
	else Goal_V.Wheel[0].angle = atan2(Goal_V.Wheel[0].v_x,Goal_V.Wheel[0].v_y)*180/pi+Goal_V.Wheel[0].angle_offset;	
	
	/*2轮速度解算*/
	Goal_V.Wheel[1].Reduction_ratio=1;
	Goal_V.Wheel[1].v_x = 1* Vel_x * Goal_V.Kxy - 0.588f * Vel_z * Goal_V.Kz;
	Goal_V.Wheel[1].v_y = 1* Vel_y * Goal_V.Kxy - 0.809f * Vel_z * Goal_V.Kz;  

	Goal_V.Wheel[1].v_out = sqrt(Goal_V.Wheel[1].v_x*Goal_V.Wheel[1].v_x + Goal_V.Wheel[1].v_y*Goal_V.Wheel[1].v_y);
	if(Goal_V.Wheel[1].v_x == 0 && Goal_V.Wheel[1].v_y==0) Goal_V.Wheel[1].angle = Goal_Vlast.Wheel[1].angle;
	else Goal_V.Wheel[1].angle = atan2(Goal_V.Wheel[1].v_x,Goal_V.Wheel[1].v_y)*180/pi+Goal_V.Wheel[1].angle_offset;	

	/*3轮速度解算*/
	Goal_V.Wheel[2].Reduction_ratio=1;
	Goal_V.Wheel[2].v_x = 1* Vel_x * Goal_V.Kxy + 0.588f * Vel_z * Goal_V.Kz;
	Goal_V.Wheel[2].v_y = 1* Vel_y * Goal_V.Kxy - 0.809f * Vel_z * Goal_V.Kz;  

	Goal_V.Wheel[2].v_out = sqrt(Goal_V.Wheel[2].v_x*Goal_V.Wheel[2].v_x + Goal_V.Wheel[2].v_y*Goal_V.Wheel[2].v_y);
	if(Goal_V.Wheel[2].v_x == 0 && Goal_V.Wheel[2].v_y==0) Goal_V.Wheel[2].angle = Goal_Vlast.Wheel[2].angle;
	else Goal_V.Wheel[2].angle = atan2(Goal_V.Wheel[2].v_x,Goal_V.Wheel[2].v_y)*180/pi+Goal_V.Wheel[2].angle_offset;	
		
	/*4轮速度解算*/
	Goal_V.Wheel[3].Reduction_ratio=1;
	Goal_V.Wheel[3].v_x = 1* Vel_x * Goal_V.Kxy + 0.588f * Vel_z * Goal_V.Kz;
	Goal_V.Wheel[3].v_y = 1* Vel_y * Goal_V.Kxy + 0.809f * Vel_z * Goal_V.Kz;  

	Goal_V.Wheel[3].v_out = sqrt(Goal_V.Wheel[3].v_x*Goal_V.Wheel[3].v_x + Goal_V.Wheel[3].v_y*Goal_V.Wheel[3].v_y);
	if(Goal_V.Wheel[3].v_x == 0 && Goal_V.Wheel[3].v_y==0) Goal_V.Wheel[3].angle = Goal_Vlast.Wheel[3].angle;
	else Goal_V.Wheel[3].angle = atan2(Goal_V.Wheel[3].v_x,Goal_V.Wheel[3].v_y)*180/pi+Goal_V.Wheel[3].angle_offset;	
	

    
	//输入全0时保持上一个角度
	Goal_Vlast = Goal_V;    
	//can信号	

    motor_run();

}

/**************************************************
* @breif: 驱动6020与vesc，让6020转最小的角度(避免把轮子磨坏)
* @input: null
**************************************************/
void motor_run(void)
{
	for(uint8_t i=0; i<4; i++)
	{  
		//走劣弧
	 	while(Goal_V.Wheel[i].angle - GM6020_Get_Pos(i+1)*360/8192 >180){
			Goal_V.Wheel[i].angle -=360;
		}
		while(Goal_V.Wheel[i].angle - GM6020_Get_Pos(i+1)*360/8192 <-180){
			Goal_V.Wheel[i].angle +=360;
		}
		
		//走小于90度，再反向
		if(Goal_V.Wheel[i].angle - GM6020_Get_Pos(i+1)*360/8192 >90){
			Goal_V.Wheel[i].angle -=180;
			Goal_V.Wheel[i].v_out =-Goal_V.Wheel[i].v_out;
		}
		else if(Goal_V.Wheel[i].angle - GM6020_Get_Pos(i+1)*360/8192 <-90){
			Goal_V.Wheel[i].angle +=180;
			Goal_V.Wheel[i].v_out =-Goal_V.Wheel[i].v_out;
		}
		
		//走你

		GM6020_Set_Pos(Goal_V.Wheel[i].angle, i+1);	    
		
		if(i == 0) Goal_V.Wheel[i].v_out = Goal_V.Wheel[i].v_out; //1、3轮子方向反了
        if(i == 1) Goal_V.Wheel[i].v_out = Goal_V.Wheel[i].v_out;
        if(i == 2) Goal_V.Wheel[i].v_out = Goal_V.Wheel[i].v_out;
        if(i == 3) Goal_V.Wheel[i].v_out = Goal_V.Wheel[i].v_out;
		VESC_COMMAND_SEND(&hfdcan2, 3, i+1, Goal_V.Wheel[i].v_out);		
	}
	
}


//void Openloop_WorldCoordinate(float Vel_x,float Vel_y,float Vel_z)
//{
//	float dz= Cur_GYRO.position_z;
//	
//	while(dz>360) 	dz -= 360;
//	while(dz<0 )		dz += 360;
//	float v_x = (float)(Vel_x * cos(dz/180*pi) + Vel_y * sin(dz/180*pi));
//	float v_y = (float)(Vel_y * cos(dz/180*pi) - Vel_x * sin(dz/180*pi)); 
//		
//	Openloop_CarCoordinate(v_x, v_y, Vel_z);
//}


