#include "main.h"
#include "Elmo_fdcan.h"
#include "Resolve.h"
#include "joyhandle.h"
#include "math.h"
#include "arm_math.h"
#include "Tinn.h"
#include "steering_wheel.h"
#include "Ann.h"

int32_t Wheel_Speed[4] = {0};//分解到三个轮胎上的速度
PID_T pid_speedx={.KP=1,.PID_MAX=2,.Dead_Zone=0.08};
PID_T pid_speedy={.KP=1,.PID_MAX=2,.Dead_Zone=0.08};
PID_T pid_deg={.KP=5.5,.KI=0,.KD=15,.PID_MAX=200,.Dead_Zone=0.5f};
PID_T pid_pos={.KP=3.6,.KI=0,.KD=0,.PID_MAX=4};
uint8_t deg_pid_disable;
Ort open_loop_velocity;
extern Ort current_pos;
extern Ort current_speed;
extern ANN_PID_handle velocity_nn_x,velocity_nn_y;
float dY;
float dX;
float dZ;
extern float ababa;

Kal_Filter kal_velocity_x =
{   1.0f,				//k_flt.C_last
    0.0f,			    //k_flt.X_last
    0.0001f,	  	    //k_flt.Q
    0.04f,		  		//k_flt.R 4.0
    0.0f, 0.0f, 0.0f, 0.0f
};

Kal_Filter kal_velocity_y =
{   1.0f,				//k_flt.C_last
    0.0f,			    //k_flt.X_last
    0.0001f,	  	    //k_flt.Q
    0.04f,		  		//k_flt.R 4.0
    0.0f, 0.0f, 0.0f, 0.0f
};

/*到达指定位置*/
/********************************************************************************
  *@  name      : Set_Pos
  *@  function  : 不要碰它，这里面水很深，你把握不住
  *@  input     : 无
  *@  output    : 无
********************************************************************************/
void Set_Pos(void)
{	
    float dx,dy,ddx,ddy;//input[2];
//    ddx=Pid_Run(&pid_speedx,dX,current_speed.x);
//    ddy=Pid_Run(&pid_speedy,dY,current_speed.y);
    ddx=dX;
    ddy=dY;
//    input[0]=dX;
//    input[1]=current_speed.x;
//    ddx=*xtpredict(velocity_nn_x,input);
//    input[0]=dY;
//    input[1]=current_speed.y;
//    ddy=*xtpredict(velocity_nn_y,input);
    
    
    dx=ddx*arm_cos_f32(current_pos.z*3.1415926f/180.0f)-ddy*arm_sin_f32(current_pos.z*3.1415926f/180.0f);
    dy=ddx*arm_sin_f32(current_pos.z*3.1415926f/180.0f)+ddy*arm_cos_f32(current_pos.z*3.1415926f/180.0f);
    float pid_velocityz=0;
//    int speed;
//    speed=-dx*80638;
//    speed=-(int)((double)speed*1.414);
//    int tmp[4]={0};
//    tmp[0]=-speed;
//    tmp[1]=speed;
//    tmp[2]=-speed;
//    tmp[3]=speed;
//    for(int i=0;i<4;i++)
//    {
//        Wheel_Speed[i]=-tmp[i];
//    }
//	speed=dy*80638;
//    for(int i=0;i<4;i++)
//    {
//        tmp[i]=speed;
//        Wheel_Speed[i]+=tmp[i];
//    }
    if(deg_pid_disable==0)
    {
        if(fabs(current_pos.z+dZ)>180)
        {
            if(dZ>0)
            {
    //            tmp[0]=Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
    //            tmp[1]=-Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
    //            tmp[2]=-Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
    //            tmp[3]=Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
                pid_velocityz=Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
            }
            else
            {
    //            tmp[0]=Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
    //            tmp[1]=-Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
    //            tmp[2]=-Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
    //            tmp[3]=Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
                pid_velocityz=Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
            }
        }
        else
        {
    //        tmp[0]=Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
    //        tmp[1]=-Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
    //        tmp[2]=-Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
    //        tmp[3]=Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
            pid_velocityz=Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
        }
    }
    else
    {
        pid_velocityz=Read_Rocker(2)/3;
    }
//    for(int i=0;i<4;i++)
//    {
//        Wheel_Speed[i]+=-1000*tmp[i];
//    }
//        Wheel_Speed[0]=Wheel_Speed[0];
//        Wheel_Speed[1]=Wheel_Speed[1]/19*28.5f;
//        Wheel_Speed[2]=Wheel_Speed[2]/19*28.5f;
//        Wheel_Speed[3]=Wheel_Speed[3];

	/*解算到车轮的速度*/
    if(open_loop_velocity.x==0&&open_loop_velocity.y==0&&open_loop_velocity.z==0)
        Openloop_CarCoordinate(dx,dy,pid_velocityz);
    else
        Openloop_CarCoordinate(open_loop_velocity.x,open_loop_velocity.y,open_loop_velocity.z);
	/*限制输出*/
	//Wheel_Limit();
}

///*********************************************************************************
//  *@  name      : Pid_Run
//  *@  function  : PID函数
//  *@  input     : pid设置，目标，反馈
//  *@  output    : 输出
//  *@  note      : NULL
//*********************************************************************************/
//float Pid_Run(PID_T *Pid, float Target, float Feedback)
//{
//    Pid->Error_Last = Pid->Error;
//    Pid->Error = Target - Feedback;
//    
//    Pid->P_OUT = Pid->KP * Pid->Error;

//    Pid->I_OUT += Pid->KI * Pid->Error;
//    
//    Pid->D_OUT = Pid->KD * (Pid->Error - Pid->Error_Last);

//    Pid->I_OUT = CLAMP(Pid->I_OUT, -Pid->I_MAX, Pid->I_MAX);
//    Pid->PID_OUT = Pid->P_OUT +Pid->I_OUT+ Pid->D_OUT;
//    Pid->PID_OUT = CLAMP(Pid->PID_OUT, -Pid->PID_MAX, Pid->PID_MAX);
//    
//    return Pid->PID_OUT;
//}


/*启动电机*/
void Elmo_Run(void)
{
	for(uint8_t i=1; i<=4; i++)
	{
		Elmo_PVM(i,Wheel_Speed[i-1]);
		//HAL_Delay(1);
	}
}

/**
 * @brief  卡尔曼滤波
 * @note  None
 * @param  None
 * @retval None
  */
float Kalman_Filter(Kal_Filter* K_Flt, float Input)
{
    /*量测更新，3组方程*/
    K_Flt->Input = Input;
    K_Flt->K = (K_Flt->C_last) / (K_Flt->C_last + K_Flt->R);
    K_Flt->X  = K_Flt->X_last + K_Flt->K * (K_Flt->Input - K_Flt->X_last);
    K_Flt->C =  (1 - K_Flt->K) * (K_Flt->C_last);

    /*时间更新，2组方程*/
    K_Flt->X_last = K_Flt->X;
    K_Flt->C_last = K_Flt->C + K_Flt->Q;

    return K_Flt->X;
}
