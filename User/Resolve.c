#include "main.h"
#include "Elmo_fdcan.h"
#include "Resolve.h"
#include "joyhandle.h"
#include "math.h"
#include "arm_math.h"
#include "Tinn.h"

int32_t Wheel_Speed[4] = {0};//分解到三个轮胎上的速度
PID_T pid_speedx={.KP=1,.PID_MAX=2,.Dead_Zone=0.08};
PID_T pid_speedy={.KP=1,.PID_MAX=2,.Dead_Zone=0.08};
PID_T pid_deg={.KP=12,.PID_MAX=150};
PID_T pid_pos={.KP=3.6,.KI=0,.KD=0,.PID_MAX=4};
extern Ort current_pos;
extern Ort current_speed;
extern Tinn velocity_nn_x,velocity_nn_y;
float dY;
float dX;
float dZ;
extern float ababa;

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
    int speed;
    speed=-dx*80638;
    speed=-(int)((double)speed*1.414);
    int tmp[4]={0};
    tmp[0]=-speed;
    tmp[1]=speed;
    tmp[2]=-speed;
    tmp[3]=speed;
    for(int i=0;i<4;i++)
    {
        Wheel_Speed[i]=-tmp[i];
    }
	speed=dy*80638;
    for(int i=0;i<4;i++)
    {
        tmp[i]=speed;
        Wheel_Speed[i]+=tmp[i];
    }
    
    if(fabs(current_pos.z+dZ)>180)
    {
        if(dZ>0)
        {
            tmp[0]=Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
            tmp[1]=-Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
            tmp[2]=-Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
            tmp[3]=Pid_Run(&pid_deg,dZ-180,-current_pos.z+180)*0.5f;
        }
        else
        {
            tmp[0]=Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
            tmp[1]=-Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
            tmp[2]=-Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
            tmp[3]=Pid_Run(&pid_deg,dZ+180,-current_pos.z-180)*0.5f;
        }
    }
    else
    {
        tmp[0]=Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
        tmp[1]=-Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
        tmp[2]=-Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
        tmp[3]=Pid_Run(&pid_deg,dZ,-current_pos.z)*0.5f;
    }
    for(int i=0;i<4;i++)
    {
        Wheel_Speed[i]+=-1000*tmp[i];
    }
        Wheel_Speed[0]=Wheel_Speed[0];
        Wheel_Speed[1]=Wheel_Speed[1]/19*28.5f;
        Wheel_Speed[2]=Wheel_Speed[2]/19*28.5f;
        Wheel_Speed[3]=Wheel_Speed[3];

	/*解算到车轮的速度*/

	/*限制输出*/
	//Wheel_Limit();
}

/*********************************************************************************
  *@  name      : Pid_Run
  *@  function  : PID函数
  *@  input     : pid设置，目标，反馈
  *@  output    : 输出
  *@  note      : NULL
*********************************************************************************/
float Pid_Run(PID_T *Pid, float Target, float Feedback)
{
    Pid->Error_Last = Pid->Error;
    Pid->Error = Target - Feedback;
    
    Pid->P_OUT = Pid->KP * Pid->Error;

    Pid->I_OUT += Pid->KI * Pid->Error;
    
    Pid->D_OUT = Pid->KD * (Pid->Error - Pid->Error_Last);

    Pid->I_OUT = CLAMP(Pid->I_OUT, -Pid->I_MAX, Pid->I_MAX);
    Pid->PID_OUT = Pid->P_OUT +Pid->I_OUT+ Pid->D_OUT;
    Pid->PID_OUT = CLAMP(Pid->PID_OUT, -Pid->PID_MAX, Pid->PID_MAX);
    
    return Pid->PID_OUT;
}


/*启动电机*/
void Elmo_Run(void)
{
	for(uint8_t i=1; i<=4; i++)
	{
		Elmo_PVM(i,Wheel_Speed[i-1]);
		//HAL_Delay(1);
	}
}

/*限制区间函数*/
