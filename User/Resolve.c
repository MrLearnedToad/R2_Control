#include "main.h"
#include "Elmo_fdcan.h"
#include "Resolve.h"
#include "joyhandle.h"
#include "math.h"
#include "arm_math.h"
#include "Tinn.h"
#include "steering_wheel.h"
#include "Ann.h"
#include "fdcan_bsp.h"

int32_t Wheel_Speed[4] = {0};//�ֽ⵽������̥�ϵ��ٶ�
PID_T pid_speedx={.KP=1,.PID_MAX=2,.Dead_Zone=0.08};
PID_T pid_speedy={.KP=1,.PID_MAX=2,.Dead_Zone=0.08};
PID_T pid_deg={.KP=4.5,.KI=0,.KD=4,.PID_MAX=100,.Dead_Zone=0.5f};
PID_T pid_pos={.KP=3.6,.KI=0,.KD=0,.PID_MAX=2};
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
    0.001f,	  	    //k_flt.Q
    0.03f,		  		//k_flt.R 4.0
    0.0f, 0.0f, 0.0f, 0.0f
};//X�����ٶȿ������˲�

Kal_Filter kal_velocity_y =
{   1.0f,				//k_flt.C_last
    0.0f,			    //k_flt.X_last
    0.001f,	  	    //k_flt.Q
    0.03f,		  		//k_flt.R 4.0
    0.0f, 0.0f, 0.0f, 0.0f
};//y�����ٶȿ������˲�

/*����ָ��λ��*/
/********************************************************************************
  *@  name      : Set_Pos
  *@  function  : ��Ҫ������������ˮ�������ղ�ס
  *@  input     : ��
  *@  output    : ��
********************************************************************************/
void Set_Pos(void)
{	
    float dx,dy,ddx,ddy;//input[2];
//    ddx=Pid_Run(&pid_speedx,dX,current_speed.x);
//    ddy=Pid_Run(&pid_speedy,dY,current_speed.y);
//    ddx=dX;
//    ddy=dY;
    ddx=ANN_pid_run(&velocity_nn_x,dX,current_speed.x);//�������ٶȿ���
    ddy=ANN_pid_run(&velocity_nn_y,dY,current_speed.y);
       //̫ܳ�ˣ��������Ȼ�ܺ�ʹ
//    input[0]=dX;
//    input[1]=current_speed.x;
//    ddx=*xtpredict(velocity_nn_x,input);
//    input[0]=dY;
//    input[1]=current_speed.y;
//    ddy=*xtpredict(velocity_nn_y,input);
    
    static float last_velocity_z=0;
    
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
    extern int judge_sign(double num);
    if(fabs(last_velocity_z-pid_velocityz)>10)
    {
        pid_velocityz=last_velocity_z+judge_sign(pid_velocityz-last_velocity_z)*10;
    }

    last_velocity_z=pid_velocityz;
    
	/*���㵽���ֵ��ٶ�*/
    if(open_loop_velocity.x==0&&open_loop_velocity.y==0&&open_loop_velocity.z==0)
    {
        Openloop_CarCoordinate(dx,dy,pid_velocityz);
        send_velocity(dx,dy,pid_velocityz);
    }
    else
    {
        Openloop_CarCoordinate(open_loop_velocity.x,open_loop_velocity.y,open_loop_velocity.z);
        send_velocity(open_loop_velocity.x,open_loop_velocity.y,open_loop_velocity.z);
    }
	/*�������*/
	//Wheel_Limit();
}

///*********************************************************************************
//  *@  name      : Pid_Run
//  *@  function  : PID����
//  *@  input     : pid���ã�Ŀ�꣬����
//  *@  output    : ���
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


/*�������*/
void Elmo_Run(void)
{
	for(uint8_t i=1; i<=4; i++)
	{
		Elmo_PVM(i,Wheel_Speed[i-1]);
		//HAL_Delay(1);
	}
}

/**
 * @brief  �������˲�
 * @note  None
 * @param  None
 * @retval None
  */
float Kalman_Filter(Kal_Filter* K_Flt, float Input)
{
    /*������£�3�鷽��*/
    K_Flt->Input = Input;
    K_Flt->K = (K_Flt->C_last) / (K_Flt->C_last + K_Flt->R);
    K_Flt->X  = K_Flt->X_last + K_Flt->K * (K_Flt->Input - K_Flt->X_last);
    K_Flt->C =  (1 - K_Flt->K) * (K_Flt->C_last);

    /*ʱ����£�2�鷽��*/
    K_Flt->X_last = K_Flt->X;
    K_Flt->C_last = K_Flt->C + K_Flt->Q;

    return K_Flt->X;
}

void send_velocity(float x,float y,float z)
{
    uint8_t transmit_buffer[8]={0};
    *(short*)(transmit_buffer+0)=x*2048.0f;
    *(short*)(transmit_buffer+2)=y*2048.0f;
    *(short*)(transmit_buffer+4)=z*128.0f;
    transmit_buffer[7]=Read_Button(23);
    FDCAN_SendData(&hfdcan1,transmit_buffer,0x234,8);
}
