#ifndef __RESOLVE_H__
#define __RESOLVE_H__
#include "main.h"
#include "user_task.h"

#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

typedef struct
{
    float Error;
    float Error_Last;
    float Error_Sum;

    float KP;
    float KI;
    float KD;

    float I_Limit;
    float I_Coefficient;
    float Dead_Zone;

    float P_OUT;
    float I_OUT;
    float D_OUT;
    float PID_OUT;

    float I_MAX;
    float PID_MAX;

} PID_T;

extern int32_t Wheel_Speed[4];//分解到三个轮胎上的速度
extern PID_T pid_pos;
/*内部调用的函数*/




/*外部可调用*/
void Elmo_Run(void);
void Set_Pos(void);
float Pid_Run(PID_T *Pid, float Target, float Feedback);

#endif
