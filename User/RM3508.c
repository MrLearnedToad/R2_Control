#include "RM3508.h"
#include "string.h"
#include "fdcan.h"

/*这里改了一下，没有了PID，只有发送和接收的处理*/

//采样时间在5~10ms
const uint8_t RM3508_Reduction_Ratio[8] = {19, 19, 19, 19, 19, 19, 19, 19}; //电机减速比数组

//用于存储电机反馈的全局变量
uint8_t RM3508_Feedback_Buf[8][7]; //电机反馈值(全局变量)
int RM3508_Pos[8];                 //每一个元素对应一个ID的电机的信息
uint8_t RM3508_Sendbuf[8] = {0};   // CAN发送数据

/*********************************************************************************
 *@  name      : RM3508_Set_I
 *@  function  : RM3508电机电流设置
 *@  input     : 目标电流，电机id
 *@  output    : 成功返回0，失败返回1
 *********************************************************************************/
uint8_t RM3508_Set_I(int target_i, uint8_t motor_id)
{
    if (motor_id >= 1 && motor_id <= 8)
    {
        int send_id = 0;
        send_id = send_id;

        if (target_i <= -10000)
            target_i = -10000;
        else if (target_i >= 10000)
            target_i = 10000;

        if (motor_id <= 4)
            send_id = 0x200;
        else
        {
            send_id = 0x1ff;
            motor_id -= 4;
        }

        RM3508_Sendbuf[2 * motor_id - 2] = target_i >> 8;     //电流值高8位
        RM3508_Sendbuf[2 * motor_id - 1] = target_i & 0x00ff; //电流值低8位

        //		RM3508_CAN_Send_Data(&hcan1, RM3508_Sendbuf, send_id ,8);
        return 0;
    }
    else
        return 1;
}

/*********************************************************************************
 *@  name      : RM3508_Get_Feedback
 *@  function  : 获取RM3508电机的反馈并存入全局变量RM3508_Feedback_Buf[8][7];
 *@  input     : message_id,message数组指针
 *@  output    :
 *********************************************************************************/
void RM3508_Get_Feedback(uint32_t std_id, uint8_t *data_p)
{
    int i;
    for (i = 1; i < 9; i++) //前四电机匹配
    {
        if (std_id == 0x200 + i)
        {
            memcpy(RM3508_Feedback_Buf[i - 1], data_p, 7);
            RM3508_Pos_Rec(i);
            return;
        }
    }
}
/*********************************************************************************
 *@  name      :RM3508_Get_Real_I
 *@  function  : 获取RM3508电机的实际转矩信息
 *@  input     : 电机id号
 *@  output    : 对应id电机的转矩,读取失败返回0
 *********************************************************************************/
int RM3508_Get_Torque(uint8_t motor_id)
{
    int torque = 0;
    if (RM3508_Feedback_Buf[motor_id - 1][2] >> 7 == 1)
        torque = -(0xffff - ((RM3508_Feedback_Buf[motor_id - 1][4] << 8) + RM3508_Feedback_Buf[motor_id - 1][5]));
    else
        torque = (RM3508_Feedback_Buf[motor_id - 1][4] << 8) + RM3508_Feedback_Buf[motor_id - 1][5];
    return torque;
}
/*********************************************************************************
 *@  name      : RM3508_Get_Speed
 *@  function  : 获取RM3508电机的反馈的速度信息
 *@  input     : 电机id号
 *@  output    : 对应id电机的速度,读取失败返回0
 *********************************************************************************/
int RM3508_Get_Speed(uint8_t motor_id)
{
    int speed = 0;
    if (RM3508_Feedback_Buf[motor_id - 1][2] >> 7 == 1)
        speed = -(0xffff - ((RM3508_Feedback_Buf[motor_id - 1][2] << 8) + RM3508_Feedback_Buf[motor_id - 1][3]));
    else
        speed = (RM3508_Feedback_Buf[motor_id - 1][2] << 8) + RM3508_Feedback_Buf[motor_id - 1][3];
    return speed;
}
/*********************************************************************************
 *@  name      : RM3508_Get_Pos
 *@  function  : 获取RM3508电机当前的位置信息
 *@  input     : 电机id号
 *@  output    : 对应id电机的位置，编码器的CNT值
 *********************************************************************************/
int RM3508_Get_Pos(uint8_t motor_id)
{
    return RM3508_Pos[motor_id - 1];
}
/*********************************************************************************
 *@  name      : RM3508_Pos_Rec
 *@  function  : 获取RM3508电机的反馈的位置信息
 *@  input     : 电机id号
 *@  output    : 对应id电机的位置信息,读取失败返回-1
 *********************************************************************************/
static int32_t RM3508_base[8] = {0}; //用来标记已经转过的圈数，一圈8192

void RM3508_Pos_Rec(uint8_t motor_id)
{
    int id = motor_id - 1;
    int32_t RM3508_tmp[8];

    static int32_t RM3508tmp_pre[8] = {0};

    RM3508_tmp[id] = (RM3508_Feedback_Buf[id][0] << 8) + RM3508_Feedback_Buf[id][1];
    if (RM3508_tmp[id] - RM3508tmp_pre[id] > 4095) //转过8191到0时记录圈数
        RM3508_base[id] -= 8191;
    else if (RM3508_tmp[id] - RM3508tmp_pre[id] < -4095)
        RM3508_base[id] += 8191;

    RM3508tmp_pre[id] = RM3508_tmp[id];
    RM3508_Pos[id] = RM3508_base[id] + RM3508_tmp[id];
}
/********************************************************************************
 *@  name      : Ang2Cnt
 *@  function  : 角度转换为实际电机应该转动的位置数 //未经减速箱的转轴转一圈数值为8192
 *@  input     : 目标角度（任意角度），电机id  //id不同，减速比不同
 *@  output    : 电机位置
 ********************************************************************************/
int RM3508_Ang2Cnt(float angle, int ID)
{

    int cnt;
    cnt = (int)(RM3508_CNT_PER_ROUND_OUT(ID) * angle / 360);
    return cnt;
}
/********************************************************************************
 *@  name      : Cnt2Ang
 *@  function  : 电机位置转换为角度 //未经减速箱的转轴转一圈数值为8192
 *@  input     : 电机位置，电机id  //id不同，减速比不同
 *@  output    : 电机转过的角度
 ********************************************************************************/
double RM3508_Cnt2Ang(int32_t cnt, int ID)
{
    double angled;
    angled = (double)((cnt * 360.0) / RM3508_CNT_PER_ROUND_OUT(ID));
    return angled;
}

/*将3508电机的当前值设置为任意位置*/
void RM3508_Set_NowPos(uint8_t ID, int32_t Pos_Angle)
{
    uint8_t id;
    id = ID - 1;
    RM3508_base[id] = Pos_Angle;
}

/*RM3508温度反馈*/
uint8_t RM3508_Temperature(uint8_t id)
{
    uint8_t Tem;
    Tem = RM3508_Feedback_Buf[id - 1][6];
    return Tem;
}

/*得到3508的CAN发送数据*/
uint8_t *Get_RM3508_Send(void)
{
    return RM3508_Sendbuf;
}
