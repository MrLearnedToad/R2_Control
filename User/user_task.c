#include "user_task.h"
#include "my_robot.h"
#include "math.h"
#include "Resolve.h"
#include "fdcan_bsp.h"
#include "move.h"
/*函数指针区*/
int (*autodirveshortdistance)(mission_queue*)=auto_drive_shortdistance;
int (*autodrivelongdistance)(mission_queue*)=auto_drive_longdistance;
int (*grabposset)(mission_queue*)=grab_pos_set;
int (*hookgrasp)(mission_queue*)=hook_grasp;
int (*hookrelease)(mission_queue*)=hook_release;
int (*switcherdirectionset)(mission_queue*)=switcher_direction_set;
int (*catapultactivate)(mission_queue*)=catapult_activate;
int (*autopickup)(mission_queue *current_task)=auto_pick_up;
int (*autoplace)(mission_queue *current_task)=auto_place;
int (*posregulatorposset)(mission_queue *current_task)=pos_regulator_pos_set;
int (*pickupactivatorposset)(mission_queue *current_task)=pick_up_activator_pos_set;
int (*autoturn)(mission_queue *current_task)=auto_turn;
/*全局变量区*/
Ort target_pos;
int current_target_ID=0;
float short_drive_deadzone=0.2f;
extern uint8_t get_block_flag;
uint8_t thread_lock=0;
uint8_t final_point_lock;
/*********************************************************************************
  *@  name      : auto_drive_shortdistance
  *@  function  : 机器人短距离移动函数
  *@  input     : current_task
  *@  output    : NULL 
  *@  note      : NULL
*********************************************************************************/
int auto_drive_shortdistance(mission_queue *current_task)
{
    static int flag_running=0;
    double distance;
//    Ort velocity_vector;
    distance=sqrtf((current_pos.x-current_task->info.x)*(current_pos.x-current_task->info.x)+(current_pos.y-current_task->info.y)*(current_pos.y-current_task->info.y));
//    pid_distance=-Pid_Run(&pid_pos,0,distance);
//    velocity_vector.x=(current_task->info.x-current_pos.x)/distance*pid_distance;
//    velocity_vector.y=(current_task->info.y-current_pos.y)/distance*pid_distance;
//    dX=velocity_vector.x;
//    dY=velocity_vector.y;
    if(flag_running==0)
    {
        pre_plan(current_task->info);
        global_clock=3;
        flag_running=1;
        flags[auto_drive_status]=moving;
    }
    
    if(distance<0.004||global_clock>500||(Read_Rocker(2)*Read_Rocker(2)+Read_Rocker(3)*Read_Rocker(3))>=100)
    {
//        dX=0;
//        dY=0;
        current_task->flag_finish=1;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
    }
    if(distance<short_drive_deadzone&&current_task->info.z>6)
    {
//        dX=0;
//        dY=0;
        current_task->flag_finish=1;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
    }
    return 0;
}

int auto_turn(mission_queue *current_task)
{
    dZ=current_task->info.z;
    current_task->flag_finish=1;
    return 0;
}

/*********************************************************************************
  *@  name      : auto_drive_longdistance
  *@  function  : 机器人长距离移动函数
  *@  input     : current_task
  *@  output    : NULL 
  *@  note      : NULL
*********************************************************************************/
uint8_t msg;
TaskHandle_t task_handle_temp;
int auto_drive_longdistance(mission_queue *current_task)
{
    
    static int flag_running=0,flag_start_signal_send=0,error_sum=0;
    static Ort current_point;
    uint8_t set_flags[20];
    Ort info;
    for(int i=0;i<20;i++)
    {
        set_flags[i]=either;
    }
    barrier *barr=find_barrier(1);
    
    if(final_point.x==0||final_point.y==0)
    {
        msg=7;
        current_task->flag_finish=1;
        final_point_lock=0;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
        flag_start_signal_send=0;
        xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
        error_sum=0;
        send_log(8,0,0,0,0,&huart3);
        return 0;
    }
    
    if(flag_running==0)//开始起步部分
    {
        flag_running=1;
        current_point=planned_path[0];
        dZ=-atan2f(current_point.x-current_pos.x,current_point.y-current_pos.y)*180.0f/3.1415926f;
        pre_plan(current_point);   
        flags[auto_drive_status]=moving;
        global_clock=3;
        return 0;
    }
    dZ=-atan2f(barr->location.x-current_pos.x,barr->location.y-current_pos.y)*180.0f/3.1415926f;
    if(flag_start_signal_send==0&&fabs(current_pos.z+dZ)<20)
    {
        msg=6;     
        xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
        flag_start_signal_send=1;
    }
    
    if(current_point.x!=final_point.x||current_point.y!=final_point.y)
    {
        if(((current_pos.x-current_point.x)*(current_pos.x-current_point.x)+(current_pos.y-current_point.y)*(current_pos.y-current_point.y))<=deadzone*deadzone)//检测是否到达路径点附近，到达且不是最后一个则前往下一个路径点
        {
            for(int i=0;i<5;i++)
            {
                if(pow(current_pos.x-planned_path[i].x,2)+pow(current_pos.y-planned_path[i].y,2)>0.04f&&planned_path[i].x>0)
                {
                    current_point=planned_path[i];
                    planned_path[i].x=-1;
                    break;
                }
                if(i==4)
                {
                    error_sum++;
                    if(error_sum<=5)
                        break;
                    msg=7;
                    current_task->flag_finish=1;
                    final_point_lock=0;
                    flags[auto_drive_status]=current_task->info.z;
                    flag_running=0;
                    flags[drivemode]=manualmode;
                    flag_start_signal_send=0;
                    xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
                    error_sum=0;
                    send_log(9,0,0,0,0,&huart3);
                    return 0;
                }
            }
                
            
            if(((final_point.x-current_point.x)*(final_point.x-current_point.x)+(final_point.y-current_point.y)*(final_point.y-current_point.y))<0.00010f)
            {
                dZ=-atan2f(barr->location.x-current_point.x,barr->location.y-current_point.y)*180.0f/3.1415926f;
                final_point_lock=1;
            }
            thread_lock=1;
            pre_plan(current_point);
            //send_log(0x05,current_point.x,current_point.y,0,0,&huart3);
            global_clock=3;
            thread_lock=0;
        }
    }
    if(((final_point.x-current_pos.x)*(final_point.x-current_pos.x)+(final_point.y-current_pos.y)*(final_point.y-current_pos.y))<0.09f||
        (Read_Rocker(1)*Read_Rocker(1)+Read_Rocker(0)*Read_Rocker(0))>=100||(Read_Rocker(2)*Read_Rocker(2)+Read_Rocker(3)*Read_Rocker(3))>=100)//到达目的地附近
    {
        msg=7;
        error_sum=0;
        current_task->flag_finish=1;
        final_point_lock=0;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
        flag_start_signal_send=0;
        send_log(10,final_point.x,final_point.y,(float)dX,(float)dY,&huart3);
        if(((final_point.x-current_pos.x)*(final_point.x-current_pos.x)+(final_point.y-current_pos.y)*(final_point.y-current_pos.y))<0.10f)
            add_mission(AUTOPLACE,set_flags,0,&info);
        xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
        return 0;
    }
    return 0;
}
/*********************************************************************************
  *@  name      : grab_pos_set
  *@  function  : 抬升机构位置设定
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int grab_pos_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[grab_status]==stop)
    {
        flags[grab_status]=moving;
        can_msg[0]=(int)current_task->info.x;
        flags[grab_pos]=can_msg[0];
        FDCAN_SendData(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[0]==1)
    {
        current_task->flag_finish=1;
        flags[grab_status]=stop;
        cmd_feedback[0]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : hook_grasp
  *@  function  : 夹块
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int hook_grasp(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[hook_status]==stop)
    {
        flags[hook_status]=moving;
        can_msg[1]=1;
        flags[hook_pos]=grasp;
        FDCAN_SendData(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[1]==1)
    {
        current_task->flag_finish=1;
        flags[hook_status]=stop;
        cmd_feedback[1]=0;
    }
    return 0;
}    

/*********************************************************************************
  *@  name      : hook_release
  *@  function  : 释放
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int hook_release(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[hook_status]==stop)
    {
        flags[hook_status]=moving;
        can_msg[2]=1;
        flags[hook_pos]=release;
        FDCAN_SendData(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[2]==1)
    {
        current_task->flag_finish=1;
        flags[hook_status]=stop;
        cmd_feedback[2]=0;
    }
    return 0;
}    

/*********************************************************************************
  *@  name      : switcher_direction_set
  *@  function  : 设定翻转位置
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int switcher_direction_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[switcher_status]==stop)
    {
        flags[switcher_status]=moving;
        flags[switcher_pos]=current_task->info.x;
        can_msg[3]=current_task->info.x+1;
        FDCAN_SendData(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[3]==1)
    {
        current_task->flag_finish=1;
        flags[switcher_status]=stop;
        cmd_feedback[3]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : catapult_activate
  *@  function  : NULL
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int catapult_activate(mission_queue *current_task)
{
    
    return 0;
}

/*********************************************************************************
  *@  name      : pos_regulator_pos_set
  *@  function  : NULL
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int pos_regulator_pos_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[regulator_status]==stop)
    {
        flags[regulator_status]=moving;
        if(current_task->info.x!=-1)
            flags[regulator_vertical_pos]=current_task->info.x;
        if(current_task->info.y!=-1)
            flags[regulator_horizontal_pos]=current_task->info.y;
        if(current_task->info.z!=-1)
            flags[regulator_catapult_pos]=current_task->info.z;
        if(current_task->info.z!=-1)
        {
            can_msg[5]=current_task->info.z+1;
        }
        else
        {
            if(current_task->info.x==down)
            {
                if(current_task->info.y==backward)
                {
                    can_msg[4]=1;
                }
                else
                {
                    can_msg[4]=2;
                }
            }
            else
            {
                if(current_task->info.y==backward)
                {
                    can_msg[4]=3;
                }
                else
                {
                    can_msg[4]=4;
                }
            }
        }
        
        FDCAN_SendData(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[4]==1||cmd_feedback[5]==1)
    {
        current_task->flag_finish=1;
        flags[regulator_status]=stop;
        if(cmd_feedback[4]==1)
            cmd_feedback[4]=0;
        else
            cmd_feedback[5]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : pick_up_activator_pos_set
  *@  function  : NULL
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int pick_up_activator_pos_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[activator_status]==stop)
    {
        flags[activator_status]=moving;
        flags[activator_pos]=7-current_task->info.x;
        can_msg[6]=current_task->info.x+1;
        FDCAN_SendData(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[6]==1)
    {
        current_task->flag_finish=1;
        flags[activator_status]=stop;
        cmd_feedback[6]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : pick_up
  *@  function  : 抓取塔块流程函数
  *@  input     : 塔块朝向
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void pick_up(uint8_t pos,uint8_t mode)
{
    
    uint8_t set_flags[20]={either};
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    Ort info={.x=0,.y=0,.z=0};
    if(mode==automode)
        add_mission(HOOKRELEASE,set_flags,1,&info);
        
    set_flags[hook_status]=stop;
    if(pos==up||pos==down)
    {
        if(flags[switcher_pos]!=up&&flags[switcher_pos]!=down)
            info.x=up;
        else
            info.x=flags[switcher_pos];
    }
    else
    {
        info.x=forward;
        add_mission(SWITCHERDIRECTIONSET,set_flags,1,&info);
    }
        
    info.x=0;
    set_flags[hook_status]=either;
    
    if(pos==forward||pos==backward)
    {
        set_flags[grab_status]=stop;
        set_flags[switcher_status]=stop;
        info.x=tower_bottom2;
        add_mission(GRABPOSSET,set_flags,1,&info);
        set_flags[grab_status]=either;
        info.x=0;
    }
    else
    {
        set_flags[grab_status]=stop;
        set_flags[switcher_status]=stop;
        info.x=tower_bottom;
        add_mission(GRABPOSSET,set_flags,1,&info);
        set_flags[grab_status]=either;
        info.x=0;
    }

    set_flags[hook_pos]=release;
    set_flags[hook_status]=stop;
    set_flags[grab_status]=stop;
    set_flags[switcher_status]=stop;
    if(mode==automode)
        set_flags[auto_drive_status]=moving_partially_complete1;
    add_mission(HOOKGRASP,set_flags,1,&info);
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    
    if(pos!=up)
    {
        set_flags[grab_status]=stop;
        
        set_flags[hook_status]=stop;
        if(mode==automode)
            set_flags[auto_drive_status]=moving_partially_complete1;
        info.x=block_num;
        add_mission(GRABPOSSET,set_flags,1,&info);
        for(int i=0;i<total_flags;i++)
        {
            set_flags[i]=either;
        }
        info.x=0;
    
        set_flags[grab_status]=stop;
        set_flags[hook_status]=stop;
        if(pos==down)
        {
            if(flags[switcher_pos]==up)
                info.x=down;
            else
                info.x=up;
        }
        else if(pos==forward)
        {
            info.x=up;
        }
        else
        {
            info.x=down;
        }
        add_mission(SWITCHERDIRECTIONSET,set_flags,0,&info);
        info.x=0;
        set_flags[hook_status]=either;
    }
    else
    {
        
        set_flags[hook_status]=stop;
        info.x=block_num;
        add_mission(GRABPOSSET,set_flags,1,&info);
    }
    return;
}

/*********************************************************************************
  *@  name      : auto_pick_up
  *@  function  : 自动抓取塔块
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int auto_pick_up(mission_queue *current_task)
{
    static uint8_t flag_running,count=0;
    barrier *target;
    static uint8_t direction;
    Ort grasp_pos;
    double distance,distance_2_move;
    uint8_t set_flags[20];
    instruction_refresh();
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    if(flag_running==0)
    {         
        flags[lock_mode_status]=moving;
        flag_running=1;
        flags[auto_drive_status]=stop;
    }
    
    if(count==0)
    {
        target=find_barrier(block_num);
        if(target==NULL)
        {
            flags[lock_mode_status]=stop;
            current_task->flag_finish=1;
            flag_running=0;
            count=0;
            return 0;
        }
        
        target_pos.x=target->location.x;
        target_pos.y=target->location.y;
        target_pos.z=atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
        dZ=-target_pos.z;
        count=1;
    }
    if(Read_Button(13)==1)
        count=0;
        
    if(flags[auto_drive_status]==stop&&count==1)
    {
        short_drive_deadzone=0.1f;
        target_pos.z=atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
        dZ=-target_pos.z;
        distance=sqrtf((current_pos.x-target_pos.x)*(current_pos.x-target_pos.x)+(current_pos.y-target_pos.y)*(current_pos.y-target_pos.y));
        distance_2_move=distance-0.50f;
        grasp_pos.x=current_pos.x+(target_pos.x-current_pos.x)*distance_2_move/distance;
        grasp_pos.y=current_pos.y+(target_pos.y-current_pos.y)*distance_2_move/distance;
        grasp_pos.z=moving_partially_complete1;
        flags[auto_drive_status]=moving;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&grasp_pos);
        direction=(uint8_t)target->location.z;
        pick_up(target->location.z,automode);
    }
    if(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_ORE) != RESET) //如果发生了上溢错误，就将标志位清零，并重新开始接收头帧
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
    }
    if(flags[auto_drive_status]==moving_partially_complete1&&flags[hook_status]==stop)
    {
        remove_barrier(block_num);
//        if(direction==up||direction==down)
//        {
//            grasp_pos=evaluate_approach_pos(1,1.5f);
//            if(grasp_pos.x>0)
//            {
//              add_mission(AUTODRIVELONGDISTANCE,set_flags,1,&grasp_pos);
//            }
//        }
        
        flags[lock_mode_status]=stop;
        current_task->flag_finish=1;
        flag_running=0;
        count=0;
        return 0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : auto_place
  *@  function  : 自动搭塔
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int auto_place(mission_queue *current_task)
{
    static uint8_t flag_running=0;
    Ort release_pos,stop_pos;
    double distance,distance_2_move;
    uint8_t set_flags[20];
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    if(flag_running==0)
    {         
        flag_running=1;
        flags[lock_mode_status]=moving;
        if(flags[auto_drive_status]!=moving)
            flags[auto_drive_status]=stop;
    }
    target_pos=find_barrier(1)->location;
    stop_pos=evaluate_place_pos(1,0.8);
    if(flags[auto_drive_status]==stop&&flag_running==1)
    { 
        flags[auto_drive_status]=moving;
        short_drive_deadzone=0.12f;
        dZ=stop_pos.z;
        flag_running=2;
        stop_pos.z=moving_partially_complete1;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,0,&stop_pos);
        distance=sqrtf((stop_pos.x-target_pos.x)*(stop_pos.x-target_pos.x)+(stop_pos.y-target_pos.y)*(stop_pos.y-target_pos.y));
        distance_2_move=distance-0.58f;
        release_pos.x=stop_pos.x+(target_pos.x-stop_pos.x)*distance_2_move/distance;
        release_pos.y=stop_pos.y+(target_pos.y-stop_pos.y)*distance_2_move/distance;
        release_pos.z=moving_complete2;
        flags[auto_drive_status]=moving;
        place_block(block_num);
        set_flags[grab_status]=stop;
        set_flags[auto_drive_status]=moving_partially_complete1;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&release_pos);
        set_flags[grab_status]=stop;
        set_flags[hook_status]=stop;
        set_flags[auto_drive_status]=moving_complete2;
        release_pos.x=stop_pos. x-(release_pos.x-target_pos.x)*0.2f;
        release_pos.y=stop_pos.y-(release_pos.y-target_pos.y)*0.2f;
        release_pos.z=moving_complete3;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&release_pos);
        release_pos.x=tower_bottom2;
        set_flags[grab_status]=stop;
        set_flags[hook_status]=stop;
        set_flags[auto_drive_status]=moving_complete3;
        add_mission(GRABPOSSET,set_flags,0,&release_pos);
    }
    if(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_ORE) != RESET) //如果发生了上溢错误，就将标志位清零，并重新开始接收头帧
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
    }
    if(flags[auto_drive_status]==moving_complete3)
    {
        get_block_flag=0;
        remove_barrier(block_num);
        pos_reset=1;
        current_task->flag_finish=1;
        block_num++;
        flags[lock_mode_status]=stop;
        flag_running=0;
        return 0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : place_block
  *@  function  : 搭塔流程函数
  *@  input     : 塔块编号
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void place_block(uint8_t tower_num)
{
    uint8_t set_flags[20]={either};
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    Ort info={.x=0,.y=0,.z=0};
    
    
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    info.x=0;
    
    set_flags[grab_status]=stop;
    set_flags[auto_drive_status]=moving_complete2;
    add_mission(HOOKRELEASE,set_flags,0,&info);
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    info.x=0;
    
    return;
}
