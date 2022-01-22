#include "user_task.h"
#include "my_robot.h"
#include "math.h"
#include "Resolve.h"
#include "fdcan_bsp.h"
#include "move.h"
/*函数指针区*/
int (*autodirveshortdistance)(mission_queue*)=auto_drive_shortdistance;
int (*grabposset)(mission_queue*)=grab_pos_set;
int (*hookgrasp)(mission_queue*)=hook_grasp;
int (*hookrelease)(mission_queue*)=hook_release;
int (*switcherdirectionset)(mission_queue*)=switcher_direction_set;
int (*catapultactivate)(mission_queue*)=catapult_activate;
int (*autopickup)(mission_queue *current_task)=auto_pick_up;
int (*autoplace)(mission_queue *current_task)=auto_place;
Ort target_pos;

/*********************************************************************************
  *@  name      : auto_drive_shortdistance
  *@  function  : 机器人短距离移动函数
  *@  input     : current_task
  *@  output    : NULL 
  *@  note      : NULL
*********************************************************************************/
int auto_drive_shortdistance(mission_queue *current_task)
{
    flags[auto_drive_status]=moving;
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
        global_clock=0;
        flag_running=1;
    }
    
    if(distance<0.010)
    {
//        dX=0;
//        dY=0;
        current_task->flag_finish=1;
        flags[auto_drive_status]=moving_complete;
        flag_running=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : auto_drive_longdistance
  *@  function  : 机器人长距离移动函数
  *@  input     : current_task
  *@  output    : NULL 
  *@  note      : NULL
*********************************************************************************/
int auto_drive_longdistance(mission_queue *current_task)
{
    flags[auto_drive_status]=moving;
    static int flag_running=0;
    static int barrier_id=0;
    check_point *check_point_temp0,*check_point_temp1,*check_point_temp2;
    
    if(flag_running==0)//开始起步部分
    {
        flag_running=1;
        check_point_head=static_path_planning(current_pos,current_task->info);
        pre_plan(check_point_head->pos);   
        global_clock=0;
    }
    
    if(check_point_head!=NULL)//检测路径点队列是否为空
    {
        #if 1
        barrier_id=check_barrier(current_pos,check_point_head->pos);//检测当前路径上受否有障碍，返回第一个检测到的障碍的编号
        if (barrier_id!=0&&global_clock>=turn_end_time)//若检测到了障碍，且不在弯道当中，进行一次局部路径规划避障
        {
            if(barrier_id>50)//如果路径点被障碍物遮挡,向后搜索直到找到一个未被遮挡的路径点
            {
                check_point_temp1=check_point_head;
                while(check_point_temp1->next!=NULL&&check_barrier(check_point_temp1->pos,check_point_temp1->next->pos)>50)
                {
                    check_point_temp1=check_point_temp1->next;
                }
                if(check_point_temp1->next!=NULL)
                    check_point_temp0=static_path_planning(current_pos,check_point_temp1->next->pos);
                else
                    check_point_temp0=check_point_head;
                check_point_head=check_point_temp0;
                while(check_point_temp0->next!=NULL)
                {
                    check_point_temp0=check_point_temp0->next;
                }
                check_point_temp0->next=check_point_temp1->next->next;
                pre_plan(check_point_head->pos);
                global_clock=0;
            }
            else//如果是中间出现障碍物
            {
                check_point_temp0=static_path_planning(current_pos,check_point_head->next->pos);
                check_point_temp1=check_point_head->next;
                check_point_head=check_point_temp0;
                while(check_point_temp0->next!=NULL)
                {
                    check_point_temp0=check_point_temp0->next;
                }
                check_point_temp0->next=check_point_temp1->next;
                pre_plan(check_point_head->pos);
                global_clock=0;
            }
            barrier_id=0;
        }
            
        check_point_temp1=check_point_head;
        check_point_temp2=NULL;
        while (check_point_head->next!=NULL&&global_clock>=turn_end_time)//检测是否可以抄近路,不在弯道当中时，则立刻尝试抄近路
        {           
            check_point_temp1=check_point_temp1->next;
            barrier_id=check_barrier(current_pos,check_point_temp1->pos);
            if (barrier_id==0)
            {
                check_point_temp2=check_point_temp1;
            }
        }
        if(check_point_temp2!=NULL)
        {
            check_point_head=check_point_temp2;
            pre_plan(check_point_head->pos);
            global_clock=0;
        }
        
        
        check_point_temp1=check_point_head;
        while(check_point_temp1->next!=NULL)//检测之后的路径点中间是否有障碍,非相邻路径点间如果没有障碍则抄近路，相邻两个路径点间有障碍则重新进行局部路径规划
        {
            check_point_temp2=check_point_temp1->next;
            while (check_point_temp2!=NULL)
            {
                barrier_id=check_barrier(current_pos,check_point_temp2->pos);
                if (barrier_id!=0&&check_point_temp2==check_point_temp1->next)
                {
                    if(barrier_id<50)
                    {
                        check_point_temp0=static_path_planning(check_point_temp1->pos,check_point_temp2->pos);
                        check_point_temp1->next = check_point_temp0;
                        while(check_point_temp0->next!=NULL)
                        {
                            check_point_temp0=check_point_temp0->next;
                        }
                        check_point_temp0->next=check_point_temp2->next;
                    }
                }
                if (barrier_id==0&&check_point_temp1->next!=check_point_temp2)
                {
                    check_point_temp1->next=check_point_temp2;
                }
            }
            check_point_temp1=check_point_temp1->next;
        }
        #endif
        if(((current_pos.x-check_point_head->pos.x)*(current_pos.x-check_point_head->pos.x)+(current_pos.y-check_point_head->pos.y)*(current_pos.y-check_point_head->pos.y))<=deadzone*deadzone)//检测是否到达路径点附近，到达且不是最后一个则前往下一个路径点
        {
            if (check_point_head->next!=NULL)
            {
                check_point_head=check_point_head->next;
                pre_plan(check_point_head->pos);
                global_clock=0;
            }
        }
    }
    
    if(((current_task->info.x-current_pos.x)*(current_task->info.x-current_pos.x)+(current_task->info.y-current_pos.y)*(current_task->info.y-current_pos.y))<0.010f)//到达目的地附近
    {
        current_task->flag_finish=1;
        flags[auto_drive_status]=moving_complete;
        flag_running=0;
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
  *@  name      : pick_up
  *@  function  : 抓取塔块流程函数
  *@  input     : 塔块朝向
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void pick_up(uint8_t pos)
{
    
    uint8_t set_flags[20]={either};
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    Ort info={.x=0,.y=0,.z=0};
    
    add_mission(HOOKRELEASE,set_flags,1,&info);
    
    set_flags[grab_status]=stop;
    info.x=1;
    add_mission(GRABPOSSET,set_flags,1,&info);
    set_flags[grab_status]=either;
    info.x=0;
    
    set_flags[hook_status]=stop;
    info.x=pos;
    add_mission(SWITCHERDIRECTIONSET,set_flags,1,&info);
    info.x=0;
    set_flags[hook_status]=either;

    set_flags[hook_pos]=release;
    set_flags[hook_status]=stop;
    set_flags[grab_status]=stop;
    set_flags[switcher_status]=stop;
    set_flags[auto_drive_status]=moving_complete;
    add_mission(HOOKGRASP,set_flags,1,&info);
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    
    if(pos!=up)
    {
        set_flags[grab_status]=stop;
        set_flags[grab_pos]=tower_bottom;
        set_flags[hook_status]=stop;
        set_flags[auto_drive_status]=moving_complete;
        info.x=tower_block_2;
        add_mission(GRABPOSSET,set_flags,1,&info);
        for(int i=0;i<total_flags;i++)
        {
            set_flags[i]=either;
        }
        info.x=0;
    
        set_flags[grab_status]=stop;
        set_flags[hook_status]=stop;
        info.x=up;
        add_mission(SWITCHERDIRECTIONSET,set_flags,1,&info);
        info.x=0;
        set_flags[hook_status]=either;
        
        info.x=tower_bottom;
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
    static int count2=0;
    static Ort ortbuffer[20];
    Ort grasp_pos;
    double tmp=0,distance,distance_2_move;
    uint8_t set_flags[20];
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    uint8_t msg_buffer[30];
    if(flag_running==0)
    {         
        flags[lock_mode_status]=moving;
        if(flags[auto_drive_status]==moving_complete)
            flags[auto_drive_status]=stop;
        msg_buffer[0]='?';
        msg_buffer[1]='!';
        msg_buffer[2]=1;
        for(int i=3;i<15;i++)msg_buffer[i]=0;
        msg_buffer[15]='!';
        for(int j=0;j<8;j++)
        {
        HAL_UART_Transmit(&huart8,msg_buffer,16,200);
        osDelay(5);
        }
        osDelay(800);
        flag_running=1;
        return 0;
    }
    if(count<20)
    {
        if(count2>10&&(target_relative_pos.y!=0))
        {
            ortbuffer[count].x=cosf(-current_pos.z*3.1415926f/180.0f)*target_relative_pos.x+(-sinf(-current_pos.z*3.1415926f/180.0f))*target_relative_pos.y+current_pos.x;
            ortbuffer[count].y=sinf(-current_pos.z*3.1415926f/180.0f)*target_relative_pos.x+cosf(-current_pos.z*3.1415926f/180.0f)*target_relative_pos.y+current_pos.y;
            count++;
        }
    }
    else
    {
        for(int i=0;i<20;i++)
        {
            tmp+=ortbuffer[i].x;
        }
        target_pos.x=tmp/20.0f;
        tmp=0;
        for(int i=0;i<20;i++)
        {
            tmp+=ortbuffer[i].y;
        }
        target_pos.y=tmp/20.0f;
    }
    if(Read_Button(13)==1)
        count=0;
        
    if(flags[auto_drive_status]==stop&&count2==200)
    {
        target_pos.z=atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
        dZ=-target_pos.z;
        distance=sqrtf((current_pos.x-target_pos.x)*(current_pos.x-target_pos.x)+(current_pos.y-target_pos.y)*(current_pos.y-target_pos.y));
        distance_2_move=distance-0.39;
        grasp_pos.x=current_pos.x+(target_pos.x-current_pos.x)*distance_2_move/distance;
        grasp_pos.y=current_pos.y+(target_pos.y-current_pos.y)*distance_2_move/distance;
        flags[auto_drive_status]=moving;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&grasp_pos);
        pick_up(target_relative_pos.z);
    }
    if(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_ORE) != RESET) //如果发生了上溢错误，就将标志位清零，并重新开始接收头帧
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
        HAL_UART_Receive_IT(&huart8,&rxtemp, 1);
    }
    if(flags[auto_drive_status]==moving_complete)
    {
        flags[lock_mode_status]=stop;
        current_task->flag_finish=1;
        flag_running=0;
        count=0;
        count2=0;
        msg_buffer[0]='?';
        msg_buffer[1]='!';
        msg_buffer[2]=2;
        for(int i=3;i<15;i++)msg_buffer[i]=0;
        msg_buffer[15]='!';
        for(int j=0;j<4;j++)
        {
        HAL_UART_Transmit(&huart8,msg_buffer,16,200);
            osDelay(10);
        }
        return 0;
    }
    if(count2<200)
    {
        count2++;
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
    static uint8_t flag_running,count=0;
    static int count2=0;
    static Ort ortbuffer[20];
    Ort release_pos;
    double tmp=0,distance,distance_2_move;
    uint8_t set_flags[20];
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    uint8_t msg_buffer[30];
    if(flag_running==0)
    {         
        flags[lock_mode_status]=moving;
        if(flags[auto_drive_status]==moving_complete)
            flags[auto_drive_status]=stop;
        msg_buffer[0]='?';
        msg_buffer[1]='!';
        msg_buffer[2]=3;
        for(int i=3;i<15;i++)msg_buffer[i]=0;
        msg_buffer[15]='!';
        for(int j=0;j<8;j++)
        {
        HAL_UART_Transmit(&huart8,msg_buffer,16,200);
        osDelay(5);
        }
        osDelay(200);
        flag_running=1;
        return 0;
    }
    if(count<20)
    {
        if(count2>10&&(target_relative_pos.y!=0))
        {
            ortbuffer[count].x=cosf(-current_pos.z*3.1415926f/180.0f)*target_relative_pos.x+(-sinf(-current_pos.z*3.1415926f/180.0f))*target_relative_pos.y+current_pos.x;
            ortbuffer[count].y=sinf(-current_pos.z*3.1415926f/180.0f)*target_relative_pos.x+cosf(-current_pos.z*3.1415926f/180.0f)*target_relative_pos.y+current_pos.y;
            count++;
        }
    }
    else
    {
        for(int i=0;i<20;i++)
        {
            tmp+=ortbuffer[i].x;
        }
        target_pos.x=tmp/20.0f;
        tmp=0;
        for(int i=0;i<20;i++)
        {
            tmp+=ortbuffer[i].y;
        }
        target_pos.y=tmp/20.0f;
    }
    if(Read_Button(13)==1)
        count=0;
    target_pos.z=atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
    dZ=-target_pos.z;    
    if(flags[auto_drive_status]==stop&&count2==200&&Read_Button(20)==1)
    {
        distance=sqrtf((current_pos.x-target_pos.x)*(current_pos.x-target_pos.x)+(current_pos.y-target_pos.y)*(current_pos.y-target_pos.y));
        distance_2_move=distance-0.39;
        release_pos.x=current_pos.x+(target_pos.x-current_pos.x)*distance_2_move/distance;
        release_pos.y=current_pos.y+(target_pos.y-current_pos.y)*distance_2_move/distance;
        flags[auto_drive_status]=moving;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&release_pos);
        place_block(tower_block_2);
    }
    if(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_ORE) != RESET) //如果发生了上溢错误，就将标志位清零，并重新开始接收头帧
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
        HAL_UART_Receive_IT(&huart8,&rxtemp, 1);
    }
    if(flags[auto_drive_status]==moving_complete)
    {
        current_task->flag_finish=1;
        count=0;
        count2=0;
        flags[lock_mode_status]=stop;
        flag_running=0;
        msg_buffer[0]='?';
        msg_buffer[1]='!';
        msg_buffer[2]=4;
        for(int i=3;i<15;i++)msg_buffer[i]=0;
        msg_buffer[15]='!';
        for(int j=0;j<4;j++)
        {
        HAL_UART_Transmit(&huart8,msg_buffer,16,200);
            osDelay(10);
        }
        return 0;
    }
    if(count2<200)
    {
        count2++;
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
    
    set_flags[auto_drive_status]=moving;
    info.x=tower_num;
    //add_mission(GRABPOSSET,set_flags,0,&info);
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    info.x=0;
    
    set_flags[grab_status]=stop;
    set_flags[auto_drive_status]=moving_complete;
    add_mission(HOOKRELEASE,set_flags,1,&info);
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    info.x=0;
    
    return;
}
