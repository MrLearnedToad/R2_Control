#include "move.h"
#include "arm_math.h"
#include "my_robot.h"

Ort speed_plan[1500];
Ort pos_plan[1500];
float acceleration_plan[1500][2];//加速度计划数组，第一位为x方向加速度，第二位为y方向加速度
check_point *check_point_head;
barrier *barrier_head=NULL;
extern Ort current_pos;
extern Ort current_speed;
int flag_current_path_change=0;
int flag_final_goal_change=0;
int turn_end_time=0;
double dT=0.01;
Ort output;
int flag_center_access=1;
float speed_st[1500]={0};

plan_control_block pcb[2]={{
        .acceleration_limit_increase=1.8f,
        .acceleration_limit_decrease=1.8f,
        .acceleration_limit_turn=2.2f,
        .speed_limit=1.6f,
        .turn_speed_limit=1.8f
    },{
        .acceleration_limit_increase=3.0f,
        .acceleration_limit_decrease=3.0f,
        .acceleration_limit_turn=3.4f,
        .speed_limit=3.0f,
        .turn_speed_limit=1.9f
    }
};

Ort planned_path[5];
Ort final_point;

float my_sqrt(float n)
{
    float a;
    arm_sqrt_f32(n,&a);
    return a;
}

/*********************************************************************************
  *@  name      : pre_plan
  *@  function  : 进行两个路径点之间的速度规划和路径规划，并将结果存储在speed_plan和pos_plan数组中
  *@  input     : NULL
  *@  output    : NULL
  *@  note      : 下一步引入机器人角度的规划，继续修复速度规划bug（在终点震荡）
*********************************************************************************/
void pre_plan(Ort pos_Goal)
{
    //定义：时间单位为s，距离单位为m，速度单位为m/s，加速度单位为m/s^2
    /**/
    
    
//    pos_Goal.x=check_point_head->pos.x;
//    pos_Goal.y=check_point_head->pos.y;

//    double maxspeed=speed_limit;
//    double maxA=acceleration_limit_increase;
//    double maxD=acceleration_limit_decrease;
//    double maxT=acceleration_limit_turn;
//    double dT=control_period;
    
    double maxspeed=pcb[Read_Button(22)].speed_limit;
    double maxA=pcb[Read_Button(22)].acceleration_limit_increase;
    double maxD=pcb[Read_Button(22)].acceleration_limit_decrease;
    double maxT=pcb[Read_Button(22)].acceleration_limit_turn;
    double dT=control_period;

    int i=0;
    double tmp;
    double speed=0;
    double path_len;
    Ort direction;
    Ort direction_dV;
    
    Ort dV;
    path_len=sqrt((pos_Goal.x-current_pos.x)*(pos_Goal.x-current_pos.x)+(pos_Goal.y-current_pos.y)*(pos_Goal.y-current_pos.y));
    direction.x=(pos_Goal.x-current_pos.x)/path_len;
    direction.y=(pos_Goal.y-current_pos.y)/path_len;
    
//    speed=sqrt(current_speed.x*current_speed.x+current_speed.y*current_speed.y);
    speed=sqrt(dX*dX+dY*dY);
    if(speed>pcb[Read_Button(22)].turn_speed_limit)
    {
        speed=pcb[Read_Button(22)].turn_speed_limit;
    }
//    dV.x=speed*direction.x-current_speed.x;
//    dV.y=speed*direction.y-current_speed.y;
    dV.x=speed*direction.x-dX;
    dV.y=speed*direction.y-dY;
    
    tmp=sqrt(dV.x*dV.x+dV.y*dV.y);
    direction_dV.x=dV.x/tmp;
    direction_dV.y=dV.y/tmp;

//    speed_plan[i].x=current_speed.x;
//    speed_plan[i].y=current_speed.y;
    speed_plan[i].x=dX;
    speed_plan[i].y=dY;
    
    pos_plan[i].x=current_pos.x;
    pos_plan[i].y=current_pos.y;
    acceleration_plan[i][0]=0;
    acceleration_plan[i][1]=0;
    while (fabs(((direction.x*speed_plan[i].x+direction.y*speed_plan[i].y)
        /(sqrt(direction.x*direction.x+direction.y*direction.y)*sqrt(speed_plan[i].x*speed_plan[i].x+speed_plan[i].y*speed_plan[i].y)))-1)>0.0001f)
    {
        if(i==0)
        {
//            speed_plan[i].x=current_speed.x;
//            speed_plan[i].y=current_speed.y;
            speed_plan[i].x=dX;
            speed_plan[i].y=dY;
            
            pos_plan[i].x=current_pos.x;
            pos_plan[i].y=current_pos.y;
            acceleration_plan[i][0]=0;
            acceleration_plan[i][1]=0;
        }

        if(fabs(dV.x)<fabs(direction_dV.x*maxT*dT)) 
        {
            speed_plan[i+1].x=speed_plan[i].x+dV.x;
            acceleration_plan[i+1][0]=dV.x/dT;
        }
        else 
        {
            speed_plan[i+1].x=speed_plan[i].x+maxT*dT*direction_dV.x;
            acceleration_plan[i+1][0]=maxT*direction_dV.x;
        }
        if(fabs(dV.y)<fabs(direction_dV.y*maxT*dT)) 
        {
            speed_plan[i+1].y=speed_plan[i].y+dV.y;
            acceleration_plan[i+1][1]=dV.y/dT;
        }
        else 
        {
            speed_plan[i+1].y=speed_plan[i].y+maxT*dT*direction_dV.y;
            acceleration_plan[i+1][1]=maxT*direction_dV.y;
        }
        pos_plan[i+1].x=pos_plan[i].x+dT*(speed_plan[i].x+speed_plan[i+1].x)/2;
        pos_plan[i+1].y=pos_plan[i].y+dT*(speed_plan[i].y+speed_plan[i+1].y)/2;
            
        i++;
        path_len=sqrt((pos_Goal.x-pos_plan[i].x)*(pos_Goal.x-pos_plan[i].x)+(pos_Goal.y-pos_plan[i].y)*(pos_Goal.y-pos_plan[i].y));
        direction.x=(pos_Goal.x-pos_plan[i].x)/path_len;
        direction.y=(pos_Goal.y-pos_plan[i].y)/path_len;
        dV.x=speed*direction.x-speed_plan[i].x;
        dV.y=speed*direction.y-speed_plan[i].y;
        tmp=sqrt(dV.x*dV.x+dV.y*dV.y);
        direction_dV.x=dV.x/tmp;
        direction_dV.y=dV.y/tmp;
        
    }
    
    turn_end_time=i;
    int j=0;
    if(2*path_len*maxD<speed*speed)
    {
        while (speed>0.001)
        {
            if(j==0) speed_st[j]=speed;
            if(speed<dT*maxD) 
            {
                speed=0;
                speed_st[j+1]=0;
            }
            else 
            {
                speed_st[j+1]=speed_st[j]-maxD*dT;
                speed-=maxD*dT;
            }
            j++;
        }
    }
    else if(path_len<0.5*(speed+maxspeed)*((maxspeed-speed)/maxA)+0.5*(maxspeed/maxD)*maxspeed)
    {
        maxspeed=sqrt((2*path_len*maxA*maxD+speed*speed*maxD)/(maxA+maxD));
        while (speed<maxspeed-0.0001)
        {
            if(j==0) speed_st[j]=speed;
            if((maxspeed-0.0001)-speed<dT*maxA) 
            {
                speed=maxspeed;
                speed_st[j+1]=maxspeed;
            }
            else 
            {
                speed_st[j+1]=speed_st[j]+maxA*dT;
                speed+=maxA*dT;
            }
            j++;
        }
        while (speed>0.001)
        {
            if(j==0) speed_st[j]=speed;
            if(speed<dT*maxD) 
            {
                speed=0;
                speed_st[j+1]=0;
            }
            else 
            {
                speed_st[j+1]=speed_st[j]-maxD*dT;
                speed-=maxD*dT;
            }
            j++;
        }      
    }
    else
    {
        tmp=speed;
        while (fabs(speed-maxspeed)>0.0001)
        {
            if(j==0) speed_st[j]=speed;
            if(fabs((maxspeed-0.0001)-speed)<dT*maxA) 
            {
                speed=maxspeed;
                speed_st[j+1]=maxspeed;
            }
            else 
            {
                speed_st[j+1]=speed_st[j]+maxA*dT*(fabs(maxspeed-speed)/(maxspeed-speed));
                speed+=maxA*dT*(fabs(maxspeed-speed)/(maxspeed-speed));
            }
            j++;
        }
        tmp=j+0.93f*(path_len-(0.5f*(tmp+maxspeed)*((maxspeed-tmp)/maxA)+0.5f*(maxspeed/maxD)*maxspeed))/maxspeed/dT;
        for(;j<=tmp;j++)
        {
            speed_st[j+1]=speed_st[j];
        }
        while (speed>0.001)
        {
            if(j==0) speed_st[j]=speed;
            if(speed<dT*maxD) 
            {
                speed=0;
                speed_st[j+1]=0;
            }
            else 
            {
                speed_st[j+1]=speed_st[j]-maxD*dT;
                speed-=maxD*dT;
            }
            j++;
        }
    }

    path_len=sqrt((pos_Goal.x-pos_plan[i].x)*(pos_Goal.x-pos_plan[i].x)+(pos_Goal.y-pos_plan[i].y)*(pos_Goal.y-pos_plan[i].y));
    direction.x=(pos_Goal.x-pos_plan[i].x)/path_len;
    direction.y=(pos_Goal.y-pos_plan[i].y)/path_len;
    dV.x=speed_st[0]*direction.x-speed_plan[i].x;
    dV.y=speed_st[0]*direction.y-speed_plan[i].y;
    tmp=sqrt(dV.x*dV.x+dV.y*dV.y);
    direction_dV.x=dV.x/tmp;
    direction_dV.y=dV.y/tmp;
    for (int k = 0; k <=j; k++)
    {
        speed_plan[i+1].x=speed_st[k]*direction.x;
        speed_plan[i+1].y=speed_st[k]*direction.y;
        acceleration_plan[i+1][0]=(speed_plan[i+1].x-speed_plan[i].x)/dT;
        acceleration_plan[i+1][1]=(speed_plan[i+1].y-speed_plan[i].y)/dT;
        pos_plan[i+1].x=pos_plan[i].x+dT*(speed_plan[i].x+speed_plan[i+1].x)/2;
        pos_plan[i+1].y=pos_plan[i].y+dT*(speed_plan[i].y+speed_plan[i+1].y)/2;
        path_len=sqrt((pos_Goal.x-pos_plan[i].x)*(pos_Goal.x-pos_plan[i].x)+(pos_Goal.y-pos_plan[i].y)*(pos_Goal.y-pos_plan[i].y));
        direction.x=(pos_Goal.x-pos_plan[i].x)/path_len;
        direction.y=(pos_Goal.y-pos_plan[i].y)/path_len;
        i++;
        
    }
    for(;i<1499;i++)
    {
        speed_plan[i+1].x=0;
        speed_plan[i+1].y=0;
        acceleration_plan[i+1][0]=0;
        acceleration_plan[i+1][1]=0;
        pos_plan[i+1].x=pos_Goal.x;
        pos_plan[i+1].y=pos_Goal.y;
    }
    return;
}    

/*********************************************************************************
  *@  name      : add_barrier
  *@  function  : 将新的可变障碍添加到barrier队列中
  *@  input     : 障碍位置pos，障碍的半径range，障碍的ID（使用者设定）
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void add_barrier(Ort pos,double range,int barrier_id,int deg)
{
    barrier *tmp;
    tmp=barrier_head;
    if(barrier_head==NULL)
    {
        barrier_head=(barrier*)malloc(sizeof(barrier));
        if(barrier_head==NULL)
        {
            Error_Handler();
        }
        tmp=barrier_head;
    }
    else
    {
        while (tmp->next!=NULL)
        {
            tmp=tmp->next;
        }
        tmp->next=(barrier*)malloc(sizeof(barrier));
        if(barrier_head==NULL)
        {
            Error_Handler();
        }
        tmp=tmp->next;
    }
    tmp->barrier_ID=barrier_id;
    tmp->location.x=pos.x;
    tmp->location.y=pos.y;
    tmp->location.z=pos.z;
    tmp->range=range;
    tmp->last_update_time=0;
    tmp->next = NULL;
    tmp->deg=deg;
    return;
}

/*********************************************************************************
  *@  name      : update_barrier
  *@  function  : 更新可变障碍的位置和半径,如果没有找到则自动添加
  *@  input     : 障碍ID，新的位置pos，新的半径range
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void update_barrier(int barrier_ID,Ort pos,double range,int deg)
{
    barrier *tmp=barrier_head;
    int find_flag=0;
    while (tmp!=NULL)
    {
            
        if(tmp->barrier_ID==barrier_ID)
        {
        
            tmp->location.x=pos.x;
            tmp->location.y=pos.y;
            tmp->location.z=pos.z;
            tmp->range=range;
            tmp->deg=deg;
            extern Ort debug;
            debug.z=tmp->last_update_time;
            tmp->last_update_time=0;
            find_flag=1;
        }
        tmp=tmp->next;
     }
    if(find_flag==0)
    {
        add_barrier(pos,range,barrier_ID,deg);
    }
    return;
}    

/*********************************************************************************
  *@  name      : find_barrier
  *@  function  : 搜索障碍结构体的地址
  *@  input     : 障碍ID
  *@  output    : 指向指定障碍结构体的指针
  *@  note      : NULL
*********************************************************************************/
barrier* find_barrier(int barrier_ID)
{
    barrier *tmp=barrier_head;
    while (tmp!=NULL)
    {
        if(tmp->barrier_ID==barrier_ID)
        {
            return tmp;
        }
        tmp=tmp->next;
    }
    return NULL;
}

/*********************************************************************************
  *@  name      : remove_barrier
  *@  function  : 移除指定的可变障碍
  *@  input     : 障碍ID
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void remove_barrier(int barrier_ID)
{
    barrier *tmp=barrier_head;
    barrier *tmp2;
    if(tmp->barrier_ID==barrier_ID)
    {
        free(tmp);
        barrier_head=NULL;
        return;
    }
    if(barrier_head->next==NULL)
    {
        return;
    }
    while (tmp->next!=NULL)
    {
        tmp2=tmp;
        tmp=tmp->next;
        if(tmp->barrier_ID==barrier_ID)
        {
            tmp2->next=tmp->next;
            free(tmp);
        }
    }
    return;
}

/*********************************************************************************
  *@  name      : check_dead_barrier
  *@  function  : 检查长期未更新的可变障碍
  *@  input     : NULL
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void check_dead_barrier(void)
{
    barrier *tmp=barrier_head,*tmp2;
    while(tmp!=NULL)
    {
        if(tmp->last_update_time>2000&&tmp->barrier_ID!=1&&tmp->barrier_ID<12)
        {
            tmp2=tmp->next;
            remove_barrier(tmp->barrier_ID);
            tmp=tmp2;
            continue;
        }
        if(tmp->barrier_ID<block_num&&tmp->barrier_ID!=1)
        {
            tmp2=tmp->next;
            remove_barrier(tmp->barrier_ID);
            tmp=tmp2;
            continue;
        }
        tmp->last_update_time++;
        tmp=tmp->next;
    }
    return;
}

/*********************************************************************************
  *@  name      : correct_brrier
  *@  function  : 校准后更新物品队列位置信息
  *@  input     : NULL
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void correct_brrier(Ort old_pos,Ort new_pos)
{
    barrier *temp=barrier_head;
    new_pos.x=new_pos.x-old_pos.x;
    new_pos.x=new_pos.y-old_pos.y;
    if(block_num<7)
    {
        while(temp!=NULL)
        {
            if(temp->barrier_ID!=1)
            {
                temp->location.x+=new_pos.x;
                temp->location.y+=new_pos.y;
            }
            temp=temp->next;
        }
    }
    else
    {
        while(temp!=NULL)
        {
            if(temp->barrier_ID!=12)
            {
                temp->location.x+=new_pos.x;
                temp->location.y+=new_pos.y;
            }
            temp=temp->next;
        }
    }
    return;
}

/*********************************************************************************
  *@  name      : cal_average
  *@  function  : 平均值计算函数
  *@  input     : 数组a[][]，两个指针指向数组两个元素
  *@  output    : 两个元素（含头尾）之间所有元素的平均值
  *@  note      : NULL
*********************************************************************************/
double cal_average(uint8_t a[5000][2],int head,int end)
{
    int total=0;
    for (int i = head; i <=end; i++)
    {
        total+=*(a[i]+1);
    }
    return (double)total/(double)(end-head+1);
}

/*********************************************************************************
  *@  name      : check_barrier
  *@  function  : 检查两个坐标之间是否有障碍
  *@  input     : 起点坐标pos1，终点坐标pos2
  *@  output    : 第一个检测到的障碍ID
  *@  note      : 有问题
*********************************************************************************/
int check_barrier(Ort pos1,Ort pos2,float offset_len)
{
    barrier *tmp2=barrier_head;
    if(barrier_head==NULL)return 0;
    double k,b;
    k=(pos1.y-pos2.y)/(pos1.x-pos2.x);
    b=pos1.y-k*pos1.x;

//    if(flag_center_access==0)//是否允许进入中央区？
//    {
//        if(pos2.x<1.5+0.25&&pos2.y>2.5-0.25)//判断终点是否在对方放球区
//        {
//            return -3;//对方放球区
//        }
//        if(pos2.y>9.5-0.25)//判断终点是否在对方区
//        {
//            return -2;//对方区
//        }
//        if (pos2.y>2.5-0.25&&pos2.x>1.5-0.25&&pos2.x<10.5+0.25)//判断终点是否在中央场地
//        {
//            return -1;//中央区域
//        }
//        if(pos1.x<10.5+0.25&&pos2.y>2.5-0.25)//判断路径是否穿过中央区域
//        {
//            return -4;//穿过中央区域
//        }
//    }
//    if(pos2.x<1.5+0.25&&pos2.y>2.5-0.25)//判断终点是否在对方放球区
//    {
//        return -3;//对方放球区
//    }
//    if(pos2.y>9.5-0.25)//判断终点是否在对方区
//    {
//        return -2;//对方区
//    }
//    if(pos1.x<1.5+0.25&&pos2.y>2.5-0.25)//判断路径是否穿过对方放球区
//    {
//        return -5;//穿过对方放球区
//    }

    while (tmp2!=NULL)
    { 
        if(4*pow(tmp2->location.x-b*k+k*tmp2->location.y,2)-4*(k*k+1)*(pow(tmp2->location.x,2)+pow(b-tmp2->location.y,2)-pow(tmp2->range+offset_len,2))>0 //????
            &&tmp2->barrier_ID!=block_num&&(tmp2->barrier_ID==1||tmp2->barrier_ID==12))
        {
//            if(((pos2.x-tmp2->location.x)*(pos2.x-tmp2->location.x)+(pos2.y-tmp2->location.y)*(pos2.y-tmp2->location.y))<pow(tmp2->range+offset_len,2))
//            {
//                return 100-tmp2->barrier_ID;
//            }
            if(((tmp2->location.x-pos1.x)*(pos2.x-pos1.x)+(tmp2->location.y-pos1.y)*(pos2.y-pos1.y))>0&&((tmp2->location.x-pos2.x)*(pos1.x-pos2.x)+(tmp2->location.y-pos2.y)*(pos1.y-pos2.y))>0)
                return tmp2->barrier_ID;//如果检测到了移动障碍物则返回障碍物的ID
        }
        tmp2 = tmp2->next;
    }

    

    return 0;
}

/*********************************************************************************
  *@  name      : check_barrier_point
  *@  function  : 检查坐标附近是否有障碍
  *@  input     : 坐标pos
  *@  output    : 第一个检测到的障碍ID
  *@  note      : NULL
*********************************************************************************/
int check_barrier_point(Ort pos)
{
    barrier *tmp2=barrier_head;
    if(barrier_head==NULL)return 0;
    while (tmp2!=NULL)
    {         
        if(((pos.x-tmp2->location.x)*(pos.x-tmp2->location.x)+(pos.y-tmp2->location.y)*(pos.y-tmp2->location.y))<(tmp2->range+0.35)*(tmp2->range+0.35))
        {
            return tmp2->barrier_ID;
        }
        tmp2 = tmp2->next;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : shift
  *@  function  : 重建小根堆
  *@  input     : 堆数组地址，根节点编号，堆长度
  *@  output    : NULL
  *@  note      : A*算法的一部分，不用管
*********************************************************************************/
void shift(heap_node *root[],int i,int len)
{
    heap_node *temp=root[i];
    short fin = 0;
    while (fin==0&&i<=len)
    {
        if (i*2+1<=len&&temp->cost>root[2*i+1]->cost&&root[2*i]->cost>root[2*i+1]->cost)
        {
            root[i] = root[2 * i + 1];
            i = 2 * i + 1;
            continue;
        }
        else if (i*2<=len&&temp->cost>root[2*i]->cost)
        {
            root[i] = root[2 * i];
            i = 2 * i;
            continue;
        }
        else
        {
            root[i] = temp;
            fin = 1;
        }
    }    
}

/*********************************************************************************
  *@  name      : add_node
  *@  function  : 添加新的堆节点
  *@  input     : 堆数组地址，当前堆长，需要添加的堆节点
  *@  output    : NULL
  *@  note      : 不用管
*********************************************************************************/
void add_node(heap_node *root[],int len,heap_node *item)
{
    len++;
    root[len] = item;
    int i = len / 2;
    for (; i > 0; i--)
    {
        shift(root, i, len);
    }
    return;
}

/*********************************************************************************
  *@  name      : out_node
  *@  function  : 将堆顶节点出堆
  *@  input     : 堆数组地址，当前堆长
  *@  output    : NULL
  *@  note      : 依然不用管
*********************************************************************************/
heap_node* out_node(heap_node *root[],int len)
{
    heap_node *result;
    result = root[1];
    root[1] = root[len];
    len--;
    shift(root, 1, len);
    return result;
}

/*********************************************************************************
  *@  name      : static_path_planning
  *@  function  : 静态全局路径规划函数，起点为机器人当前位置
  *@  input     : 终点坐标
  *@  output    : NULL 
  *@  note      : NULL
*********************************************************************************/
check_point* static_path_planning(Ort start_coordinate,Ort current__final_goal)
{
    if(current__final_goal.x>12||current__final_goal.x<0||current__final_goal.y>12||current__final_goal.y<0)
    {
        return NULL;
    }
    
    heap_node *boundary[1000];
    heap_node *searched_path[3000];
    uint8_t result[3000][2];
    heap_node *tmp2,*tmp3;
    int boundary_len=0;
    int searched_path_len = 0, result_len = 0;
    uint8_t map[120][120]={0};  //地图方格单位：1分米
    barrier *tmp=barrier_head;
    short barrier_x,barrier_y,barrier_range;
    int finalx = current__final_goal.x * 10;
    int finaly = current__final_goal.y * 10;
    /*
    if (flag_center_access==0)//是否允许进入中央区域
    {
        for(int i=15;i<=105+3;i++)//标记中央区为禁区（“2”）
        {
            for (int j = 25-3; j <=95; j++)
            {
                map[i][j]=2;
            }
        }
    }
    for(int i=0;i<=15+3;i++)//标记对方发球区为禁区（“2”）
    {
        for (int j = 25-3; j <=95; j++)
        {
            map[i][j]=2;
        }
    }
    for(int i=0;i<=119;i++)//标记对方区为禁区（“2”）
    {
        for (int j = 95-3; j <=119; j++)
        {
            map[i][j]=2;
        }
    }
    */
    while (tmp!=NULL)//导入可变障碍到地图中，标记障碍为“2”
    {
        if(tmp->barrier_ID==current_target_ID)//当前目标豁免
        {   
            tmp=tmp->next;
            continue;
        }
        barrier_x=tmp->location.x*10;
        barrier_y=tmp->location.y*10;
        barrier_range=tmp->range*10;
        for (int i = barrier_x-barrier_range*10-6; i <=barrier_x+barrier_range*10+6; i++)
        {
            for(int j = barrier_y-barrier_range*10-6; j<= barrier_y+barrier_range*10+6; j++)
            {
                if(i<120&&j<120&&i>=0&&j>=0)
                {
                    if (((barrier_x-i)*(barrier_x-i)+(barrier_y-j)*(barrier_y-j))<=(barrier_range+6)*(barrier_range+6))
                    {
                        map[i][j]=2;//将障碍标为2
                    }
                }
            }
        }
        tmp=tmp->next;
    }

    
    
    tmp2 = (heap_node *)malloc(sizeof(heap_node));
    tmp2->cost_current = 0;
    tmp2->last_x = -1;
    tmp2->last_y = -1;
    tmp2->x = start_coordinate.x*10;
    tmp2->y = start_coordinate.y*10;
    tmp2->cost = sqrt((finalx - tmp2->x)*(finalx - tmp2->x) + (finaly - tmp2->y)*(finaly - tmp2->y));
    map[tmp2->x][tmp2->y] = 4;//标记边界为4
    add_node(boundary, boundary_len, tmp2);//将起点加入到边界，准备开始搜索
    boundary_len++;
    
    while (boundary_len!=0)//开始搜索
    {
        tmp2 = out_node(boundary, boundary_len);
        map[tmp2->x][tmp2->y] = 1;//将走过的路标记为1
        boundary_len--;
        searched_path[searched_path_len] = tmp2;
        searched_path_len++;
        
        if(searched_path_len==2999||boundary_len>990)//溢出保护
        {
            for(int i=0;i<searched_path_len;i++)
                free(searched_path[i]);
            for(int i=1;i<boundary_len;i++)
                free(boundary[i]);
            return NULL;
        }
        
        if (tmp2->x==finalx&&tmp2->y==finaly)
        {
            break;
        }
        if(tmp2->x+1<120&&map[tmp2->x+1][tmp2->y]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x + 1;
            tmp3->y = tmp2->y;
            tmp3->cost_current = tmp2->cost_current+1.0f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
        if(tmp2->x-1>=0&&map[tmp2->x-1][tmp2->y]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x - 1;
            tmp3->y = tmp2->y;
            tmp3->cost_current = tmp2->cost_current+1.0f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
        if(tmp2->y+1<120&&map[tmp2->x][tmp2->y+1]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x;
            tmp3->y = tmp2->y + 1;
            tmp3->cost_current = tmp2->cost_current+1.0f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
        if(tmp2->y-1>=0&&map[tmp2->x][tmp2->y-1]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x;
            tmp3->y = tmp2->y - 1;
            tmp3->cost_current = tmp2->cost_current+1.0f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
        if(tmp2->x+1<120&&tmp2->y+1<120&&map[tmp2->x+1][tmp2->y+1]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x + 1;
            tmp3->y = tmp2->y + 1;
            tmp3->cost_current = tmp2->cost_current+1.414f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
        if(tmp2->x+1<120&&tmp2->y-1>=0&&map[tmp2->x+1][tmp2->y-1]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x + 1;
            tmp3->y = tmp2->y - 1;
            tmp3->cost_current = tmp2->cost_current+1.414f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
        if(tmp2->x-1>=0&&tmp2->y+1<120&&map[tmp2->x-1][tmp2->y+1]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x - 1;
            tmp3->y = tmp2->y + 1;
            tmp3->cost_current = tmp2->cost_current+1.414f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
        if(tmp2->x-1>=0&&tmp2->y-1>=0&&map[tmp2->x-1][tmp2->y-1]==0)
        {
            tmp3=(heap_node *)malloc(sizeof(heap_node));
            tmp3->x = tmp2->x - 1;
            tmp3->y = tmp2->y - 1;
            tmp3->cost_current = tmp2->cost_current+1.414f;
            tmp3->last_x = tmp2->x;
            tmp3->last_y = tmp2->y;
            tmp3->cost =tmp3->cost_current+sqrt((finalx - tmp3->x)*(finalx - tmp3->x) + (finaly - tmp3->y)*(finaly - tmp3->y));
            map[tmp3->x][tmp3->y] = 4;
            add_node(boundary, boundary_len,tmp3);
            boundary_len++;
        }
    }
    
    while (tmp2->last_x!=-1)
    {
        result[result_len][0] = tmp2->x;
        result[result_len][1] = tmp2->y;
        result_len++;
        for (int i = 0; i < searched_path_len; i++)
        {
            if(searched_path[i]->x==tmp2->last_x&&searched_path[i]->y==tmp2->last_y)
            {
                tmp2 = searched_path[i];
                break;
            }
        }
    }
    
    result[result_len][0] = tmp2->x;
    result[result_len][1] = tmp2->y;
    result_len++;
    
    check_point *path=(check_point *)malloc(sizeof(check_point));
    check_point *tmp4=path;
    int j=0;
    double R2;//相关指数
    double kl,bl;//直线的斜率与截距
    double RSS=0,AVG=0,DX=0;//RSS残差平方和，AVG平均数，DX方差
    for (int i = result_len-1; i > 0; )
    {
        j=i-1;
        while ((R2<0.01&&j>0&&i-j+1<35&&j>6)||(i-j+1<8&&j>0))//设定最大偏差，最大，最小长度
        {
            if((result[i][0]-result[j][0])!=0)
            {
                kl=(result[i][1]-result[j][1])/(result[i][0]-result[j][0]);
                bl=result[i][1]-kl*result[i][0];
                AVG=cal_average(result,j,i);
                RSS=0;
                AVG=0;
                DX=0;
                for (int k = i; k>=j; k--)
                {
                    DX+=(result[k][1]-AVG)*(result[k][1]-AVG);
                    RSS+=(result[k][1]-(kl*result[k][0]+bl))*(result[k][1]-(kl*result[k][0]+bl));
                }
                R2=RSS/DX;
            }
            j--;
        }
        tmp4->pos.x=result[j][0]/10.0f;
        tmp4->pos.y=result[j][1]/10.0f;
        tmp4->next=(check_point *)malloc(sizeof(check_point));
        if(j!=0)tmp4=tmp4->next;
        i=j;
    }
    tmp4->next=NULL;
    for(int i=0;i<searched_path_len;i++)
        free(searched_path[i]);
    for(int i=1;i<boundary_len;i++)
        free(boundary[i]);
    return path;
}

/*********************************************************************************
  *@  name      : dynamic_path_planning
  *@  function  : 动态局部路径规划
  *@  input     : 起点坐标pos1，终点坐标pos2，需要避开的障碍ID
  *@  output    : 返回一个中继路径点
  *@  note      : 下一步计划加入边界避障，另外还是觉得函数算的不太对
*********************************************************************************/
check_point* dynamic_path_planning(Ort pos1,Ort pos2,int barrier_id)
{
    barrier *barrier_goal;
    check_point *rtn = (check_point *)malloc(sizeof(check_point));
    float x0, x1, x2, y0, y1, y2, R;
    float k1, k2, k3, k4;
    Ort point1, point2;
    x1 = pos1.x;
    x2 = pos2.x;
    y1 = pos1.y;
    y2 = pos2.y;
    if(barrier_id>0)//如果障碍是非边界障碍
    {
        barrier_goal = find_barrier(barrier_id);
        x0 = barrier_goal->location.x;
        y0 = barrier_goal->location.y;
        R = barrier_goal->range + 0.3;
        k1=(2*(x0-x1)*(y0-y1)+sqrtf((2*(x0-x1)*(y0-y1))*(2*(x0-x1)*(y0-y1))-4*((x0-x1)*(x0-x1)-R*R)*((y0-y1)*(y0-y1)-R*R)))/(2*((x0-x1)*(x0-x1)-R*R));
        k2=(2*(x0-x1)*(y0-y1)-sqrtf((2*(x0-x1)*(y0-y1))*(2*(x0-x1)*(y0-y1))-4*((x0-x1)*(x0-x1)-R*R)*((y0-y1)*(y0-y1)-R*R)))/(2*((x0-x1)*(x0-x1)-R*R));
        k3=(2*(x0-x2)*(y0-y2)+sqrtf((2*(x0-x2)*(y0-y2))*(2*(x0-x2)*(y0-y2))-4*((x0-x2)*(x0-x2)-R*R)*((y0-y2)*(y0-y2)-R*R)))/(2*((x0-x2)*(x0-x2)-R*R));
        k4=(2*(x0-x2)*(y0-y2)-sqrtf((2*(x0-x2)*(y0-y2))*(2*(x0-x2)*(y0-y2))-4*((x0-x2)*(x0-x2)-R*R)*((y0-y2)*(y0-y2)-R*R)))/(2*((x0-x2)*(x0-x2)-R*R));
        if(k1*k3<0)
        {
            point1.x = ((k1 * x1 - y1) * -1 - (k3 * x2 - y2) * -1) / (k3 - k1);
            point1.y = (k1 * (k3 * x2 - y2) - k3 * (k1 * x1 - y1)) / (k3 - k1);
            point2.x = ((k2 * x1 - y1) * -1 - (k4 * x2 - y2) * -1) / (k4 - k2);
            point2.y = (k2 * (k4 * x2 - y2) - k4 * (k2 * x1 - y1)) / (k4 - k2);
        }
        else
        {
            point1.x = ((k1 * x1 - y1) * -1 - (k4 * x2 - y2) * -1) / (k4 - k1);
            point1.y = (k1 * (k4 * x2 - y2) - k4 * (k1 * x1 - y1)) / (k4 - k1);
            point2.x = ((k2 * x1 - y1) * -1 - (k3 * x2 - y2) * -1) / (k3 - k2);
            point2.y = (k2 * (k3 * x2 - y2) - k3 * (k2 * x1 - y1)) / (k3 - k2);
        }

        
    }
    if (barrier_id<0)//如果是边界，则返回一个固定的经停点
    {
        if(barrier_id==-4)
        {
            rtn->next=NULL;
            rtn->pos.x=10.5+0.3;
            rtn->pos.y=2.5-0.3;
            return rtn;
        }
        if(barrier_id==-5)
        {
            rtn->next=NULL;
            rtn->pos.x=1.5+0.3;
            rtn->pos.y=2.5-0.3;
            return rtn;
        }
    }
    
    
    if(check_barrier(pos1,point1,0.5)==0)
    {
        rtn->pos.x = point1.x;
        rtn->pos.y = point1.y;
    }
    else
    {
        rtn->pos.x = point2.x;
        rtn->pos.y = point2.y;
    }
    return rtn;
}

/*********************************************************************************
  *@  name      : move_execute
  *@  function  : 执行移动函数（开 始 飙 车）
  *@  input     : 当前的终点坐标
  *@  output    : NULL
  *@  note      : 下一步计划调整决策逻辑，决定具体输出方案(如果终点在某一个方块附近？考虑加入检测豁免)
*********************************************************************************/
void move_execute(Ort current__final_goal)
{
    static Ort last_final_goal;
    int static time_tick;//计时器静态变量，定时器设定为0.01s
    check_point *new_path,*tmp1,*tmp2,*tmp3;
    int barrier_id=0;
    tmp1=check_point_head;
    if(fabs(current__final_goal.x*current__final_goal.y-last_final_goal.x*last_final_goal.y)>0.001)//判断终点是否改变
    {
        flag_final_goal_change=1; 
        last_final_goal.x=current__final_goal.x;
        last_final_goal.y=current__final_goal.y;
        last_final_goal.z=current__final_goal.z;
    }
    else flag_final_goal_change=0;
    if(flag_final_goal_change==0)//如果终点未改变，则继续当前路径
    {
        if(check_point_head!=NULL)//检测路径点队列是否为空
        {
            #if 1
            barrier_id=check_barrier(current_pos,check_point_head->pos,0.5);//检测当前路径上受否有障碍，返回第一个检测到的障碍的编号
            if (barrier_id!=0&&time_tick>=turn_end_time)//若检测到了障碍，且不在弯道当中，进行一次局部路径规划避障
            {
                if(barrier_id>50)//如果路径点被障碍物遮挡
                {
                    barrier_id=100-barrier_id;
                    barrier_id=check_barrier(current_pos,check_point_head->next->pos,0.5f);
                    if(barrier_id<50&&barrier_id!=0)//如果下一个路径点还被遮挡，则重新进行全局路径规划，否则则删除当前目标点，直接尝试前往下一个路径点
                    {
                        new_path=dynamic_path_planning(current_pos,check_point_head->next->pos,barrier_id);
                        new_path->next=check_point_head->next;
                        check_point_head=new_path;
                    }
                    else if (barrier_id==0)
                    {
                        check_point_head=check_point_head->next;
                    }
                    
                    else
                    {
                        static_path_planning(current_pos,current__final_goal);
                    }
                }
                else
                {
                    new_path=dynamic_path_planning(current_pos,check_point_head->pos,barrier_id);
                    tmp3 = check_point_head->next;
                    check_point_head = new_path;
                    check_point_head->next = tmp3;
                }
                flag_current_path_change=1;
                barrier_id=0;
            }
            
            while (tmp1->next!=NULL&&time_tick>=turn_end_time)//检测是否可以抄近路,不在弯道当中时，则立刻抄近路
            {
                tmp1=tmp1->next;
                barrier_id=check_barrier(current_pos,tmp1->pos,0.5f);
                if (barrier_id==0)
                {
                    check_point_head=tmp1;
                    flag_current_path_change=1;
                }
            }
            
            tmp1=check_point_head;
            while(tmp1->next!=NULL)//检测之后的路径点中间是否有障碍,非相邻路径点间如果没有障碍则抄近路，相邻两个路径点间有障碍则重新进行局部路径规划
            {
                tmp2=tmp1->next;
                while (tmp2!=NULL)
                {
                    barrier_id=check_barrier(current_pos,tmp1->pos,0.5f);
                    if (barrier_id!=0&&tmp2==tmp1->next)
                    {
                        if(barrier_id<50)
                        {
                            new_path=dynamic_path_planning(tmp1->pos,tmp2->pos,barrier_id);
                            tmp3 = tmp1->next;
                            tmp1->next = new_path;
                            new_path->next = tmp3;
                        }
                    }
                    if (barrier_id==0&&tmp1->next!=tmp2)
                    {
                        tmp1->next=tmp2;
                    }
                }
                tmp1=tmp1->next;
            }
            #endif
            if(((current_pos.x-check_point_head->pos.x)*(current_pos.x-check_point_head->pos.x)+(current_pos.y-check_point_head->pos.y)*(current_pos.y-check_point_head->pos.y))<=deadzone*deadzone)//检测是否到达路径点附近，到达且不是最后一个则前往下一个路径点
            {
                if (check_point_head->next!=NULL)
                {
                    check_point_head=check_point_head->next;
                    flag_current_path_change=1;
                }
            }
        }
        else//如果路径点队列为空，则进行全局路径规划
        {
            static_path_planning(current_pos,current__final_goal);
            flag_current_path_change=1;
        }
    }
    else//如果终点改变，则重新进行全局路径规划
    {
        static_path_planning(current_pos,current__final_goal);
        flag_current_path_change=1;
    }
    if(flag_current_path_change==1)//若当前路径需要改变，则立刻进行一次速度规划和路径规划并重置计时器
    {
        pre_plan(check_point_head->pos);
        flag_current_path_change=0;
        time_tick=0;
    }
    
    
//    output.x=speed_plan[time_tick].x+Pid_Run(&pid_pos,pos_plan[time_tick].x,current_pos.x);
//    output.y=speed_plan[time_tick].y+Pid_Run(&pid_pos,pos_plan[time_tick].y,current_pos.y);
    
    if(time_tick<2499)//执行完毕后，定时器加1
    {
        time_tick++;
    }
}

/*********************************************************************************
  *@  name      : coordinate_transform
  *@  function  : 相对坐标到绝对坐标的转换
  *@  input     : 相对坐标
  *@  output    : 绝对坐标
  *@  note      : NULL
*********************************************************************************/
Ort coordinate_transform(Ort relative_pos,Ort target_pos)
{
    Ort absolute_pos;
    absolute_pos.x=cosf(-target_pos.z*3.1415926f/180.0f)*relative_pos.x+(-sinf(-target_pos.z*3.1415926f/180.0f))*relative_pos.y+target_pos.x;
    absolute_pos.y=sinf(-target_pos.z*3.1415926f/180.0f)*relative_pos.x+cosf(-target_pos.z*3.1415926f/180.0f)*relative_pos.y+target_pos.y;
    absolute_pos.z=relative_pos.z;
    return absolute_pos;
}

/*********************************************************************************
  *@  name      : evaluate_place_pos
  *@  function  : 
  *@  input     : 
  *@  output    : 
  *@  note      : NULL
*********************************************************************************/
Ort evaluate_place_pos(int target_ID,float dist)
{
    Ort target=find_barrier(target_ID)->location;
    float dist2=sqrt(pow(target.x-current_pos.x,2)+pow(target.y-current_pos.y,2));
    static float temp;
    if(target_ID==1||target_ID==12)
    {
          temp=-atan2f(target.x-current_pos.x,target.y-current_pos.y)*180.0f/3.1415926f;
//        if(temp>=-157.5f&&temp<-112.5f)
//        {
//            target.x=target.x-dist;
//            target.y=target.y+dist;
//            target.z=-135;
//        }
//        else if(temp>=-112.5f&&temp<-67.5f)
//        {
//            target.x=target.x-dist;
//            target.y=target.y;
//            target.z=-90;
//        }
//        else if(temp>=-67.5f&&temp<-22.5f)
//        {
//            target.x=target.x-dist;
//            target.y=target.y-dist;
//            target.z=-45;
//        }
//        else if(temp>=-22.5f&&temp<22.5f)
//        {
//            target.x=target.x;
//            target.y=target.y-dist;
//            target.z=0;
//        }
//        else if(temp>=22.5f&&temp<67.5f)
//        {
//            target.x=target.x+dist;
//            target.y=target.y-dist;
//            target.z=45;
//        }
//        else if(temp>=67.5f&&temp<112.5f)
//        {
//            target.x=target.x+dist;
//            target.y=target.y;
//            target.z=90;
//        }
//        else if(temp>=112.5f&&temp<157.5f)
//        {
//            target.x=target.x+dist;
//            target.y=target.y+dist;
//            target.z=135;
//        }
//        else
//        {
//            target.x=target.x;
//            target.y=target.y+dist;
//            target.z=180;
//        }

        if(dist2<dist)
        {
            dist=dist2;
        }

        if(temp>=-135&&temp<-45)
        {
            target.x=target.x-dist;
            target.y=target.y;
            target.z=-90;
        }
        else if(temp>=-45&&temp<45)
        {
            target.x=target.x;
            target.y=target.y-dist;
            target.z=0;
        }
        else if(temp>=45&&temp<135)
        {
            target.x=target.x+dist;
            target.y=target.y;
            target.z=90;
        }
        else
        {
            target.x=target.x;
            target.y=target.y+dist;
            target.z=180;
        }
    }
    else
    {
        temp=-atan2f(target.x-current_pos.x,target.y-current_pos.y)*180.0f/3.1415926f;
        if(dist2<dist)
        {
            dist=dist2;
        }
        Ort ret,direction;
        ret.z=-atan2f(target.x-current_pos.x,target.y-current_pos.y)*180.0f/3.1415926f;
        direction.x=(-target.x+current_pos.x)/my_sqrt(pow(target.x-current_pos.x,2)+pow(target.y-current_pos.y,2));
        direction.y=(-target.y+current_pos.y)/my_sqrt(pow(target.x-current_pos.x,2)+pow(target.y-current_pos.y,2));
        ret.x=target.x+direction.x*dist;
        ret.y=target.y+direction.y*dist;
        target=ret;
    }
    return target;
}

/*********************************************************************************
  *@  name      : evaluate_approach_pos
  *@  function  : 
  *@  input     : 
  *@  output    : 
  *@  note      : NULL
*********************************************************************************/
Ort evaluate_approach_pos(Ort start_location,int target_ID,float dist)
{
    Ort target=find_barrier(target_ID)->location,temp;
    float current_deg=atan2f(start_location.y-target.y,start_location.x-target.x);
    int direction=1;
    float offset_deg=0;
    current_target_ID=target_ID;
    temp.x=target.x+dist*arm_cos_f32(direction*offset_deg+current_deg);
    temp.y=target.y+dist*arm_sin_f32(direction*offset_deg+current_deg);
    while(check_barrier(temp,target,0.20f)!=0)
    {
        offset_deg+=0.0466332312f;
        direction=-direction;
        temp.x=target.x+dist*arm_cos_f32(direction*offset_deg+current_deg);
        temp.y=target.y+dist*arm_sin_f32(direction*offset_deg+current_deg);
        if(offset_deg>1.570796326f)
        {
            temp.x=-1;
            return temp;
        }
    }
    temp.z=-atan2f(target.x-temp.x,target.y-temp.y)*180.0f/3.1415926f;
    
    current_target_ID=0;
    return temp;
}

void update_check_point(Ort point,uint8_t id)
{
    planned_path[id]=point;
    return;
}

float cal_distance(Ort src,Ort tar)
{
    return my_sqrt(pow(src.x-tar.x,2)+pow(src.y-tar.y,2));
}

/*********************************************************************************
  *@  name      : Pid_Run
  *@  function  : PID函数
  *@  input     : pid设置，目标，反馈
  *@  output    : 输出
  *@  note      : NULL              
*********************************************************************************/
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
//    Pid->PID_OUT = Pid->P_OUT + Pid->I_OUT + Pid->D_OUT;
//    Pid->PID_OUT = CLAMP(Pid->PID_OUT, -Pid->PID_MAX, Pid->PID_MAX);
//    
//    return Pid->PID_OUT;
//}

/*
int main()
{
    flag_center_access=0;
    check_point *c;
    Ort a1, a2, a3;
    a3.x = 0;
    a3.y = 5;
    add_barrier(a3, 1,1);
    a3.x = 2;
    a3.y = 3;
    add_barrier(a3, 2,2);
    current_pos.x=0;
    current_pos.y=0;
    current_speed.x=0;
    current_speed.y=0;
    Ort final;
    final.x=2;
    final.y=6;
    static_path_planning(final);
    //dynamic_path_planning(current_pos,final,2);
    int i,j;
    check_point_head=(check_point*)malloc(sizeof(check_point));
    check_point_head->pos.x=10;
    check_point_head->pos.y=6;
    check_point_head->next=NULL;
    pre_plan();
    while (pos_plan[i].x!=pos_plan[i+2].x)
    {
        i++;
    }
    printf("角度\n");
    for (int j = 0; j < i-1; j++)
    {
        printf("%lf ",180*atan2f(0-pos_plan[j].x,6-pos_plan[j].y)/3.1415926);
    }
    printf("加速度\n");
    for (int j = 0; j < i-1; j++)
    {
        printf("(%lf,%lf) ",acceleration_plan[j][0],acceleration_plan[j][1]);
    }
    printf("\n");
    for (int j = 0; j < i-1; j++)
    {
        printf("(%lf,%lf) ",speed_plan[j].x,speed_plan[j].y);
    }
    
    
    
    return 0;
    
}

*/
