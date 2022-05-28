/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_task.h"
#include "my_robot.h"
#include "nrf.h"
#include "math.h"
#include "fdcan_bsp.h"
#include "move.h"
#include "Tinn.h"
#include "rgb.h"
#include "arm_math.h"
#include "VESC_CAN.h"
#include "Remote_Control.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t flags[20]={0}; //标志位区
uint8_t backup_flags[20]={0};//备份标志位区
mission_queue *mission_queue_head=NULL;//任务队列队头
mission_queue *backup_mission_queue_head=NULL;//备份任务队列队头
uint8_t task_reset=0;
uint8_t block_num=0;//计数器
uint8_t backup_block_num=0;//备份计数器
mission_queue *running_task;
extern Ort debug;
uint8_t get_block_flag=0;
uint8_t focus_mode=0;
uint8_t lock_status=0;
extern Tinn velocity_nn_x,velocity_nn_y;
SemaphoreHandle_t fdcan_queue_mutex;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Robot */
osThreadId_t RobotHandle;
const osThreadAttr_t Robot_attributes = {
  .name = "Robot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for manualmove */
osThreadId_t manualmoveHandle;
const osThreadAttr_t manualmove_attributes = {
  .name = "manualmove",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for error_detector */
osThreadId_t error_detectorHandle;
const osThreadAttr_t error_detector_attributes = {
  .name = "error_detector",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for debug_msg */
osThreadId_t debug_msgHandle;
const osThreadAttr_t debug_msg_attributes = {
  .name = "debug_msg",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void add_mission(int mission_name,uint8_t *request,uint8_t flag_nessary,Ort *info);//任务添加函数
void task_handler(void *task_info);//任务执行函数
void save_status(void);
void load_status(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void RobotTask(void *argument);
void manual_move(void *argument);
void errordetector(void *argument);
void send_debug_msg(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Robot */
  RobotHandle = osThreadNew(RobotTask, NULL, &Robot_attributes);

  /* creation of manualmove */
  manualmoveHandle = osThreadNew(manual_move, NULL, &manualmove_attributes);

  /* creation of error_detector */
  error_detectorHandle = osThreadNew(errordetector, NULL, &error_detector_attributes);

  /* creation of debug_msg */
  debug_msgHandle = osThreadNew(send_debug_msg, NULL, &debug_msg_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    mission_queue *task_temp,*task_temp2;
    int flag_task_unavailable=0;
    TaskHandle_t *task_handle_temp;
    fdcan_queue_mutex=xSemaphoreCreateMutex();
  /* Infinite loop */
  for(;;)
  {
    if(mission_queue_head!=NULL)
      {
          task_temp=mission_queue_head;
          do
          {
              for(int i=0;i<total_flags;i++)
              {
                  if(task_temp->request[i]!=flags[i]&&task_temp->request[i]!=either)
                  {
                      flag_task_unavailable=1;
                      break;
                  }
              }
              if(flag_task_unavailable==0)
              {
                  task_handle_temp=(TaskHandle_t*)malloc(sizeof(TaskHandle_t));
                  task_temp->handle=task_handle_temp;
                  if(task_temp->taskname==autodrivelongdistance)
                  {
                      xTaskCreate(task_handler,"abab",30000,(void*)task_temp,(osPriority_t) osPriorityHigh,task_handle_temp);
                  }
                  else
                      xTaskCreate(task_handler,"abab",0x100,(void*)task_temp,(osPriority_t) osPriorityNormal,task_handle_temp);
                  if(task_temp==mission_queue_head)
                  {
                      mission_queue_head=mission_queue_head->next;
                      break;
                  }
                  else
                  {
                      task_temp2=mission_queue_head;
                      while(task_temp2->next!=task_temp)
                      {
                          task_temp2=task_temp2->next;
                      }
                      task_temp2->next=task_temp->next;
                      break;
                  }
              }
              else if(task_temp->flag_necessary==1)
              {
                  flag_task_unavailable=0;
                  break;
              }
              flag_task_unavailable=0;
              task_temp=task_temp->next;
          }while(task_temp!=NULL);
      }
    osDelay(2);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_RobotTask */
/**
* @brief Function implementing the Robot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotTask */
void RobotTask(void *argument)
{
  /* USER CODE BEGIN RobotTask */
    uint8_t set_flags[20]={either};
    uint8_t last_key_status[29]={0};
    Ort info={.x=0,.y=0,.z=0};
    int count=0,last_target_id=0;
    
    
    barrier *target,target_temp;
  /* Infinite loop */
  for(;;)
  {
	  instruction_refresh();
      if(Read_Button(0)==1&&last_key_status[0]==0)
      {
//          info.x=1;
//          add_mission(0,set_flags,0,&info);
          dZ=0;
          last_key_status[0]=1;
      }
      else if(Read_Button(1)==1&&last_key_status[1]==0)
      {
          info.z=moving_complete1;
          add_mission(AUTODRIVELONGDISTANCE,set_flags,1,&info);
//          dZ=180;
          last_key_status[1]=1;
      }
      else if(Read_Button(2)==1&&last_key_status[2]==0)
      {
          if(flags[grab_pos]>1)
            info.x=flags[grab_pos]-1;
          add_mission(GRABPOSSET,set_flags,1,&info);
          last_key_status[2]=1;
      }
      else if(Read_Button(3)==1&&last_key_status[3]==0)
      {
          if(flags[grab_pos]<6)
            info.x=flags[grab_pos]+1;
          add_mission(GRABPOSSET,set_flags,1,&info);
          last_key_status[3]=1;
      }
      else if(Read_Button(4)==1&&last_key_status[4]==0)
      {
//          if(flags[regulator_horizontal_pos]==backward)
//          {
//              info.x=up;
//              info.y=forward;
//              info.z=-1;
//          }
//          else
//          {
//              info.x=down;
//              info.y=backward;
//              info.z=-1;
//          }
          if(flags[regulator_R_pos]==sweep)
          {
              info.z=flags[regulator_L_pos];
          }
          else
          {
              info.z=sweep;
          }
          add_mission(POSREGULATORPOSSET,set_flags,0,&info);
          last_key_status[4]=1;
      }
      else if(Read_Button(5)==1&&last_key_status[5]==0)
      {
          if(flags[regulator_L_pos]==regulate)
          {
              info.x=-1;
              info.y=-1;
              info.z=standby;
          }
          else
          {
              info.x=-1;
              info.y=-1;
              info.z=regulate;
          }
          add_mission(POSREGULATORPOSSET,set_flags,0,&info);
          last_key_status[5]=1;
      }
      else if(Read_Button(6)==1&&last_key_status[6]==0)
      {
          add_mission(2,set_flags,0,&info);
          last_key_status[6]=1;
      }
      else if(Read_Button(7)==1&&last_key_status[7]==0)
      {
          add_mission(3,set_flags,0,&info);
          last_key_status[7]=1;
      }
      else if(Read_Button(8)==1&&last_key_status[8]==0)
      {
          info.x=up;
          add_mission(4,set_flags,0,&info);
          last_key_status[8]=1;
      }
      else if(Read_Button(9)==1&&last_key_status[9]==0)
      {
          info.x=forward;
          add_mission(4,set_flags,0,&info);
          last_key_status[9]=1;
      }
      else if(Read_Button(10)==1&&last_key_status[10]==0)
      {
          info.x=down;
          add_mission(4,set_flags,0,&info);
          last_key_status[10]=1;
      }
      else if(Read_Button(11)==1&&last_key_status[11]==0)
      {
          mission_queue_head=NULL;
          last_key_status[11]=1;
      }
      else if(Read_Button(12)==1&&last_key_status[12]==0&&flags[lock_mode_status]==stop)
      {
          info.x=block_num;
          get_block_flag=1;
          add_mission(AUTOPICKUP,set_flags,0,&info);
          focus_mode=0;
          last_key_status[12]=1;
      }
      else if(Read_Button(13)==1&&last_key_status[13]==0)
      {
          mission_queue_head=NULL;
          info.x=tower_bottom;
          add_mission(GRABPOSSET,set_flags,0,&info);
          info.x=block_num;
          add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
          info.x=up;
          add_mission(SWITCHERDIRECTIONSET,set_flags,0,&info);
          info.x=0;
          info.z=standby;
          add_mission(POSREGULATORPOSSET,set_flags,0,&info);
          get_block_flag=0;          
          last_key_status[13]=1;
      }
      else if(Read_Button(14)==1&&last_key_status[14]==0)
      {
          extern float short_drive_deadzone;
          //flags[auto_drive_status]=moving;
          short_drive_deadzone=0.10f;
          info.x=current_pos.x+1.5f;
          info.y=current_pos.y+1.5f;
          info.z=moving_partially_complete1;
          add_mission(AUTODRIVESHORTDISTANCE,set_flags,0,&info);
          set_flags[auto_drive_status]=moving_partially_complete1;
          info.x=current_pos.x+1.5f;
          info.y=current_pos.y;
          info.z=moving_partially_complete2;
          add_mission(AUTODRIVESHORTDISTANCE,set_flags,0,&info);
          set_flags[auto_drive_status]=moving_partially_complete2;
          info.x=current_pos.x;
          info.y=current_pos.y;
          info.z=moving_partially_complete3;
          add_mission(AUTODRIVESHORTDISTANCE,set_flags,0,&info);
          last_key_status[14]=1;
      }
      else if(Read_Button(15)==1&&last_key_status[15]==0&&flags[lock_mode_status]==stop&&flags[hook_pos]==grasp)
      {
          add_mission(AUTOPLACE,set_flags,0,&info); 
          focus_mode=0;
          last_key_status[15]=1;
      }
      else if(Read_Button(16)==1&&last_key_status[16]==0)
      {
          if(flags[activator_pos]<6)
             info.x=flags[activator_pos]+1;
          add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
          last_key_status[16]=1;
      }
      else if(Read_Button(17)==1&&last_key_status[17]==0)
      {
          if(block_num<11)
            block_num++;
          last_key_status[17]=1;
      }
      else if(Read_Button(18)==1&&last_key_status[18]==0)
      {
          if(flags[activator_pos]>1)
             info.x=flags[activator_pos]-1;
          add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
          last_key_status[18]=1;
      }
      else if(Read_Button(19)==1&&last_key_status[19]==0)
      {
          if(block_num>2)
              block_num--;
          last_key_status[19]=1;
      }
      else if(Read_Button(20)==1&&last_key_status[20]==0)
      {
          if(focus_mode==1)
              focus_mode=0;
          else
              focus_mode=1;
          last_key_status[20]=1;
      }
      else if(Read_Button(21)==1)//全局重启
      {
          __disable_irq();//关闭总中断
          NVIC_SystemReset();//请求单片机重启
          last_key_status[21]=1;
      }
      else if(Read_Button(26)==1&&last_key_status[26]==0)
      {
          
          last_key_status[26]=1;
      }
      else if(Read_Button(27)==1&&last_key_status[27]==0)
      {
          
          last_key_status[27]=1;
      }
      
      if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)&&flags[regulator_R_pos]==sweep)
      { 
          info.x=100;
          add_mission(TASKQUEUEDELAY,set_flags,0,&info);
          set_flags[delay_status]=delay_complete;
          info.z=standby;
          flags[regulator_R_pos]=standby;
          add_mission(POSREGULATORPOSSET,set_flags,0,&info);
      }
      
      for(int i=0;i<total_flags;i++)
      {
          set_flags[i]=either;
      }
      for(int i=0;i<=28;i++)
      {
          if(Read_Button(i)==0&&last_key_status[i]==1)
          {
              last_key_status[i]=0;
          }
      }
      //task_reset=Read_Button(14);
                  
      target=find_barrier(block_num);
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)||HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
      {
            
            extern uint16_t RGB_DEFAULT[2];
            RGB_DEFAULT[0]=30;
            RGB_Color(&htim8,TIM_CHANNEL_3,RGB_DEFAULT,0.2f);
      }
      else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
      {
            extern uint16_t RGB_DEFAULT[2];
            RGB_DEFAULT[0]=180;
            RGB_Color(&htim8,TIM_CHANNEL_3,RGB_DEFAULT,0.2f);
//          if(activator_flag==0)
//          {
//            add_mission(HOOKGRASP,set_flags,0,&info);
//              activator_flag=1;
//          }
      }
      else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))
      {
          extern uint16_t RGB_DEFAULT[2];
            RGB_DEFAULT[0]=0;
            RGB_Color(&htim8,TIM_CHANNEL_3,RGB_DEFAULT,1.0f);
      }
      else
      {
          extern uint16_t RGB_DEFAULT[2];
          RGB_DEFAULT[0]=270;
          RGB_Color(&htim8,TIM_CHANNEL_3,RGB_DEFAULT,0.2f);
      }
      
    
      if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))&&get_block_flag!=1)
      {
          if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
          {
            focus_mode=0;
            if(get_block_flag==0)
                pick_up(target->location.z,manualmode,0);
            else
                pick_up(target->location.z,manualmode,1);
            get_block_flag=1;
          }
      }
      if(target!=NULL)
      {
//          debug.x=fabs(current_pos.z-atan2f(target->location.x-current_pos.x,target->location.y-current_pos.y)*180.0f/3.1415926f);
//          debug.y=fabs(sqrt(pow(current_pos.x-target->location.x,2)+pow(current_pos.y-target->location.y,2))-0.595f);
          //((fabs(sqrt(pow(current_pos.x-target->location.x,2)+pow(current_pos.y-target->location.y,2))-114.595f)<0.05f&&target->last_update_time<=500&&flags[auto_drive_status]!=moving)||
          
          if(fabs(current_pos.z-atan2f(target->location.x-current_pos.x,target->location.y-current_pos.y)*180.0f/3.1415926f)<50.0f
              &&(pow(target->location.x-current_pos.x,2)+pow(target->location.y-current_pos.y,2)<2.3f)
              &&get_block_flag==0&&target->last_update_time<300
              &&flags[grab_pos]==tower_bottom&&flags[grab_status]==stop)
          {
              if(target->location.z==forward||target->location.z==backward)
              {
                  info.x=block_num+5;
                  if(block_num>6)
                      info.x-=5;
                  add_mission(GRABPOSSET,set_flags,0,&info);
                  info.x=tower_block_5;
                  add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
                  info.x=forward;
                  add_mission(SWITCHERDIRECTIONSET,set_flags,0,&info);
                  info.z=regulate;
                  set_flags[grab_status]=stop;
                  add_mission(POSREGULATORPOSSET,set_flags,0,&info);
                  get_block_flag=2;
              }
//              else
//                  get_block_flag=3;
          }
      }
      extern PID_T pid_deg;
      static int lock_flag=0;
      debug.x=((arm_sin_f32(current_pos.z*3.1415926f/180.f)*(target->location.x-current_pos.x)+arm_cos_f32(current_pos.z*3.1415926f/180.f)*(target->location.y-current_pos.y))/sqrt(pow(target->location.x-current_pos.x,2)+pow(target->location.y-current_pos.y,2)));
      debug.y=pow(target->location.x-current_pos.x,2)+pow(target->location.y-current_pos.y,2);
      if(target!=NULL&&focus_mode==1&&((arm_sin_f32(current_pos.z*3.1415926f/180.f)*(target->location.x-current_pos.x)+arm_cos_f32(current_pos.z*3.1415926f/180.f)*(target->location.y-current_pos.y))/sqrt(pow(target->location.x-current_pos.x,2)+pow(target->location.y-current_pos.y,2)))>0.766f&&pow(target->location.x-current_pos.x,2)+pow(target->location.y-current_pos.y,2)<5.76f&&target->last_update_time<300)
      {
          lock_status=1;
         if(last_target_id!=target->barrier_ID)
         {
             target_temp=*target;
             last_target_id=target->barrier_ID;
         }
         if(count>10&&lock_flag==1)
         {
             target_temp=*target;
             count=0;
         }
         else if(lock_flag==0&&count>25)
         {
             target_temp=*target;
             count=0;
             lock_flag=1;
         }
         if(pow(target_temp.location.x-current_pos.x,2)+pow(target_temp.location.y-current_pos.y,2)>0.09f)
            dZ=-atan2f(target_temp.location.x-current_pos.x,target_temp.location.y-current_pos.y)*180.0f/3.1415926f;
         count++;
        pid_deg.PID_MAX=75.0f;
      }
      else
      {
          pid_deg.PID_MAX=200.0f;
          lock_flag=0;
          lock_status=0;
      }
      
      
      
    osDelay(10);
  }
  /* USER CODE END RobotTask */
}

/* USER CODE BEGIN Header_manual_move */
/**
* @brief Function implementing the manualmove thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_manual_move */
void manual_move(void *argument)
{
  /* USER CODE BEGIN manual_move */
    
  /* Infinite loop */
  for(;;)
  {
          instruction_refresh();
      if((Read_Rocker(1)*Read_Rocker(1)+Read_Rocker(0)*Read_Rocker(0))>=100||(Read_Rocker(2)*Read_Rocker(2)+Read_Rocker(3)*Read_Rocker(3))>=100)
      {
        flags[0]=manualmode;
        static int rocker[4]={0};
        if((Read_Rocker(1)*Read_Rocker(1)+Read_Rocker(0)*Read_Rocker(0))>=100&&abs(Read_Rocker(1))<=190&&abs(Read_Rocker(0))<=190)
        {
            rocker[0]=Read_Rocker(0);
            rocker[1]=Read_Rocker(1);
        }
        else
        {
            rocker[0]=0;
            rocker[1]=0;
        }
        dX=(0.00005876f*rocker[0]*rocker[0]*rocker[0]+0.2471129f*rocker[0])/90.0f;
        dY=(0.00005876f*rocker[1]*rocker[1]*rocker[1]+0.2471129f*rocker[1])/90.0f;
        if((Read_Rocker(2)*Read_Rocker(2)+Read_Rocker(3)*Read_Rocker(3))>=100&&abs(Read_Rocker(2))<=190&&abs(Read_Rocker(3))<=190)
        {
            rocker[2]=-Read_Rocker(2);
            rocker[3]=-Read_Rocker(3);
            if(lock_status==0)
                dZ+=rocker[2]/1000.0f;
            else
            {
                dX+=-rocker[2]/90.0f*arm_cos_f32(current_pos.z*3.1415926f/180.0f);
                dY+=rocker[2]/90.0f*arm_sin_f32(current_pos.z*3.1415926f/180.0f);
            }
            if(dZ>=180)
            {
                dZ-=360.0f;
            }
            if(dZ<=-180)
            {
                dZ+=360.0f;
            }
        }
        else
        {
            rocker[2]=0;
            rocker[3]=0;
        }
        
        dX+=-rocker[3]/90.0f*arm_sin_f32(current_pos.z*3.1415926f/180.0f);
        dY+=-rocker[3]/90.0f*arm_cos_f32(current_pos.z*3.1415926f/180.0f);
//        if(Read_Button(22)==1)
//        {
//            dX/=10;
//            dY/=10;
//        }
      }
    osDelay(1);
  }
  /* USER CODE END manual_move */
}

/* USER CODE BEGIN Header_errordetector */
/**
* @brief Function implementing the error_detector thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_errordetector */
void errordetector(void *argument)
{
  /* USER CODE BEGIN errordetector */
    uint8_t system_status[10];
    uint8_t transmit_buffer[8]={0};
    uint8_t temp=0;
  /* Infinite loop */
  for(;;)
  {
      if(block_color==up)
        system_status[0]=1;//1bit
      else
        system_status[0]=0;
      if(communciation_error_counter==255)
        system_status[1]=1;
      else
          system_status[1]=0;
      system_status[2]=flags[grab_status];
      if(gyro.error_counter==1000)
          system_status[3]=1;
      else
          system_status[3]=0;
      system_status[4]=flags[regulator_status];
//      system_status[5]=flags[activator_status];
      system_status[5]=deg_pid_disable;
      system_status[6]=focus_mode;
      if(block_num<7)//right(0) or left(1)
          system_status[7]=0;
      else
          system_status[7]=1;
      temp=0;
      for(int i=0;i<8;i++)
      {
          temp=temp|(system_status[i]<<i);
      }
      transmit_buffer[0]=temp;
      if(block_num<7)
          system_status[0]=7-block_num;//3bit
      else
          system_status[0]=12-block_num;//3bit
      for(int i=1;i<=4;i++)
      {
          if(VESC_Feedback[i].error_flag==1)
          {
              system_status[1]=1;//1bit
              break;
          }
          else
              system_status[1]=0;
      }
      system_status[2]=flags[auto_drive_status];//4bit
      temp=0;
      temp=temp|(system_status[0]<<5);
      temp=temp|(system_status[1]<<4);
      temp=temp|(system_status[2]);
      transmit_buffer[1]=temp;
      transmit_buffer[2]=current_pos.x*16.0f;
      transmit_buffer[3]=current_pos.y*16.0f;
      transmit_buffer[4]=(current_pos.z+180)/2;
      transmit_buffer[5]=fabs(current_speed.x)*32.0f;
      transmit_buffer[6]=fabs(current_speed.y)*32.0f;
      for(int i=0;i<8;i++)
        nRF24L01_ack_pay.Ack_Buf[i]=transmit_buffer[i];
      Ack_load(7);
      osDelay(10);
  }
  /* USER CODE END errordetector */
}

/* USER CODE BEGIN Header_send_debug_msg */
/**
* @brief Function implementing the debug_msg thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send_debug_msg */
void send_debug_msg(void *argument)
{
  /* USER CODE BEGIN send_debug_msg */
  /* Infinite loop */
  for(;;)
  {
    //   if(flags[auto_drive_status]==moving)
    //     send_log2(current_pos.x,current_pos.y,pos_plan[global_clock].x,pos_plan[global_clock].y,&huart3);
    if(fdcan_msg_queue_head!=NULL)
    {
        xSemaphoreTake(fdcan_queue_mutex,5);
        FDCAN_SendData(fdcan_msg_queue_head->fdcan,fdcan_msg_queue_head->msg,fdcan_msg_queue_head->ID,fdcan_msg_queue_head->len);
        fdcan_msg_queue *temp=fdcan_msg_queue_head;
        fdcan_msg_queue_head=fdcan_msg_queue_head->next;
        free(temp);
        xSemaphoreGive(fdcan_queue_mutex);
    }
    
    osDelay(20);
  }
  /* USER CODE END send_debug_msg */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void add_mission(int mission_name,uint8_t *request,uint8_t flag_nessary,Ort *info)//然后这里会添加任务
{
    mission_queue *temp,*searcher;
    temp=(mission_queue*)malloc(sizeof(mission_queue));
    switch(mission_name)//根据需要添加的任务的序号为结构体中的函数指针变量赋值，使用时可能需要频繁更改
    {
        case 0:{
            temp->taskname=grabposset;
            break;
        }
        case 1:{
            temp->taskname=autodirveshortdistance;
            break;
        }
        case 2:{
            temp->taskname=hookgrasp;
            break;
        }
        case 3:{
            temp->taskname=hookrelease;
            break;
        }
        case 4:{
            temp->taskname=switcherdirectionset;
            break;
        }
        case 5:{
            temp->taskname=catapultactivate;
            break;
        }
        case 6:{
            temp->taskname=autopickup;
            break;
        }
        case 7:{
            temp->taskname=autoplace;
            break;
        }
        case 8:{
            temp->taskname=autodrivelongdistance;
            break;
        }
        case 9:{
            temp->taskname=posregulatorposset;
            break;
        }
        case 10:{
            temp->taskname=pickupactivatorposset;
            break;
        }
        case 12:{
            temp->taskname=taskqueuedelay;
            break;
        }
        case 13:{
            temp->taskname=moveforward;
            break;
        }
        case 14:{
            temp->taskname=fuckblock;
            break;
        }
    }   
    temp->flag_necessary=flag_nessary;//决定是否为关键任务（不可跳过）
    temp->flag_finish=0;
    temp->next=NULL;
    temp->handle=NULL;
    temp->info.x=info->x;
    temp->info.y=info->y;
    temp->info.z=info->z;
    for(int i=0;i<total_flags;i++)
    {
        temp->request[i]=request[i];
    }
    if(mission_queue_head==NULL)
    {
        mission_queue_head=temp;
    }
    else
    {
        searcher=mission_queue_head;
        while(searcher->next!=NULL)
        {
            searcher=searcher->next;
        }
        searcher->next=temp;
    }
    return;
}

void save_status(void)
{
    for(int i=0;i<total_flags;i++)
    {
        backup_flags[i]=flags[i];
    }
    backup_mission_queue_head=mission_queue_head;
    backup_block_num=block_num;
    return;
}

void load_status(void)
{
    Ort info;
    TaskHandle_t temp;
    uint8_t setflags[20];
    for(int i=0;i<total_flags;i++)
    {
        setflags[i]=either;
    }
    task_reset=1;
    mission_queue_head=NULL;//暂定为清空任务队列
    temp=xTaskGetHandle("defaultTask");
    vTaskSuspend(temp);
    osDelay(100);
    for(int i=0;i<total_flags;i++)
    {
        flags[i]=backup_flags[i];
    }
    info.x=flags[grab_pos];
    add_mission(GRABPOSSET,setflags,0,&info);
    info.x=flags[switcher_pos];
    add_mission(SWITCHERDIRECTIONSET,setflags,0,&info);
    block_num=backup_block_num;
    for(int i=0;i<8;i++)
        cmd_feedback[i]=0;
    vTaskResume(temp);
    task_reset=0;
    return;
}

void task_handler(void *task_info)//条件满足的时候这里会开始执行任务，任务具体函数在user_task.c里面
{
    mission_queue *current_task=(mission_queue*)task_info;
    TaskHandle_t current_handle=*(current_task->handle);
    while(1)
    {
        if(current_task->flag_finish==0)
        {
            current_task->taskname(current_task);
        }
        else
        {
            free(current_task);
            vTaskDelete(current_handle);
        }
        if(task_reset==1)
        {
            free(current_task);
            for(int i=0;i<total_flags;i++)
                flags[i]=0;
            
            vTaskDelete(current_handle);
        }
        osDelay(10);
    }
}

void manual_assistant(void)
{
    barrier *target;
    float deg;
    Ort vec_deg1,vec_deg2,vec_tar,vel;
    target=find_barrier(block_num);
    if(target==NULL)
        return;
    deg=target->deg;
    vec_deg1.x=arm_sin_f32(deg*3.1415926f/180.0f);
    vec_deg1.y=arm_cos_f32(deg*3.1415926f/180.0f);
    vec_deg2.x=-vec_deg1.x;
    vec_deg2.y=-vec_deg1.y;
    vec_tar.x=(current_pos.x-target->location.x)/my_sqrt(pow(current_pos.x-target->location.x,2)+pow(current_pos.y-target->location.y,2));
    vec_tar.y=(current_pos.y-target->location.y)/my_sqrt(pow(current_pos.x-target->location.x,2)+pow(current_pos.y-target->location.y,2));
    
}

void send_msg_synchronal(void *id)
{
    uint8_t ID=*(uint8_t*)id;
    uint8_t msg_buffer[16];
    msg_buffer[0]='?';
    msg_buffer[1]='!';
    msg_buffer[2]=ID;
    msg_buffer[15]='!';
    for(int i=0;i<10;i++)
    {
        HAL_UART_Transmit(&huart8,msg_buffer,16,200);
        osDelay(5);
    }
    vTaskDelete(xTaskGetCurrentTaskHandle());
    return;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
