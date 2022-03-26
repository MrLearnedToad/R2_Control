/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Elmo_fdcan.h"
#include "nrf.h"
#include "Resolve.h"
#include "fdcan_bsp.h"
#include "my_robot.h"
#include "math.h"
#include "usart.h"
#include "move.h"
#include "Remote_Control.h"
#include "arm_math.h"
#include "HMI.h"
#include "Tinn.h"
#include "rgb.h"
#include "GM6020.h"
#include "VESC_CAN.h"
#include "Ann.h"
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

/* USER CODE BEGIN PV */
extern Kal_Filter kal_velocity_x,kal_velocity_y;
extern float dX;
extern float dY;
extern float dZ;
Ort debug;
float last_pos_x=0,last_pos_y=0;
uint8_t rxtemp=0;
int rocker[4];
uint8_t turret_buffer[8];
uint8_t dma_buffer[30]={0};
uint8_t Rx_buffer[16]={0};
uint8_t huart3_rxbuffer[16]={0};
Ort current_acceration;
Ort current_speed;
Ort current_pos={.x=6,.y=0.41f};
Ort correction_value={.x=6,.y=0.41f};
uint8_t cmd_feedback[8];
Ort pos_buffer[9];
Ort speed_buffer[6];
Ort target_relative_pos;
int global_clock;
int speed_clock;
float vx[10],vy[10],ababa;
Ort pos_log[10];
uint8_t pos_reset=0;
Tinn velocity_nn_x,velocity_nn_y;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void acceration_limit(void);
void speed_cal(void);
void update_target_info(uint8_t *data);
void DMA_recieve(void);
void executive_auto_move(void);
void send_log(uint8_t ID,float data1,float data2,float data3,float data4,UART_HandleTypeDef *uart);
void send_msg(void);
void send_init_msg(UART_HandleTypeDef *uart,uint8_t ID);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_UART8_Init();
  MX_TIM8_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
nrf_mode = true;
Handle_Init(&hspi1,01);
HAL_Delay(20);
FDCAN1_Init(&hfdcan1);
HAL_Delay(200);
HAL_TIM_Base_Start_IT(&htim7);
HAL_Delay(200);
FDCAN2_Init(&hfdcan2);
HAL_Delay(20);
HAL_UART_Receive_DMA(&huart8,dma_buffer,30);
HAL_UART_Receive_IT(&huart3,huart3_rxbuffer,16);

extern uint16_t RGB_DEFAULT[2];
RGB_DEFAULT[0]=0;
RGB_Color(&htim8,TIM_CHANNEL_3,RGB_DEFAULT,0.2f);
RGB_Init(&htim8,TIM_CHANNEL_3);

for(int i=0;i<2000;i++)
{
    HAL_Delay(1);
}

for(int i=0;i<10;i++)
{
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))
    {        
        send_init_msg(&huart8,0x02);
    }
    else
    {
        send_init_msg(&huart8,0x03);
    }
    HAL_Delay(10);
}
HAL_TIM_Base_Start_IT(&htim6);
extern uint8_t block_num;
//float 
//tnn_buffer1_1[12]={-0.416729,-0.337441,0.190049,0.40271,0.229096,0.463848,0.240863,0.377448,-0.0322686,0.223719,0.123895,0.563927},
//tnn_buffer1_2[2]={-0.257041,0.419105},
//tnn_buffer2_1[12]={0.13623,-0.440966,0.308962,0.0377088,0.434576,0.490001,0.0025015,0.445981,-0.0441973,0.395057,0.0753355,0.442013},
//tnn_buffer2_2[2]={0.326608,-0.0178685};
block_num=2;
Ort base={.x=8,.y=6};
update_barrier(1,base,0.7f);


hmi_init(&huart3);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 48;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int judge_sign(double num)
{
    if(num>0)
        return 1;
    if(num<0)
        return -1;
    if(num==0)
        return 0;
    return 0;
}

void acceration_limit()
{
    static float lastdx=0,lastdy=0;
    float current_acceration;
    
    current_acceration=sqrtf((dX-lastdx)*(dX-lastdx)+(dY-lastdy)*(dY-lastdy));
    if((lastdx*lastdx+lastdy*lastdy)>0.0004f||(dX*dX+dY*dY)>0.0004f)
    {
        if(current_acceration>0.02f)
        {
            dX=judge_sign(dX-lastdx)*0.02f+lastdx;
            dY=judge_sign(dY-lastdy)*0.02f+lastdy;
        }
    }
        lastdx=dX;
        lastdy=dY;
    return;
}

void send_msg(void)
{
    uint8_t msg_buffer[30];
    msg_buffer[0]='?';
    msg_buffer[1]='!';
    msg_buffer[2]=0x05;
    int ax,ay,az;
    ax=current_pos.x*1000;
    ay=current_pos.y*1000;
    memcpy(msg_buffer+3,&ax,4);
    memcpy(msg_buffer+7,&ay,4);
    az=(int)(current_pos.z*1000);
    if(az<0)
        az=360000+az;
    memcpy(msg_buffer+11,&az,4);
    msg_buffer[15]='!';
    HAL_UART_Transmit(&huart8,msg_buffer,16,20);
    //HAL_UART_Transmit(&huart3,msg_buffer,16,200);
    return;
}

void send_init_msg(UART_HandleTypeDef *uart,uint8_t ID)
{
    uint8_t msg_buffer[16];
    msg_buffer[0]='?';
    msg_buffer[1]='!';
    msg_buffer[2]=ID;
//    int ax,ay,az;
//    ax=data1*1000;
//    ay=data2*1000;
//    memcpy(msg_buffer+3,&ax,4);
//    memcpy(msg_buffer+7,&ay,4);
//    az=(int)(gyro.z*1000);
//    memcpy(msg_buffer+11,&az,4);
    msg_buffer[15]='!';
    HAL_UART_Transmit(uart,msg_buffer,16,200);
    return;
}

void executive_auto_move(void)
{
    double distance,pid_distance;
    distance=sqrtf((current_pos.x-pos_plan[global_clock].x)*(current_pos.x-pos_plan[global_clock].x)+(current_pos.y-pos_plan[global_clock].y)*(current_pos.y-pos_plan[global_clock].y));
    pid_distance=-Pid_Run(&pid_pos,0,distance);
    dX=(pos_plan[global_clock].x-current_pos.x)/distance*pid_distance+0.6f*speed_plan[global_clock].x;
    dY=(pos_plan[global_clock].y-current_pos.y)/distance*pid_distance+0.6f*speed_plan[global_clock].y;
    return;
}

void send_log(uint8_t ID,float data1,float data2,float data3,float data4,UART_HandleTypeDef *uart)
{
    uint8_t abaaba[18];
    abaaba[0]=0x02;
    abaaba[17]=ID;
    memcpy(abaaba+1,&data1,4);
    memcpy(abaaba+5,&data2,4);
    memcpy(abaaba+9,&data3,4);
    memcpy(abaaba+13,&data4,4);
    HAL_UART_Transmit(uart,abaaba,18,1);
    return;
}

void send_log2(float data1,float data2,float data3,float data4,UART_HandleTypeDef *uart)
{
    uint8_t abaaba[20];
    memcpy(abaaba,&data1,4);
    memcpy(abaaba+4,&data2,4);
    memcpy(abaaba+8,&data3,4);
    memcpy(abaaba+12,&data4,4);
    abaaba[19]=0x7f;
    abaaba[18]=0x80;
    abaaba[17]=0x00;
    abaaba[16]=0x00;
    HAL_UART_Transmit(uart,abaaba,20,1);
    return;
}

void speed_cal(void)
{
//    static float lastx[2]={0},lasty[2]={0};
//    static int last_gyrox=0,last_gyroy=0;
    
    
//    current_acceration.x=(current_speed.x-lastsx)/0.001f;
//    current_acceration.y=(current_speed.y-lastsy)/0.001f;
    

//    lastsx=current_speed.x;
//    lastsy=current_speed.y;
    
//    current_pos.x=current_pos.x+((float)(gyro.x-last_gyrox)/1000.0f)*arm_cos_f32(-correction_value.z*3.1415926f/180.0f)+((float)(gyro.y-last_gyroy)/1000.0f)*(-arm_sin_f32(-correction_value.z*3.1415926f/180.0f));
//    current_pos.y=current_pos.y+((float)(gyro.x-last_gyrox)/1000.0f)*arm_sin_f32(-correction_value.z*3.1415926f/180.0f)+((float)(gyro.y-last_gyroy)/1000.0f)*(arm_cos_f32(-correction_value.z*3.1415926f/180.0f));
    current_pos.x=(float)gyro.x/1000.0f+correction_value.x;
    current_pos.y=(float)gyro.y/1000.0f+correction_value.y;
    current_pos.z=gyro.z+correction_value.z;
//    last_gyrox=gyro.x;
//    last_gyroy=gyro.y;
    while(fabs(current_pos.z)>180)
    {
        if(current_pos.z>0)
            current_pos.z-=360.0f;
        else
            current_pos.z+=360.0f;
    }
    
    for(int i=4;i>=0;i--)
    {
        vx[i+1]=vx[i];
        vy[i+1]=vy[i];
    }
    vx[0]=current_pos.x;
    vy[0]=current_pos.y;
//    current_speed.x=(vx[0]+vx[1]+vx[2]-vx[3]-vx[4]-vx[5])/0.036f;
//    current_speed.y=(vy[0]+vy[1]+vy[2]-vy[3]-vy[4]-vy[5])/0.036f;
    current_speed.x=Kalman_Filter(&kal_velocity_x,(vx[0]-vx[1])/0.004f);
    current_speed.y=Kalman_Filter(&kal_velocity_x,(vy[0]-vy[1])/0.004f);
    //send_log(0x01,vx[time],vy[time],current_speed.x,current_speed.y,&huart3);
    return;
}

void update_target_info(uint8_t *data)
{
    int temp[3];
    Ort temp1;
    uint8_t temp2,cmd,id;
    cmd=data[2];
    barrier *base;
        
    if(cmd>0&&cmd<6)
    {
        memcpy(temp,data+3,4);
        memcpy(temp+1,data+7,4);
        temp2=data[11];
        if(fabs(temp[0]/1000.0f)>5||fabs(temp[1]/1000.0f)>5||fabs(temp[0]/1000.0f)<0.01||fabs(temp[1]/1000.0f)<0.01)
        {
            return;
        }
        cmd=7-cmd;
        temp1.x=(float)temp[0]/1000.0f;
        temp1.y=(float)(temp[1]+200)/1000.0f;
        temp1.z=temp2;
        temp1=coordinate_transform(temp1,current_pos);
        if(cmd>=block_num)
            update_barrier(cmd,temp1,0.5f-(cmd-2)*0.075f);
    }
    else if(cmd==6)
    {
        memcpy(temp,data+3,4);
        memcpy(temp+1,data+7,4);
        temp2=data[11];
        temp1.x=(float)(temp[0])/1000.0f;
        temp1.y=(float)(temp[1])/1000.0f;
        temp1.x-=pos_log[0].x-pos_log[5].x;
        temp1.y-=pos_log[0].y-pos_log[5].y;
        temp1.z=temp2;
        if(fabs(temp1.x)>14||fabs(temp1.y)>14)
            return;
        //send_debug_msg(&huart8,temp1.x,temp1.y,0x06);
        //temp1=coordinate_transform(temp1,pos_log[5]);        
        update_barrier(1,temp1,0.7);
    }
    else if(cmd==7)
    {
        memcpy(temp,data+3,4);
        memcpy(temp+1,data+7,4);
        id=data[11];
        temp1.x=(float)(temp[0])/1000.0f;
        temp1.y=(float)(temp[1])/1000.0f;
        if(fabs(temp1.x)>14||fabs(temp1.y)>14||temp1.x<0.5f||temp1.y<0.5f)
            return;
        update_check_point(temp1,id);
    }
    else if(cmd==8&&final_point_lock==0)
    {
        base=find_barrier(1);
        memcpy(temp,data+3,4);
        memcpy(temp+1,data+7,4);
        temp1.x=(float)(temp[0])/1000.0f;
        temp1.y=(float)(temp[1])/1000.0f;
        if(fabs(temp1.x)>14||fabs(temp1.y)>14||temp1.x<0.5f||temp1.y<0.5f)
            return;
        final_point=temp1;
        final_point.z=-atan2f(base->location.x-temp1.x,base->location.y-temp1.y)*180.0f/3.1415926f;
    }
//    else if(cmd>=7&&cmd<50)
//    {
//        memcpy(temp,data+3,4);
//        memcpy(temp+1,data+7,4);
//        memcpy(temp+2,data+11,4);
//        temp1.x=(float)(temp[0])/1000.0f;
//        temp1.y=(float)(temp[1])/1000.0f;
//        temp1.z=(float)(temp[2])/1000.0f;
//        if(fabs(temp1.x)>14||fabs(temp1.y)>14||temp1.z>2||temp1.z<0)
//            return;
//        temp1.x-=pos_log[0].x-pos_log[5].x;
//        temp1.y-=pos_log[0].y-pos_log[5].y;
//        if(fabs(temp1.x)>14||fabs(temp1.y)>14)
//            return;      
//        update_barrier(cmd,temp1,temp1.z+0.3f);
//    }
//    else if(cmd>=50&&cmd!=114)
//    {
//        remove_barrier(cmd-50);
//    }
    else if(cmd==114)
    {
        memcpy(temp,data+3,4);
        memcpy(temp+1,data+7,4);
        
        temp1.x=(float)temp[0]/1000.0f;
        temp1.y=(float)temp[1]/1000.0f;
        memcpy(&temp1.z,data+11,4);
        if(pos_reset==1)
        {
            pos_reset=0;
            correction_value.x=temp1.x-pos_log[5].x+6.0f;
            correction_value.y=temp1.y-pos_log[5].y+0.41f;
            //correction_value.z=temp1.z-current_pos.z;
        }
    }
    return;
}

void DMA_recieve(void)
{
    for(int i=8;i>=0;i--)
    {
        pos_log[i+1].x=pos_log[i].x;
        pos_log[i+1].y=pos_log[i].y;
        pos_log[i+1].z=pos_log[i].z;
    }
    pos_log[0].x=current_pos.x;
    pos_log[0].y=current_pos.y;
    pos_log[0].z=current_pos.z;
    
    int i,j;
    for(i=0;i<15;i++)
    {
        if(dma_buffer[i]=='?'&&dma_buffer[i+1]=='!')
        {
            break;
        }
    }
    if(i==15)
        return;
    for(int j=i;j<30;j++)
    {
        if(dma_buffer[i+1]=='!'&&j-i+1==16)
        {
            break;
        }
    }
    if(j==30)
        return;
    for(int k=0;k<16;k++)
    {
        Rx_buffer[k]=dma_buffer[i+k];
    }
    update_target_info(Rx_buffer);
    return;
}

float *Fifo_update(float num1,float num2,float num3,float num4,int retnum)
{
    static float buffer[10][4];
    for(int i=8;i>=0;i--)
    {
        buffer[i+1][0]=buffer[i][0];
        buffer[i+1][1]=buffer[i][1];
        buffer[i+1][2]=buffer[i][2];
        buffer[i+1][3]=buffer[i][3];
    }
    buffer[0][0]=num1;
    buffer[0][1]=num2;
    buffer[0][2]=num1;
    buffer[0][3]=num1;
    return buffer[retnum];
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    static int log_clock=0;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else if(htim->Instance == TIM6)	/**/
	{
		static uint8_t ID=2,flag_sendlog=0;
        
        if(flags[auto_drive_status]!=moving)
            acceration_limit();

        
        check_dead_barrier();
        if(global_clock<1499&&thread_lock==0)
            global_clock++;
        if(flags[auto_drive_status]==moving)
        {
            executive_auto_move();
            
//            if(global_clock%8==0)
//                send_log(ID,current_pos.x,current_pos.y,pos_plan[global_clock].x,pos_plan[global_clock].y,&huart3);
            flag_sendlog=0;
        }
        if(flags[auto_drive_status]!=moving&&flags[auto_drive_status]!=stop&&flag_sendlog==0)
        {
            ID++;
            flag_sendlog=1;
        }
        Set_Pos();
//        Elmo_Run();
        
//        GM6020_Set_Speed(0,1);
        //VESC_COMMAND_SEND(&hfdcan2,3,1,(int)debug.x);  
        if(drivemode==manualmode)
        {
            dX=0;
            dY=0;
        }
        send_msg();
        if(log_clock==10)
        {
            send_log2(-current_pos.z,dZ,0,0,&huart3);
            log_clock=0;
        }
        log_clock++;
//        send_debug_msg(&huart8,0,0,0);
//        if(fabs(dX)>2.0f)
//            send_log(0x03,dX,Read_Rocker(0),Read_Rocker(1),0,&huart3);
        
        
	}
	else if(htim->Instance == TIM7)
	{
		
        speed_clock++;
        if(speed_clock==30)
            DMA_recieve();
        if(speed_clock==40)
        {
            speed_cal();
            speed_clock=0;
        }
	}
    else if(htim->Instance == TIM16)
    {
        
    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
