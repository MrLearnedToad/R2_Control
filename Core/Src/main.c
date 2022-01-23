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
extern float dX;
extern float dY;
extern float dZ;
int debug;
float last_pos_x=0,last_pos_y=0;
uint8_t rxtemp=0;
int rocker[4];
int debug;
uint8_t turret_buffer[8];
uint8_t dma_buffer[30]={0};
uint8_t Rx_buffer[16]={0};
Ort current_acceration;
Ort current_speed;
Ort current_pos;
Ort correction_value;
uint8_t cmd_feedback[8];
Ort pos_buffer[9];
Ort target_relative_pos;
int global_clock;
float vx[5],vy[5];
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
  MX_TIM16_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
nrf_mode = true;
Handle_Init(&hspi1,77);
HAL_Delay(200);
FDCAN2_Init(&hfdcan1);
HAL_Delay(200);
Elmo_Init(&hfdcan2,&htim7);
HAL_Delay(200);
Elmo_Pre_PVM(0);
HAL_Delay(20);
HAL_UART_Receive_DMA(&huart8,dma_buffer,30);
HAL_Delay(200);
HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_Base_Start_IT(&htim16);
extern uint8_t block_num;
block_num=2;
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
void acceration_limit()
{
    static float lastdx=0,lastdy=0;
    float current_acceration;
    current_acceration=sqrtf((dX-lastdx)*(dX-lastdx)+(dY-lastdy)*(dY-lastdy));
    if((lastdx*lastdx+lastdy*lastdy)>0.16f)
    {
    if(current_acceration>0.02f)
    {
        dX=(dX-lastdx)*0.02f+lastdx;
        dY=(dY-lastdy)*0.02f+lastdy;
    }
    }
        lastdx=dX;
        lastdy=dY;
    return;
}

void executive_auto_move(void)
{
    double distance,pid_distance;
    distance=sqrtf((current_pos.x-pos_plan[global_clock].x)*(current_pos.x-pos_plan[global_clock].x)+(current_pos.y-pos_plan[global_clock].y)*(current_pos.y-pos_plan[global_clock].y));
    pid_distance=-Pid_Run(&pid_pos,0,distance);
    dX=(pos_plan[global_clock].x-current_pos.x)/distance*pid_distance+0.5f*speed_plan[global_clock].x;
    dY=(pos_plan[global_clock].y-current_pos.y)/distance*pid_distance+0.5f*speed_plan[global_clock].y;
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

void speed_cal(void)
{
    static float lastx=0,lasty=0,lastsx=0,lastsy=0;
    static int time=0,last_gyrox=0,last_gyroy=0;
    
    vx[time]=(current_pos.x-lastx)/0.001f;
    vy[time]=(current_pos.y-lasty)/0.001f;
    time++;
    
    current_speed.x=(vx[0]+vx[1]+vx[2]+vx[3]+vx[4])/5.0f;
    current_speed.y=(vy[0]+vy[1]+vy[2]+vy[3]+vy[4])/5.0f;
    
    if(time==5)
        time=0;
    
    current_acceration.x=(current_speed.x-lastsx)/0.001f;
    current_acceration.y=(current_speed.y-lastsy)/0.001f;
 
    lastx=current_pos.x;
    lasty=current_pos.y;
    lastsx=current_speed.x;
    lastsy=current_speed.y;
    
    current_pos.x=current_pos.x+((float)(gyro.x-last_gyrox)/1000.0f)*cos(-correction_value.z*3.1415926f/180.0f)+((float)(gyro.y-last_gyroy)/1000.0f)*(-sin(-correction_value.z*3.1415926f/180.0f));
    current_pos.y=current_pos.y+((float)(gyro.x-last_gyrox)/1000.0f)*sin(-correction_value.z*3.1415926f/180.0f)+((float)(gyro.y-last_gyroy)/1000.0f)*(cos(-correction_value.z*3.1415926f/180.0f));
//    current_pos.x=(float)gyro.x/1000.0f;
//    current_pos.y=(float)gyro.y/1000.0f;
    current_pos.z=gyro.z+correction_value.z;
    last_gyrox=gyro.x;
    last_gyroy=gyro.y;
    while(fabs(current_pos.z)>180)
    {
        if(current_pos.z>0)
            current_pos.z-=360.0f;
        else
            current_pos.z+=360.0f;
    }
    //send_log(0x01,vx[time],vy[time],current_speed.x,current_speed.y,&huart3);
    return;
}

void update_target_info(uint8_t *data)
{
    int temp[2];

    uint8_t temp2;
    memcpy(temp,data+3,4);
    memcpy(temp+1,data+7,4);
    temp2=data[11];
    if(fabs(temp[0]/1000.0f)>5||fabs(temp[1]/1000.0f)>5||fabs(temp[0]/1000.0f)<0.01||fabs(temp[1]/1000.0f)<0.01)
    {
        return;
    }
    target_relative_pos.x=(float)temp[0]/1000.0f;
    target_relative_pos.y=(float)temp[1]/1000.0f;
    target_relative_pos.z=temp2;
    return;
}

void DMA_recieve(void)
{
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
    
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else if(htim->Instance == TIM6)	/**/
	{
		
        acceration_limit();
        DMA_recieve();
        Set_Pos();
        Elmo_Run();
        if(global_clock<2499)
            global_clock++;
        if(flags[auto_drive_status]==moving)
        {
            executive_auto_move();
        }
        else if(flags[auto_drive_status]==moving_complete)
        {
            dX=0;
            dY=0;
        }
        //send_log(0x01,vx[1],vy[1],current_speed.x,current_speed.y,&huart3);
	}
	else if(htim->Instance == TIM7)
	{
		Elmo_SendCmd(&hfdcan2);
//        if(count%10==0)
//        {
//            pos_buffer[count/10].x=gyro.x;
//            pos_buffer[count/10].y=gyro.y;
//        }
//        count++;
//        if(count==90)
//           count=0;
	}
    else if(htim->Instance == TIM16)
    {
        speed_cal();
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
