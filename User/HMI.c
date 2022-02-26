#include "main.h"
#include "HMI.h"
#include "string.h"

uint8_t hmi_Rec_flag;
uint8_t hmi_RxData[16];
UART_HandleTypeDef *hmi_huart;
int hmi_data[3];

void hmi_init(UART_HandleTypeDef *huart)
{
    hmi_huart=huart;
    HAL_UART_Receive_IT(hmi_huart, hmi_RxData, sizeof(hmi_RxData));
}

void hmi_send_msg(UART_HandleTypeDef *huart,float data1,float data2,float data3,uint8_t ID)
{
    //if(hmi_Rec_flag)
    {
        uint8_t transmit_buffer[14] = {0x86, ID};
        int tmp=data1;
        memcpy(&transmit_buffer[2], &tmp, sizeof(tmp));
        tmp=data2;
        memcpy(&transmit_buffer[6], &tmp, sizeof(tmp));
        tmp=data3;
        memcpy(&transmit_buffer[10], &tmp, sizeof(tmp));
        HAL_UART_Transmit(huart, transmit_buffer, sizeof(transmit_buffer), 20); 
    }
    HAL_UART_Receive_IT(hmi_huart, hmi_RxData, sizeof(hmi_RxData));
}

void hmi_IT_handler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == hmi_huart->Instance)
    {
		if(hmi_RxData[0] == 0x70 && hmi_RxData[13] == 0xff)
        {
			uint8_t tmp[4];
			memcpy(tmp, &hmi_RxData[1], 4);
			hmi_data[0] = *(int32_t*)tmp;
			memcpy(tmp, &hmi_RxData[5], 4);
			hmi_data[1] = *(int32_t*)tmp;
			memcpy(tmp, &hmi_RxData[9], 4);
			hmi_data[2] = *(int32_t*)tmp;
			hmi_RxData[13] = 0x00;			
		}
		hmi_Rec_flag = 1;
	}
}
