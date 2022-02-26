#ifndef _HMI_H_
#define _HMI_H_

#include "main.h"

extern uint8_t hmi_RxData[16];
extern int hmi_data[3];
void hmi_send_msg(UART_HandleTypeDef *huart,float data1,float data2,float data3,uint8_t ID);
void hmi_init(UART_HandleTypeDef *huart);
void hmi_IT_handler(UART_HandleTypeDef *huart);

#endif
