#ifndef _RGB_H_
#define _RGB_H_

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define RGB_NUM 2   /*RGB数量*/
#define RGB_HIGH 56 /*Period  的 0.75*/
#define RGB_LOW 18  /*Period  的 0.25*/

/*对外调用*/
void RGB_Init(TIM_HandleTypeDef *htim, uint8_t Channel);
void RGB_Color(TIM_HandleTypeDef *htim, uint8_t Channel, uint16_t *Color_360, float Bright);

/*对内调用*/
void RGB_Buffer_Input(uint16_t Color_RGB[][3]);
void RGB_Change(uint16_t *Color_360, float Bright);

#endif
