#include "rgb.h"

#include "main.h"
#include "dma.h"
#include "tim.h"

uint16_t Color_RGB[RGB_NUM][3] = {0};				  /*RGB 数值*/
uint16_t RGB_WriteBuff[230 + RGB_NUM * 24 + 1] = {0}; /*DMA发送数组*/
uint16_t RGB_DEFAULT[2] = {0, 0};					  /*初始化颜色，红色*/

/**
 * @brief RGB 初始化，配置初始颜色，打开TIM_DMA
 * @param htim 定时器句柄
 * @param Channel 通道
 */
void RGB_Init(TIM_HandleTypeDef *htim, uint8_t Channel)
{
	RGB_Change(RGB_DEFAULT, 0.2f);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)RGB_WriteBuff, 278);
}


/**
 * @brief RGB 颜色改变
 * @param Color_360 色环值数组  0 ~ 360
 * @param Bright 亮度，0 ~ 1.0f
 */
void RGB_Color(TIM_HandleTypeDef *htim, uint8_t Channel, uint16_t *Color_360, float Bright)
{
	HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_3);
	RGB_Change(Color_360, Bright);
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, (uint32_t *)RGB_WriteBuff, 278);
}

/**
 * @brief TIM_DMA 发送数据填充
 * @param Color_RGB RGB 数组指针
 */
void RGB_Buffer_Input(uint16_t Color_RGB[][3])
{
	uint32_t RGB_Index = 0;
	for (int8_t j = 0; j < RGB_NUM; j++)
	{
		RGB_Index = (j + 1) * 24;

		for (int8_t i = 7; i >= 0; i--)
			RGB_WriteBuff[213 + RGB_Index - i] = ((Color_RGB[j][1] >> i) & 0x01) ? RGB_HIGH : RGB_LOW;

		for (int8_t i = 7; i >= 0; i--)
			RGB_WriteBuff[221 + RGB_Index - i] = ((Color_RGB[j][0] >> i) & 0x01) ? RGB_HIGH : RGB_LOW;

		for (int8_t i = 7; i >= 0; i--)
			RGB_WriteBuff[229 + RGB_Index - i] = ((Color_RGB[j][2] >> i) & 0x01) ? RGB_HIGH : RGB_LOW;
	}
}

/**
 * @brief RGB颜色变化，只改变数组，不发送数据
 * @param Color_360 色环值数组  0 ~ 360
 * @param Bright 亮度，0 ~ 1.0f
 */
void RGB_Change(uint16_t *Color_360, float Bright)
{
	float Scale = 0.0f;
	/*360度映射到RGB*/
	for (uint16_t i = 0; i < RGB_NUM; i++)
	{
		if (Color_360[i] < 60)
		{
			Scale = (float)Color_360[i] / 60;
			Color_RGB[i][0] = 255 * Bright;
			Color_RGB[i][1] = 255 * Scale * Bright;
			Color_RGB[i][2] = 0;
		}
		else if (Color_360[i] >= 60 && Color_360[i] < 120) /**/
		{
			Scale = (float)(Color_360[i] - 60) / 60;
			Color_RGB[i][0] = 255 * (1 - Scale) * Bright;
			Color_RGB[i][1] = 255 * Bright;
			Color_RGB[i][2] = 0;
		}
		else if (Color_360[i] >= 120 && Color_360[i] < 180)
		{
			Scale = (float)(Color_360[i] - 120) / 60;
			Color_RGB[i][0] = 0;
			Color_RGB[i][1] = 255 * Bright;
			Color_RGB[i][2] = 255 * Scale * Bright;
		}
		else if (Color_360[i] >= 180 && Color_360[i] < 240)
		{
			Scale = (float)(Color_360[i] - 180) / 60;
			Color_RGB[i][0] = 0;
			Color_RGB[i][1] = 255 * (1 - Scale) * Bright;
			Color_RGB[i][2] = 255 * Bright;
		}
		else if (Color_360[i] >= 240 && Color_360[i] < 300)
		{
			Scale = (float)(Color_360[i] - 240) / 60;
			Color_RGB[i][0] = 255 * Scale * Bright;
			Color_RGB[i][1] = 0;
			Color_RGB[i][2] = 255 * Bright;
		}
		else if (Color_360[i] >= 300 && Color_360[i] < 360)
		{
			Scale = (float)(Color_360[i] - 300) / 60;
			Color_RGB[i][0] = 255 * Bright;
			Color_RGB[i][1] = 0;
			Color_RGB[i][2] = 255 * (1 - Scale) * Bright;
		}
	}
	RGB_Buffer_Input(Color_RGB);
}
