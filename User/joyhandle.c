#include "joyhandle.h"

#include "nrf.h"
#include "string.h"

uint8_t nrf_cmd[32];	  //初始数据
int16_t nrf_trans_cmd [7]; //转换后数据
uint8_t button [26];		  //按键

int16_t abc = 0;
bool nrf_mode = 1; // nrf工作模式
bool last_nrf_mode;

int same_times = 0; //计划在定时器中统计cmd 连续 不变的数目，t>10认定为nrf断开

/**********************************************************************
 *@ 	name		: instruction_refresh
 *@	functio	: 手柄数据接收
 *@	input		: none
 *@	output	: none
 ***********************************************************************/
void instruction_refresh(void)
{
	//	if(NRF24L01_Check(&hspi4))
	//	{
	//		nrf_mode = false;
	//
	//		for(int i; i<7; i++)		nrf_trans_cmd[i] = 0;             //nrf断线，cmd清零
	//		for(int i; i<26; i++)		button[i] = 0;
	//	}
	if (nrf_mode)
	{
		if (!last_nrf_mode)
		{
			//			Handle_Init(&hspi3,false);
			last_nrf_mode = true;
		}

		int16_t *nrf_p;
		int recvui = NRF24L01_RxPacket(&hspi1, nrf_cmd);
		if (recvui == 0 && nrf_cmd[13] == 0)
		{
			int i_nrf = 0;
			nrf_p = (int16_t *)nrf_cmd;
			for (i_nrf = 0; i_nrf < 7; i_nrf++)
			{
				nrf_trans_cmd[i_nrf] = nrf_p[i_nrf];
			}
		}

		abc = nrf_trans_cmd[0];
	}
}

void instruction_sand(void)
{
	if (NRF24L01_Check(&hspi1))
	{
		nrf_mode = false;
	}
	if (nrf_mode)
	{
		if (!last_nrf_mode)
		{
			last_nrf_mode = true;
		}

		int recvui = NRF24L01_TxPacket(&hspi1, nrf_cmd);

		abc = nrf_trans_cmd[0];
	}
}

/**********************************************************************
 *@ 	name		: read_rocker
 *@	functio	: 获得摇杆数据		编号0~3
 *@	input		: none
 *@	output	: none
 ***********************************************************************/
int16_t Read_Rocker(int id)
{
	return -nrf_trans_cmd[2 + id];
}

uint8_t Read_Button(int id)
{
	return button[id];
}

/**********************************************************************
*@ 	name		: read_keys
*@	functio	: 获得按键数据，放在 boll button[26]中
*@	input		: none
*@	output	: none

		 20        |		  21
 16	 17	 06    | 04  08  09
 18	 19	 07    | 05  10  11
 |22| |23|     |     12  13
   00 |24|     |          |25|
02    03       |     15  14
   01          |

***********************************************************************/
void read_keys(void)
{

	button[0] = (nrf_trans_cmd[0] & 0x0001);
	button[1] = (nrf_trans_cmd[0] & 0x0002) >> 1;
	button[2] = (nrf_trans_cmd[0] & 0x0004) >> 2;
	button[3] = (nrf_trans_cmd[0] & 0x0008) >> 3;
	button[4] = (nrf_trans_cmd[0] & 0x0010) >> 4;
	button[5] = (nrf_trans_cmd[0] & 0x0020) >> 5;
	button[6] = (nrf_trans_cmd[0] & 0x0040) >> 6;
	button[7] = (nrf_trans_cmd[0] & 0x0080) >> 7;
	button[8] = (nrf_trans_cmd[1] & 0x0001);
	button[9] = (nrf_trans_cmd[1] & 0x0002) >> 1;
	button[10] = (nrf_trans_cmd[1] & 0x0004) >> 2;
	button[11] = (nrf_trans_cmd[1] & 0x0008) >> 3;
	button[12] = (nrf_trans_cmd[1] & 0x0010) >> 4;
	button[13] = (nrf_trans_cmd[1] & 0x0020) >> 5;
	button[14] = (nrf_trans_cmd[1] & 0x0040) >> 6;
	button[15] = (nrf_trans_cmd[1] & 0x0080) >> 7;
	button[16] = (nrf_trans_cmd[0] & 0x0400) >> 10;
	button[17] = (nrf_trans_cmd[0] & 0x0800) >> 11;
	button[18] = (nrf_trans_cmd[0] & 0x1000) >> 12;
	button[19] = (nrf_trans_cmd[0] & 0x2000) >> 13;
	button[20] = (nrf_trans_cmd[0] & 0x4000) >> 14;
	button[21] = (nrf_trans_cmd[0] & 0x8000) >> 15;
	button[22] = (nrf_trans_cmd[6] & 0x0008) >> 3;
	button[23] = (nrf_trans_cmd[6] & 0x0004) >> 2;
	button[24] = (nrf_trans_cmd[6] & 0x0002) >> 1;
	button[25] = (nrf_trans_cmd[6] & 0x0001);
}
