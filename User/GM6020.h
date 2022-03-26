#ifndef __GM6020_H
#define __GM6020_H	 
#include "fdcan.h"
#include "string.h"
#include "fdcan_bsp.h"

/*需自己配置CAN发送和接收回调*/
//void CAN_USER_Init(CAN_HandleTypeDef * hcan)
//{
//    CAN_FilterTypeDef sFilterConfig;
//    HAL_StatusTypeDef HAL_Status;
//    HAL_Status = HAL_Status;

//    sFilterConfig.FilterBank = 0;  //过滤器组
//    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
//    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

//    sFilterConfig.FilterIdHigh = 0x0000;  //filter id
//    sFilterConfig.FilterIdLow = 0x0000;
//    sFilterConfig.FilterMaskIdHigh = 0x0000;
//    sFilterConfig.FilterMaskIdLow = 0x0000;
//    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  //用FIFO0接收

//    sFilterConfig.FilterActivation = ENABLE;
//    sFilterConfig.SlaveStartFilterBank = 14;

//    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
//    HAL_CAN_Start(hcan); //开启CAN
//    HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

//    if(HAL_Status != HAL_OK)
//    {
//        Error_Handler();
//    }
//}

//uint8_t CAN_Send_Data(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID, uint16_t Len)
//{
//	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
//	uint8_t FreeTxNum = 0;
//	CAN_TxHeaderTypeDef TxMessage;
//	
//	TxMessage.IDE = CAN_ID_STD;  //标准帧,CAN_ID_EXT扩展帧;
//	TxMessage.RTR = CAN_RTR_DATA;  //数据帧,CAN_RTR_REMOTE遥控帧
//	TxMessage.StdId = ID;
//	TxMessage.DLC = Len;
//	
//	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);	
//		
//	while(FreeTxNum==0)  //等待空邮箱
//	{
//		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
//	}
//	
//	HAL_Delay(1); //没有延时很有可能会发送失败
//	
//	HAL_RetVal = HAL_CAN_AddTxMessage(hcan,&TxMessage,pData,(uint32_t*)CAN_TX_MAILBOX0);
//	
//	if(HAL_RetVal!=HAL_OK)
//	{
//		return 2;
//	}
//	
//	return 0;
//}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	CAN_RxHeaderTypeDef RxMessage;
//	uint8_t Rx_Data[8] = {0};
//	
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, Rx_Data);	
//	GM6020_Get_Feedback(RxMessage.StdId, Rx_Data);
//	GM6020_Pos_Rec(2);
//	
//}

#define CNT_PER_ROUND			(8192)								//编码器线程
#define GM_CNT_PER_ROUND_OUT(x)	(CNT_PER_ROUND*GM6020_Reduction_Ratio[(x-1)])
#define CLAMP(x, lower, upper)	(x >= upper ? upper : (x <= lower ? lower : x))

typedef struct
{
	int Err;
	int Err_1;
	int Err_sum;
	
	float Kp;
	float Ki;
	float Kd;
	
	int Max;
	int Min;

	float P_Out;
	float I_Out;
	float D_Out;
	float PID_Out;
}GM6020_PID;
extern uint8_t can_Sendbuf[8];
extern uint8_t GM6020_Feedback_Buf[8][6];      //电机反馈报文
extern int     GM6020_Pos[8];

uint8_t		GM6020_Set_V(int target_i,uint8_t motor_id);
void	    GM6020_Set_Speed(int goal_speed,int ID);
void	    GM6020_Set_Pos(int goal_angle,int ID);
void        GM6020_ANNPID_Init(void);

void	GM6020_Get_Feedback(uint32_t std_id,uint8_t* data_p);
int		GM6020_Get_Torque(uint8_t motor_id);
int		GM6020_Get_Speed(uint8_t motor_id);
int		GM6020_Get_Pos(uint8_t motor_id);
void	GM6020_Pos_Rec(uint8_t motor_id);									//CAN中断中实时更新位置信息

int		GM6020_Ang2Cnt(float angle,int ID);
double	GM6020_Cnt2Ang(int32_t cnt,int ID);

uint8_t GM6020_CAN_Send_Data(FDCAN_HandleTypeDef* hfdcan, uint8_t *pData, uint16_t ID, uint16_t Len);

/*速度输出死区电流*/
#define GM6020_Death_Val		(8000)
/*绝对值函数*/
#define GM6020_FABS(x)			((x>=0) ? (x) : (-x))
/*速度输出死区函数*/
#define GM6020_Death_Speed(x,y)		((GM6020_FABS(x)<(y)) ? (0) : (x))

#endif
