#include "VESC_CAN.h"
#include "fdcan_bsp.h"
/*电机的返回参数*/
Motor_INFO VESC_Feedback[VESC_MAX_ID+1];
uint8_t TXDATA[4];
/*电机速度PID参数*/
static VESC_PID VESC_SPEED_PID[VESC_MAX_ID+1] = {
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=1
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=2
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=3
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=4
};


/*电机位置PID参数*/
static VESC_PID VESC_POS_PID[VESC_MAX_ID+1] = {
	{.Kp = 1, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=1
	{.Kp = 1, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=2
	{.Kp = 1, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=3
	{.Kp = 1, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=4
};

void VESC_CAN_SENDDATA(FDCAN_HandleTypeDef *hfdcan,uint32_t Ext,uint8_t pData[]);
void VESC_ENCODE_VALUE(uint8_t pData[],int set_value);

/*********************************************************************************
  *@  name      : VESC_SET_POS
  *@  function  : 位置PID控制
  *@  input     : ID， 目标位置
  *@  output    : 无
*********************************************************************************/
void VESC_SET_POS(uint8_t ID, int Goal_pos,FDCAN_HandleTypeDef *hfdcan)
{
	VESC_POS_PID[ID].Err_1 = VESC_POS_PID[ID].Err;
	VESC_POS_PID[ID].Err = Goal_pos - VESC_Feedback[ID].cur_pos;
	VESC_POS_PID[ID].Err_sum += VESC_POS_PID[ID].Err;
	VESC_POS_PID[ID].P_Out = VESC_POS_PID[ID].Kp * VESC_POS_PID[ID].Err;
	VESC_POS_PID[ID].I_Out = VESC_POS_PID[ID].Ki * VESC_POS_PID[ID].Err_sum;
	VESC_POS_PID[ID].D_Out = VESC_POS_PID[ID].Kd * (VESC_POS_PID[ID].Err - VESC_POS_PID[ID].Err_1);
	
	VESC_POS_PID[ID].I_Out = CLAMP(VESC_POS_PID[ID].I_Out, -1000, 1000);
	VESC_POS_PID[ID].PID_Out = VESC_POS_PID[ID].P_Out + VESC_POS_PID[ID].I_Out + VESC_POS_PID[ID].D_Out;
	VESC_POS_PID[ID].PID_Out = CLAMP(VESC_POS_PID[ID].PID_Out,  VESC_POS_PID[ID].Min,  VESC_POS_PID[ID].Max);
	
	VESC_SET_SPEED(ID, VESC_POS_PID[ID].PID_Out,hfdcan);
}

/*********************************************************************************
  *@  name      : VESC_SET_SPEED
  *@  function  : 速度PID控制
  *@  input     : ID， 目标转速
  *@  output    : 无
*********************************************************************************/
void VESC_SET_SPEED(uint8_t ID,int Goal_rpm,FDCAN_HandleTypeDef *hfdcan)
{
	VESC_SPEED_PID[ID].Err_1 = VESC_SPEED_PID[ID].Err;
	VESC_SPEED_PID[ID].Err = Goal_rpm - VESC_GET_SPEED(ID);
	VESC_SPEED_PID[ID].Err_sum += VESC_SPEED_PID[ID].Err;
	VESC_SPEED_PID[ID].Err_sum = CLAMP(VESC_SPEED_PID[ID].Err_sum,-10,10);
	
	VESC_SPEED_PID[ID].P_Out = VESC_SPEED_PID[ID].Kp * VESC_SPEED_PID[ID].Err;
	VESC_SPEED_PID[ID].I_Out = VESC_SPEED_PID[ID].Ki * VESC_SPEED_PID[ID].Err_sum;
	VESC_SPEED_PID[ID].D_Out = VESC_SPEED_PID[ID].Kd * (VESC_SPEED_PID[ID].Err - VESC_SPEED_PID[ID].Err_1);
	
	VESC_SPEED_PID[ID].I_Out = CLAMP(VESC_SPEED_PID[ID].I_Out, -1000, 1000);
	VESC_SPEED_PID[ID].PID_Out = VESC_SPEED_PID[ID].P_Out + VESC_SPEED_PID[ID].I_Out + VESC_SPEED_PID[ID].D_Out;
	VESC_SPEED_PID[ID].PID_Out = CLAMP(VESC_SPEED_PID[ID].PID_Out,VESC_SPEED_PID[ID].Min,VESC_SPEED_PID[ID].Max);
	
	VESC_COMMAND_SEND(hfdcan, VESC_SET_CURRENT, ID, VESC_SPEED_PID[ID].PID_Out); 
}



/*********************************************************************************
  *@  name      : VESC_COMMAND_SEND
	*@  function  : 发送命令给VESC
  *@  input     : CAN线、命令、VESC的id值、目标值
  *@  output    : 无
*********************************************************************************/
void VESC_COMMAND_SEND(FDCAN_HandleTypeDef *hfdcan,uint32_t cmd,uint8_t id,float set_value)
{
	/*定义发送数组*/
	uint8_t pData[4]={0};
	
	/*与接收端进行设置同步*/
	switch(cmd){
		case VESC_SET_DUTY:
			set_value*=100000;break;
		case VESC_SET_CURRENT:
			set_value*=1000;break;
		case VESC_SET_CURRENT_BRAKE:
			set_value*=1000;break;
		case VESC_SET_RPM:
			set_value*=1;break;
		case VESC_SET_POSITION:
			set_value*=1000000;break;
	}
	if(VESC_Feedback[id].last_call_time<254)
        VESC_Feedback[id].last_call_time++;
    else
        VESC_Feedback[id].error_flag=1;
    
	/*将值解码至数据帧*/
	VESC_ENCODE_VALUE(pData,set_value);
	
	/*得到其扩展帧格式*/
	uint32_t IDE = (cmd<<8) | ((uint32_t)id);
	memcpy(TXDATA,pData,4);
	/*发送指令*/
	VESC_CAN_SENDDATA(hfdcan,IDE,pData);
}


/*********************************************************************************
  *@  name      : VESC_ENCODE_VALUE
  *@  function  : 将输入值拆分入四个字节的数组中 
  *@  input     : set_value pData数组指针
  *@  output    : 无
*********************************************************************************/
void VESC_ENCODE_VALUE(uint8_t pData[],int set_value)
{
	for(int i=3;i>=0;i--)
		pData[i] = (set_value >> ((3-i)*8)) & 0xFF;
}


/*********************************************************************************
  *@  name      : VESC_CAN_SENDDATA
  *@  function  : 发送CAN信号给VESC
	*@  input     : CAN线、扩展帧、需要发送的数组指针
  *@  output    : 无
*********************************************************************************/
void VESC_CAN_SENDDATA(FDCAN_HandleTypeDef *hfdcan,uint32_t Ext,uint8_t pData[])
{
	FDCAN_SendData_Ext(hfdcan,pData,Ext,FDCAN_DLC_BYTES_4,FDCAN_DATA_FRAME);
}






/*********************************************************************************
  *@  name      : VESC_CAN_DECODE
  *@  function  : 获取VESC的反馈并存入全局变量VESC_Feedback中
  *@  input     : 扩展帧、数据帧
  *@  output    : 无
*********************************************************************************/
void VESC_CAN_DECODE(uint32_t ExtID,uint8_t pData[])
{
	/*判断是否为VESC发送来的信息*/
	if(ExtID>>8 == 0x000009){
		
		/*id值为扩展帧后8位*/
		uint8_t id = ExtID & 0x00000FF;
		
		/*将此id的数据解码,得到当前电流和转速*/
		VESC_Feedback[id].cur_rpm = VESC_DECODE_RPM(pData);
		VESC_Feedback[id].cur_current = VESC_DECODE_CURRENT(pData);
		VESC_Feedback[id].cur_pos = VESC_DECODE_POS(pData);
        VESC_Feedback[id].last_call_time=0;
        VESC_Feedback[id].error_flag=0;
	}
}


/*********************************************************************************
  *@  name      : VESC_DECODE_RPM
  *@  function  : 将获取到的反馈值解码为转速
  *@  input     : 反馈数组指针
  *@  output    : 电机当前转速
*********************************************************************************/
uint32_t VESC_DECODE_RPM(uint8_t pData[])
{
	int ret_rpm = 0;
	
	ret_rpm = ((uint32_t)pData[0]<<24)|
						((uint32_t)pData[1]<<16)|
						((uint32_t)pData[2]<<8)|
						((uint32_t)pData[3]);
	
	return ret_rpm;
}


/*********************************************************************************
  *@  name      : VESC_DECODE_CURRENT
  *@  function  : 将获取到的反馈值解码为电流值
  *@  input     : 反馈数组指针
  *@  output    : 电机当前电流值
*********************************************************************************/
float VESC_DECODE_CURRENT(uint8_t pData[])
{
	short ret_current = ((uint32_t)pData[4]<<8)|
										((uint32_t)pData[5]);
	
	return (float)ret_current /10 ;
}


/*********************************************************************************
  *@  name      : VESC_DECODE_CURRENT
  *@  function  : 将获取到的反馈值解码为电流值
  *@  input     : 反馈数组指针
  *@  output    : 电机当前电流值
*********************************************************************************/
int VESC_DECODE_POS(uint8_t pData[])
{
	int ret_pos = ((uint32_t)pData[6]<<8) |
								((uint32_t)pData[7]);
	return (float)ret_pos / 100000;
}

/*********************************************************************************
  *@  name      : VESC_PID_INIT
  *@  function  : 设置速度环和位置环的PID
  *@  input     : 无
  *@  output    : 无
*********************************************************************************/
//void VESC_PID_INIT()
//{
//	/*id为2的VESC初始化PID设定*/
//	VESC_SPEED_PID[2].Kp=0.001;
//	VESC_SPEED_PID[2].Ki=0;
//	VESC_SPEED_PID[2].Kd=0;
//	VESC_POS_PID[2].Kp=1;
//	VESC_POS_PID[2].Ki=0;
//	VESC_POS_PID[2].Kd=0;
//	
//}


/*********************************************************************************
  *@  name      : VESC_GET_SPEED
  *@  function  : 获取电机当前转速
  *@  input     : ID
  *@  output    : 当前转速
*********************************************************************************/
int VESC_GET_SPEED(uint8_t ID)
{
	 return VESC_Feedback[ID].cur_rpm;
}


/*********************************************************************************
  *@  name      : VESC_SET_CurrentBrake
  *@  function  : 刹车
  *@  input     : ID  
	*								Current 按导电滑环电流看，不要超过20A
  *@  output    : null
*********************************************************************************/
void VESC_SET_CurrentBrake(uint8_t ID, int Current,FDCAN_HandleTypeDef *hfdcan)
{
	VESC_COMMAND_SEND(hfdcan, 2, ID, Current * 1000);
}





