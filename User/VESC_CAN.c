#include "VESC_CAN.h"
#include "fdcan_bsp.h"
/*����ķ��ز���*/
Motor_INFO VESC_Feedback[VESC_MAX_ID+1];
uint8_t TXDATA[4];
/*����ٶ�PID����*/
static VESC_PID VESC_SPEED_PID[VESC_MAX_ID+1] = {
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=1
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=2
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=3
	{.Kp = 0.001f, .Ki = 0, .Kd = 0, .Max = 10, .Min = -10},//ID=4
};


/*���λ��PID����*/
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
  *@  function  : λ��PID����
  *@  input     : ID�� Ŀ��λ��
  *@  output    : ��
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
  *@  function  : �ٶ�PID����
  *@  input     : ID�� Ŀ��ת��
  *@  output    : ��
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
	*@  function  : ���������VESC
  *@  input     : CAN�ߡ����VESC��idֵ��Ŀ��ֵ
  *@  output    : ��
*********************************************************************************/
void VESC_COMMAND_SEND(FDCAN_HandleTypeDef *hfdcan,uint32_t cmd,uint8_t id,float set_value)
{
	/*���巢������*/
	uint8_t pData[4]={0};
	
	/*����ն˽�������ͬ��*/
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
    
	/*��ֵ����������֡*/
	VESC_ENCODE_VALUE(pData,set_value);
	
	/*�õ�����չ֡��ʽ*/
	uint32_t IDE = (cmd<<8) | ((uint32_t)id);
	memcpy(TXDATA,pData,4);
	/*����ָ��*/
	VESC_CAN_SENDDATA(hfdcan,IDE,pData);
}


/*********************************************************************************
  *@  name      : VESC_ENCODE_VALUE
  *@  function  : ������ֵ������ĸ��ֽڵ������� 
  *@  input     : set_value pData����ָ��
  *@  output    : ��
*********************************************************************************/
void VESC_ENCODE_VALUE(uint8_t pData[],int set_value)
{
	for(int i=3;i>=0;i--)
		pData[i] = (set_value >> ((3-i)*8)) & 0xFF;
}


/*********************************************************************************
  *@  name      : VESC_CAN_SENDDATA
  *@  function  : ����CAN�źŸ�VESC
	*@  input     : CAN�ߡ���չ֡����Ҫ���͵�����ָ��
  *@  output    : ��
*********************************************************************************/
void VESC_CAN_SENDDATA(FDCAN_HandleTypeDef *hfdcan,uint32_t Ext,uint8_t pData[])
{
	FDCAN_SendData_Ext(hfdcan,pData,Ext,FDCAN_DLC_BYTES_4,FDCAN_DATA_FRAME);
}






/*********************************************************************************
  *@  name      : VESC_CAN_DECODE
  *@  function  : ��ȡVESC�ķ���������ȫ�ֱ���VESC_Feedback��
  *@  input     : ��չ֡������֡
  *@  output    : ��
*********************************************************************************/
void VESC_CAN_DECODE(uint32_t ExtID,uint8_t pData[])
{
	/*�ж��Ƿ�ΪVESC����������Ϣ*/
	if(ExtID>>8 == 0x000009){
		
		/*idֵΪ��չ֡��8λ*/
		uint8_t id = ExtID & 0x00000FF;
		
		/*����id�����ݽ���,�õ���ǰ������ת��*/
		VESC_Feedback[id].cur_rpm = VESC_DECODE_RPM(pData);
		VESC_Feedback[id].cur_current = VESC_DECODE_CURRENT(pData);
		VESC_Feedback[id].cur_pos = VESC_DECODE_POS(pData);
        VESC_Feedback[id].last_call_time=0;
        VESC_Feedback[id].error_flag=0;
	}
}


/*********************************************************************************
  *@  name      : VESC_DECODE_RPM
  *@  function  : ����ȡ���ķ���ֵ����Ϊת��
  *@  input     : ��������ָ��
  *@  output    : �����ǰת��
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
  *@  function  : ����ȡ���ķ���ֵ����Ϊ����ֵ
  *@  input     : ��������ָ��
  *@  output    : �����ǰ����ֵ
*********************************************************************************/
float VESC_DECODE_CURRENT(uint8_t pData[])
{
	short ret_current = ((uint32_t)pData[4]<<8)|
										((uint32_t)pData[5]);
	
	return (float)ret_current /10 ;
}


/*********************************************************************************
  *@  name      : VESC_DECODE_CURRENT
  *@  function  : ����ȡ���ķ���ֵ����Ϊ����ֵ
  *@  input     : ��������ָ��
  *@  output    : �����ǰ����ֵ
*********************************************************************************/
int VESC_DECODE_POS(uint8_t pData[])
{
	int ret_pos = ((uint32_t)pData[6]<<8) |
								((uint32_t)pData[7]);
	return (float)ret_pos / 100000;
}

/*********************************************************************************
  *@  name      : VESC_PID_INIT
  *@  function  : �����ٶȻ���λ�û���PID
  *@  input     : ��
  *@  output    : ��
*********************************************************************************/
//void VESC_PID_INIT()
//{
//	/*idΪ2��VESC��ʼ��PID�趨*/
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
  *@  function  : ��ȡ�����ǰת��
  *@  input     : ID
  *@  output    : ��ǰת��
*********************************************************************************/
int VESC_GET_SPEED(uint8_t ID)
{
	 return VESC_Feedback[ID].cur_rpm;
}


/*********************************************************************************
  *@  name      : VESC_SET_CurrentBrake
  *@  function  : ɲ��
  *@  input     : ID  
	*								Current �����绬������������Ҫ����20A
  *@  output    : null
*********************************************************************************/
void VESC_SET_CurrentBrake(uint8_t ID, int Current,FDCAN_HandleTypeDef *hfdcan)
{
	VESC_COMMAND_SEND(hfdcan, 2, ID, Current * 1000);
}





