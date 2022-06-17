#include "GM6020.h"
#include "fdcan_bsp.h"
#include "Ann.h"
//����ʱ����5~10ms

const uint8_t GM6020_Reduction_Ratio[8] = {1, 1, 1, 1, 1, 1, 1, 1}; //������ٱ�����

//���ڴ洢���������ȫ�ֱ���
uint8_t GM6020_Feedback_Buf[8][6];		//�������ֵ(ȫ�ֱ���)
int GM6020_Pos[8];					//ÿһ��Ԫ�ض�Ӧһ��ID�ĵ������Ϣ
ANN_PID_handle GM6020_Speed_ANNPID[8];
uint8_t GM6020_Temp_error_counter[8];

//PID������ʼ��	(ÿ�����һ�ײ���)
GM6020_PID GM6020_Speed_Pid[8] =
{
    {.Kp = 110, .Ki = 3, .Kd = 14, .Max = 300000, .Min = -300000}, //ID = 1
    {.Kp = 110, .Ki = 3, .Kd = 14, .Max = 300000, .Min = -300000}, //ID = 2
    {.Kp = 110, .Ki = 3, .Kd = 14, .Max = 300000, .Min = -300000}, //ID = 3
    {.Kp = 110, .Ki = 3, .Kd = 14, .Max = 300000, .Min = -300000}, //ID = 4
    {.Kp = 50, .Ki = 1, .Kd = 5, .Max = 300000, .Min = -300000}, //ID = 5
    {.Kp = 50, .Ki = 1, .Kd = 5, .Max = 300000, .Min = -300000}, //ID = 6
    {.Kp = 50, .Ki = 1, .Kd = 5, .Max = 300000, .Min = -300000}, //ID = 7
    {.Kp = 50, .Ki = 1, .Kd = 5, .Max = 300000, .Min = -300000}, //ID = 8
};


GM6020_PID GM6020_Pos_Pid[8] =
{
    {.Kp = 0.75, .Ki = 0.001, .Kd = 0.1, .Max = 320, .Min = -320},	//ID = 1
    {.Kp = 0.35, .Ki = 0.001, .Kd = 0.1, .Max = 320, .Min = -320},	//ID = 2
    {.Kp = 0.35, .Ki = 0.001, .Kd = 0.1, .Max = 320, .Min = -320},	//ID = 3
    {.Kp = 0.35, .Ki = 0.001, .Kd = 0.1, .Max = 320, .Min = -320},	//ID = 4
    {.Kp = 1.3, .Ki = 0, .Kd = 0.05, .Max = 320, .Min = -320},	//ID = 5
    {.Kp = 1.3, .Ki = 0, .Kd = 0.05, .Max = 320, .Min = -320},	//ID = 6
    {.Kp = 1.3, .Ki = 0, .Kd = 0.05, .Max = 320, .Min = -320},	//ID = 7
    {.Kp = 1.3, .Ki = 0, .Kd = 0.05, .Max = 320, .Min = -320},	//ID = 8
};

void GM6020_ANNPID_Init(void)
{
    for(int i=0;i<8;i++)
        ANN_pid_init(&GM6020_Speed_ANNPID[i]);
    for(int i=0;i<400;i++)
    {
        for(int j=0;j<8;j++)
            GM6020_Set_Speed(20,j);
        HAL_Delay(12);
    }
    
    for(int i=0;i<400;i++)
    {
        for(int j=0;j<8;j++)
            GM6020_Set_Speed(-20,j);
        HAL_Delay(12);
    }
    NNlearn=0;
    for(int i=0;i<8;i++)
    {
        GM6020_Speed_ANNPID[i].pid_handle.KP*=0.7f;
        GM6020_Speed_ANNPID[i].pid_handle.KI*=0.5f;
        GM6020_Speed_ANNPID[i].pid_handle.KD*=0.7f;
    }
    return;
}

/*********************************************************************************
  *@  name      : GM6020_Set_V
  *@  function  : GM6020�����ѹ����
  *@  input     : Ŀ���ѹ�����id
  *@  output    : �ɹ�����0��ʧ�ܷ���1
*********************************************************************************/
uint8_t can_Sendbuf[8];
uint8_t GM6020_Set_V(int target_v, uint8_t motor_id)
{
    if( motor_id >= 1 && motor_id <= 8 )
    {
        int send_id = 0;
        send_id = send_id;

        if( target_v <= -30000 )
            target_v = -30000;
        else if( target_v >= 30000 )
            target_v = 30000;

        if(motor_id <= 4)
            send_id = 0x1ff;
        else
        {
            send_id = 0x2ff;
            motor_id -= 4;
        }

        can_Sendbuf[2 * motor_id - 2] = target_v >> 8;          //��ѹֵ��8λ
        can_Sendbuf[2 * motor_id - 1] = target_v & 0x00ff;      //��ѹֵ��8λ
				
		//		CAN_SendData(&hcan1,can_Sendbuf,send_id);
       GM6020_CAN_Send_Data(&hfdcan2, can_Sendbuf, send_id, 8);
		
        return 0;
    }
    else
        return 1;
}


/********************************************************************************
  *@  name      : GM6020_Set_Speed
  *@  function  : GM6020�ٶ�����
  *@  input     : Ŀ���ٶȣ�-320~320���ɽ��ܣ������id
  *@  output    : ��
********************************************************************************/
void GM6020_Set_Speed(int goal_speed, int ID)
{
    uint8_t id = ID - 1;
    GM6020_Speed_Pid[id].Err_1 = GM6020_Speed_Pid[id].Err;
    GM6020_Speed_Pid[id].Err 	= goal_speed - GM6020_Get_Speed(ID);
    GM6020_Speed_Pid[id].Err_sum += GM6020_Speed_Pid[id].Err;
    GM6020_Speed_Pid[id].Err_sum = CLAMP(GM6020_Speed_Pid[id].Err_sum, -10000, 10000);

    GM6020_Speed_Pid[id].P_Out = GM6020_Speed_Pid[id].Kp * GM6020_Speed_Pid[id].Err;
    GM6020_Speed_Pid[id].I_Out = GM6020_Speed_Pid[id].Ki * GM6020_Speed_Pid[id].Err_sum;
    GM6020_Speed_Pid[id].D_Out = GM6020_Speed_Pid[id].Kd * (GM6020_Speed_Pid[id].Err - GM6020_Speed_Pid[id].Err_1);

    GM6020_Speed_Pid[id].PID_Out = GM6020_Speed_Pid[id].P_Out + GM6020_Speed_Pid[id].I_Out + GM6020_Speed_Pid[id].D_Out;
    GM6020_Speed_Pid[id].PID_Out = CLAMP(GM6020_Speed_Pid[id].PID_Out, GM6020_Speed_Pid[id].Min, GM6020_Speed_Pid[id].Max);
	
	
		GM6020_Death_Speed((GM6020_Speed_Pid[id].PID_Out),GM6020_Death_Val);
	
    GM6020_Set_V(GM6020_Speed_Pid[id].PID_Out, ID);
//    GM6020_Set_V(ANN_pid_run(&GM6020_Speed_ANNPID[id],goal_speed,GM6020_Get_Speed(ID)), ID);
}


/********************************************************************************
  *@  name      : GM6020_Set_Pos
  *@  function  : GM6020λ������
  *@  input     : Ŀ��Ƕȣ�����Ƕȣ������id
  *@  output    : ��
********************************************************************************/
void GM6020_Set_Pos(int goal_angle, int ID)
{
	int goal_cnt = GM6020_Ang2Cnt(goal_angle, ID);
    uint8_t id = ID - 1;
    GM6020_Pos_Pid[id].Err_1 	= GM6020_Pos_Pid[id].Err;
    GM6020_Pos_Pid[id].Err 	= goal_cnt - GM6020_Get_Pos(ID);

    GM6020_Pos_Pid[id].Err_sum += GM6020_Pos_Pid[id].Err;
    GM6020_Pos_Pid[id].Err_sum = CLAMP(GM6020_Pos_Pid[id].Err_sum, -20000, 20000);

    GM6020_Pos_Pid[id].P_Out = GM6020_Pos_Pid[id].Kp * GM6020_Pos_Pid[id].Err;
    GM6020_Pos_Pid[id].I_Out = GM6020_Pos_Pid[id].Ki * GM6020_Pos_Pid[id].Err_sum;
    GM6020_Pos_Pid[id].D_Out = GM6020_Pos_Pid[id].Kd * (GM6020_Pos_Pid[id].Err - GM6020_Pos_Pid[id].Err_1);

    GM6020_Pos_Pid[id].PID_Out = GM6020_Pos_Pid[id].P_Out + GM6020_Pos_Pid[id].I_Out + GM6020_Pos_Pid[id].D_Out;
    GM6020_Pos_Pid[id].PID_Out = CLAMP(GM6020_Pos_Pid[id].PID_Out, GM6020_Pos_Pid[id].Min, GM6020_Pos_Pid[id].Max );

    GM6020_Set_Speed(GM6020_Pos_Pid[id].PID_Out, ID);

}
/*********************************************************************************
  *@  name      : GM6020_Get_Feedback
  *@  function  : ��ȡGM6020����ķ���������ȫ�ֱ���GM6020_Feedback_Buf[8][6];
  *@  input     : message_id,message����ָ��
  *@  output    :
*********************************************************************************/
void GM6020_Get_Feedback(uint32_t std_id, uint8_t* data_p)
{
    int i;
    for(i = 1; i < 9; i++)  //ǰ�ĵ��ƥ��
    {
			
        if(std_id == 0x204 + i)
        {
            memcpy(GM6020_Feedback_Buf[i - 1], data_p, 6);
			GM6020_Pos_Rec(i);
            if(data_p[6]>=100)
            {
                if(GM6020_Temp_error_counter[i]<255)
                    GM6020_Temp_error_counter[i]++;
                else
                    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
            }
            return;
        }
    }
}
/*********************************************************************************
  *@  name      : GM6020_Get_Real_I
  *@  function  : ��ȡGM6020�����ʵ��ת����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����ת��,��ȡʧ�ܷ���0
*********************************************************************************/
int GM6020_Get_Torque(uint8_t motor_id)
{
    int torque = 0;
    if(GM6020_Feedback_Buf[motor_id - 1][2] >> 7 == 1)
        torque = -( 0xffff - (  (GM6020_Feedback_Buf[motor_id - 1][4] << 8) + GM6020_Feedback_Buf[motor_id - 1][5])  ) ;
    else
        torque = (GM6020_Feedback_Buf[motor_id - 1][4] << 8) + GM6020_Feedback_Buf[motor_id - 1][5];
    return torque;
}
/*********************************************************************************
  *@  name      : GM6020_Get_Speed
  *@  function  : ��ȡGM6020����ķ������ٶ���Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid������ٶ�,��ȡʧ�ܷ���0
*********************************************************************************/
int GM6020_Get_Speed(uint8_t motor_id)
{
    int speed = 0;
    if(GM6020_Feedback_Buf[motor_id - 1][2] >> 7 == 1)
        speed = -( 0xffff - (  (GM6020_Feedback_Buf[motor_id - 1][2] << 8) + GM6020_Feedback_Buf[motor_id - 1][3])  ) ;
    else
        speed = (GM6020_Feedback_Buf[motor_id - 1][2] << 8) + GM6020_Feedback_Buf[motor_id - 1][3];
    return speed;
}
/*********************************************************************************
  *@  name      : GM6020_Get_Pos
  *@  function  : ��ȡGM6020�����ǰ��λ����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����λ�ã���������CNTֵ
*********************************************************************************/
int GM6020_Get_Pos(uint8_t motor_id)
{
    return GM6020_Pos[motor_id - 1];
}
/*********************************************************************************
  *@  name      : GM6020_Pos_Rec
  *@  function  : ��ȡGM6020����ķ�����λ����Ϣ
  *@  input     : ���id��
  *@  output    : ��Ӧid�����λ����Ϣ,��ȡʧ�ܷ���-1
*********************************************************************************/
void GM6020_Pos_Rec(uint8_t motor_id)
{
    int id = motor_id - 1;
    int32_t	GM6020_tmp[8];
    static int32_t	GM6020_base[8] = {0};	//��������Ѿ�ת����Ȧ����һȦ8192
    static int32_t GM6020tmp_pre[8] = {0};

    GM6020_tmp[id] = (GM6020_Feedback_Buf[id][0] << 8) + GM6020_Feedback_Buf[id][1];
    if ( GM6020_tmp[id] - GM6020tmp_pre[id] > 4095 )  //ת��8191��0ʱ��¼Ȧ��
        GM6020_base[id] -= 8191;
    else if ( GM6020_tmp[id] - GM6020tmp_pre[id] < -4095 )
        GM6020_base[id] += 8191;

    GM6020tmp_pre[id] = GM6020_tmp[id];
    GM6020_Pos[id] = GM6020_base[id] + GM6020_tmp[id];

}
/********************************************************************************
  *@  name      : GM6020_Ang2Cnt
  *@  function  : �Ƕ�ת��Ϊʵ�ʵ��Ӧ��ת����λ���� //δ���������ת��תһȦ��ֵΪ8192
  *@  input     : Ŀ��Ƕȣ�����Ƕȣ������id  //id��ͬ�����ٱȲ�ͬ
  *@  output    : ���λ��
********************************************************************************/
int GM6020_Ang2Cnt(float angle, int ID)
{

    int cnt;
    cnt = (int)(GM_CNT_PER_ROUND_OUT(ID) * angle / 360);
    return cnt;
}
/********************************************************************************
  *@  name      : GM6020_Cnt2Ang
  *@  function  : ���λ��ת��Ϊ�Ƕ� //δ���������ת��תһȦ��ֵΪ8192
  *@  input     : ���λ�ã����id  //id��ͬ�����ٱȲ�ͬ
  *@  output    : ���ת���ĽǶ�
********************************************************************************/
double GM6020_Cnt2Ang(int32_t cnt, int ID)
{
    int angled;
    angled = (double)((cnt * 360.0) / GM_CNT_PER_ROUND_OUT(ID));
    return angled;
}

/*CAN���ͺ���*/
uint8_t GM6020_CAN_Send_Data(FDCAN_HandleTypeDef* hfdcan, uint8_t *pData, uint16_t ID, uint16_t Len)
{
//	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
//	uint8_t FreeTxNum = 0;
//	CAN_TxHeaderTypeDef TxMessage;
//	
//	TxMessage.IDE = CAN_ID_STD;  //��׼֡,CAN_ID_EXT��չ֡;
//	TxMessage.RTR = CAN_RTR_DATA;  //����֡,CAN_RTR_REMOTEң��֡
//	TxMessage.StdId = ID;
//	TxMessage.DLC = Len;
//	
//	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);	
//		
//	while(FreeTxNum==0)  //�ȴ�������
//	{
//		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
//	}
//	
////	HAL_Delay(1); //û����ʱ���п��ܻᷢ��ʧ��
//	
//	HAL_RetVal = HAL_CAN_AddTxMessage(hcan,&TxMessage,pData,(uint32_t*)CAN_TX_MAILBOX0);
	
    FDCAN_SendData(hfdcan,pData,ID,Len);
 
//	if(HAL_RetVal!=HAL_OK)
//	{
//		return 2;
//	}
	
	return 0;
}




