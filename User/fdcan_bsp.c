#include "fdcan_bsp.h"
#include "Remote_Control.h"
#include "GM6020.h"
#include "VESC_CAN.h"
/**
 * @brief FDCAN1 ��ʼ��������FIFO0
 * @param hfdcan hfdcan1
 * @return uint8_t �ɹ�����0
 */
uint8_t FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef RXFilter;

    //����RX�˲���
    RXFilter.IdType = FDCAN_STANDARD_ID;             /*��׼ID*/
    RXFilter.FilterIndex = 0;                        /*�˲�������*/
    RXFilter.FilterType = FDCAN_FILTER_MASK;         /*���������ͣ�����ģʽ*/
    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /*������������ RXFIFO0*/
    RXFilter.FilterID1 = 0x0000;                     /*32λID*/
    RXFilter.FilterID2 = 0x0000;                     /*���FDCAN����Ϊ��ͳģʽ�Ļ���������32λ����*/

    /*�˲�����ʼ��*/
    if (HAL_FDCAN_ConfigFilter(hfdcan, &RXFilter) != HAL_OK)
    {
        return 2;
    }
    /* ����FDCAN1�˲���0ȫ������  */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK)
    {
        return 2;
    }
    HAL_FDCAN_Start(hfdcan);                                                  /*����FDCAN*/
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); /*��FIFO�жϽ���*/
    return 0;
}

/**
 * @brief FDCAN2��ʼ��������FIFO1
 * @param hfdcan hfdcan2
 * @return uint8_t �ɹ�����0
 */
uint8_t FDCAN2_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef RXFilter;

    //����RX�˲���
    RXFilter.IdType = FDCAN_STANDARD_ID;             /*��׼ID*/
    RXFilter.FilterIndex = 0;                        /*�˲�������*/
    RXFilter.FilterType = FDCAN_FILTER_MASK;         /*���������ͣ�����ģʽ*/
    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; /*������������ RXFIFO0*/
    RXFilter.FilterID1 = 0x0000;                     /*32λID*/
    RXFilter.FilterID2 = 0x0000;                     /*���FDCAN����Ϊ��ͳģʽ�Ļ���������32λ����*/
    /*�˲�����ʼ��*/
    if (HAL_FDCAN_ConfigFilter(hfdcan, &RXFilter) != HAL_OK)
    {
        return 2;
    }
    
    RXFilter.IdType = FDCAN_EXTENDED_ID;             /*��׼ID*/
    RXFilter.FilterIndex = 2;                        /*�˲�������*/
    RXFilter.FilterType = FDCAN_FILTER_MASK;         /*���������ͣ�����ģʽ*/
    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; /*������������ RXFIFO0*/
    RXFilter.FilterID1 = 0x0000;                     /*32λID*/
    RXFilter.FilterID2 = 0x0000;                     /*���FDCAN����Ϊ��ͳģʽ�Ļ���������32λ����*/
    /*�˲�����ʼ��*/
    if (HAL_FDCAN_ConfigFilter(hfdcan, &RXFilter) != HAL_OK)
    {
        return 2;
    }
    
    /* ����FDCAN1�˲���0ȫ������  */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK)
    {
        return 2;
    }
    
    *(uint32_t*)(0x4000A400+0x0080)=0x00000024;
    
    HAL_FDCAN_Start(hfdcan);                                                  /*����FDCAN*/
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); /*��FIFO�жϽ���*/
    return 0;
}

/**
 * @brief FDCAN���ͺ���
 * @param hfdcan hfdcan���
 * @param TxData ��������
 * @param StdId ��׼֡ID
 * @param Length ���� 1~8
 * @return uint8_t ���ͳɹ�����0
 */
uint8_t FDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t StdId, uint32_t Length)
{

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = StdId;             /*32λ ID*/
    TxHeader.IdType = FDCAN_STANDARD_ID;     /*��׼ID*/
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; /*����֡*/
    TxHeader.DataLength = Length << 16;      /*���ݳ�����ר�ŵĸ�ʽ  FDCAN_DLC_BYTES_8*/
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           /*�ر������л�*/
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            /*��ͳ��CANģʽ*/
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /*�޷����¼�*/
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
        return 1; //����
    return 0;
}

uint8_t FDCAN_SendData_Ext(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t ExtId, uint32_t Length, uint32_t Data_type)
{

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = ExtId;             /*32λ ID*/
    TxHeader.IdType = FDCAN_EXTENDED_ID;     /*��չID*/
    TxHeader.TxFrameType = Data_type; /*֡����*/
    TxHeader.DataLength = Length;      /*���ݳ�����ר�ŵĸ�ʽ*/
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           /*�ر������л�*/
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            /*��ͳ��CANģʽ*/
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /*�޷����¼�*/
    TxHeader.MessageMarker = 0;
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
        return 1; //����
    return 0;
}

/**
 * @brief FDCAN������Ϣ����
 * @param hfdcan
 * @param StdId
 * @param RxData
 */
__weak void FDCAN_Message_Decode(FDCAN_HandleTypeDef *hfdcan, uint32_t StdId, uint8_t *RxData)
{
    if (hfdcan->Instance == FDCAN1)
    {
        
        GYRO_Resolve(StdId,RxData);
        if(StdId==0x233)
        {
            for(int i=0;i<8;i++)
            {
                if(RxData[i]==1)
                {
                    cmd_feedback[i]=1;
                }
            }
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        GM6020_Get_Feedback(StdId,RxData);
    }
}

/*FIFO 0���ջص�*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t RxData[8] = {0};
    FDCAN_RxHeaderTypeDef RxHeader;

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

    FDCAN_Message_Decode(hfdcan, RxHeader.Identifier, RxData);
}

/*FIFO 1���ջص�*/
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t RxData[64] = {0};
    FDCAN_RxHeaderTypeDef RxHeader;

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData);
    if(RxHeader.IdType==FDCAN_STANDARD_ID)
        FDCAN_Message_Decode(hfdcan, RxHeader.Identifier, RxData);
    else
    {
        VESC_CAN_DECODE(RxHeader.Identifier,RxData);
    }
}
