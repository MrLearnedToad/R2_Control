#include "Elmo_fdcan.h"

#ifdef ID_0
static CANQUEUE QUEUE_CAN_ID0;
#endif
#ifdef ID_1
static CANQUEUE QUEUE_CAN_ID1;
#endif
#ifdef ID_2
static CANQUEUE QUEUE_CAN_ID2;
#endif
#ifdef ID_3
static CANQUEUE QUEUE_CAN_ID3;
#endif
#ifdef ID_4
static CANQUEUE QUEUE_CAN_ID4;
#endif
#ifdef ID_5
static CANQUEUE QUEUE_CAN_ID5;
#endif
#ifdef ID_6
static CANQUEUE QUEUE_CAN_ID6;
#endif
#ifdef ID_7
static CANQUEUE QUEUE_CAN_ID7;
#endif
#ifdef ID_8
static CANQUEUE QUEUE_CAN_ID8;
#endif
#ifdef ID_9
static CANQUEUE QUEUE_CAN_ID9;
#endif
#ifdef ID_10
static CANQUEUE QUEUE_CAN_ID10;
#endif
#ifdef ID_11
static CANQUEUE QUEUE_CAN_ID11;
#endif
#ifdef ID_12
static CANQUEUE QUEUE_CAN_ID12;
#endif
#ifdef ID_13
static CANQUEUE QUEUE_CAN_ID13;
#endif
#ifdef ID_14
static CANQUEUE QUEUE_CAN_ID14;
#endif
#ifdef ID_15
static CANQUEUE QUEUE_CAN_ID15;
#endif

CANQUEUE *QUEUE_CAN_IDx; //ָ����е�ָ��

static Elmo elmo[ELMO_NUM + 1];
static Elmo elmogroup;
// static CAN_TypeDef* can = CAN1;         //��ʼ��ѡ��CANx�Ľӿڱ���
static uint8_t CAN_Error = 0;       //���CAN����ʧ�ܣ�CAN_Error��1
static uint32_t Elmo_Init_Flag = 0; //������¼ELMOʵ�ʵĳ�ʼ����״̬
static int32_t MotorPos[16] = {0};

// extern Arm_Struct Im_Arm;
/*
********************************************************************************
*@  name      : CAN_init
*@  function  : Initialization for CAN
*@  input     : CANx      CAN1 or CAN2
*@              PPr       TIM7����ռ���ȼ�
*@              SPr       TIM7�Ĵ����ȼ�
*@  output    : 0            ��ʼ���ɹ�
*@              1         ��ʼ��ʧ��
********************************************************************************
*/
uint32_t Elmo_Init(FDCAN_HandleTypeDef *hfdcan, TIM_HandleTypeDef *htim)
{
    uint8_t i = 0;
    int temp = 0;
    /* Ӳ����ȫ�ֱ�����ʼ�� */

    Elmo_TIM_init(htim);
    Elmo_CAN_USER_Init(hfdcan);

    /*���������ʼ��*/
    Variate_init();

    /* ��elmo����ڵ�ID */
    for (i = 0; i <= ELMO_NUM; i++)
    {
        elmo[i].NodeID = i;
    }

    /* ��elmo������ID */
    elmogroup.NodeID = 64;

    /* ��ȫ��ڵ����ͨ�Ÿ�λ */
    for (i = 1; i <= ELMO_NUM; i++)
    {
        NMTCmd(&elmo[i], NMT_RESET_COMMUNICATION);
        Elmo_Delay100us_IDx(&elmo[i], 50);
    }

    /* �Լ칦�ܺ�����CAN����ELMO��ʼ���ɹ���������ֵ0�������ӦELMOλ��0���������û�����ߣ�����0x80000000 */
    if ((temp = Self_test(htim)) == 0x80000000)
    {
        return temp;
    }

    //    /* �ȴ�Elmo�������,�����յ�Boot up���� */
    //    Elmo_Delay100us_IDx(&elmo[elmoID],50);

    /* CANOpenͨ�Ų�����ʼ�� */
    /* RPDO1->0x6040,ָ����,2�ֽ�,�첽���� */
    // ������Ĭ��ӳ��,����Ҫ�޸� //

    /* RPDO2->0x2012,�����Ʊ�������,4�ֽ�,�첽���� */
    // ������Ĭ��ӳ��,����Ҫ�޸� //

    /* ����TPDO,Debugʱ����,������ʱ��ùر� */
    RSDO(&elmo[0], 0x1A00, 0x00, 0); //����PDO1
    Elmo_Delay100us_IDx(&elmo[0], 150);
    RSDO(&elmo[0], 0x1A01, 0x00, 0); //����PDO2 ���ʹ�õ���Pdo2
    Elmo_Delay100us_IDx(&elmo[0], 150);

    /* ����NMT����״̬ */
    Elmo_Delay100us_IDx(&elmo[0], 40);
    NMTCmd(&elmo[0], NMT_ENTER_OPERATIONAL);
    Elmo_Delay100us_IDx(&elmo[0], 40);

    /* �ر����� */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[0], 80);

    /* ��ʼ�����ٶ� */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"PM", 0, TYPE_INTEGER, 0x01);
    Elmo_Delay100us_IDx(&elmo[0], 40);
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"AC", 0, TYPE_INTEGER, 1000000000);
    Elmo_Delay100us_IDx(&elmo[0], 40);
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"DC", 0, TYPE_INTEGER, 1000000000);
    Elmo_Delay100us_IDx(&elmo[0], 40);

    /* Enter in SCM */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
    Elmo_Delay100us_IDx(&elmo[0], 40);
    elmogroup.CurOPMode = UM_SCM;
    for (i = 0; i <= ELMO_NUM; i++)
    {
        elmo[i].CurOPMode = UM_SCM;
    }

    /* ʹ������ */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    Elmo_Delay100us_IDx(&elmo[0], 250);
    Elmo_Delay100us_IDx(&elmo[0], 250);
    Elmo_Delay100us_IDx(&elmo[0], 50);
    return temp;
}

void Elmo_Reinit(uint8_t elmoID)
{
    RSDO(&elmo[elmoID], 0x6040, 0x00, 0x80); //��������־λ
    Elmo_Delay100us_IDx(&elmo[elmoID], 20);

    RSDO(&elmo[elmoID], 0x2f41, 0x00, 0x02); //  ����PPM
    Elmo_Delay100us_IDx(&elmo[elmoID], 20);

    NMTCmd(&elmo[elmoID], NMT_RESET_COMMUNICATION);
    Elmo_Delay100us_IDx(&elmo[elmoID], 30);

    /* ����NMT����״̬ */
    NMTCmd(&elmo[elmoID], NMT_ENTER_OPERATIONAL);
    Elmo_Delay100us_IDx(&elmo[elmoID], 30);

    /* ��ʼ�����ٶ� */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"PM", 0, TYPE_INTEGER, 0x01);
    Elmo_Delay100us_IDx(&elmo[elmoID], 20); //�����ٶ�1000000000 1000000
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"AC", 0, TYPE_INTEGER, 1000000000);
    Elmo_Delay100us_IDx(&elmo[elmoID], 40); //�����ٶ�1000000000
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"DC", 0, TYPE_INTEGER, 1000000000);
    Elmo_Delay100us_IDx(&elmo[elmoID], 40);

    /* �ر����� */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[elmoID], 30);

    /* ʹ������ */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    Elmo_Delay100us_IDx(&elmo[elmoID], 60);
}
/*
********************************************************************************
*@  name      : Elmo_PTM
*@  function  : ��������ģʽ����
*@  input     : elmoID    ȡelmo�ڵ�ID,�����ڵ��̿����е���elmo[1]~elmo[4]
*@               torque    Ŀ��ת��(A)
*@  output    : 0         �������óɹ�
*@              1         ��������ʧ��
********************************************************************************
*/
uint8_t Elmo_PTM(uint8_t elmoID, float torque)
{
    uint8_t i = 0;

    /* ���ʹ�ù㲥����, ��ǰģʽ��Ϊ����ģʽ,�л�Ϊ����ģʽ*/
    if (elmoID == 0 && elmo[elmoID].CurOPMode != UM_TCM)
    {
        if (elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 100);
        }

        /* ������״̬������ֵΪTCM */
        elmogroup.CurOPMode = UM_TCM;
        for (i = 0; i <= ELMO_NUM; i++)
        {
            elmo[i].CurOPMode = UM_TCM;
        }

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_TCM);
        Elmo_Delay100us_IDx(&elmo[elmoID], 10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID], 30);
    }
    /* ʹ�õ���ELMO����,��ǰģʽ��Ϊ����ģʽ,�л�Ϊ����ģʽ */
    else if (elmo[elmoID].CurOPMode != UM_TCM)
    {
        if (elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 100);
        }
        elmo[elmoID].CurOPMode = UM_TCM;
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_TCM);
        Elmo_Delay100us_IDx(&elmo[elmoID], 10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID], 30);

        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */
        if (ELMO_NUM == 1)
        {
            elmo[0].CurOPMode = UM_TCM;
        }
        /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
        else
        {
            for (i = 1; i < ELMO_NUM; i++)
            {
                if (elmo[i].CurOPMode == elmo[i + 1].CurOPMode)
                {
                    elmo[0].CurOPMode = UM_TCM;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }
        }
    }

    /* ����Ŀ��ת�ز����е���ģʽ */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"TC", 0, TYPE_FLOAT, f2h(torque));
    Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    return 0;
}
/*
********************************************************************************
*@  name      : Elmo_Pre_PVM
*@  function  : �ٶ�ģʽԤ������
*@  input     : elmoID   ȡelmo�ڵ�ID
*@  output    : none
********************************************************************************
*/
void Elmo_Pre_PVM(uint8_t elmoID)
{
    uint8_t i = 0;

    /* ���ʹ�ù㲥����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ */
    if (elmoID == 0 && elmo[elmoID].CurOPMode != UM_SCM)
    {
        if (elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 100);
        }

        /* ������״̬������ֵΪSCM */
        elmogroup.CurOPMode = UM_SCM;
        for (i = 0; i <= ELMO_NUM; i++)
        {
            elmo[i].CurOPMode = UM_SCM;
        }

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
        Elmo_Delay100us_IDx(&elmo[elmoID], 10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID], 30);
    }
    /* ʹ�õ���ELMO����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ*/
    else if (elmo[elmoID].CurOPMode != UM_SCM)
    {
        if (elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 100);
        }

        elmo[elmoID].CurOPMode = UM_SCM;
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
        Elmo_Delay100us_IDx(&elmo[elmoID], 10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID], 30);

        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */
        if (ELMO_NUM == 1)
        {
            elmo[0].CurOPMode = UM_SCM;
        }
        /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
        else
        {
            for (i = 1; i < ELMO_NUM; i++)
            {
                if (elmo[i].CurOPMode == elmo[i + 1].CurOPMode)
                {
                    elmo[0].CurOPMode = UM_SCM;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }
        }
    }
}

/*
********************************************************************************
*@  name      : Elmo_PVM
*@  function  : �����ٶ�ģʽ����
*@  input     : elmoID   ȡelmo�ڵ�ID
*@              speed    Ŀ���ٶ�(cnt/s)
*@  output    : 0         �������óɹ�
*@              1         ��������ʧ��
********************************************************************************
*/

uint8_t Elmo_PVM(uint8_t elmoID, int32_t speed)
{
    //		uint8_t i = 0;
    //
    //

    ////    //����*******************************
    ////    #ifdef CASE5_ENABLE
    ////    if(elmoID == 8 && Im_Arm.Arm_Init_success == 1)
    ////    {
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode=1;
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed=speed;
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos=0;
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t=OSTime;
    ////        SLIDEWAY_ELMO_SAVE_count++;
    ////        if(SLIDEWAY_ELMO_SAVE_count>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
    ////    }
    ////        if(elmoID == 10 && Im_Arm.Arm_Init_success == 1)
    ////    {
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode=1;
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed=speed;
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos=0;
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t=OSTime;
    ////        SLIDEWAY_ELMO_SAVE_count_Y++;
    ////        if(SLIDEWAY_ELMO_SAVE_count_Y>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count_Y=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
    ////    }
    ////    #endif
    //        //����*******************************
    //
    //
    //
    //    /* ���ʹ�ù㲥����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ */
    //    if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_SCM)
    //    {
    //        if(elmo[elmoID].CurOPMode != UM_IDLE)
    //        {
    //            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    //            Elmo_Delay100us_IDx(&elmo[elmoID],100);
    //        }
    //
    //        /* ������״̬������ֵΪSCM */
    //        elmogroup.CurOPMode = UM_SCM;
    //        for(i=0;i<=ELMO_NUM;i++)
    //        {
    //            elmo[i].CurOPMode = UM_SCM;
    //        }

    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],10);

    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],30);
    //
    //    }
    //    /* ʹ�õ���ELMO����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ*/
    //    else if(elmo[elmoID].CurOPMode != UM_SCM)
    //    {
    //        if(elmo[elmoID].CurOPMode != UM_IDLE)
    //        {
    //            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    //            Elmo_Delay100us_IDx(&elmo[elmoID],100);
    //        }

    //        elmo[elmoID].CurOPMode = UM_SCM;
    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],10);

    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],30);
    //
    //        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */
    //        if(ELMO_NUM == 1)
    //        {
    //            elmo[0].CurOPMode = UM_SCM;
    //        }
    //        /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
    //        else
    //        {
    //            for(i=0;i<ELMO_NUM;i++)
    //            {
    //                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
    //                {
    //                    elmo[0].CurOPMode = UM_SCM;
    //                }
    //                else
    //                {
    //                    elmo[0].CurOPMode = UM_UNC;
    //                    break;
    //                }
    //            }
    //        }
    //    }
    //	Elmo_Pre_PVM(elmoID);

    /* ����Ŀ���ٶ� */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"JV", 0, TYPE_INTEGER, speed);
    Elmo_Delay100us_IDx(&elmo[elmoID], 10);

    /* �����ٶ�ģʽ */
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
    Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    return 0;
}

/*
********************************************************************************
*@  name      : Elmo_RunPPM
*@  function  : ����ƽ���ٶȵ�λ��ģʽ
*@  input     : elmoID    ȡelmo�ڵ�ID
*@              speed     �ٶ�(cnt/s)
*@              position  Ŀ��λ��(cnt)
*@              PPMmode   ����ģʽ
*@                    POS_ABS // PPM���з�ʽ:����λ��
*@                    POS_REL // PPM���з�ʽ:���λ��
*@  output    : 0         �������óɹ�
*@              1         ��������ʧ��
********************************************************************************
*/
void Elmo_Pre_PPM(uint8_t elmoID)
{
    uint8_t i = 0;
    /* ���ʹ�ù㲥����,��ǰģʽ��Ϊλ��ģʽ,�л�Ϊλ��ģʽ */
    if (elmoID == 0 && elmo[elmoID].CurOPMode != UM_PCM)
    {
        if (elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 90);
        }
        /* ������״̬������ֵΪPCM */
        elmogroup.CurOPMode = UM_PCM;
        for (i = 0; i <= ELMO_NUM; i++)
        {
            elmo[i].CurOPMode = UM_PCM;
        }

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_PCM);
        Elmo_Delay100us_IDx(&elmo[elmoID], 20);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID], 30);
    }
    /* ʹ�õ���ELMO����,��ǰģʽ��Ϊλ��ģʽ,�л�Ϊλ��ģʽ */
    else if (elmo[elmoID].CurOPMode != UM_PCM)
    {
        if (elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 90);
        }

        elmo[elmoID].CurOPMode = UM_PCM;
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_PCM);
        Elmo_Delay100us_IDx(&elmo[elmoID], 200);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID], 30);

        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */
        if (ELMO_NUM == 1)
        {
            elmo[0].CurOPMode = UM_PCM;
        }
        /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
        else
        {
            for (i = 1; i < ELMO_NUM; i++)
            {
                if (elmo[i].CurOPMode == elmo[i + 1].CurOPMode)
                {
                    elmo[0].CurOPMode = UM_PCM;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }
        }
    }
}

/*********************************************************************************************************************/
uint8_t Elmo_PPM(uint8_t elmoID, uint32_t speed, int32_t position, uint8_t PPMmode)
{
    //    uint8_t i = 0;

    //    //����*******************************
    ////    #ifdef CASE5_ENABLE
    ////    if(elmoID == 8 && Im_Arm.Arm_Init_success == 1)
    ////    {
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode=2;
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed=0;
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos=position;
    ////        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t=OSTime;
    ////        SLIDEWAY_ELMO_SAVE_count++;
    ////        if(SLIDEWAY_ELMO_SAVE_count>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
    ////    }
    ////        if(elmoID == 10 && Im_Arm.Arm_Init_success == 1)
    ////    {
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode=2;
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed=speed;
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos=0;
    ////        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t=OSTime;
    ////        SLIDEWAY_ELMO_SAVE_count_Y++;
    ////        if(SLIDEWAY_ELMO_SAVE_count_Y>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count_Y=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
    ////    }
    ////    #endif
    //        //����*******************************

    //    /* ���ʹ�ù㲥����,��ǰģʽ��Ϊλ��ģʽ,�л�Ϊλ��ģʽ */
    //    if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_PCM)
    //    {
    //        if(elmo[elmoID].CurOPMode != UM_IDLE)
    //        {
    //            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    //            Elmo_Delay100us_IDx(&elmo[elmoID],90);
    //        }
    //        /* ������״̬������ֵΪPCM */
    //        elmogroup.CurOPMode = UM_PCM;
    //        for(i=0;i<=ELMO_NUM;i++)
    //        {
    //            elmo[i].CurOPMode = UM_PCM;
    //        }

    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_PCM);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],20);
    //
    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],30);
    //    }
    //    /* ʹ�õ���ELMO����,��ǰģʽ��Ϊλ��ģʽ,�л�Ϊλ��ģʽ */
    //    else if(elmo[elmoID].CurOPMode != UM_PCM)
    //    {
    //        if(elmo[elmoID].CurOPMode != UM_IDLE)
    //        {
    //            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    //            Elmo_Delay100us_IDx(&elmo[elmoID],90);
    //        }

    //        elmo[elmoID].CurOPMode = UM_PCM;
    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_PCM);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],200);

    //        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    //        Elmo_Delay100us_IDx(&elmo[elmoID],30);

    //        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */
    //        if(ELMO_NUM == 1)
    //        {
    //            elmo[0].CurOPMode = UM_PCM;
    //        }
    //        /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
    //        else
    //        {
    //            for(i=1;i<ELMO_NUM;i++)
    //            {
    //                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
    //                {
    //                    elmo[0].CurOPMode = UM_PCM;
    //                }
    //                else
    //                {
    //                    elmo[0].CurOPMode = UM_UNC;
    //                    break;
    //                }
    //            }
    //        }
    //    }
    // RPDO2_Cmd_data(Elmo *elmo, uint8_t *Cmd, uint8_t Index, uint8_t Type, uint32_t Data)
    /* ����Ŀ���ٶ� */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"SP", 0, TYPE_INTEGER, speed);
    Elmo_Delay100us_IDx(&elmo[elmoID], 20);

    /* ��������ģʽ����λ�� */
    if (PPMmode)
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PA", 0, TYPE_INTEGER, position);
        Elmo_Delay100us_IDx(&elmo[elmoID], 20);
    }
    else
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PR", 0, TYPE_INTEGER, position);
        Elmo_Delay100us_IDx(&elmo[elmoID], 20);
    }

    /* ����λ��ģʽ */
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
    Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    return 0;
}

/*
********************************************************************************
*@  name      : Elmo_Close
*@  function  : ��������رգ���������Լ�����ʻ
*@  input     : elmoID    ȡelmo�ڵ�ID,�����ڵ��̿����е���elmo[1]~elmo[4]
*@  output    : 0         �������óɹ�
*@              1         ��������ʧ��
********************************************************************************
*/
uint8_t Elmo_Close(uint8_t elmoID)
{
    uint8_t i = 0;

    /* ���ʹ�ù㲥����,������ELMOʧ�� */
    if (elmoID == 0)
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
        Elmo_Delay100us_IDx(&elmo[elmoID], 100);

        /* ������״̬������ֵΪIDLE */
        elmogroup.CurOPMode = UM_IDLE;
        for (i = 0; i <= ELMO_NUM; i++)
        {
            elmo[i].CurOPMode = UM_IDLE;
        }
    }
    /* ʹ�õ���ELMO���� */
    else
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
        Elmo_Delay100us_IDx(&elmo[elmoID], 100);
        elmo[elmoID].CurOPMode = UM_IDLE;

        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */
        if (ELMO_NUM == 1)
        {
            elmo[0].CurOPMode = UM_IDLE;
        }
        /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
        else
        {
            for (i = 1; i < ELMO_NUM; i++)
            {
                if (elmo[i].CurOPMode == elmo[i + 1].CurOPMode)
                {
                    elmo[0].CurOPMode = UM_IDLE;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }
        }
    }
    return 0;
}

/*
********************************************************************************
*@  name      : Elmo_Stop
*@  function  : ɲ�����������
*@  input     : elmoID      ȡelmo�ڵ�ID,�����ڵ��̿����е���elmo[1]~elmo[4]
*@  output    : 0         �������óɹ�
*@              1         ��������ʧ��
********************************************************************************
*/
uint8_t Elmo_Stop(uint8_t elmoID)
{
    uint8_t i = 0;

    //����*******************************
#ifdef CASE5_ENABLE
    if (elmoID == 8 && Im_Arm.Arm_Init_success == 1)
    {
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode = 3;
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed = 0;
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos = 0;
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t = OSTime;
        SLIDEWAY_ELMO_SAVE_count++;
        if (SLIDEWAY_ELMO_SAVE_count >= SLIDEWAY_ELMO_SAVE_Max)
            SLIDEWAY_ELMO_SAVE_count = SLIDEWAY_ELMO_SAVE_Max; //��ֹԽ��
    }
    if (elmoID == 10 && Im_Arm.Arm_Init_success == 1)
    {
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode = 3;
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed = 0;
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos = 0;
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t = OSTime;
        SLIDEWAY_ELMO_SAVE_count_Y++;
        if (SLIDEWAY_ELMO_SAVE_count_Y >= SLIDEWAY_ELMO_SAVE_Max)
            SLIDEWAY_ELMO_SAVE_count_Y = SLIDEWAY_ELMO_SAVE_Max; //��ֹԽ��
    }
#endif
    /* ���CAN������������Ŀ���ģʽ����ͬ���ҷ����㲥ָ����ιر� */
    if (elmoID == 0 && elmo[elmoID].CurOPMode == UM_UNC)
    {
        /* ѭ���жϸ��������״̬�����ر�ÿһ����� */
        for (i = 1; i <= ELMO_NUM; i++)
        {
            /* ��ǰģʽΪ�ͷŵ��,�ȴ򿪵�����ٱ��� */
            if (elmo[i].CurOPMode == UM_IDLE)
            {
                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
                Elmo_Delay100us_IDx(&elmo[elmoID], 30);
            }
            /* ��ǰģʽΪ����ģʽ,���л�Ϊ�ٶ�ģʽ���ٹر� */
            else if (elmo[i].CurOPMode == UM_TCM)
            {
                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);
                Elmo_Delay100us_IDx(&elmo[i], 200);

                elmo[i].CurOPMode = UM_SCM;

                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
                Elmo_Delay100us_IDx(&elmo[i], 10);

                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
                Elmo_Delay100us_IDx(&elmo[i], 30);

                /* ����Ŀ���ٶ� */
                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"JV", 0, TYPE_INTEGER, 0);
                Elmo_Delay100us_IDx(&elmo[i], 10);

                /* �����ٶ�ģʽ */
                RPDO2_Cmd_string(&elmo[i], (uint8_t *)"BG");
                Elmo_Delay100us_IDx(&elmo[i], 10);
            }

            RPDO2_Cmd_string(&elmo[i], (uint8_t *)"ST");
            Elmo_Delay100us_IDx(&elmo[i], 10);
        }

        /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ,�ܹ��������if�Ĳ���������CAN����ֻ�е��� */
        for (i = 1; i < ELMO_NUM; i++)
        {
            if (elmo[i].CurOPMode == elmo[i + 1].CurOPMode)
            {
                elmo[0].CurOPMode = UM_SCM; //�����STOPģʽ������ģʽͳһ�ˣ�ֻ�п�����SCMģʽ
            }
            else
            {
                elmo[0].CurOPMode = UM_UNC;
                break;
            }
        }
    }
    /* �����㲥ָ����������϶���Ŀ���ģʽ����ͬ����ʱ�����һ��ر� */
    else if (elmoID == 0 && elmo[elmoID].CurOPMode != UM_UNC)
    {
        /* ��ǰģʽΪ�ͷŵ��,�ȴ򿪵�����ٱ��� */
        if (elmo[elmoID].CurOPMode == UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID], 30);
        }
        /* ��ǰģʽΪ����ģʽ,���л�Ϊ�ٶ�ģʽ���ٹر� */
        else if (elmo[elmoID].CurOPMode == UM_TCM)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);

            /* �㲥ģʽ�£������ٶ�ģʽ����ΪSCM */
            for (i = 0; i <= ELMO_NUM; i++)
            {
                elmo[i].CurOPMode = UM_SCM;
            }

            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
            Elmo_Delay100us_IDx(&elmo[elmoID], 10);

            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID], 30);

            /* ����Ŀ���ٶ� */
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"JV", 0, TYPE_INTEGER, 0);
            Elmo_Delay100us_IDx(&elmo[elmoID], 10);

            /* �����ٶ�ģʽ */
            RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
            Elmo_Delay100us_IDx(&elmo[elmoID], 10);
        }

        RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"ST");
        Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    }
    /* ʹ�õ���ELMO���� */
    else
    {
        /* ��ǰģʽΪ�ͷŵ��,�ȴ򿪵�����ٱ��� */
        if (elmo[elmoID].CurOPMode == UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID], 30);
        }
        /* ��ǰģʽΪ����ģʽ,���л�Ϊ�ٶ�ģʽ���ٹر� */
        else if (elmo[elmoID].CurOPMode == UM_TCM)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);
            Elmo_Delay100us_IDx(&elmo[elmoID], 200);

            elmo[elmoID].CurOPMode = UM_SCM;

            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
            Elmo_Delay100us_IDx(&elmo[elmoID], 10);

            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID], 30);

            /* ����Ŀ���ٶ� */
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"JV", 0, TYPE_INTEGER, 0);
            Elmo_Delay100us_IDx(&elmo[elmoID], 10);

            /* �����ٶ�ģʽ */
            RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
            Elmo_Delay100us_IDx(&elmo[elmoID], 10);

            /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */
            for (i = 1; i < ELMO_NUM; i++) //�����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ
            {
                if (elmo[i].CurOPMode == elmo[i + 1].CurOPMode)
                {
                    elmo[0].CurOPMode = UM_SCM; //�����STOPģʽ������ģʽͳһ�ˣ�ֻ�п�����SCMģʽ
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }
        }

        RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"ST");
        Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    }
    return 0;
}

/*
********************************************************************************
*@  name      : Omni_Elmo_Close
*@  function  : �ͷŵ��̵��
*@  input     : ��
*@  output    : ��
********************************************************************************
*/
void Omni_Elmo_Close()
{
    RPDO2_Cmd_data(&elmo[1], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    RPDO2_Cmd_data(&elmo[2], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    RPDO2_Cmd_data(&elmo[3], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[1], 30);
    Elmo_Delay100us_IDx(&elmo[2], 30);
    Elmo_Delay100us_IDx(&elmo[3], 30);
}

/*
********************************************************************************
*@  name      : Omni_Elmo_Stop
*@  function  : ɲ�����������
*@  input     : ��
*@  output    : ��
********************************************************************************
*/
void Omni_Elmo_Stop()
{
    RPDO2_Cmd_string(&elmo[1], (uint8_t *)"ST");
    RPDO2_Cmd_string(&elmo[2], (uint8_t *)"ST");
    RPDO2_Cmd_string(&elmo[3], (uint8_t *)"ST");
    Elmo_Delay100us_IDx(&elmo[1], 10);
    Elmo_Delay100us_IDx(&elmo[2], 10);
    Elmo_Delay100us_IDx(&elmo[3], 10);
}

/*
********************************************************************************
*@  name      : Omni_Elmo_PVM
*@  function  : �����ٶ�ģʽ����
*@  input     : ��
*@  output    : ��
********************************************************************************
*/
// void Omni_Elmo_PVM()
//{
//     //����ǰ�����һ���޷�����
//     /*
//     VelX = clamp(VelX, -5, 5);
//     VelY = clamp(VelY, -5, 5);
//     VelZ = clamp(VelZ, -180, 180);
//     */
//
//     /* ����Ŀ���ٶ� */
//     RPDO2_Cmd_data(&elmo[1], (uint8_t *)"JV", 0, TYPE_INTEGER, Real2ElmoHead);
//     RPDO2_Cmd_data(&elmo[2], (uint8_t *)"JV", 0, TYPE_INTEGER, Real2ElmoLeft);
//     RPDO2_Cmd_data(&elmo[3], (uint8_t *)"JV", 0, TYPE_INTEGER, Real2ElmoRight);
//     Elmo_Delay100us_IDx(&elmo[1],10);
//     Elmo_Delay100us_IDx(&elmo[2],10);
//     Elmo_Delay100us_IDx(&elmo[3],10);

//    /* �����ٶ�ģʽ */
//    RPDO2_Cmd_string(&elmo[1], (uint8_t *)"BG");
//    RPDO2_Cmd_string(&elmo[2], (uint8_t *)"BG");
//    RPDO2_Cmd_string(&elmo[3], (uint8_t *)"BG");
//    Elmo_Delay100us_IDx(&elmo[1],10);
//    Elmo_Delay100us_IDx(&elmo[2],10);
//    Elmo_Delay100us_IDx(&elmo[3],10);
//
//    Elmo_PVM(1, Real2ElmoHead);
//    Elmo_PVM(2, Real2ElmoLeft);
//    Elmo_PVM(3, Real2ElmoRight);
//}

/*
********************************************************************************
*@  name      : Elmo_SetAcc(uint8_t elmoID, uint32_t acc, uint32_t dec)
*@  function  : �����ٶ�ģʽ��λ��ģʽ�ĵ���Ӽ���
*@  input     : elmoID     ȡelmo�Ľڵ�ID
*@                 acc        ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
*@                 dec        ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
*@  output    : 0         �������óɹ�
*@              1         ��������ʧ��
********************************************************************************
*/
uint8_t Elmo_SetAcc(uint8_t elmoID, uint32_t acc, uint32_t dec)
{
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[elmoID], 30);
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PM", 0, TYPE_INTEGER, 0x01);
    Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"AC", 0, TYPE_INTEGER, acc);
    Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"DC", 0, TYPE_INTEGER, dec);
    Elmo_Delay100us_IDx(&elmo[elmoID], 10);
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    Elmo_Delay100us_IDx(&elmo[elmoID], 100);
    return 0;
}

/*
********************************************************************************
  *@  name      : NMTCmd
  *@  function  : ����CANOPEN��NMT״̬����
  *@  input     : elmo     ȡelmo�Ľڵ�ID
                  MNTCmd   NMTָ��,NMT_xxx
  *@  output    : None
********************************************************************************
*/
static void NMTCmd(Elmo *elmo, uint8_t MNTCmd)
{
    uint16_t tmp_rear;

    switch (elmo->NodeID)
    {
#ifdef ID_0
    case 0:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID0;
        break;
#endif
#ifdef ID_1
    case 1:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID1;
        break;
#endif
#ifdef ID_2
    case 2:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID2;
        break;
#endif
#ifdef ID_3
    case 3:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID3;
        break;
#endif
#ifdef ID_4
    case 4:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID4;
        break;
#endif
#ifdef ID_5
    case 5:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID5;
        break;
#endif
#ifdef ID_6
    case 6:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID6;
        break;
#endif
#ifdef ID_7
    case 7:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID7;
        break;
#endif
#ifdef ID_8
    case 8:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID8;
        break;
#endif
#ifdef ID_9
    case 9:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID9;
        break;
#endif
#ifdef ID_10
    case 10:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID10;
        break;
#endif
#ifdef ID_11
    case 11:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID11;
        break;
#endif
#ifdef ID_12
    case 12:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID12;
        break;
#endif
#ifdef ID_13
    case 13:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID13;
        break;
#endif
#ifdef ID_14
    case 14:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID14;
        break;
#endif
#ifdef ID_15
    case 15:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID15;
        break;
#endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if (tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if (tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* ���������� */
        return;
    }
    /* ��仺���� */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID = COBID_NMT_SERVICE;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC = 2;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = MNTCmd;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = elmo->NodeID;

    /* ��Ч���ݼ�1 */
    Queue_Can_Update(QUEUE_CAN_IDx);
}

/*
********************************************************************************
  *@  name      : RSDO
  *@  function  : ʹ������SDO����ָ���
  *@  input     : elmo      ȡelmo�ڵ�ID
                                    Index     ����
                  SubIndex  ������
                  Data      ����
  *@  output    : None
********************************************************************************
*/
static void RSDO(Elmo *elmo, uint16_t Index, uint8_t SubIndex, uint32_t Data)
{
    uint16_t tmp_rear;

    switch (elmo->NodeID)
    {
#ifdef ID_0
    case 0:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID0;
        break;
#endif
#ifdef ID_1
    case 1:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID1;
        break;
#endif
#ifdef ID_2
    case 2:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID2;
        break;
#endif
#ifdef ID_3
    case 3:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID3;
        break;
#endif
#ifdef ID_4
    case 4:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID4;
        break;
#endif
#ifdef ID_5
    case 5:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID5;
        break;
#endif
#ifdef ID_6
    case 6:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID6;
        break;
#endif
#ifdef ID_7
    case 7:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID7;
        break;
#endif
#ifdef ID_8
    case 8:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID8;
        break;
#endif
#ifdef ID_9
    case 9:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID9;
        break;
#endif
#ifdef ID_10
    case 10:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID10;
        break;
#endif
#ifdef ID_11
    case 11:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID11;
        break;
#endif
#ifdef ID_12
    case 12:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID12;
        break;
#endif
#ifdef ID_13
    case 13:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID13;
        break;
#endif
#ifdef ID_14
    case 14:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID14;
        break;
#endif
#ifdef ID_15
    case 15:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID15;
        break;
#endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if (tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if (tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* ���������� */
        return;
    }
    /* ��仺���� */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID = COBID_RSDO + elmo->NodeID; // COBID
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC = 8;                           // DLC
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = 0x22;                    // CS,������δȷ��
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (Index & 0xFF);          // Index
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = (Index & 0xFF00) >> 8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = (SubIndex);    // SubIndex
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[4] = (Data & 0xFF); // Data
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[5] = (Data & 0xFF00) >> 8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[6] = (Data & 0xFF0000) >> 16;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[7] = (Data & 0xFF000000) >> 24;

    /* ��Ч���ݼ�1 */
    Queue_Can_Update(QUEUE_CAN_IDx);
}

/*
********************************************************************************
  *@  name      : RPDO2_Cmd_data
  *@  function  : ʹ�ö����Ʊ����ELMO��������ָ��
                  CANopen RPDO2 -> 0x2012 ����������-���ù���
  *@  input     : elmo   ȡelmo�ڵ�ID
                  Cmd    ����,���ַ�����ʽ����
                  Index  ������±�
                  Type   ��������
                  Data   ����
  *@  output    : None
********************************************************************************
*/
static void RPDO2_Cmd_data(Elmo *elmo, uint8_t *Cmd, uint8_t Index, uint8_t Type, uint32_t Data)
{
    uint16_t tmp_rear;
    switch (elmo->NodeID)
    {
#ifdef ID_0
    case 0:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID0;
        break;
#endif
#ifdef ID_1
    case 1:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID1;
        break;
#endif
#ifdef ID_2
    case 2:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID2;
        break;
#endif
#ifdef ID_3
    case 3:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID3;
        break;
#endif
#ifdef ID_4
    case 4:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID4;
        break;
#endif
#ifdef ID_5
    case 5:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID5;
        break;
#endif
#ifdef ID_6
    case 6:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID6;
        break;
#endif
#ifdef ID_7
    case 7:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID7;
        break;
#endif
#ifdef ID_8
    case 8:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID8;
        break;
#endif
#ifdef ID_9
    case 9:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID9;
        break;
#endif
#ifdef ID_10
    case 10:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID10;
        break;
#endif
#ifdef ID_11
    case 11:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID11;
        break;
#endif
#ifdef ID_12
    case 12:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID12;
        break;
#endif
#ifdef ID_13
    case 13:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID13;
        break;
#endif
#ifdef ID_14
    case 14:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID14;
        break;
#endif
#ifdef ID_15
    case 15:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID15;
        break;
#endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if (tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if (tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* ���������� */
        return;
    }
    /* ��仺���� */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID = COBID_RPDO2 + elmo->NodeID;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC = 8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = (*Cmd++);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (*Cmd);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = (Index);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = (Type << 7);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[4] = (Data & 0xFF);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[5] = (Data & 0xFF00) >> 8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[6] = (Data & 0xFF0000) >> 16;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[7] = (Data & 0xFF000000) >> 24;

    /* ��Ч���ݼ�1 */
    Queue_Can_Update(QUEUE_CAN_IDx);
}

/*
********************************************************************************
  *@  name      : RPDO2_Cmd_string
  *@  function  : ʹ�ö����Ʊ����ELMO�����ַ���ָ��
                  CANopen RPDO2 -> 0x2012 ����������-ִ�й���
  *@  input     : elmo   ȡelmo�ڵ�ID
                                    cmd    ����,���ַ�����ʽ����
  *@  output    : None
********************************************************************************
*/
static void RPDO2_Cmd_string(Elmo *elmo, uint8_t *Cmd)
{
    uint16_t tmp_rear;

    switch (elmo->NodeID)
    {
#ifdef ID_0
    case 0:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID0;
        break;
#endif
#ifdef ID_1
    case 1:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID1;
        break;
#endif
#ifdef ID_2
    case 2:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID2;
        break;
#endif
#ifdef ID_3
    case 3:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID3;
        break;
#endif
#ifdef ID_4
    case 4:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID4;
        break;
#endif
#ifdef ID_5
    case 5:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID5;
        break;
#endif
#ifdef ID_6
    case 6:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID6;
        break;
#endif
#ifdef ID_7
    case 7:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID7;
        break;
#endif
#ifdef ID_8
    case 8:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID8;
        break;
#endif
#ifdef ID_9
    case 9:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID9;
        break;
#endif
#ifdef ID_10
    case 10:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID10;
        break;
#endif
#ifdef ID_11
    case 11:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID11;
        break;
#endif
#ifdef ID_12
    case 12:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID12;
        break;
#endif
#ifdef ID_13
    case 13:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID13;
        break;
#endif
#ifdef ID_14
    case 14:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID14;
        break;
#endif
#ifdef ID_15
    case 15:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID15;
        break;
#endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if (tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if (tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* ���������� */
        return;
    }

    /* ��仺���� */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID = COBID_RPDO2 + elmo->NodeID;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC = 4;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = (*Cmd++);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (*Cmd);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = 0x00;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = 0x00;

    Queue_Can_Update(QUEUE_CAN_IDx);
}

/*
********************************************************************************
*@  name      : CAN_init
*@  function  : Initialization for CAN
*@  input     : None
*@  output    : None
********************************************************************************
*/
// static void CAN_init(CAN_TypeDef* CANx)
//{
//     GPIO_InitTypeDef         GPIO_InitStructure;
//     CAN_InitTypeDef          CAN_InitStructure;
//     CAN_FilterInitTypeDef    CAN_FilterInitStructure;
//     NVIC_InitTypeDef         NVIC_InitStructure;

//    if( CANx == CAN1 )
//    {
//        can = CAN1;                //Ϊ�ײ㷢�ͱ���ѡ��can��
//        /* Enable GPIO clock */
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

//        /* Enable CAN clock */
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

//        /* Configure CAN RX and TX pins */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//        GPIO_Init(GPIOA, &GPIO_InitStructure);

//        /* Connect CAN pins to AF9 */
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
//    }
//    else if( CANx == CAN2 )
//    {
//        can = CAN2;                //Ϊ�ײ㷢�ͱ���ѡ��can��
//        /* Enable GPIO clock */
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//        /* Enable CAN clock */
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

//        /* Configure CAN RX and TX pins */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//        GPIO_Init(GPIOB, &GPIO_InitStructure);

//        /* Connect CAN pins to AF9 */
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
//    }

//    /* CAN register init */
//    CAN_DeInit(CANx);
//    CAN_StructInit(&CAN_InitStructure);

//    /* CAN cell init */
//    CAN_InitStructure.CAN_TTCM = DISABLE;
//    CAN_InitStructure.CAN_ABOM = DISABLE;
//    CAN_InitStructure.CAN_AWUM = DISABLE;
//    CAN_InitStructure.CAN_NART = DISABLE;
//    CAN_InitStructure.CAN_RFLM = DISABLE;
//    CAN_InitStructure.CAN_TXFP = ENABLE;
//    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
//    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;

//    /* Baudrate = 1Mbps (CAN clocked at 42 MHz) */
//    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
//    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
//    CAN_InitStructure.CAN_Prescaler = 3;     //CAN������42/(1+9+4)/3=1Mbps
//    CAN_Init(CANx, &CAN_InitStructure);

//    /* CAN filter init */
//    CAN_FilterInitStructure.CAN_FilterNumber = 0;
//    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
//    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
//    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000<<5;
//    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000<<5;
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0006;
//    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
//    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//    CAN_FilterInit(&CAN_FilterInitStructure);

//    if( CANx == CAN1 )//�����Լ�
//    {
//        /* CAN FIFO0 message pending interrupt enable */
//        CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);

//        /* Enable the CAN1 global Interrupt */
//        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&NVIC_InitStructure);
//    }
//    else if( CANx == CAN2 )
//    {
//        /* CAN FIFO0 message pending interrupt enable */
//        CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);

//        /* Enable the CAN2 global Interrupt */
//        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&NVIC_InitStructure);
//    }
//}
// CAN_TxHeaderTypeDef TxMessage;
// CAN_RxHeaderTypeDef RxMessage;
__weak void Elmo_CAN_USER_Init(FDCAN_HandleTypeDef *hfdcan)
{
    /*
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef HAL_Status;

    sFilterConfig.FilterBank = 0;  //��������
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;  //filter id
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  //��FIFO0����
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_CAN_Start(hcan); //����CAN
    HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if(HAL_Status != HAL_OK)
    {
        Error_Handler();
    }
    */

    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN1_Init(hfdcan);
    }
    else
    {
        FDCAN2_Init(hfdcan);
    }
}

/*
********************************************************************************
*@  name      : CAN1_RX0_IRQHandler & CAN2_RX0_IRQHandler
*@  function  : CAN�жϴ�����
*@  input     : None
*@  output    : None
********************************************************************************
*/
// void CAN1_RX0_IRQHandler()
//{
//
// }
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	CAN_RxHeaderTypeDef RxMessage;
//	uint8_t RxData[8] = {0};
//
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData);
//
//	Elmo_Init_Flag |= (1<<((RxMessage.StdId&0x0F)-1));
//
//	/**********************************��ȡELMO����������*************************************/
//	if((RxMessage.StdId>COBID_TPDO2) && (RxMessage.StdId<(COBID_TPDO2 + 17)))
//		memcpy(&MotorPos[RxMessage.StdId - COBID_TPDO2-1], RxData, 4);
// }

/*
********************************************************************************
*@  name      : TIM_init
*@  function  : TIM7��ʼ����ʹCAN����ÿ100us����һ��
*@  input     : None
*@  output    : None
********************************************************************************
*/
static void Elmo_TIM_init(TIM_HandleTypeDef *htim)
{
    //    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    //    NVIC_InitTypeDef          NVIC_InitStructure;

    //    /* TIM7 clock enable */
    //    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);

    //    /* Enable the TIM7 global Interrupt */
    //    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    //    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PPr;
    //    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SPr;
    //    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //    NVIC_Init(&NVIC_InitStructure);

    //    /* Time base configuration (TIM7 clocked at 84 MHz)*/
    //    TIM_TimeBaseStructure.TIM_Period = 840;
    //    TIM_TimeBaseStructure.TIM_Prescaler = 10-1;
    //    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    //    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    //    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    //    /* TIM7 IT enable */
    //    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    //    TIM_ClearFlag(TIM7, TIM_FLAG_Update);

    //    /* TIM7 enable counter */
    //    TIM_Cmd(TIM7, ENABLE);
    HAL_TIM_Base_Start_IT(htim);
}

/*
********************************************************************************
*@  name      : TIM7_IRQHandler
*@  function  : TIM7�жϴ�������ÿ100us��������CAN����
*@  input     : None
*@  output    : None
********************************************************************************
*/
// void TIM7_IRQHandler(void)
//{

//    OSIntEnter();
//
//    /* 100us */
//    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
//    Elmo_SendCmd();
//
//    OSIntExit();
//
//}

/*
********************************************************************************
*@  name      : Self_test
*@  function  : Elmo�ϵ��Լ�
*@  input     : None
*@  output    : None
********************************************************************************
*/
int Self_test(TIM_HandleTypeDef *htim)
{
    //    uint32_t cnt_delay = 0x3FFF;

    //    do
    //    {
    //       __nop();
    //    }
    //    while( --cnt_delay );
    //	HAL_Delay(1);
    Elmo_software_delay_ms(1);
    if (CAN_Error == 1) //�����ʼ��ʧ�ܣ��ر�TIM7���жϺ�TIM7
    {
        HAL_TIM_Base_Stop_IT(htim);
        //        TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
        //        TIM_Cmd(TIM7, DISABLE);
        return 0x80000000;
    }

    Elmo_software_delay_ms(500);

    while ((Elmo_Init_Flag & ((1 << ELMO_NUM) - 1)) != ((1 << ELMO_NUM) - 1))
    {
        // canopen�豸����֮��ᷢ�ͱ��ģ����ĺ���������һ�������Լ���豸�������ߡ�
        return Elmo_Init_Flag;
    }
    //    CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
    //    CAN_ITConfig(CAN2,CAN_IT_FMP0, DISABLE);

    return 0;
}

/*
********************************************************************************
*@  name      : Variate_init
*@  function  : ������ʼ��
*@  input     : None
*@  output    : None
********************************************************************************
*/

static void Variate_init_QUEUE_CAN_IDx(CANQUEUE *QUEUE_CAN_IDx)
{
    memset(&QUEUE_CAN_IDx, 0, sizeof(QUEUE_CAN_IDx));
    memset(elmo, 0, sizeof(Elmo) * (ELMO_NUM + 1));
    memset(&elmogroup, 0, sizeof(Elmo));
    QUEUE_CAN_IDx->Front = 0;
    QUEUE_CAN_IDx->Rear = 0;
}

static void Variate_init(void)
{
#ifdef ID_0
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID0);
#endif
#ifdef ID_1
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID1);
#endif
#ifdef ID_2
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID2);
#endif
#ifdef ID_3
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID3);
#endif
#ifdef ID_4
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID4);
#endif
#ifdef ID_5
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID5);
#endif
#ifdef ID_6
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID6);
#endif
#ifdef ID_7
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID7);
#endif
#ifdef ID_8
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID8);
#endif
#ifdef ID_9
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID9);
#endif
#ifdef ID_10
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID10);
#endif
#ifdef ID_11
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID11);
#endif
#ifdef ID_12
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID12);
#endif
#ifdef ID_13
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID13);
#endif
#ifdef ID_14
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID14);
#endif
#ifdef ID_15
    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID15);
#endif

    //    VelX = 0;
    //    VelY = 0;
    //    VelZ = 0;
}

/*
********************************************************************************
*@  name      : Elmo_SendCmd_QUEUE_CAN_IDx
*@  function  : ��������п�����CAN���ģ�Ҳ�п������ӳٱ���
*@  input     : CANQUEUE *QUEUE_CAN_IDx Ҫ���͵Ļ�����
*@  output    : None
********************************************************************************
*/

void Elmo_SendCmd_QUEUE_CAN_IDx(FDCAN_HandleTypeDef *hfdcan, CANQUEUE *QUEUE_CAN_IDx) //����ָ��ָ��Ļ�����������
{
    /* �жϻ������Ƿ������� */
    if (QUEUE_CAN_IDx->Rear != QUEUE_CAN_IDx->Front)
    {
        /* ������,�ж��Ƿ�����ʱ���� */
        if (QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].COBID == CAN_ID_DELAY)
        {
            /* ����ʱָ��,�ж��Ƿ���ʱ��� */
            if (QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].DATA[0] > 1) //����һ��BUG������Ч���úܶ�
            {
                /* ��ʱδ��,��ʱʱ���1 */
                QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].DATA[0]--;
            }
            else
            {
                /* ��ʱ���,���׼�1 */
                QUEUE_CAN_IDx->Front++;
                if (QUEUE_CAN_IDx->Front >= CAN_BUF_NUM)
                {
                    QUEUE_CAN_IDx->Front = 0;
                }
            }
        }
        else
        {
            /* ������ʱָ��,����CAN���� */
            Elmo_CANSend(hfdcan, &QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front]);

            /*���׼�1*/
            QUEUE_CAN_IDx->Front++;
            if (QUEUE_CAN_IDx->Front >= CAN_BUF_NUM)
            {
                QUEUE_CAN_IDx->Front = 0;
            }
        }
    }
    else
    {
        /* ����Ϊ�� */
        return;
    }
}

/*
********************************************************************************
*@  name      : Elmo_SendCmd
*@  function  : ��������п�����CAN���ģ�Ҳ�п������ӳٱ���
*@  input     : None
*@  output    : None
********************************************************************************
*/

void Elmo_SendCmd(FDCAN_HandleTypeDef *hfdcan)
{
#ifdef ID_0
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID0);
#endif
#ifdef ID_1
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID1);
#endif
#ifdef ID_2
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID2);
#endif
#ifdef ID_3
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID3);
#endif
#ifdef ID_4
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID4);
#endif
#ifdef ID_5
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID5);
#endif
#ifdef ID_6
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID6);
#endif
#ifdef ID_7
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID7);
#endif
#ifdef ID_8
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID8);
#endif
#ifdef ID_9
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID9);
#endif
#ifdef ID_10
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID10);
#endif
#ifdef ID_11
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID11);
#endif
#ifdef ID_12
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID12);
#endif
#ifdef ID_13
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID13);
#endif
#ifdef ID_14
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID14);
#endif
#ifdef ID_15
    Elmo_SendCmd_QUEUE_CAN_IDx(hfdcan, &QUEUE_CAN_ID15);
#endif
}

/*
********************************************************************************
*@  name      : Elmo_CANSend
*@  function  : �ײ㷢��CAN����
*@  input     : None
*@  output    : None
********************************************************************************
*/
//	TxMessage.IDE = CAN_ID_STD;  //��׼֡,CAN_ID_EXT��չ֡;
//	TxMessage.RTR = CAN_RTR_DATA;  //����֡,CAN_RTR_REMOTEң��֡
static void Elmo_CANSend(FDCAN_HandleTypeDef *hfdcan, CANDATA *pCANDATA)
{
    //  uint8_t i, TransmitMailbox;
    //    uint32_t cnt_delay = 0x2FFF;
    //    CAN_TxHeaderTypeDef elmoCAN;
    //	uint8_t TxData[8];

    //    elmoCAN.IDE    =  CAN_ID_STD;                          // ��׼֡
    //    elmoCAN.RTR    =  CAN_RTR_DATA;                        // ����֡
    //    elmoCAN.StdId  =  pCANDATA->COBID;                     // COBID
    //    elmoCAN.DLC    =  pCANDATA->DLC;                       // DLC
    //    for(i=0;i<elmoCAN.DLC;i++)                             // Data
    //    {
    //       TxData[i] = pCANDATA->DATA[i];
    //    }
    //    TransmitMailbox = CAN_Transmit(can, &elmoCAN);                          // ���÷��ͱ��ĺ���
    CAN_Error = Elmo_CAN_Send_Data(hfdcan, pCANDATA->DATA, pCANDATA->COBID, pCANDATA->DLC);

    //    cnt_delay = 0x2FFF;
    //    do
    //    {
    //       __nop();
    //    }
    //    while((CAN_TransmitStatus(can,TransmitMailbox) != CANTXOK) && (--cnt_delay));
    //    if (cnt_delay <= 0x01 )
    //        CAN_Error = 1;
    //    else
    //        CAN_Error = 0;
}

/*
********************************************************************************
  *@  name      : Elmo_Delay100us_IDx
  *@  function  : �ӳٱ��������ӳ���һ�����ĵķ��ͣ���߷��ͱ��ĵ��ȶ���
                  N���Ϊ255��N��100us��
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
static void Elmo_Delay100us_IDx(Elmo *elmo, uint8_t N100us)
{
    uint16_t tmp_rear;
    switch (elmo->NodeID)
    {
#ifdef ID_0
    case 0:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID0;
        break;
#endif
#ifdef ID_1
    case 1:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID1;
        break;
#endif
#ifdef ID_2
    case 2:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID2;
        break;
#endif
#ifdef ID_3
    case 3:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID3;
        break;
#endif
#ifdef ID_4
    case 4:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID4;
        break;
#endif
#ifdef ID_5
    case 5:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID5;
        break;
#endif
#ifdef ID_6
    case 6:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID6;
        break;
#endif
#ifdef ID_7
    case 7:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID7;
        break;
#endif
#ifdef ID_8
    case 8:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID8;
        break;
#endif
#ifdef ID_9
    case 9:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID9;
        break;
#endif
#ifdef ID_10
    case 10:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID10;
        break;
#endif
#ifdef ID_11
    case 11:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID11;
        break;
#endif
#ifdef ID_12
    case 12:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID12;
        break;
#endif
#ifdef ID_13
    case 13:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID13;
        break;
#endif
#ifdef ID_14
    case 14:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID14;
        break;
#endif
#ifdef ID_15
    case 15:
        QUEUE_CAN_IDx = &QUEUE_CAN_ID15;
        break;
#endif
    }

    /* �жϻ������Ƿ����� */
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if (tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if (tmp_rear == QUEUE_CAN_IDx->Rear)
    {
        /* ���������� */
        return;
    }

    /* ��仺���� */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID = CAN_ID_DELAY; // ��ʱ�õ�COBID
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC = 1;              // DLCΪ1r
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = N100us;     // ��ʱ,��¼ʱ��

    /* ��Ч���ݼ�1 */
    Queue_Can_Update(QUEUE_CAN_IDx);
}

/*
********************************************************************************
*@  name      : Elmo_software_delay_ms
*@  function  : Elmo�����ʱ
*@  input     : None
*@  output    : None
********************************************************************************
*/
void Elmo_software_delay_ms(unsigned int t)
{
    int i;
    for (i = 0; i < t; i++)
    {
        int a = 44460; // at 168MHz 41580 is ok
        while (a--)
            ;
    }
}

/*
********************************************************************************
*@  name      : f2h
*@  function  : ��������ת��Ϊ8�ֽ�ʮ��������(IEEE754)
*@  input     : x   ������
*@  output    : None
********************************************************************************
*/
static uint32_t f2h(float x)
{
    uint32_t *p = (uint32_t *)&x;
    return ((uint32_t)*p);
}
/*
********************************************************************************
*@  name      : Elmo_Read_POS
*@  function  : ��ȡ��Ӧ��elmo�ı�����������
*@  input     : elmoID   �޷���8λ����
*@  output    : None
********************************************************************************
*/
void Elmo_Read_POS(uint8_t elmoID)
{
    //����TPDO2
    RSDO(&elmo[elmoID], 0x1A01, 0x00, 1);
    Elmo_Delay100us_IDx(&elmo[elmoID], 5);
    //��ȡelmo����������ֵ
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"PX");
    Elmo_Delay100us_IDx(&elmo[elmoID], 2);
    //�ر�TPDO2
    RSDO(&elmo[elmoID], 0x1A01, 0x00, 0);
    Elmo_Delay100us_IDx(&elmo[elmoID], 5);
}

/**
 * @brief  Elmo_Read_POS,��ȡ��Ӧ��elmo�ı�����������
 * @param  elmoID   �޷���8λ����
 *         pEncData int32_t ��ָ�룬ָ�򷵻�����
 * @retval 0   ���ɹ�
 *         1   ��ʧ��
 * @time   RSDORead
 *         -   1ms�����������1000�Σ�����ȡ(���������������յ�����)ʱ��δ���� 80000/84us = 952.38us
 *         -   �˴��ȴ�1.5ms
 * @note   �ж��ж�ȡ
 *         RxMsg.StdId == COBID_TSDO + ID;
 *         if ((RxMsg.Data[2]<<8 | RxMsg.Data[1]) == 0x6064)
 *             memcpy(&Encoder_Data, &RxMsg.Data[4],sizeof(int32_t));
 *         Ŀǰ��֧�� ID��MOTOR_ID_HIT_MAIN��MOTOR_ID_HIT_SERVE ������
 */
// uint32_t readElmo_Pos(uint8_t elmoID, int32_t *pEncData)
//{
//     uint32_t err = 1;
//     /* ����ʼ�տ���CAN1�����жϣ��ʲ����ٿ���CAN1�����ж� */
//     //CAN1->IER |= CAN_IT_FMP0;

//    /* ��ȡelmo����������ֵ */
//    Elmo_Read_POS(elmoID);
//
////    if(elmoID == MOTOR_ID_ARM_X)
////    {
////        if(xSemaphoreTake(PosArmXSem, 3*portTICK_MS) == pdTRUE)   //1.5ms  CAN������
////        {
////            *pEncData = EncoderData_1;
////            err = 0;
////        }
////        else
////            err = 1;
////    }
////    else if(elmoID == MOTOR_ID_CLIP)
////    {
////        if(xSemaphoreTake(PosClipSem, 3*portTICK_MS) == pdTRUE)   //1.5ms
////        {
////            *pEncData = EncoderData_3;
////            err = 0;
////        }
////        else
////            err = 1;
////    }
//
//    /* �ر�CAN1�����ж� */
//    //CAN1->IER &= ~CAN_IT_FMP0;
//    return err;
//}
/*
********************************************************************************
*@  name      : Elmo_Set_POS
*@  function  : ��ȡ��Ӧ��elmo�ı�����������
*@  input     : elmoID   �޷���8λ����
*@  output    : None
********************************************************************************
*/
void Elmo_Set_POS(uint8_t elmoID, int32_t POS)
{
    //�رյ������������λ��ֵ���Ⱦ�����
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[elmoID], 90);

    //�Ա�������ֵ��������
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PX", 0, TYPE_INTEGER, POS);
    Elmo_Delay100us_IDx(&elmo[elmoID], 90);

    //�������
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    Elmo_Delay100us_IDx(&elmo[elmoID], 90);
    //���µ�ǰ���״̬
    elmo[elmoID].CurOPMode = UM_UNC;
}

/*
********************************************************************************
*@  name      : Elmo_Read_ACT_CUR
*@  function  : ��ȡ��Ӧ��elmo����Ч����
*@  input     : elmoID   �޷���8λ����
*@  output    : None
********************************************************************************
*/
void Elmo_Read_ACT_CUR(uint8_t elmoID)
{
    //����TPDO2
    RSDO(&elmo[0], 0x1A01, 0x00, 1);
    Elmo_Delay100us_IDx(&elmo[elmoID], 50);
    //��ȡelmo����������ֵ
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"IQ");
    Elmo_Delay100us_IDx(&elmo[elmoID], 20);
    //�ر�TPDO2
    RSDO(&elmo[0], 0x1A01, 0x00, 0);
    Elmo_Delay100us_IDx(&elmo[elmoID], 50);
}

/*
********************************************************************************
  *@  name      : CAN_Send_Data
  *@  function  : CAN���ͱ���
  *@  input     : hcan    CAN���
                  pData   �������ݵ�ָ��
                  ID      ���ͱ��ĵ�ID
                  Len     ���ݳ���
  *@  output    : 0       ���ͳɹ�
                  2       ����ʧ��
********************************************************************************
*/
//	TxMessage.IDE = CAN_ID_STD;  //��׼֡,CAN_ID_EXT��չ֡;
//	TxMessage.RTR = CAN_RTR_DATA;  //����֡,CAN_RTR_REMOTEң��֡
static uint8_t Elmo_CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint8_t *pData, uint16_t ID, uint16_t Len)
{
    /*
    HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
    uint8_t FreeTxNum = 0;
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.StdId = ID;
    TxMessage.DLC = Len;
    TxMessage.IDE = IDE;
    TxMessage.RTR = RTR;

    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);

    while (FreeTxNum == 0) //�ȴ�������
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
    }

    //	Elmo_software_delay_ms(1); //û����ʱ���п��ܻᷢ��ʧ��,����ʱ��ֹ���ж�����

    HAL_RetVal = HAL_CAN_AddTxMessage(hcan, &TxMessage, pData, (uint32_t *)CAN_TX_MAILBOX0);

    if (HAL_RetVal != HAL_OK)
    {
        return 2;
    }

    return 0;
    */
    return FDCAN_SendData(hfdcan, pData, ID, Len);
}

/*
********************************************************************************
*@  name      : Queue_Can_Update
*@  function  : �������ݸ���
*@  input     : QUEUE_CAN_IDx ָ����е�ָ��
*@  output    : None
********************************************************************************
*/
static void Queue_Can_Update(CANQUEUE *QUEUE_CAN_IDx)
{
    /* ��Ч���ݼ�1 */
    QUEUE_CAN_IDx->Rear++;
    if (QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
    {
        QUEUE_CAN_IDx->Rear = 0;
    }
}

void Elmo_Get_Init_Flag(uint32_t StdId)
{
    Elmo_Init_Flag |= (1 << ((StdId & 0x0F) - 1));
}

void Elmo_Get_Encoder(uint32_t StdId, uint8_t *RxData)
{
    memcpy(&MotorPos[StdId - COBID_TPDO2 - 1], &RxData[4], 4);
}

int Get_Pos(uint8_t id)
{
    Elmo_Read_POS(id);
    int pos_tmp;
    pos_tmp = MotorPos[id - 1];
    return pos_tmp;
}
