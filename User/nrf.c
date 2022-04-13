//2013-10-28测试多发一收正常,包括:
//固定长度收发,使用ACK及不使用ACK
//可变长度收发,使用ACK及不使用ACK,以及使用带回传数据的ACK
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "nrf.h"
#include "stdio.h"
#include "spi.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define car_num 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void delay_us(uint32_t n);
/* Private functions ---------------------------------------------------------*/
ACK_PAYLOAD nRF24L01_ack_pay;

uint8_t nRF24L01_TxBuf[32]={0};
uint8_t nRF24L01_RxBuf[32]={0};

//接收机
uint8_t RX_ADDRESS0[RX_ADR_WIDTH] = {0x20,0x22,0x2c,0x01,0x01};	// 信道0 = TXDEV0_TX_ADDR = DEV1_RX_ADDRESS0{0x30, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS1[RX_ADR_WIDTH] = {0x20,0x22,0x2c,0x01,0x22};	// 信道1 = TXDEV1_TX_ADDR = DEV1_RX_ADDRESS0{0x31, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS2[1]= {0x32};										// 信道2 = TXDEV2_TX_ADDR = DEV1_RX_ADDRESS0{0x32, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS3[1]= {0x33};										// 信道3 = TXDEV3_TX_ADDR = DEV1_RX_ADDRESS0{0x33, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS4[1]= {0x34};										// 信道4 = TXDEV4_TX_ADDR = DEV1_RX_ADDRESS0{0x34, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS5[1]= {0x35};										// 信道5 = TXDEV5_TX_ADDR = DEV1_RX_ADDRESS0{0x35, 0x37, 0x33, 0x34, 0x30}

uint8_t tx_channel_addr[RX_ADR_WIDTH] = {0x20,0x22,0x2c,0x01,0x01};
uint8_t nRF24_SPI_Send_Byte(uint8_t data)
{
	uint8_t	ret;
	HAL_SPI_TransmitReceive(&hspi1,&data,&ret,1,100);
	return ret;	
}


/**********************************************************************	
*@	function: 就delay				
*@	input		: none							
*@	output	: none															
***********************************************************************/ 
void delay_us(uint32_t n)
{
	uint8_t i;

	while(n--)
	{
		i = 8;
		while(i--);
	}
}


/*===============================寄存器读写==========================================*/
uint8_t nRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;

	CSN_L();						// CSN low, initialize SPI communication...
	nRF24_SPI_Send_Byte(reg);		// Select register to read from..
	reg_val = nRF24_SPI_Send_Byte(0xff);// ..then read registervalue
	CSN_H();						// CSN high, terminate SPI communication

	return(reg_val);				// return register value
}

uint8_t nRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CSN_L();						// CSN low, init SPI transaction
	status = nRF24_SPI_Send_Byte(reg);// select register
	nRF24_SPI_Send_Byte(value);		// ..and write value to it..
	CSN_H();						// CSN high again

	return(status);					// return nRF24L01 status uint8_t
}

uint8_t nRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status, i;

	CSN_L();							// Set CSN low, init SPI tranaction
	status = nRF24_SPI_Send_Byte(reg);	// Select register to write to and read status uint8_t

	for(i = 0; i < Len; i++)
	{
		pBuf[i] = nRF24_SPI_Send_Byte(0xff);
	}

	CSN_H();                           

	return(status);						// return nRF24L01 status uint8_t
}

uint8_t nRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status, i;

	CSN_L();				//SPI使能       
	status = nRF24_SPI_Send_Byte(reg);   
	for(i = 0; i < Len; i++)//
	{
		nRF24_SPI_Send_Byte(pBuf[i]);
	}
	CSN_H();				//关闭SPI
	return(status);			// 
}

/*===============================end==========================================*/




/**********************************************************************	
*@	function: 寄存器初始化			
*@	input		: none							
*@	output	: none															
***********************************************************************/ 
void nRF24L01_Set_Config(void)
{	
	CE_L();

		nRF24L01_Write_Reg(nRF_WRITE_REG | SETUP_AW,    0x03);											// 0x03对应宽度5
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS1, RX_ADR_WIDTH);	// 写接收端地址
//		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P1, RX_ADDRESS1, RX_ADR_WIDTH);	// 写接收端地址
	//nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);	//2-5通道使用1字节
		nRF24L01_Write_Reg(nRF_WRITE_REG | EN_AA,       0x01);											// 频道0自动	ACK应答允许	
		nRF24L01_Write_Reg(nRF_WRITE_REG | EN_RXADDR,   0x01);											//通道0允许接收数据
		nRF24L01_Write_Reg(nRF_WRITE_REG | SETUP_RETR,  0x1a);											//自动重发延时：500+86us，10次
		nRF24L01_Write_Reg(nRF_WRITE_REG | RF_CH,       1);												//工作通道频率，收发一样即可
		nRF24L01_Write_Reg(nRF_WRITE_REG | RF_SETUP,    0x0f);											//数据传输率：2Mbps，发射功率：0DB，
		nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG,      0x0f);											//16位CRC使能，接收模式，开启全部中断
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);											//清除TX FIFO寄存器.发射模式下用
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX, 0xff);											//清除RX FIFO寄存器.接收模式下用,在传输应答信号过程中不应执行此指令
		nRF24L01_Write_Reg(nRF_WRITE_REG | DYNPD, 0x01);														//使能通道0的动态载荷长度
		nRF24L01_Write_Reg(nRF_WRITE_REG | FEATURE,  0x06);													//使能ACK带载,使能ACK的动态载荷长度

	

	CE_H();	// Set CE pin high to enable RX device
	delay_us(130);
//	CE_L();
}

/**********************************************************************	
*@	function:检测24L01是否存在		
*@	input		: none							
*@	output	: 返回值:成功 0 ； 失败	1													
***********************************************************************/ 
uint8_t nRF24L01_Check(void)		
{
	uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	uint8_t i;
	
	nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR, buf, 5);	//写入5个字节的地址.
	nRF24L01_Read_Buf(nRF_READ_REG + TX_ADDR, buf, 5);		//读出写入的地址
	for(i = 0; i < 5; i++)
	{
		if(buf[i] != 0xA5)
		{
			break;
		}
	}
	if(i != 5)
	{
		debug_out(("nRF24L01 TEST FAUSE\r\n"));
		return 1;											//检测24L01错误
	}
	else 
	{
		debug_out(("nRF24L01 TEST OK\r\n"));
		return 0;											//检测到24L01
	}
}

/**********************************************************************	
*@	function:切换模式（没啥用）	
*@	input		: none							
*@	output	: none												
***********************************************************************/ 
void nRF24L01_Set_Mode(uint8_t rx_mode)	//修改为接收/发射模式
{
	uint8_t mode = nRF24L01_Read_Reg(CONFIG);
	//debug_out(("mode=0x%02x\r\n",mode));
	if(rx_mode)	//1接收模式
	{
	 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG, mode | 0x01);	
	}
	else		//0发送模式
	{
	 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG, mode & 0xfe);	
	}
}

//==============================接收、发送数据包===========================================
uint8_t nRF24L01_RxPacket(uint8_t * rx_buf)
{
	uint8_t len = 0;
	uint8_t status;
	
//	CE_L();
	status = nRF24L01_Read_Reg(STATUS);							// 读取状态寄存其来判断数据接收状况
//	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status & RX_DR);			//接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清楚中断标志
	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);			//接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清楚中断标志
	
#if (DYNPD_ACK_DATA)
	if(status & TX_DS)//
	{
		//debug_out(("PRX TX_DS\r\n"));
	}
#endif
	if(status & RX_DR)											// 判断是否接收到数据
	{
		len = nRF24L01_Read_Reg(nRF_R_RX_PL_WID);
		if(len < 33)
		{
			nRF24L01_Read_Buf(nRF_R_RX_PLOAD, rx_buf, len);	// read receive payload from RX_FIFO buffer
		}
		else 
		{
			len = 0;
		}
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//清空缓冲区
		
		nRF24L01_ack_pay.Ack_Channel = (status & 0x0e) >> 1;
	}
	
//	CE_H();	//置高CE，激发数据发送
	return len;
}
uint8_t nRF24L01_TxPacket(uint8_t tx_channel, uint8_t * tx_buf, uint8_t len)
{
	CE_L();
	
	if((tx_channel == 0))
	{
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);//2-5通道使用1字节
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);// 写接收端地址
	}
	else
	{
		tx_channel_addr[0] = 0x30 + tx_channel;
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    tx_channel_addr, RX_ADR_WIDTH);//2-5通道使用1字节
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, tx_channel_addr, RX_ADR_WIDTH);// 写接收端地址
	}
#if DYNAMIC_PAYLOAD_ACK
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// 装载数据	
#else
	//nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// 装载数据	
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PAYLOAD_NOACK, tx_buf, len);	// 装载数据
#endif
	
	CE_H();	//置高CE，激发数据发送
	delay_us(20);
	
	while(nRF_IRQ());										//等待发送完成
	
	return (nRF24L01_Tx_Ack(&nRF24L01_ack_pay));
}
uint8_t nRF24L01_Tx_Ack(ACK_PAYLOAD *ack_pay)
{
 	uint8_t tmp = 200;
	uint8_t status;
	while(1)
	{
		status = nRF24L01_Read_Reg(STATUS);					//读取状态寄存器的值
		
		nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);	// 清除TX_DS或MAX_RT中断标志
		
#if (DYNPD_ACK_DATA)
		if(status & RX_DR)		// 判断是否接收到数据
		{
			ack_pay->Ack_Len = nRF24L01_Read_Reg(nRF_R_RX_PL_WID);
			if(ack_pay->Ack_Len < 33)
			{
				nRF24L01_Read_Buf(nRF_R_RX_PLOAD, ack_pay->Ack_Buf, ack_pay->Ack_Len);	// read receive payload from RX_FIFO buffer
				ack_pay->Ack_Status = 1;
			}
			else 
			{
				ack_pay->Ack_Len = 0;
			}
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//清空缓冲区
			
			ack_pay->Ack_Channel = (status & 0x0e) >> 1;
			debug_out(("PTX RX_DR\r\n"));
		}
#endif
		if(status & MAX_RT)		//达到最大重发次数
		{
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//清除TX FIFO寄存器 
			debug_out(("PTX MAX_RT\r\n"));
			return MAX_RT;
		}
		else if(status & TX_DS)	//发送完成
		{
			debug_out(("PTX TX_DS\r\n"));
			return TX_DS;
		}
		else					//等待
		{
			delay_us(10);
			tmp--;
			if(tmp == 0)
			{
				nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//清除TX FIFO寄存器 
				debug_out(("PTX time_out\r\n"));
				return 0xff;//其他原因发送失败
			}
		}
	}
}

/*****************************************
*@ function : 装载回传数据包
*@
*****************************************/
#if (DYNPD_ACK_DATA)//带数据的ACK功能
void nRF24L01_Rx_AckPayload(ACK_PAYLOAD ack_pay)
{
	CE_L();
	nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//清除TX FIFO寄存器 
//	switch(nRF24L01_ack_pay.Ack_Channel){
//		case 1:       
//			nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS1, RX_ADR_WIDTH);// 写接收端地址
//		break;
//		
//	}
		

	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_ACK_PAYLOAD | ack_pay.Ack_Channel, ack_pay.Ack_Buf, ack_pay.Ack_Len);// 装载数据
	CE_H();
	//debug_out(("加载带数据ACK\r\n"));
}
#endif
//==============================end===========================================

//-------------使用说明---------------------//
// 1.1.PTX发送数据到PRX
// #if (DYNAMIC_PAYLOAD) //动态数据长度
// 		nRF24L01_TxPacket(channel, tmp_data, tmp_len);
// #else                 //固定数据长度
// 		nRF24L01_TxPacket(channel, tmp_data, RX_PLOAD_WIDTH);
// #endif
// 1.2.PTX从PRX接收反向ACK数据
// #if (DYNPD_ACK_DATA)  //接收反向数据
//     if(nRF24L01_ack_pay.Ack_Status)
//     {
//         nRF24L01_ack_pay.Ack_Status = 0;
//         debug_out(("[%d]ACK(%d):%s\r\n",nRF24L01_ack_pay.Ack_Channel,nRF24L01_ack_pay.Ack_Len,nRF24L01_ack_pay.Ack_Buf));
//     }
// #endif
// 2.PRX接收PTX发送来的数据
// tmp_len = nRF24L01_RxPacket(data, &channel);//接收数据
// if(tmp_len)
// {
// #if (DYNPD_ACK_DATA)//回送反向的ACK数据
//     nRF24L01_Rx_AckPayload(nRF24L01_ack_pay);
// #endif
// 		debug_out(("接收到[%d]字节 @%d:%s\r\n", tmp_len, channel, data));
// }
//-----------------------------------------//
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
