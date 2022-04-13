/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __nRF24L01_H
#define __nRF24L01_H

//通过定义宏 NRF24SPI_Send_Byte 可以使用硬件SPI，否则使用模拟SPI
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
//#define nRF24_SPI_Send_Byte
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t		Ack_Buf[32];
	uint8_t		Ack_Len;
	uint8_t		Ack_Channel;
	uint8_t		Ack_Status;
}ACK_PAYLOAD;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
	// 1.GND	2.VCC
	// 3.CE		4.nCS		//RX/TX模式选择端		//SPI片选端SS
	// 5.SCK	6.MOSI		//SPI时钟端				//SPI主机输出从机输入端
	// 7.MISO	8.IRQ		//SPI主机输出从机输出端	//可屏蔽中断端
//==============================
//控制管脚


	#define debug_out(x)		//printf x
	#define CE_H()		( HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET) )
	#define CE_L()		( HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET) )		//GPIO_PORT_nRF->BRR  = GPIO_Pin_nRF_CE
	#define CSN_H()		( HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_SET) )		//GPIO_PORT_nRF_SPI->BSRR = GPIO_Pin_nRF_nCS
	#define CSN_L()		( HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_RESET) )		//GPIO_PORT_nRF_SPI->BRR  = GPIO_Pin_nRF_nCS

	#define nRF_IRQ()			HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port,NRF_IRQ_Pin)//GPIO_ReadInputDataBit(GPIO_PORT_nRF_IRQ, GPIO_Pin_nRF_IRQ)
//==============================
//nRF24L01寄存器操作命令
	#define nRF_READ_REG        0x00	//读配置寄存器,低5位为寄存器地址
	#define nRF_WRITE_REG       0x20	//写配置寄存器,低5位为寄存器地址
	
	#define nRF_R_RX_PLOAD		0x61	//读接收数据,1~32字节
	#define nRF_W_TX_PLOAD		0xA0	//写发送数据,1~32字节
	
	#define nRF_FLUSH_RX        0xE2	//清除RX FIFO寄存器.接收模式下用,在传输应答信号过程中不应执行此指令
	#define nRF_FLUSH_TX        0xE1	//清除TX FIFO寄存器.发射模式下用
	
	#define nRF_REUSE_TX_PL     0xE3	//数据重发,重新使用上一包数据,CE为高,数据包被不断发送.
	
	#define nRF_R_RX_PL_WID		0x60	//读RX FIFO顶层R_RX_PLOAD载荷长度,若超出32字节则清除RX FIFO
	#define nRF_W_ACK_PAYLOAD	0xA8//+ppp	//RX模式下,将PIPEppp(ppp的范围为000~101)中的AcK包与载荷一起写入,最多可以有3个挂起的ACK包.
	#define nRF_W_TX_PAYLOAD_NOACK	0xB0//发送数据,但禁止自动应答,发射模式用
	
	#define nRF_NOP				0xFF	//空操作,可以用来读状态寄存器
//=======补充参数
	#define Activate			0x50
	#define Code_Activate		0x73
//==============================
//SPI(nRF24L01)寄存器地址
	#define CONFIG				0x00//  配置收发状态，CRC校验模式以及收发状态响应方式
	#define EN_AA				0x01//  自动应答功能设置
	#define EN_RXADDR			0x02//  可用信道设置
	#define SETUP_AW			0x03// 收发地址宽度设置
	#define SETUP_RETR			0x04// 自动重发功能设置
	#define RF_CH				0x05// 工作频率设置
	#define RF_SETUP			0x06// 发射速率、功耗功能设置0b0010 0111
	#define STATUS				0x07  // 状态寄存器（写1清除对应的中断）
	#define OBSERVE_TX			0x08  // 发送监测功能
	#define CD					0x09  // 地址检测
	#define RX_ADDR_P0			0x0A  // 频道0接收数据地址（通道0的地址 注： 位39到位0可以随意改 ）
	#define RX_ADDR_P1			0x0B  // 频道1接收数据地址（通道1的地址，注：位39到位0可以随意改）
	#define RX_ADDR_P2			0x0C  // 频道2接收数据地址（通道2的地址，注：位39到位8同通道1,位7到位0可以随意改）
	#define RX_ADDR_P3			0x0D  // 频道3接收数据地址（通道3的地址，注：位39到位8同通道1,位7到位0可以随意改）
	#define RX_ADDR_P4			0x0E  // 频道4接收数据地址（通道4的地址，注：位39到位8同通道1,位7到位0可以随意改）
	#define RX_ADDR_P5			0x0F  // 频道5接收数据地址（通道5的地址，注：位39到位8同通道1,位7到位0可以随意改）
	#define TX_ADDR				0x10  // 发送地址寄存器（发射地址，注： 位39到位0可以随意改）
	#define RX_PW_P0			0x11  // 接收频道0接收数据长度
	#define RX_PW_P1			0x12  // 接收频道1接收数据长度
	#define RX_PW_P2			0x13  // 接收频道2接收数据长度
	#define RX_PW_P3			0x14  // 接收频道3接收数据长度
	#define RX_PW_P4			0x15  // 接收频道4接收数据长度
	#define RX_PW_P5			0x16  // 接收频道5接收数据长度
	#define FIFO_STATUS			0x17  // FIFO栈入栈出状态寄存器设置（只读）
//=======补充参数
	#define DYNPD				0x1C  // nRF24L01 Dynamic payload setup
	#define FEATURE				0x1D  // nRF24L01 Exclusive feature setup
//==============================
	#define MAX_RT				0x10	//达到最大发送次数中断
	#define TX_DS   			0x20	//TX发送完成中断
	#define RX_DR   			0x40	//接收到数据中断

	#define nRF_Mode_TX			0
	#define nRF_Mode_RX			1
//==============================
//24L01发送接收数据宽度定义
	#define TX_ADR_WIDTH    	5		//5字节的地址宽度
	#define RX_ADR_WIDTH    	5		//5字节的地址宽度
	#define RX_PLOAD_WIDTH  	32		//指定的用户数据宽度
	#define ACK_PLOAD_WIDTH  	5		//指定的用户数据宽度
	
	//接收机,ACK 应答通道N,接收通道0~5,(间隔250*4=1ms,2次,)速率250K,0db,校验9位,接收模式
	#define nRF_RX_CONFIG		0x0,  0x0,  0x26, 40, 0x26, 0x0f			//不要看这里，没有用到
	//发射机,ACK 应答通道0,接收通道0,间隔250*4=1ms,2次,速率250K,0db,校验9位,发送模式
	#define nRF_TX_CONFIG		0x01, 0x01, 0x1a, 40, 0x26, 0x0e			//不要看这里，没有用到
//==============================
	#define DYNAMIC_PAYLOAD		1//使用动态加载数据长度[长度相符时,接收机可以静态接收发射机的动态数据]
	#if		DYNAMIC_PAYLOAD
	#define DYNAMIC_PAYLOAD_ACK	1//使用动态数据长度时ACK功能
	#if		DYNAMIC_PAYLOAD_ACK
	#define DYNPD_ACK_DATA		1//使用动态数据长度时带数据ACK功能
	#endif
	#endif
	#define nRF_TX_Channel		0//发送通道号(相对于接收机的接收通道号)
	
	#define nRF_PTX				0//发射机标志
/* Exported functions ------------------------------------------------------- */
	void MY_GPIO_Init(void);
	void nRF24L01_Configuration(void);
	void nRF24L01_Set_Config(void);
	uint8_t nRF24L01_Check(void);
	void nRF24L01_Set_Mode(uint8_t rx_mode);
//==============================
	uint8_t nRF24L01_RxPacket(uint8_t * rx_buf);
	uint8_t nRF24L01_TxPacket(uint8_t tx_channel, uint8_t * tx_buf, uint8_t len);
	uint8_t nRF24L01_Tx_Ack(ACK_PAYLOAD *ack_pay);
	void nRF24L01_Rx_AckPayload(ACK_PAYLOAD ack_pay);
/* External variables --------------------------------------------------------*/
	extern uint8_t nRF24L01_TxBuf[32];
	extern uint8_t nRF24L01_RxBuf[32];

	extern ACK_PAYLOAD nRF24L01_ack_pay;
#endif
//==============================
