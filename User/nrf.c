#include "nrf.h"

void Handle_Init(SPI_HandleTypeDef *hspi,uint8_t Channel)
{
    Handle_CE_L();
    Handle_CS_H();

    HAL_Delay(200);

    while (NRF24L01_Check(hspi)) //检测模块
    {
        HAL_Delay(200);
    }
    NRF24L01_RX_Mode(hspi,Channel); //接收模式
}

/*****************NRF24L01模块部分*************************************************************/

const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {222, 111, 001, 100, 151}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {222, 111, 001, 100, 151}; //发送地址

/*******************************************************************************
 * function : SPI_SetSpeed(SPI_HandleTypeDef* hspi,uint8_t SPI_BaudRatePrescaler)
 * brief : hspi
                     SPI速度 = fAPB1 / 分频系数
                     fAPB1时钟为45Mhz
 * param : SPI_BaudRate_Prescaler  :  SPI_BAUDRATEPRESCALER_2  ~ SPI_BAUDRATEPRESCALER_2 256
 * return : 无
*******************************************************************************/

void SPI_SetSpeed(SPI_HandleTypeDef *hspi, uint32_t SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler)); //判断有效性
    __HAL_SPI_DISABLE(hspi);                                        //关闭SPI
    hspi->Instance->CR1 &= 0XFFC7;                                  //位3-5清零，用来设置波特率
    hspi->Instance->CR1 |= SPI_BaudRatePrescaler;                   //设置SPI速度
    __HAL_SPI_ENABLE(hspi);                                         //使能SPI
}

/*******************************************************************************
 * function : SPI_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t TxData)
 * brief :
 * param : TxData : 写入的字节
 * return : Rxdata : 读到的字节
 *******************************************************************************/

uint8_t SPI_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(hspi, &TxData, &Rxdata, 1, 1000);
    return Rxdata; //返回收到的数据
}

/*******************************************************************************
 * function : NRF24L01_Check(SPI_HandleTypeDef* hspi)
 * brief : 检测24l01是否存在
 * param : hspi
 * return : 0  成功
            1	 失败
*******************************************************************************/

uint8_t NRF24L01_Check(SPI_HandleTypeDef *hspi)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;
    SPI_SetSpeed(hspi, SPI_BAUDRATEPRESCALER_8);               // spi速度为11.25Mhz（24L01的最大SPI时钟为10Mhz,这里大一点没关系）
    NRF24L01_Write_Buf(hspi, NRF_WRITE_REG + TX_ADDR, buf, 5); //写入5个字节的地址.
    NRF24L01_Read_Buf(hspi, TX_ADDR, buf, 5);                  //读出写入的地址
    for (i = 0; i < 5; i++)
    {
        if (buf[i] != 0XA5)
            break;
    }
    if (i != 5)
    {
        return 1; //检测24L01错误
    }
    return 0; //检测到24L01
}

/*******************************************************************************
 * function : NRF24L01_Write_Reg(SPI_HandleTypeDef* hspi,uint8_t reg, uint8_t value)
 * brief : SPI 给 NRF模块 写寄存器
 * param : hspi
                     reg    寄存器地址
                     value  写入的值
 * return : status 状态值
*******************************************************************************/

uint8_t NRF24L01_Write_Reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
    uint8_t status;
    Handle_CS_L();                         //使能SPI传输
    status = SPI_ReadWriteByte(hspi, reg); //发送寄存器号
    SPI_ReadWriteByte(hspi, value);        //写入寄存器的值
    Handle_CS_H();                         //禁止SPI传输
    return (status);                       //返回状态值
}

/*******************************************************************************
 * function : NRF24L01_Read_Reg(SPI_HandleTypeDef* hspi,uint8_t reg)
 * brief : SPI 读取 NRF模块 寄存器
 * param :  hspi
                        reg    寄存器地址
 * return : reg_val 状态值
*******************************************************************************/

uint8_t NRF24L01_Read_Reg(SPI_HandleTypeDef *hspi, uint8_t reg)
{
    uint8_t reg_val;
    Handle_CS_L();                           //使能SPI传输
    SPI_ReadWriteByte(hspi, reg);            //发送寄存器号
    reg_val = SPI_ReadWriteByte(hspi, 0XFF); //读取寄存器内容
    Handle_CS_H();                           //禁止SPI传输
    return (reg_val);                        //返回状态值
}

/*******************************************************************************
 * function : NRF24L01_Read_Buf(SPI_HandleTypeDef* hspi,uint8_t reg, uint8_t *pBuf, uint8_t len)
 * brief : SPI 读取指定长度数据
 * param : hspi
                     reg    寄存器地址
                     *pBuf  数据指针
                     len    数据长度
 * return : status 状态值
*******************************************************************************/

uint8_t NRF24L01_Read_Buf(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, u8_ctr;
    Handle_CS_L();                         //使能SPI传输
    status = SPI_ReadWriteByte(hspi, reg); //发送寄存器值(位置),并读取状态值
    for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
        pBuf[u8_ctr] = SPI_ReadWriteByte(hspi, 0XFF); //读出数据
    Handle_CS_H();                                    //关闭SPI传输
    return status;                                    //返回读到的状态值
}

/*******************************************************************************
 * function : NRF24L01_Write_Buf(SPI_HandleTypeDef* hspi,uint8_t reg, uint8_t *pBuf, uint8_t len)
 * brief : SPI  发送指定长度数据
 * param : hspi
                     reg    寄存器地址
                     *pBuf  数据指针
                     len    数据长度
 * return : status 状态值
*******************************************************************************/

uint8_t NRF24L01_Write_Buf(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, u8_ctr;
    Handle_CS_L();                         //使能SPI传输
    status = SPI_ReadWriteByte(hspi, reg); //发送寄存器值(位置),并读取状态值
    for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
        SPI_ReadWriteByte(hspi, *pBuf++); //写入数据
    Handle_CS_H();                        //关闭SPI传输
    return status;                        //返回读到的状态值
}

/*******************************************************************************
 * function : NRF24L01_TxPacket(SPI_HandleTypeDef* hspi,uint8_t *txbuf)
 * brief : SPI 给NRF 发送一次数据 ,需要IRQ引脚，默认不开启
 * param : hspi
                     *txbuf    数据指针
 * return : TX_OK    发送成功
            0xff     发送失败
*******************************************************************************/

uint8_t NRF24L01_TxPacket(SPI_HandleTypeDef *hspi, uint8_t *txbuf)
{
    uint8_t sta;
    SPI_SetSpeed(hspi, SPI_BAUDRATEPRESCALER_8); // spi速度为6.75Mhz（24L01的最大SPI时钟为10Mhz）
    Handle_CE_L();
    NRF24L01_Write_Buf(hspi, WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //写数据到TX BUF  32个字节
    Handle_CE_H();                                                //启动发送
    while (Handle_IRQ_Read() != 0)
        ;                                                  //等待发送完成
    sta = NRF24L01_Read_Reg(hspi, STATUS);                 //读取状态寄存器的值
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
    if (sta & MAX_TX)                                      //达到最大重发次数
    {
        NRF24L01_Write_Reg(hspi, FLUSH_TX, 0xff); //清除TX FIFO寄存器
        return MAX_TX;
    }
    if (sta & TX_OK) //发送完成
    {
        return TX_OK;
    }
    return 0xff; //其他原因发送失败
}

/*******************************************************************************
 * function : NRF24L01_RxPacket(SPI_HandleTypeDef* hspi,uint8_t *rxbuf)
 * brief : SPI 读 NRF 一次数据
 * param : hspi
                     *rxbuf    数据指针
 * return : 0        接收成功
            1        接收失败
*******************************************************************************/

uint8_t NRF24L01_RxPacket(SPI_HandleTypeDef *hspi, uint8_t *rxbuf)
{
    uint8_t sta;
    SPI_SetSpeed(hspi, SPI_BAUDRATEPRESCALER_8);           // spi速度为6.75Mhz（24L01的最大SPI时钟为10Mhz）
    sta = NRF24L01_Read_Reg(hspi, STATUS);                 //读取状态寄存器的值
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
    if (sta & RX_OK)                                       //接收到数据
    {
        NRF24L01_Read_Buf(hspi, RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); //读取数据
        NRF24L01_Write_Reg(hspi, FLUSH_RX, 0xff);                    //清除RX FIFO寄存器
        return 0;
    }
    return 1; //没收到任何数据
}

/*******************************************************************************
 * function : NRF24L01_RX_Mode(SPI_HandleTypeDef* hspi)
 * brief : 		初始化NRF为RX模式
                            设置RX地址   写RX数据宽度   选择RF频道
 * param : hspi
 * return : 无
*******************************************************************************/

void NRF24L01_RX_Mode(SPI_HandleTypeDef *hspi,uint8_t Channel)
{
    Handle_CE_L();
    NRF24L01_Write_Buf(hspi, NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + EN_AA, 0x01);                                     //使能通道0的自动应答
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + EN_RXADDR, 0x01);                                 //使能通道0的接收地址
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + RF_CH, Channel);                                       //设置RF通信频率
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                        //选择通道0的有效数据宽度
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + RF_SETUP, 0x27);                                  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + CONFIG, 0x0f);                                    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
    Handle_CE_H();                                                                             // CE为高,进入接收模式
}

/*******************************************************************************
 * function : NRF24L01_TX_Mode(SPI_HandleTypeDef* hspi)
 * brief : 		初始化NRF为TX模式
                            设置TX地址  写TX数据宽度  设置RX自动应答的地址   填充TX发送数据  选择RF频道
 * param : hspi
 * return : 无
*******************************************************************************/

void NRF24L01_TX_Mode(SPI_HandleTypeDef *hspi)
{
    Handle_CE_L();
    NRF24L01_Write_Buf(hspi, NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);    //写TX节点地址
    NRF24L01_Write_Buf(hspi, NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + EN_AA, 0x01);                                     //使能通道0的自动应答
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + EN_RXADDR, 0x01);                                 //使能通道0的接收地址
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + SETUP_RETR, 0x1a);                                //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + RF_CH, 76);                                       //设置RF通道为40
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + RF_SETUP, 0x27);                                  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    NRF24L01_Write_Reg(hspi, NRF_WRITE_REG + CONFIG, 0x0e);                                    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    Handle_CE_H();                                                                             // CE为高,10us后启动发送
}
