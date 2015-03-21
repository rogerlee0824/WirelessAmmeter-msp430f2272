#include "24lc08.h"

/************************************************************************
*	以下程序为IC卡读写程序,由多个函数组成								*
*	程序用途: 读取/写入I2C地址的内容									*
*************************************************************************/
// 经示波器测试，为2.3us.
static void wait_1us(volatile INT32U cnt)
{ 
    while(cnt--)
	{
		asm("nop");
	}
}

static void I2C_Start(void)
{
	ATSDA_HIGH;
	wait_1us(10);
	wait_1us(10);
	ATSCL_HIGH;
	wait_1us(10);
	wait_1us(10);
	ATSDA_LOW;
	wait_1us(10);
	ATSCL_LOW;
	wait_1us(10);
	wait_1us(10);
}

/* at24cxx卡停止函数*/
static void at24cxx_stop(void)
{
	ATSCL_LOW;
	wait_1us(10);
	wait_1us(10);
	ATSDA_LOW;
	wait_1us(10);
	wait_1us(10);
	ATSCL_HIGH;
	wait_1us(10);
	ATSDA_HIGH;
	wait_1us(10);
	wait_1us(10);
}

/* at24cxx卡串行输入函数 */
static void at24cxx_si(INT8U si)
{
	unsigned char i=8;
	ATSDA_HIGH;
	while(i--)
	{
		if((si & 0x80))
		{
			ATSDA_HIGH;
		}
		else
		{
			ATSDA_LOW;
		}
		wait_1us(10);
		ATSCL_HIGH;
		wait_1us(10);
		ATSCL_LOW;
		wait_1us(10);
		si <<= 1;
	}
	ATSDA_LOW;       // acknowledge
	wait_1us(10);
	ATSCL_HIGH;
	wait_1us(10);
	ATSCL_LOW;
	wait_1us(10);
	ATSDA_HIGH;
	wait_1us(10);
}

/*at24cxx卡串行输出函数*/
static INT8U at24cxx_so(void)
{
	INT8U so = 0,i=8;
	ATSDA_HIGH;
    GPIO_SDA_DIR_INPUT;
    do{
        so <<= 1;
        if (ATSDA)
        {
            so |=0x01;
        }
        wait_1us(10);
        ATSCL_HIGH;
        wait_1us(10);
        ATSCL_LOW;
        wait_1us(10);
    }while(--i);
	GPIO_SDA_DIR_OUTPUT;
	return(so);
}

/* 读at24c32(/64)卡一个字节函数 */
INT8U at24cxx_read(INT8U addr)
{
	INT8U temp;
	I2C_Start();
	at24cxx_si(SLAVE_ADDR);
	at24cxx_si(addr);
	I2C_Start();
	at24cxx_si(SLAVE_ADDR|1);
	temp = at24cxx_so();
	at24cxx_stop();
	return (temp);
}

/*-------------------------------------------------------------------------*/
/* 向at24c32(/64)卡写一个字节函数 */
void at24cxx_write(INT8U addr,
                   INT8U save_data)
{
	I2C_Start();
	at24cxx_si(SLAVE_ADDR);
	at24cxx_si(addr);
	at24cxx_si(save_data);
	at24cxx_stop();
}

/*************************************************************************
 *函数功能：   		向24Cxx中一次性写入Length个字节
 *参数ucStartAddr： 起始地址
 *参数pBuffer：     要写入的数据缓存。
**************************************************************************/
void WriteBytesto24cxx(INT8U ucStartAddr,
                       INT8U *pucBuffer,
                       INT8U Length)
{
    INT8U i = 0;
	I2C_Start();
	at24cxx_si(SLAVE_ADDR);
	at24cxx_si((ucStartAddr<<4));
	wait_1us(20);
	for(i = 0;i < Length;i++)
	{
		at24cxx_si(*(pucBuffer + i));
	}
    wait_1us(10);
	at24cxx_stop();
}

/*************************************************************************
 *函数功能：   		向24Cxx中一次性写入16个字节
 *参数ucStartAddr： 起始地址
 *参数pBuffer：     要写入的数据缓存。
**************************************************************************/
void PageWriteto24cxx(INT8U ucStartAddr,
                      INT8U *pucBuffer)
{
    INT8U i = 0;
	I2C_Start();
	at24cxx_si(SLAVE_ADDR);
	at24cxx_si((ucStartAddr<<4));
	wait_1us(20);
	for(i = 0;i < 16;i++)
	{
		at24cxx_si(*(pucBuffer + i));
	}
    wait_1us(10);
	at24cxx_stop();
}
