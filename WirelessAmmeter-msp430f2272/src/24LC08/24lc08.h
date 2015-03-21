#ifndef _RX_8025T_H_
#define _RX_8025T_H_

#include <msp430x41x2.h>	
#include <hw_types.h>

#define READ          	1
#define WRITE         	0

#define SLAVE_ADDR			    0xa0

#define ATSDA_LOW               P5OUT &= ~BIT6
#define ATSDA_HIGH              P5OUT |= BIT6
#define ATSCL_LOW               P5OUT &= ~BIT7
#define ATSCL_HIGH              P5OUT |= BIT7
#define ATSDA		            (P5IN & BIT6)

#define GPIO_SDA_DIR_INPUT      P5DIR &= ~BIT6  // P5.6 inputs
#define GPIO_SDA_DIR_OUTPUT     P5DIR |= BIT6   // P5.6 outputs  
 
extern INT8U at24cxx_read(INT8U addr);
extern void at24cxx_write(INT8U addr,INT8U save_data);

/*************************************************************************
 *函数功能：   		向24Cxx中一次性写入16个字节
 *参数ucStartAddr： 起始地址
 *参数pBuffer：     要写入的数据缓存。
**************************************************************************/
extern void PageWriteto24cxx(INT8U ucStartAddr,INT8U *pBuffer);

/*************************************************************************
 *函数功能：   		从24Cxx的Block0中读取ucSize个字节或者将ucSize个字节写入
 *			   		到24Cxx的Block0中
 *参数ucStartAddr： 起始地址
 *参数pBuffer：     数据缓存。读取时，存档读出来的数据；写入时，存放需要
 *	     			写入的数据
 *参数ucSize：		读取或写入的数据的长度
 *参数ucDataDirection：读取或写入的操作。0——写入；1——读取。
**************************************************************************/
extern void ReadOrWriteBytes_24cxx(INT8U ucStartAddr,
                                   INT8U *pucBuffer,
							   	   INT8U ucSize,
                                   INT8U ucDataDirection);

/*************************************************************************
 *函数功能：   		向24Cxx中一次性写入Length个字节
 *参数ucStartAddr： 起始地址
 *参数pBuffer：     要写入的数据缓存。
**************************************************************************/
extern void WriteBytesto24cxx(INT8U ucStartAddr,
                              INT8U *pucBuffer,
                              INT8U Length);

#endif
