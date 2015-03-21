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
 *�������ܣ�   		��24Cxx��һ����д��16���ֽ�
 *����ucStartAddr�� ��ʼ��ַ
 *����pBuffer��     Ҫд������ݻ��档
**************************************************************************/
extern void PageWriteto24cxx(INT8U ucStartAddr,INT8U *pBuffer);

/*************************************************************************
 *�������ܣ�   		��24Cxx��Block0�ж�ȡucSize���ֽڻ��߽�ucSize���ֽ�д��
 *			   		��24Cxx��Block0��
 *����ucStartAddr�� ��ʼ��ַ
 *����pBuffer��     ���ݻ��档��ȡʱ���浵�����������ݣ�д��ʱ�������Ҫ
 *	     			д�������
 *����ucSize��		��ȡ��д������ݵĳ���
 *����ucDataDirection����ȡ��д��Ĳ�����0����д�룻1������ȡ��
**************************************************************************/
extern void ReadOrWriteBytes_24cxx(INT8U ucStartAddr,
                                   INT8U *pucBuffer,
							   	   INT8U ucSize,
                                   INT8U ucDataDirection);

/*************************************************************************
 *�������ܣ�   		��24Cxx��һ����д��Length���ֽ�
 *����ucStartAddr�� ��ʼ��ַ
 *����pBuffer��     Ҫд������ݻ��档
**************************************************************************/
extern void WriteBytesto24cxx(INT8U ucStartAddr,
                              INT8U *pucBuffer,
                              INT8U Length);

#endif
