#include "updateLCD.h"
#include "uart.h"

#define RADIO_PACKET_SIZE           18
#define PACKET_HEADER               0xAA
#define PACKET_ACK_HEADER           0xA6
#define PACKET_END           	    0xBB
#define ReadMeterCMD		    	0x11   //抄表
#define ReadMeterACKCMD		    	0x21
#define CheckMeterCMD		    	0xee   //写表头及数据
#define CheckMeterACKCMD	    	0x2e
#define OpenValveCMD		    	0x13
#define OpenValveACKCMD		    	0x23
#define CloseValveCMD		    	0x14
#define CloseValveACKCMD	    	0x24
#define CalibrateDataCMD            0x15   //校准数据
#define CalibrateDataACKCMD         0x25

void UpdataLCD(INT8U *pucSrcBuffer)
{
	INT8U ucTxBuffer[12] = {0},ucTemp[6] = {0};
	INT8U i = 0;
    
	ucTxBuffer[0] = PACKET_HEADER;
	ucTxBuffer[1] = PACKET_HEADER;
	ucTxBuffer[2] = 0x68;
	ucTxBuffer[3] = 6;
    
	// Prepares the data to be displayed.
	for(i = 0;i < 5;i ++)
	{
		ucTemp[i] = (i % 2) ? (pucSrcBuffer[4-(i/2)] >> 4) : pucSrcBuffer[4-(i/2)] & 0x0F;
	}
	
	// Loads the data to be displayed.
	for(i = 4;i < 10;i ++)
	{
		ucTxBuffer[i] = ucTemp[i - 4];
	}
	ucTxBuffer[11] = PACKET_END;
	
	// Sends the data to LCD by uart in 2400bps.
	UartSendMultiBytes(ucTxBuffer,12);
}