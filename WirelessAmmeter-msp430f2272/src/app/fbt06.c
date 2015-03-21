#include "fbt06.h"
#include "uart.h"
#include "main.h"
#include "msp430x22x2.h"
#include <string.h>

INT8U Fbt06TestCmd[] 						= "AT";
INT8U Fbt06SetBaudCmd[] 					= "AT+BAUD";
INT8U Fbt06SetPasswordCmd[] 				= "AT+PIN1234";
INT8U Fbt06SetDefaultCmd[] 					= "AT+DEFAULT";
INT8U Fbt06InquiryRoleCmd[] 				= "AT+ROLE";
INT8U Fbt06SetMasterCmd[] 					= "AT+ROLE1";
INT8U Fbt06SetSlaveCmd[] 					= "AT+ROLE0";
INT8U Fbt06BindCmd[] 						= "AT+BIND";
INT8U Fbt06ClearCmd[] 						= "AT+CLEAR";
INT8U Fbt06AutoConnectDeviceCmd[] 			= "AT+AUTOCONN";
INT8U Fbt06InquiryDeviceStatusCmd[] 		= "AT+STATE";

INT8S ucFBT06Status;

INT8S FBT06BoardTest(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[] 				= "OK";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= 4;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06TestCmd,sizeof(Fbt06TestCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,2))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardSetDefault(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[] 				= "OK";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= 4;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06SetDefaultCmd,sizeof(Fbt06SetDefaultCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,2))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardInquiryRole(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[]				= "+ROLE=";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= 9;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06InquiryRoleCmd,sizeof(Fbt06InquiryRoleCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,6))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardChangeRole(INT8U role)
{
	INT16U	i;
	INT8U   ucReturnOKBuffer[]				= "+ROLE=";

	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= 6;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	if(!role)
	{
		UartSendATCmd(Fbt06SetSlaveCmd,sizeof(Fbt06SetSlaveCmd));
	}
	else
	{
		UartSendATCmd(Fbt06SetMasterCmd,sizeof(Fbt06SetMasterCmd));
	}
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnOKBuffer,6))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardBind(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[]				= "+BIND=";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= 23;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06BindCmd,sizeof(Fbt06BindCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,6))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardClear(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[]				= "OK";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= sizeof(ucReturnBuffer) + 1;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06ClearCmd,sizeof(Fbt06ClearCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,2))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardSearchDevice(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[]				= "OK";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= sizeof(ucReturnBuffer) + 1;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06ClearCmd,sizeof(Fbt06ClearCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,2))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardAutoConnectDevice(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[]				= "+AUTOCONN=";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= 13;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06AutoConnectDeviceCmd,sizeof(Fbt06AutoConnectDeviceCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,sizeof(Fbt06AutoConnectDeviceCmd)-1))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

INT8S FBT06BoardInquiryDeviceStatus(void)
{
	INT16U	i;
	INT8U   ucReturnBuffer[]				= "+STATE=";
	
	pucUartRXBuffer 						= ucUartRXBuffer;
	ucUartRXSize							= 0;
	ucUartRXMaxSize							= 8;
	for(i = 0;i< UART_RX_LEN;i++)
	{
		ucUartRXBuffer[i] 					= 0;
	}
	UartSendATCmd(Fbt06InquiryDeviceStatusCmd,sizeof(Fbt06InquiryDeviceStatusCmd));
	while(!wUartRXCompletedFlag);
	wUartRXCompletedFlag					= 0;
	if(!memcmp(ucUartRXBuffer,ucReturnBuffer,sizeof(Fbt06InquiryDeviceStatusCmd)-1))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void FBT06SendData(void *pvBuffer,INT16U len)
{
	if((P3IN & BIT3))
	{
		UartSendMultiBytes(pvBuffer,len);
	}
}

void FBT06Init(void)
{
	ucFBT06Status 							= FBT06BoardInquiryDeviceStatus();
}