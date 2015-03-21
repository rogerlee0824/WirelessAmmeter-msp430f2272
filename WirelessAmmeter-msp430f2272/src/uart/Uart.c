#include "uart.h"
#include "msp430x41x2.h"

void UartSendByte(INT8U chr)
{
    UCA0TXBUF = chr;
	while(!(IFG2&UCA0TXIFG));
}

void UartSendMultiBytes(void *pvBuffer,INT16U wSize)
{
    while(wSize--)
    {
        UartSendByte(*((INT8U *)pvBuffer));
        pvBuffer = (INT8U *)pvBuffer + 1;
    }
}

void UartSendATCmd(void *pvBuffer,INT16U wSize)
{
	wSize--;
	while(wSize--)
    {
        UartSendByte(*((INT8U *)pvBuffer));
        pvBuffer = (INT8U *)pvBuffer + 1;
    }
	UartSendByte('\r');
	UartSendByte('\n');
}