#ifndef _UART_H_
#define _UART_H_
#include "hw_types.h"

void UartSendByte(INT8U chr);
void UartSendMultiBytes(void *pvBuffer,INT16U wSize);
void UartSendATCmd(void *pvBuffer,INT16U wSize);

#endif   // END OF _UART_H_