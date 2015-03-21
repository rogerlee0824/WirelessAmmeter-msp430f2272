#ifndef _MAIN_H_
#define _MAIN_H_

#include "hw_types.h"

#define UART_RX_LEN      30

extern INT8U ucUartRXBuffer[];
extern INT8U *pucUartRXBuffer;
extern INT8U ucUartRXMaxSize;
extern INT8U ucUartRXSize;
extern INT16U wUartRXCompletedFlag;

#endif
