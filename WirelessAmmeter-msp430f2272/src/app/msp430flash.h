#ifndef _MSP430FLASH_H_
#define _MSP430FLASH_H_

#include "hw_types.h"

void MSP430EraseFlashSeg(INT16U *pSegAddr);

void MSP430SeqByteWriteFlash(INT8U *pFlashStartAddr,INT8U *pRAMStartAddr,INT16U num);

void MSP430SeqByteReadFlash(INT8U *pFlashStartAddr,INT8U *pRAMStartAddr,INT16U num);

void MSP430SeqHalfWordWriteFlash(INT16U *pFlashStartAddr,INT16U *pRAMStartAddr,INT16U num);

void MSP430SeqHalfWordReadFlash(INT16U *pFlashStartAddr,INT16U *pRAMStartAddr,INT16U num);

#endif // End of _MSP430FLASH_H_