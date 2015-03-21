#include "msp430flash.h"
#include "msp430x22x2.h"

/*************************************************************************
 * @brief  Erase flash in segment.
 * @param  pSegAddr:The first addr of segments to erase.
 * @return void
 *************************************************************************/
void MSP430EraseFlashSeg(INT16U *pSegAddr)
{
    _DINT();
    
    FCTL1     = FWKEY + ERASE;          // Erase individual segment only
    FCTL3     = FWKEY;
    
    *pSegAddr = 0xff;                   // Dummy write
    while(FCTL3 & BUSY);                // Waiting for BUSY reset
    
    FCTL3     = FWKEY + LOCK + WAIT;    // Lock
    
    _EINT(); 
}

/*************************************************************************
 * @brief  Sequentially Write to flash in byte.
 * @param  INT8U *pFlashStartAddr
           INT8U *pRAMStartAddr
           INT16U num
 * @return void
 *************************************************************************/
void MSP430SeqByteWriteFlash(INT8U *pFlashStartAddr,
                             INT8U *pRAMStartAddr,
                             INT16U num)
{
    INT16U i;
    _DINT();
    WDTCTL=WDTPW+WDTHOLD;//关闭看门狗 
    while((FCTL3&BUSY)==BUSY); //FLASH是否忙
    FCTL3 = FWKEY;       // 清除Lock位
    FCTL1 = FWKEY + ERASE;// 允许擦除
  
    *pFlashStartAddr = 0; //空写擦除本段信息存储器
    FCTL1 = FWKEY + WRT; // 允许编程写入
    FCTL1     = FWKEY + WRT;            // Byte/wordwrite
    FCTL3     = FWKEY;                  // Unlock
    
    for(i = 0;i < num;i ++)
    {
        *(pFlashStartAddr + i) = *(pRAMStartAddr + i);
	    while(FCTL3 & BUSY);            // Waiting for busy clear
    }
    
    FCTL1     = FWKEY;
    FCTL3     = FWKEY + LOCK + WAIT;    // Lock
    
    _EINT();
    WDTCTL = WDT_ARST_1000;//喂狗 
}

/*************************************************************************
 * @brief  Sequentially read flash in byte.
 * @param  INT8U *pFlashStartAddr
           INT8U *pRAMStartAddr
           INT16U num
 * @return void
 *************************************************************************/
void MSP430SeqByteReadFlash(INT8U *pFlashStartAddr,
                            INT8U *pRAMStartAddr,
                            INT16U num)
{
    INT16U i;
    
    _DINT(); 
    WDTCTL=WDTPW+WDTHOLD;//关闭看门狗
    FCTL1     = FWKEY;
    FCTL3     = FWKEY + LOCK + WAIT;    // Lock
    for(i = 0;i < num;i ++)
    {
       *(pRAMStartAddr + i) = *(pFlashStartAddr + i);
    }
    
    _EINT();
    WDTCTL = WDT_ARST_1000;//喂狗 
}

/*************************************************************************
 * @brief  Sequentially Write to flash in half-word.
 * @param  INT16U *pFlashStartAddr
           INT16U *pRAMStartAddr
           NT16U num
 * @return void
 *************************************************************************/
void MSP430SeqHalfWordWriteFlash(INT16U *pFlashStartAddr,
                                 INT16U *pRAMStartAddr,
                                 INT16U num)
{
    INT16U i;
    _DINT(); 
    
    FCTL1     = FWKEY + WRT;            // Byte/wordwrite
    FCTL3     = FWKEY;                  // Unlock
    
    for(i = 0;i < num;i ++)
    {
        *(pFlashStartAddr + i) = *(pRAMStartAddr + i);
	    while(FCTL3 & BUSY);            // Waiting for busy clear
    }
    
    FCTL1     = FWKEY;
    FCTL3     = FWKEY + LOCK + WAIT;    // Lock
    
    _EINT();
}

/*************************************************************************
 * @brief  Sequentially read flash in half-word.
 * @param  INT16U *pFlashStartAddr
           INT16U *pRAMStartAddr
           INT16U num
 * @return void
 *************************************************************************/
void MSP430SeqHalfWordReadFlash(INT16U *pFlashStartAddr,
                                INT16U *pRAMStartAddr,
                                INT16U num)
{
    INT16U i;
    
    _DINT(); 
    
    FCTL1     = FWKEY;
    FCTL3     = FWKEY + LOCK + WAIT;    // Lock
    for(i = 0;i < num;i ++)
    {
       *(pRAMStartAddr + i) = *(pFlashStartAddr + i);
    }
    
    _EINT();
}