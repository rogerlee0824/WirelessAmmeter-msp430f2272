#include "main.h"
#include "msp430.h"
#include "init.h"
#include "uart.h"
#include "delay.h"
#include "fbt06.h"
#include "cc112x_spi.h"
#include "hal_spi_rf_trxeb.h"
#include "cc1120_rx_sniff_mode_reg_config.h"

#define RADIO_PACKET_SIZE               18	// 1 < PKTLEN < 126
#define PACKET_HEADER           	0xAA
#define PACKET_END           		0xBB
#define ReadMeterCMD			0x11
#define ReadMeterACKCMD			0x21
#define CheckMeterCMD			0xee
#define CheckMeterACKCMD		0x2e
#define OpenValveCMD			0x13
#define OpenValveACKCMD			0x23
#define CloseValveCMD			0x14
#define CloseValveACKCMD		0x24

// Defined for Uart 
INT8U ucUartRXBuffer[UART_RX_LEN]={0};
INT8U ucUartTXMaxSize       	= 0;
INT8U ucUartRXMaxSize       	= 18;
INT8U ucUartRXSize          	= 0;
uint16 u16UartRXCompletedFlag 	= 0;
uint16 u16UartRXStartFlag	= 0;
INT8U  rfxrecv                  = 0;

INT8U flag_Port2Bit2Int		= 0;
INT8U flag_Port2Bit3Int		= 0;
volatile INT8U x = 0,y = 0,count = 0;
INT8U ucAllowBit2INT = 1,ucAllowBit3INT = 1;

// Defined for RF
uint8 ucRFTxBuffer[RADIO_PACKET_SIZE + 1];
uint8 ucRFRxBuffer[RADIO_PACKET_SIZE + 1];
uint8 RXpacketSemaphore;
uint8 TXpacketSemaphore;

void main(void)
{
   uint8 i;
   WDTCTL = WDTPW + WDTHOLD;                 	// Stop watchdog timer
   _DINT();
   WDTCTL = WDT_ARST_1000;//启动1S周期的看门狗
   
   // Inits MSP430F2272.
   initMCU();	
   
   // Reset the cc1120 module.
   //rf_PowerUpReset();
   
   // Write register settings
   //registerConfig();	
   
   LED2OFF();
   
   P1IES  &= ~BIT1;
   P1IE   |= BIT1;   
   P1IES  |= BIT0; // The P1IFGx flag is set with a  high-to-low transition
   P1IE   |= BIT0;
   P1IFG  &= 0x00;
   
   //CACTL1&=~CAON;//关闭比较器A
   
   RXpacketSemaphore = ISR_IDLE;
   TXpacketSemaphore = ISR_IDLE; 
   trxSpiCmdStrobe(CC112X_SRX);
   
   //将蓝牙模块置为低功耗模式
   //UartSendMultiBytes(&(ucUartRXBuffer[0]),15); 
    // Infinite loop
    
    ucUartRXMaxSize  = 18;
    while(1) 
    {	
		_EINT();
		WDTCTL=WDTPW+WDTHOLD;//进入低功耗之前关闭看门狗
		__bis_SR_register(LPM3_bits + GIE);
		_NOP();_NOP();
		WDTCTL = WDT_ARST_1000;//喂狗  
		
		if(u16UartRXCompletedFlag)
		{
			P2IFG  &= 0x00;
			u16UartRXCompletedFlag	= 0;
			WDTCTL = WDT_ARST_1000;//喂狗  
			LED2ON();
                         
			ucRFTxBuffer[0] = RADIO_PACKET_SIZE;
			for(i = 0;i < 18;i ++)
			{
				ucRFTxBuffer[i+1] = ucUartRXBuffer[i];
			}
			WDTCTL = WDT_ARST_1000;//喂狗	
                //无线发送时，应将接收中断关闭，发送完成后，再打开
			P1IE   &= ~BIT5; //关闭P1.5中断                
			SendPacketByRF(ucRFTxBuffer, RADIO_PACKET_SIZE + 1);
			P1IE   |= BIT5;
			WDTCTL = WDT_ARST_1000;//喂狗 
			LED2OFF();
	    }
		//无线接收改为中断
		if(RXpacketSemaphore == ISR_ACTION_REQUIRED)
		{
			//rfxrecv = 0;
			RXpacketSemaphore = ISR_IDLE;
			WDTCTL = WDT_ARST_1000;//喂狗 
			if(runRX(ucRFRxBuffer))
			{
				UartSendMultiBytes(&(ucRFRxBuffer[1]),18);
			}
			trxSpiCmdStrobe(CC112X_SRX);
			WDTCTL = WDT_ARST_1000;//喂狗 
		}
    }
}

/*******************************************************************************
*   @fn         USCIA0RX_ISR
*   @brief      ISR for packet handling in RX. Sets packet semaphore
*               and clears interrupt flag
*   @param      none
*   @return     none
*******************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
{
	uint8 ucIFG2,ucTemp;
	
	ucIFG2  = IFG2;
	if(ucIFG2 & UCA0RXIFG)
	{
		ucTemp	= UCA0RXBUF;
		if((ucTemp == PACKET_HEADER) &&(u16UartRXStartFlag == 0))
		{
			ucUartRXBuffer[0] = ucTemp;
			ucUartRXSize	++;
			u16UartRXStartFlag = 1;
		}
		else if(u16UartRXStartFlag == 1)
		{
			ucUartRXBuffer[ucUartRXSize] = ucTemp;   
			ucUartRXSize	++;
			if(ucUartRXSize >= ucUartRXMaxSize)
			{
				ucUartRXSize = 0;
				u16UartRXStartFlag = 0;
				if(ucTemp==0xBB)//有效数据，进入主循环
				{
			  		u16UartRXCompletedFlag = 1;
					LPM3_EXIT;
				}
			}
		}
	}
}
/*******************************************************************************
*   @fn         radioRxISR
*   @brief      ISR for packet handling in RX. Sets packet semaphore
*               and clears interrupt flag
*   @param      none
*   @return     none
*******************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void radioRxISR(void) 
{
	uint8 ucP1IFG;	
	ucP1IFG = P1IFG;
	if(ucP1IFG & BIT0)							// Interrupt from P1.0
	{
		P1IFG  &= ~BIT0;						// Clear isr flag
		// Set packet semaphore
  		TXpacketSemaphore = ISR_ACTION_REQUIRED;
	}
	else if(ucP1IFG & BIT1)						// Interrupt from P1.1
	{
		P1IFG  &= ~BIT1;						// Clear isr flag
		// Set packet semaphore
  		RXpacketSemaphore = ISR_ACTION_REQUIRED;
		LPM3_EXIT;
	}
	else
	{}
}

/*******************************************************************************
*   @fn         ISR_PORT2
*   @brief      Calculate.
*   @param      none
*   @return     none
************************************************ *******************************/
#pragma vector=PORT2_VECTOR
__interrupt void ISR_PORT2(void)//计数中断
{
    INT8U ucP2IFG;	
    
    ucP2IFG = P2IFG;
    if(ucP2IFG & BIT2)			            // Interrupt from P2.2
    {   	
		P2IFG  &= ~BIT2;	                // Clear the interrupt flag
    }
}