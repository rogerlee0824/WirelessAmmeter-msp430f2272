#include "main.h"
#include "msp430x22x2.h"
#include "init.h"
#include "uart.h"
#include "delay.h"
#include "encryption.h"
#include "fbt06.h"
#include "motor.h"
#include "updateLCD.h"
#include "msp430flash.h"
#include "cc112x_spi.h"
#include "hal_spi_rf_trxeb.h"
#include "cc1120_rx_sniff_mode_reg_config.h"

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

#define V_STATUS                    0x01   //阀状态
#define F_STATUS                    0x02   //FLASH状态
#define M_STATUS                    0x04   //防磁状态
#define B_STATUS                    0x08   //电池电量状态

//信息存储区010FFh -- 01000h

// Defined for Uart 
INT8U ucUartRXBuffer[UART_RX_LEN];
//INT8U *pucUartRXBuffer;
INT8U ucUartTXMaxSize       			= 0;
INT8U ucUartRXMaxSize       			= 0;
INT8U ucUartRXSize          			= 0;
INT16U wUartRXCompletedFlag 			= 0;
uint32 u32AmmeterCount				= 0;

volatile INT8U x = 0,                         y = 0;
INT8U ucAllowBit2INT = 1,ucAllowBit3INT = 1;

INT8U ucRFTxBuffer[19];
INT8U ucRFRxBuffer[128];
INT8U Buffer_count[5];							

// Defined for Radios
uint8  RXpacketSemaphore;
uint8  TXpacketSemaphore;
uint8  ucRadioWorkingStatus;

#pragma location=0x1000
no_init INT8U ucAmmeterMACAddr[5];	

#pragma location=0x1080
no_init INT8U ucAmmeterCount[5];							

INT8U ucAmmeterMACAddr_temp[5] = {0x00,0x00,0x00,0x00,0x02};
uint8 u8Buffer_count[5] = {0x00,0x00,0x00,0x12,0x34};		// Save the data,左起第一个字节用作状态字节			
uint8 u8Buffer_count_temp[5] = {0x00,0x00,0x00,0x00,0x00};	// Save the data

unsigned char Counter_flag=0;         
unsigned char Data_Receive_Pointer=0;   
unsigned char Receive_Flag=0;   
unsigned char Receive_Complete_Flag=0;
unsigned char flag_save_count = 0;
unsigned char timerflag = 0;

static INT8U memcmp(INT8U *pucBuf0,INT8U *pucBuf1,INT8U len);
void HEX_to_BCD(unsigned long ulCounterTmp,uint8 *pucArrar);
uint32 BCD_to_HEX(uint8 *pu8Arrar);
uint8 createTxPacket(uint8 *pucSrcBuffer,uint8 *pucDestBuffer);
void CountMeter(void);

uint8 gpioConfigSlave[] = 
{
    //gpio2,re rx=0
    //gpio3,te tx=0
    50,//gpio3
    0x01,//gpio2
    0xB0,
    0x06,
};
uint8 preambleConfig[] = 
{
   0x10,
   0x2A,
};
uint8 worConfig[] = 
{
   0x20,
   0x63,
   0x7B,
   //0x42,
   //0x2f,
};

void main(void)
{
    uint8 i;
	WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    _DINT();               //关闭全局中断
    WDTCTL = WDT_ARST_1000;//启动1S周期的看门狗
	initMCU();
	rf_PowerUpReset();
        trxSpiCmdStrobe(CC112X_SRES);
        cc112xSpiWriteReg(CC112X_IOCFG3, &gpioConfigSlave[0],4);
	registerConfig();
        cc112xSpiWriteReg(CC112X_WOR_CFG0,&worConfig[0],3);
        cc112xSpiWriteReg(CC112X_PREAMBLE_CFG1,&preambleConfig[0],2);  
        calibrateRCOsc();
	RXpacketSemaphore = ISR_IDLE;
        TXpacketSemaphore = ISR_IDLE;
        ucRadioWorkingStatus = 0;
       
    
	P1IES  |= BIT0;	// The P1IFGx flag is set with a high-to-low transition
        P1IE   |= BIT0;	// Enable P1.4,5,6 interrupt
        P1IES  &= ~BIT1;// The P1IFGx flag is set with a low/high transition
        P1IE   |= BIT1;	// Enable P1.4,5,6 interrupt
        P1IES  |= BIT2;
        //P1IE   |= BIT2;
        P1IFG  &= 0x00;	// The P1IFGx flag is cleared  

	//MSP430SeqByteReadFlash(&ucAmmeterCount[0],u8Buffer_count,5);
        u32AmmeterCount = BCD_to_HEX(u8Buffer_count);
       
	trxSpiCmdStrobe(CC112X_SWOR); 
        //trxSpiCmdStrobe(CC112X_SPWD);
        TACTL = TASSEL0 + TACLR; 
        CCR0=3700; //定时1ms
        TACTL|=MC0; //增计数模式
        //CCTL0 = CCIE; // CCR0 中断使能
        WDTCTL = WDT_ARST_1000;//喂狗
        LED2OFF();
        RE();
        //IDLE();
	while(1)
	{ 
        UpdataLCD(u8Buffer_count);	
       
		 _EINT();
		WDTCTL = WDTPW + WDTHOLD;//进入低功耗之前关闭看门狗           
	    __bis_SR_register(LPM3_bits + GIE);	
		_NOP();_NOP();
		WDTCTL = WDT_ARST_1000;//喂狗 
		if(RXpacketSemaphore == ISR_ACTION_REQUIRED) 
	    {
			WDTCTL = WDT_ARST_1000;
        	RXpacketSemaphore = ISR_IDLE;
			LED1ON();
			i = ReadDataFromRadio(ucRFRxBuffer);
			LED1OFF();

			if(i>0 && createTxPacket(ucRFRxBuffer,ucRFTxBuffer))
			{
				trxSpiCmdStrobe(CC112X_SPWD);                   
				for(i =(2 - ucRFTxBuffer[2]); i>0; i--)
				{
					Delay1mS(400);
					WDTCTL = WDT_ARST_1000;
				}
				LED2ON();
				WDTCTL = WDT_ARST_250;
				TXpacketSemaphore = ISR_IDLE;
				trxSpiCmdStrobe(CC112X_SIDLE);
				TE();
				SendPacketByRF(ucRFTxBuffer, RADIO_PACKET_SIZE + 1);
				P1IFG = 0;                                
				LED2OFF();
				 //RE();
			}
			trxSpiCmdStrobe(CC112X_SWORRST);
			trxSpiCmdStrobe(CC112X_SWOR);
			//IDLE();
			RE();
				   
			WDTCTL = WDT_ARST_1000;
			continue;
	   }
	}//while结束
}

uint8 createTxPacket(uint8 *pucSrcBuffer,uint8 *pucDestBuffer)
{
	INT8U i;	
	if((pucSrcBuffer[0] == RADIO_PACKET_SIZE) && (pucSrcBuffer[1] == PACKET_HEADER) && (pucSrcBuffer[18] == PACKET_END))
	{
		pucDestBuffer[0] = RADIO_PACKET_SIZE;
		pucDestBuffer[1] = PACKET_ACK_HEADER;
		pucDestBuffer[2] = pucSrcBuffer[2];
                //先判定是否为出厂写表头及数据
		if(pucSrcBuffer[3] == CheckMeterCMD)
		{
		return 1;
	}
	else
	{
		return 0;
	}	
}

static INT8U memcmp(INT8U *pucBuf0,INT8U *pucBuf1,INT8U len)
{
	INT8U i;
	
	for(i = 0;i < len;i ++)
	{
		if(pucBuf0[i] != pucBuf1[i])
		{
			return 0;
		}
	}
	return 1;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
{
    INT8U ucIFG2;
    unsigned char temp=0;
    _EINT();//允许中断重入，防止计数丢失
	
    ucIFG2  = IFG2;    
    if(ucIFG2 & UCA0RXIFG)
    {
      	temp=UCA0RXBUF;
      	if(temp==0xAA&&Receive_Flag==0)
      	{      
        	ucUartRXBuffer[0]=temp;
        	Data_Receive_Pointer=1;
        	Receive_Flag =1;
      	}
      	else  if(Receive_Flag==1)
      	{
        	ucUartRXBuffer[Data_Receive_Pointer]=UCA0RXBUF;
        	temp=ucUartRXBuffer[Data_Receive_Pointer];
        	Data_Receive_Pointer++;
        	if(Data_Receive_Pointer>=18)
        	{  
          		Receive_Flag =0;
          		Data_Receive_Pointer=0;    
          		if(temp==0xBB)//有效数据，进入主循环
	  			{
					Receive_Complete_Flag =1;
					LPM3_EXIT;
	  			}
        	}
      	}
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void radioRxISR(void) 
{
        uint8 ucP1IFG;	
	ucP1IFG = P1IFG;
	if(ucP1IFG & BIT0)							// Interrupt from P1.0
	{
		P1IFG  &= ~BIT0;      // Clear isr flag
  		//TXpacketSemaphore = ISR_ACTION_REQUIRED;
                //RE();
                //LED1ON();
	}
	else if(ucP1IFG & BIT1)						// Interrupt from P1.1
	{
		P1IFG  &= ~BIT1; // Clear isr flag
                RE();
  		RXpacketSemaphore = ISR_ACTION_REQUIRED;		
		LPM3_EXIT;
	}
	else if(ucP1IFG & BIT2)
        {
                P1IFG  &= ~BIT2; // Clear isr flag
                if(P1IES & BIT2)//下降沿，无线芯片工作
                {
                  LED1ON();
                  RE();
                }
                else
                {
                  LED1OFF();
                  IDLE();
                }
                P1IES ^= BIT2;
                P1IFG &= ~BIT2;
                P1IE  |= BIT2;
                
                //CCTL0 = CCIE; // CCR0 中断使能
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
    ucP2IFG &= P2IE;
    
    if(ucP2IFG & BIT4)			// Interrupt from P2.4
    {   	
		P2IFG  &= ~BIT4;	                // Clear the interrupt flag
		P2IE   &= ~BIT4;                    //关闭2.4中断
		P2IE   |= BIT3;                     //开启2.3中断
		x = 1;
    }
    if(ucP2IFG & BIT3)			// Interrupt from P2.3
    {   		
		P2IE    &= ~BIT3;
		P2IE    |= BIT4;
		P2IFG   &= ~BIT3;			// Clear the interrupt flags	
		y = 1;
    }
    if(x == 1 && y == 1)
    {
		flag_save_count = 1;
		LPM3_EXIT;
    }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA(void)//定时器中断
{
   CCTL0 &= ~CCIE;
   IDLE();
   LED1OFF();
}
