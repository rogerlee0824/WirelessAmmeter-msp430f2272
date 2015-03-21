//*****************************************************************************
//! @file       cc112x_easy_link_tx.c
//! @brief      This program sets up an easy link between two trxEB's with
//!             CC112x EM's connected. 
//!             The program can take any recomended register settings exported
//!             from SmartRF Studio 7 without any modification with exeption 
//!             from the assumtions decribed below.
//
//              Notes: The following asumptions must be fulfilled for 
//              the program to work:
//              
//              1. GPIO2 has to be set up with GPIO2_CFG = 0x06
//                 PKT_SYNC_RXTX for correct interupt
//              2. Packet engine has to be set up with status bytes enabled 
//                 PKT_CFG1.APPEND_STATUS = 1
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "msp430x41x2.h"
#include "hal_spi_rf_trxeb.h"
#include "cc112x_spi.h"
#include "stdlib.h"
#include "cc112x_easy_link_reg_config.h"
#include "init.h"
#include "delay.h"

/******************************************************************************
* DEFINES
*******************************************************************************/
#define ISR_ACTION_REQUIRED 1
#define ISR_IDLE            0

#define PKTLEN              32 								// 1 < PKTLEN < 126

#define GPIO3               0x04
#define GPIO2               0x08
#define GPIO0               0x80

/******************************************************************************
* LOCAL VARIABLES
*******************************************************************************/
volatile uint8  packetSemaphore;
uint32 packetCounter = 0;

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
void registerConfig(void);
static void manualCalibration(void);
void runTX(void);
void createPacket(uint8 randBuffer[]);

/******************************************************************************
 * @fn          runTX
 * @brief       Continuously sends packets on button push until button is pushed
 *              again. After the radio has gone into TX the function waits for 
 *              interrupt that packet has been sent. Updates packet counter and
 *              display for each packet sent.
 * @param       none
 * @return      none
*******************************************************************************/
void runTX(void) 
{
  	// Initialize packet buffer of size PKTLEN + 1
  	uint8 txBuffer[PKTLEN] = {0};

	P1IES  |= BIT4 | BIT5 | BIT6;						// The P1IFGx flag is set with a 
											  			// high-to-low transition
    P1IFG  &= 0x00;							  			// The P1IFGx flag is cleared
    P1IE   |= BIT4 | BIT5 | BIT6;						// Enable P1.4,5,6 interrupt

  	// Infinite loop
  	while(TRUE) 
	{
		LED2ON();
        // Update packet counter
        packetCounter++;
        
        // Create a random packet with PKTLEN + 2 byte packet 
        // counter + n x random bytes
        createPacket(txBuffer);

        // Write packet to TX FIFO
        cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));

        // Strobe TX to send packet
        trxSpiCmdStrobe(CC112X_STX);

        // Wait for interrupt that packet has been sent. 
        // (Assumes the GPIO connected to the radioRxTxISR function is set 
        // to GPIOx_CFG = 0x06)
        while(packetSemaphore != ISR_ACTION_REQUIRED);

        // Clear semaphore flag
        packetSemaphore = ISR_IDLE;
		Delay1mS(100);
		LED2OFF();
		Delay1mS(5000);
		Delay1mS(5000);
		
    }
}

void runTX2(void) 
{
  	// Initialize packet buffer of size PKTLEN + 1
  	uint8 txBuffer[PKTLEN] = {0};

  	// Infinite loop
  	while(TRUE) 
	{
		LED2ON();
        // Update packet counter
        packetCounter++;
        
        // Create a random packet with PKTLEN + 2 byte packet 
        // counter + n x random bytes
        createPacket(txBuffer);
		
		SendPacketByRF(txBuffer,sizeof(txBuffer));

		Delay1mS(100);
		LED2OFF();
		Delay1mS(5000);
		Delay1mS(5000);
    }
}

void SendPacketByRF(uint8 *u8pData,uint8 u8Len) 
{
	P1IES  |= BIT4 | BIT5 | BIT6;						// The P1IFGx flag is set with a 
											  			// high-to-low transition
    P1IFG  &= 0x00;							  			// The P1IFGx flag is cleared
    P1IE   |= BIT4 | BIT5 | BIT6;						// Enable P1.4,5,6 interrupt

	// Write packet to TX FIFO
	cc112xSpiWriteTxFifo(u8pData, u8Len);

	// Strobe TX to send packet
	trxSpiCmdStrobe(CC112X_STX);

	// Wait for interrupt that packet has been sent. 
	// (Assumes the GPIO connected to the radioRxTxISR function is set 
	// to GPIOx_CFG = 0x06)
	while(packetSemaphore != ISR_ACTION_REQUIRED);

	// Clear semaphore flag
	packetSemaphore = ISR_IDLE;
}

void rf_PowerUpReset(void)
{
    TRXEM_SPI_BEGIN(); 								// Pulse CSn low then high 
    Delay10us(2); 									// Delay 20us 
    TRXEM_SPI_END(); 								// Pulse CSn high 
    Delay10us(6); 									// Delay 60us 
    
    // pull CSn low and wait for SO to go low 
    TRXEM_SPI_BEGIN();
	while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
    cc112xSpiWriteReg(0x00, 0x00,1); 							// SPI写字节，不是写命令子程序 
    
    // wait for SO to go low again, reset is complete at that point 
    while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
    TRXEM_SPI_END(); 								// Pulse CSn high 
}

/*******************************************************************************
* @fn          radioTxISR
* @brief       ISR for packet handling in RX. Sets packet semaphore 
*              and clears isr flag.
* @param       none
* @return      none
*******************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void radioTxISR(void) 
{
	INT8U ucP1IFG;
	
	ucP1IFG = P1IFG;
	if(ucP1IFG & BIT4)                                	// Interrupt from P1.4
	{   		
		P1IFG  &= ~BIT4;								// Clear isr flag
		
		// Set packet semaphore
  		packetSemaphore = ISR_ACTION_REQUIRED;  
	}
}

/*******************************************************************************
* @fn          registerConfig
* @brief       Write register settings as given by SmartRF Studio found in
*              cc112x_easy_link_reg_config.h
* @param       none
* @return      none
*******************************************************************************/
void registerConfig(void) 
{
  	uint8 writeByte;

  	// Reset radio
  	trxSpiCmdStrobe(CC112X_SRES);

  	// Write registers to radio
  	for(uint16 i = 0; i < (sizeof  preferredSettings/sizeof(registerSetting_t)); i++) 
  	{
    	writeByte 			= preferredSettings[i].data;
    	cc112xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
  	}
	
	// Calibrate radio according to errata
  	manualCalibration();
}

/******************************************************************************
 * @fn          createPacket
 * @brief       This function is called before a packet is transmitted. It fills
 *              the txBuffer with a packet consisting of a length byte, two
 *              bytes packet counter and n random bytes.
 *              The packet format is as follows:
 *              |--------------------------------------------------------------|
 *              |           |           |           |         |       |        |
 *              | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
 *              |           |           |           |         |       |        |
 *              |--------------------------------------------------------------|
 *               txBuffer[0] txBuffer[1] txBuffer[2]  ......... txBuffer[PKTLEN]
 * @param       Pointer to start of txBuffer
 * @return      none
 ******************************************************************************/
void createPacket(uint8 txBuffer[]) 
{
  	txBuffer[0] = 0x55;        					// Address
	txBuffer[1] = PKTLEN;                        		// Length byte
  	txBuffer[2] = (uint8)packetCounter;        		// LSB of packetCounter
	txBuffer[3] = (uint8)(packetCounter >> 8); 		// MSB of packetCounter
  	
  	// Fill rest of buffer with random bytes
  	for(uint8 i = 4; i < (PKTLEN); i++) 
  	{
    	txBuffer[i] = i;
  	}
}

/******************************************************************************
 * @fn          manualCalibration
 * @brief       calibrates radio according to CC112x errata                
 * @param       none
 * @return      none
*******************************************************************************/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void manualCalibration(void) 
{
  uint8 original_fs_cal2;
  uint8 calResults_for_vcdac_start_high[3];
  uint8 calResults_for_vcdac_start_mid[3];
  uint8 marcstate;
  uint8 writeByte;
    
  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    
  // 2) Start with high VCDAC (original VCDAC_START + 2):
  cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
  writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);
    
  // 3) Calibrate and wait for calibration to be done (radio back in IDLE state)
  trxSpiCmdStrobe(CC112X_SCAL);
    
  do {
    cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
  } while (marcstate != 0x41);
    
  // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);
    
  // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    
  // 6) Continue with mid VCDAC (original VCDAC_START):
  writeByte = original_fs_cal2;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);
    
  // 7) Calibrate and wait for calibration to be done (radio back in IDLE state)
  trxSpiCmdStrobe(CC112X_SCAL);
    
  do {
    cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
  } while (marcstate != 0x41);
    
  // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);
    
  // 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result
  if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
    writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
  } else {
    writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
  }
}