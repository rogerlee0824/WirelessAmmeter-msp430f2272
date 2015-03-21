#include "delay.h"

// 10ms for 12Mhz
void Delay10mS(INT16U cnt) 
{
	while(cnt--) 
	{
		Delay1mS(10); 
	}
}

// 1ms for 12Mhz
void Delay1mS(INT16U cnt) 
{
	INT16U i;
	while(cnt--) 
	{
		for(i = 0;i < 705;i ++)
        { 
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
        }
	}
}

// 10us for 16Mhz
void Delay10us(INT16U cnt) 
{
	INT16U i;
	while(cnt--) 
	{
		for(i = 0;i < 20;i ++)
        { 
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
			//asm("nop");
        }
	}
}
// 1us for 16Mhz
void Delay1us(INT16U cnt)
{
	INT16U i;
	
	while(cnt--) 
	{
		for(i = 0;i < 2;i ++)
        { 
			asm("nop");
        	asm("nop");
			asm("nop");
        	asm("nop");
			//asm("nop");
        	//asm("nop");
			//asm("nop");
        	//asm("nop");
			//asm("nop");
        }
	}
}
