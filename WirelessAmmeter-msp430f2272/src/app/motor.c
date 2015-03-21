#include "motor.h"
#include "msp430x22x2.h"
#include "delay.h"

void TurnOFF(void)
{
    int i;
    P2DIR |= BIT6; 
    P2OUT  |= BIT6;
    Delay1mS(10);
    P2OUT  |= BIT5;
    for(i=0;i<150;i++)
    {
        WDTCTL = WDT_ARST_16;   
        Delay1mS(10);
        if(!(P2IN & BIT7))break;
    }
    P2OUT &= ~(BIT5|BIT6);
    //P2DIR &= ~(BIT5|BIT6);
}

void TurnON(void)
{
    int i;
    P2DIR |= BIT6;
    P2OUT |= BIT6; 
    Delay1mS(10);
    P2OUT  &= ~BIT5;
    for(i=0;i<150;i++)
    {
        if(P2IN & BIT7)break;
        WDTCTL = WDT_ARST_16;   
        Delay1mS(10);
    }
    P2OUT &= ~(BIT5|BIT6);
    //P2DIR &= ~(BIT5|BIT6);
}

INT8U OpenValve(void)
{
	if(!(P2IN & BIT7))
	{
		TurnON();
		if((P2IN & BIT7))			// Open valve successfully
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	return 0;	
}

INT8U CloseValve(void)
{
	if((P2IN & BIT7))
	{
		TurnOFF();
		if(!(P2IN & BIT7))			// Close valve successfully
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	return 0;		
}