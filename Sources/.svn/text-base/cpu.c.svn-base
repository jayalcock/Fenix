/****************************************************************************
* File:  		cpu.c
* Author: 		Jay Alcock
* Date: 		01JUN2012
* Description:          CPU setup and control functions
*
* History:
*
****************************************************************************/

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#define _cpu_c_

#include "cpu.h"

/*=========================================================================*/
/*  Functions                                                              */
/*=========================================================================*/

//==========================================================================
// Title: 		cpuSetupHardware
// Author:		Jay Alcock
// Date:		01JUN12
// Description:         Configures default GPIO(fast + outputs), PLL (48MHz), MAM, 
//                      peripheral power (all off) and disables all interrupts.
// Inputs:		None
// Outputs:             None
//==========================================================================
void cpuSetupHardware(void)
{

    SCS = SCS_GPIO0M | SCS_GPIO1M;	    //Configure GPIO as fast GPIO on both ports
    PINSEL0 = PINSEL0_ALL_GPIO;		    //Configure pin functions.  All pins are set to GPIO, not including the Debug
    PINSEL1 = PINSEL1_ALL_GPIO;		    //port (P1.26) and the Trace port (P1.25..P1.16).

    FIO0DIR = ~GPIO_IO_ALL;                 //Set all GPIO to input (safer than an output, which may be driving a high
    FIO1DIR = ~GPIO_IO_JTAG;                //into a closed switch such as BSL).

    PLL0CFG = (PLL0CFG_MUL4 | PLL0CFG_DIV2);//Setup the PLL to multiply the 12Mhz XTAL input by 4, divide by 2 (48Mhz)
    PLL0FEED = PLL0FEED_FEED1;              //Feed
    PLL0FEED = PLL0FEED_FEED2;

    PLL0CON  = PLL0CON_PLLE;                //Activate the PLL by turning it on then feeding the correct sequence of bytes
    PLL0FEED = PLL0FEED_FEED1;              //Feed
    PLL0FEED = PLL0FEED_FEED2;

    while (!(PLL0STAT & PLL0STAT_PLOCK));   //Wait for the PLL to lock...

    PLL0CON  = PLL0CON_PLLC | PLL0CON_PLLE; //...before connecting it using the feed sequence again
    PLL0FEED = PLL0FEED_FEED1;              //Feed
    PLL0FEED = PLL0FEED_FEED2;

    MAMTIM = MAMTIM_3;			    //Setup and turn on the MAM.  Three cycle access is used due to the fast PLL used.
    MAMCR = MAMCR_FULL;			    //It is possible faster overall performance could be obtained by tuning the MAM and PLL settings.

    APBDIV = APBDIV_50;			    //Setup the peripheral bus to be the half the PLL output (24Mhz)
//    PCONP = PCONP_ALLOFF;		    //Disable power to all modules

//    VICIntEnClr = VICIntEnClr_MASK;       //Make sure all interrupts disabled
//
//    VICProtection = VICProtection_VIC_User_Mode; //Enabled user-mode code to change VIC state
//
//    VICIntSelect = VICIntSelect_All_IRQ;   //Assign all interrupts to IRQ category
  
//    SCB_MEMMAP = SCB_MEMMAP_URM;            //Use user-mode ram mapping
//    SCB_MEMMAP = SCB_MEMMAP_UFL;          //Use user-mode flash mapping
  
    UART0init();
    SetUpInt();
    I2CInit();
    PWMInit();
    FIO0DIR |= GPIO_IO_P13;
}


