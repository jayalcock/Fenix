/****************************************************************************
* File:  		irq.c
* Author: 		Jay Alcock
* Date: 		03SEP12
* Description:          IRQ Handlers
*
* History:
*
****************************************************************************/

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#define _IRQ_C_

#include "irq.h"


/*=========================================================================*/
/*  Functions                                                              */
/*=========================================================================*/
void SetUpInt(void)
{  
    //Interrupt slot 9 -> I2C (priority 1)
    ctl_set_isr(9, 1, CTL_ISR_TRIGGER_FIXED, IRQHandler2, 0); 
    ctl_unmask_isr(9);
     
    //Interrupt slot 6 -> UART0 (priority 2) 
    ctl_set_isr(6, 5, CTL_ISR_TRIGGER_FIXED, IRQHandler1, 0); 
    ctl_unmask_isr(6);
   
}

//==========================================================================
// Title:		IRQHandler1
// Author:		Jay Alcock
// Date:		
// Description:         Processes UART0 input
//
// Inputs:		None
// Outputs:             None
//==========================================================================
void IRQHandler1(void)
{    
    // increment active interrupt count for rtos
    ++ctl_interrupt_count;
    
    // copy char from receive buffer
    char tempChar = U0RBR;	    
    
    switch(tempChar)
    {
	case '/':   // system start
	    ctl_events_set_clear(&e1, START, NULL);
	break;
	
	case ' ': // kill switch
	    uart_printf(0, "killed");
	    kill = 1;
	    if(!kill) 
		kill = 1;
	    if(kill)
		kill = 0;
	break;
	
	case 't':   // all motors max speed
	    motor1Output=motor2Output=motor3Output=motor4Output = 100;
	    motor1basic=motor2basic=motor3basic=motor4basic =100;
	break;
	
	case 'f':   // all motors min speed
	    motor1Output=motor2Output=motor3Output=motor4Output = 0;
	    motor1basic=motor2basic=motor3basic=motor4basic = 0;
	break;

	case 'p':   
	    collectiveUp();
	break;
    
	case 'l':   
	    collectiveDown(); 
	break;
    
	case 'w':	   
	    pitchForward(); 
	break;
    
	case 's':	   
	    pitchBackward(); 
	break;
    
	case 'a':	   
	    rollLeft(); 
	break;
    
	case 'd':   
	    rollRight(); 
	break;

	case 'q':   
	    yawLeft(); 
	break;

	case 'e':	   
	    yawRight(); 
	break;
	
	case '.':
	    pidRst = 1;
	break;
	
	case 'i':
	    pidItrim += 0.01;
	break;
	
	case 'j':
	    pidItrim -= 0.01;
	break;
	
	case 'o':
	    pidPtrim += 0.1;
	break;
	
	case 'k':
	    pidPtrim -= 0.1;
	break;
	

    }
	    
	//    if (tempChar == 'z')	    //toggles between values printed to UART
//    {
//	outputVar += 1; 
//	
//	if(outputVar >= 4)
//	    outputVar = 0;
//    }

    ctl_interrupt_count--;
   
}

//==========================================================================
// Title:		IRQHandler2
// Author:		Jay Alcock
// Date:		
// Description:         I2C processing
//
// Inputs:		None
// Outputs:             None
//==========================================================================
void IRQHandler2(void)
{
    ++ctl_interrupt_count;
    
    U8 i2cStatus;
    static int i2cDataCntr;
    
    i2cStatus = I2C0STAT;		    // Read the I2C status buffer
    
    switch (i2cStatus)
    {
	case 0x00:		    // Bus Error
	    I2C0CONSET = I2C0CONSET_AA | I2C0CONSET_STO;    // Acknowledge and set stop
	    while(!(I2C0CONSET & I2C0CONSET_STO));		// ensure stopped
	    ctl_events_set_clear(&e1, I2C_FREE, NULL);
	    break;
	
	case 0x08:		    // Start condition
 	    I2C0DAT = i2cAddr;				    // Write Slave Address with R/W bit to I2DAT. 
      	    I2C0CONSET = I2C0CONSET_AA;			    // Acknowledge
 	    I2C0CONCLR = I2C0CONCLR_STAC;// |I2C0CONCLR_SIC;		// Clear interrupt 
	    i2cDataCntr = 0;				    // Initialize Master data counter.
	    break;
			    
	case 0x10:		    // Repeated start	
	    break;
	
	case 0x18:		    // Slave address transmitted and ACK received
	    I2C0DAT = i2cTxBuff[i2cDataCntr];		    // Write register to be read from to I2DAT. 
	    I2C0CONSET = I2C0CONSET_AA;			    // Acknowledge
	    i2cDataCntr++;				    // Increment Master data counter
	    break;
	
	case 0x28:		    // Transmit data  
	    if(i2cDataCntr >= i2cDataLen)
	    {
	    	I2C0CONSET = I2C0CONSET_AA | I2C0CONSET_STO ;
		while(!(I2C0CONSET & I2C0CONSET_STO)); 
		ctl_events_set_clear(&e1, I2C_FREE, NULL);
	    }
	    else
	    {
		I2C0DAT = i2cTxBuff[i2cDataCntr];	    // Write register to be read from to I2DAT. 
		I2C0CONSET = I2C0CONSET_AA;		    // Acknowledge
		i2cDataCntr++;		
	    }
	    break;
	
	case 0x38:		    //	Arbitration has been lost during Slave Address + Write or data.
	    I2C0CONSET = I2C0CONSET_AA | I2C0CONSET_STA;    // Acknowledge and start
	    break;
							
	case 0x40:		    // address + read transmitted and ACK received   
	    I2C0CONSET = I2C0CONSET_AA;			    // Acknowledge 
	    break;
	
	case 0x20:		    // Slave address + write transmitted and NOT acknowledged
	case 0x30:		    // Data transmitted and NOT Ack received
	case 0x48:		    // Slave address + read transmitted and NOT acknowledged
    	    I2C0CONSET = I2C0CONSET_AA | I2C0CONSET_STO;	// Acknowledge and set stop
	    while(!(I2C0CONSET & I2C0CONSET_STO)); // ensure stopped
	    ctl_events_set_clear(&e1, I2C_FREE, NULL);
	    break;
	
	case 0x50:		    // Read data
	    i2cRxBuff[i2cDataCntr] = I2C0DAT;		    // Read data from I2C data buffer
	    i2cDataCntr++;				    // Increment Master data counter.
	    if(i2cDataCntr == i2cDataLen)
	    	I2C0CONCLR = I2C0CONCLR_AAC;// | I2C0CONCLR_SIC; // Clear interrupt 
	    else
		I2C0CONSET = I2C0CONSET_AA;		    // Acknowledge
	    
	    break;  
	case 0x58:		    // Read data and stop
	    i2cRxBuff[i2cDataCntr] = I2C0DAT;		    // Read data from I2C data buffer	
	    I2C0CONSET = I2C0CONSET_AA | I2C0CONSET_STO;    // Acknowledge and set stop
	    ctl_events_set_clear(&e1, I2C_FREE, NULL);
	    break;
    }
    I2C0CONCLR = I2C0CONCLR_SIC;	    // Clear interrupt 

    ctl_interrupt_count--;
}
//==========================================================================
// Title:		IRQHandler3
// Author:		Jay Alcock
// Date:		11OCT13
// Description:         PWM Update
//
// Inputs:		None
// Outputs:             None
//==========================================================================
//void IRQHandler3(void)
//{  
//    PWMUpdate();
//    T1_IR = T_IR_MR1;				// Clear interrupt source
//    VIC_VectAddr = VICVectAddr_Clear;           // Clear the priority hardware
//}