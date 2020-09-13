/****************************************************************************
* File:  		I2C.c
* Author: 		Jay Alcock
* Date: 		17SEP12
* Description:          
*
* History:
*
****************************************************************************/

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#define _I2C_C_

#include "I2C.h"

volatile int i2cDataLen;
volatile U8 i2cAddr, i2cTxBuff[2], i2cRxBuff[2]; 
CTL_EVENT_SET_t e1;
    

/*=========================================================================*/
/*  Functions                                                              */
/*=========================================================================*/

//==========================================================================
// Title: 		I2CInit
// Author:		Jay Alcock
// Date:		
// Description:         
//
// Inputs:		None
// Outputs:             None
//==========================================================================
void I2CInit(void)
{ 
    PINSEL0 = (PINSEL0 & ~(PINSEL0_P0_2_MASK | PINSEL0_P0_3_MASK)) | //Set pins to I2C
		(PINSEL0_P0_2_SCL0 | PINSEL0_P0_3_SDA0);	
    
    PCONP |= PCONP_PCI2C0;		// Power to I2C bus  
    
    I2C0CONCLR = I2C0CONCLR_SIC | I2C0CONCLR_STAC | I2C0CONCLR_AAC; // clear all I2C settings
    I2C0CONSET = I2C0CONSET_I2EN;	// Enable I2C interface

    I2C0SCLL = 60;			// i2c bitfrequency = pclk/(scll+sclh)
    I2C0SCLH = 60;			// = 200kHz
    
    // set up accelerometers
    I2CWriteByte(accelAdrr, CTRL_REG1_A, 0x47);	//50hz output rate, all axis' on
    I2CWriteByte(accelAdrr, CTRL_REG4_A, 0x98);	//FSD = +/-4G, High res output
    
    // set up magnetomometer
    I2CWriteByte(magAdrr, CRA_REG_M, 0x18);	//75Hz output data rate, temp sensor off
    I2CWriteByte(magAdrr, CRB_REG_M, 0x20);	//FSD = +/-1.3 Gauss
    I2CWriteByte(magAdrr, MR_REG_M, 0x00);	//Continuous conversion mode    
    
    // set up gyros
    I2CWriteByte(gyroAdrr, CTRL_REG1, 0x0F);	//output data rate = 95Hz/12.5 cutoff, all axis' on
    I2CWriteByte(gyroAdrr, CTRL_REG2, 0x00);	//high pass filter normal
    I2CWriteByte(gyroAdrr, CTRL_REG4, 0x10);	//FSD = 500deg/s
    I2CWriteByte(gyroAdrr, CTRL_REG5, 0x00);	//HPF/FIFO = off, output sel 0
}
//==========================================================================
// Title: 		I2CSetAddr
// Author:		Jay Alcock
// Date:		Sets slave I2C address
// Description:         
//
// Inputs:		device address
// Outputs:             None
//==========================================================================
void I2CSetAddr (U8 addr)
{
    // wait if busy
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);
    i2cAddr = addr;			// address + write bit
}

//==========================================================================
// Title: 		I2CSetReg
// Author:		Jay Alcock
// Date:		
// Description:         Sets slave I2C register
//
// Inputs:		register
// Outputs:             None
//==========================================================================
void I2CSetReg (U8 reg)
{
    // wait if busy		
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);
    i2cTxBuff[0] = reg;			// point register address val to buffer  
} 
    
//==========================================================================
// Title: 		I2CWriteByte
// Author:		Jay Alcock
// Date:		
// Description:         Writes data to a given address and register
//
// Inputs:		device address, register, data to be tx
// Outputs:             None
//==========================================================================
void I2CWriteByte (U8 addr, U8 reg, U8 data)
{
    // wait if busy
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);
    I2CSetAddr(addr);		// set register address
    I2CSetReg(reg);		// set register address
    i2cTxBuff[1] = data;	// point data to buffer
    i2cDataLen = 2;		// set data length
    ctl_events_set_clear(&e1, NULL, I2C_FREE);	// bus busy

    I2C0CONSET = I2C0CONSET_STA;// start i2c
    // wait if busy    
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);
  
   
}
//==========================================================================
// Title: 		I2CReadByte
// Author:		Jay Alcock
// Date:		
// Description:		Gets data from a given address and register 
//
// Inputs:		None
// Outputs:             None
//==========================================================================
U8 I2CReadByte(U8 addr, U8 reg)
{   
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);  
    I2CSetAddr (addr);
    I2CSetReg (reg);
    i2cDataLen = 1;		    // set data length
    ctl_events_set_clear(&e1, NULL, I2C_FREE);
    I2C0CONSET = I2C0CONSET_STA;    // start i2c
    
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);
    I2CSetAddr(addr | 0x01);	    // address add read bit
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);
    i2cDataLen = 1;		    // set data length
    ctl_events_set_clear(&e1, NULL, I2C_FREE);
    I2C0CONSET = I2C0CONSET_STA;    // start i2c

    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e1, I2C_FREE, 1, ctl_get_current_time()+100);

    return i2cRxBuff[0];
}

//==========================================================================
// Title: 		I2CDisable
// Author:		Jay Alcock
// Date:		
// Description:		Disables I2C
//
// Inputs:		None
// Outputs:             None
//==========================================================================
void I2CDisable (void)
{
    I2C0CONCLR |= I2C0CONCLR_I2ENC;	//Disable I2C interface
    PCONP &= ~PCONP_PCI2C0;		//turn off power to I2C bus
}
