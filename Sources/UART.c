/****************************************************************************
* File:  		UART.c
* Author: 		Jay Alcock
* Date: 		04SEP12
* Description:          Control for onboard UART
*
* History:
*
****************************************************************************/

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#define _UART_C_

#include "UART.h"

/*=========================================================================*/
/*  Functions                                                              */
/*=========================================================================*/

//==========================================================================
// Title:               UART0init
// Author:		Jay Alcock
// Date:		04SEP12
// Description:         Initialization of UART0
//
// Inputs:		None
// Outputs:             None
//==========================================================================
void UART0init(void)
{

    PCONP |= PCONP_PCUART0;                                                   //Power to UART0
    PINSEL0 |= PINSEL0_P0_0_TXD0 | PINSEL0_P0_1_RXD0;                         //Initialize Pin Select Block for Tx and Rx
    U0FCR = U0FCR_FIFO_Enable | U0FCR_Rx_FIFO_Reset;// | U0FCR_Tx_FIFO_Reset; //Reset and Enable FIFO's
    U0LCR = U0LCR_Divisor_Latch_Access_Bit | U0LCR_Word_Character_Length;     //Set DLAB and word length set to 8bits
    U0DLL = U0DLL_Baud_19200; //U0DLL_Baud_57600; //0x9C                                          //Baud rate set to 57600 for 24MHz PCLK pg.151 of user manual
    U0DLM = U0DLM_Divisor_0;                                                  //Divisor latch at 0
    U0FDR = U0FDR_MulVal_1;                                                   //Set multiplier to 1
    U0LCR = U0LCR_Clear_DLAB;                                                 //Clear DLAB 
    U0IER = U0IER_RBR_Interrupt_Enable;                                       //Enable UART0 Interrupt

}

//==========================================================================
// Title:               writeToUART
// Author:		Jay Alcock
// Date:		06NOV13
// Description:         write to UART0 task
//
// Inputs:		void task pointer
// Outputs:             none
//==========================================================================
void writeToUART(void *p)
{
    void *varPtr;
    char i, iterations;
    
    while(1)
    {
	// wait for enable bit to be set
	ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS_WITH_AUTO_CLEAR , &e1, UART_WRITE, 0, 0);
	
	//store number of values as it decreases as values are read from queue
	iterations = sensorValsQ.n;
	
	//write to uart for length of queue
	for(i = 0; i < iterations; i++)
	{
	    ctl_message_queue_receive(&sensorValsQ, &varPtr, 0 , 0);
//	    uart_printf(0, "%6.2f ", *(float *)varPtr);
	     uart_printf(0, "%d ", *(char *)varPtr);
	}
	//carriage return once all data is printed
	uart_printf(0, "\n");
    }
  
}
//==========================================================================
// Title:               uart_printf
// Author:		Jay Alcock
// Date:		21Mar13
// Description:         printf to uart
//
// Inputs:		uart number, data to be transmitted
// Outputs:             number of characters to be printed
//==========================================================================
int uart_printf(int uart, const char *fmt, ...)
{
    int n;
    va_list ap;
    __printf_t iod;
    va_start(ap, fmt);
    iod.string = NULL;
    iod.maxchars = INT_MAX;
    iod.output_fn = uart ? uart1_putc : uart0_putc;
    n = __vfprintf(&iod, fmt, ap);
    va_end(ap);
    return n;
}

//==========================================================================
// Title:               uart0_putc
// Author:		Jay Alcock
// Date:		21Mar13
// Description:         put char to uart
//
// Inputs:		char
// Outputs:             int - successful tx
//==========================================================================
int uart0_putc(int ch, __printf_t *ctx)
{
//    if(ch == '\n')		    //if character is a line feed, also carriage return
//	uart0_putc('\r', 0);
    
	while (!(U0LSR & 0x20));    //loop until transmit holding register empty
    
	return (U0THR = ch);	    //copy char to transmit holding register
}

//==========================================================================
// Title:               uart1_putc
// Author:		Jay Alcock
// Date:		21Mar13
// Description:         put char to uart
//
// Inputs:		char
// Outputs:             int - successful tx
//==========================================================================
int uart1_putc(int ch, __printf_t *ctx)
{
//    if(ch == '\n')		    //if character is a line feed, also carriage return
//	uart0_putc('\r', 0);
//    
    while (!(U1LSR & 0x20));	    //loop until transmit holding register empty
    
    return (U1THR = ch);	    //copy char to transmit holding register
}

//==========================================================================
// Title:               getChar
// Author:		Jay Alcock
// Date:		04SEP12
// Description:         Receives a character to the serial port
//
// Inputs:		None
// Outputs:             Char from serial port
//==========================================================================
int getcUART(void) 
{
  while (!(U0LSR & 0x01));

  return (U0RBR);
}