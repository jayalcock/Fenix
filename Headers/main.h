/****************************************************************************
* File:  		main.h
* Author: 		Jay Alcock
* Date: 		01JUN2012
* Description:          main.c header file
*
* History:
*
****************************************************************************/

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#ifndef _MAIN_H_
#define _MAIN_H_

#include <string.h>
#include <ctl_api.h>
#include <cross_studio_io.h>
#include <stdbool.h>

#include "cpu.h"
#include "lpc2148.h"
#include "I2C.h"
#include "irq.h"
#include "uart.h"
#include "sensors.h"
#include "PWM.h"
#include "flightControl.h"

/*=========================================================================*/
/*  DEFINE: All Structures and Common Constants                            */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Prototypes                                                     */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/
//Events
#define I2C_FREE	(1<<0)
#define UART_WRITE	(1<<1)
#define UPDATE_PWM	(1<<2)
#define UPDATE_PID	(1<<3)
#define START		(1<<4)
#define ZERODONE	(1<<5)

#define STACKSIZE 64    

/*=========================================================================*/
/*  DEFINE: Definition of all global Data                                  */
/*=========================================================================*/
extern volatile BOOL kill, pidRst;
extern volatile int i2cDataLen, outputVar;
extern volatile N8 motor1basic, motor2basic, motor3basic, motor4basic;
extern volatile N8 motor1Output, motor2Output, motor3Output, motor4Output;
extern volatile U8 i2cAddr, *txBuffPtr, *rxBuffPtr, i2cTxBuff[2], i2cRxBuff[2];
extern volatile float pidPtrim, pidItrim;

extern CTL_EVENT_SET_t e1;
extern CTL_EVENT_SET_t sensorEvents;

extern CTL_MESSAGE_QUEUE_t sensorValsQ, motorValsQ, PIDQ;

/*=========================================================================*/
/*  DEFINE: Definition of all local Procedures                             */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: All code exported                                              */
/*=========================================================================*/

#endif /* MAIN_H_ */