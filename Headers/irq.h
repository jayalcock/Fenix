#ifndef _IRQ_H_
#define _IRQ_H_

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#include <ctl_api.h>
#include "lpc2148.h"
#include "main.h"

//#include "flightControl.h"

/*=========================================================================*/
/*  DEFINE: All Structures and Common Constants                            */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Prototypes                                                     */
/*=========================================================================*/
void SetUpInt(void);
void IRQHandler0(void);
void IRQHandler1(void);
void IRQHandler2(void);
void IRQHandler3(void);

/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Definition of all local Procedures                             */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: All code exported                                              */
/*=========================================================================*/ 

#endif /* IRQ_H_ */
