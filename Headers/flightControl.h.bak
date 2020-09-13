#ifndef _flightControl_H_
#define _flightControl_H_

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#include "main.h"
#include "math.h"

/*=========================================================================*/
/*  DEFINE: All Structures and Common Constants                            */
/*=========================================================================*/
typedef struct PID 
{
    float dt, Kp, Ki, Kd, actual, setpoint, errorOld, error, 
    integral, derivative, output, deadband, MAXout, MINout;
} PID;

/*=========================================================================*/
/*  DEFINE: Prototypes                                                     */
/*=========================================================================*/
void hoverPID(void *p);
void PIDcalc(PID *vals);
void collectiveUp(void);
void collectiveDown(void);
void pitchForward(void);
void pitchBackward(void);
void rollRight(void);
void rollLeft(void);
void yawRight(void);
void yawLeft(void);

/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/
#define PIDtimestep 20
/*=========================================================================*/
/*  DEFINE: Definition of all local Procedures                             */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: All code exported                                              */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Global Variables                                               */
/*=========================================================================*/ 


#endif /* _flightControl_H_ */
