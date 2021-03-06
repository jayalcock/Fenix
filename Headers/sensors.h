#ifndef _SENSORS_H_
#define _SENSORS_H_

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#include "math.h"
#include "I2C.h"
#include "main.h"

/*=========================================================================*/
/*  DEFINE: All Structures and Common Constants                            */
/*=========================================================================*/
//typedef struct vector
//{
//    N16 x, y, z;
//    double abs;
//} vector;


/*=========================================================================*/
/*  DEFINE: Prototypes                                                     */
/*=========================================================================*/
void readSensors(void *p);
void readAcc(float *roll, float *pitch);
void readGyro(float *gyrX, float *gyrY, float *gyrZ);
void readMag(float *magX, float *magY, float *magZ);
void zeroAccel(float *xPtr, float *yPtr, float *zPtr);
void zeroGyro(float *x, float *y, float *z);
void writeToUART(void *p);

void test(void *p);
/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/
#define accPost	    0x01
#define gyrPost	    0x02
#define magPost	    0x03
#define accelMax    4	    // +- 4G
#define accelGain   0.002   // g/LSB
#define gyroMax	    500	    // +- 500dps
#define gyroGain    0.0175  // degrees/s/digit
#define magMax	    4.0	    // +- 4gauss
#define magGain	    1100.0  // LSB/gauss
#define magZGain    980.0   // LSB/gauss
#define N16Max	    32767
#define timeStep    20	    //20ms
#define PI	    3.14159265358979323846
#define pitchGain   0.95
#define rollGain    0.95
#define samplesNo   250	    // number of averaging samples for accel zeroing



/*=========================================================================*/
/*  DEFINE: Definition of all local Procedures                             */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: All code exported                                              */
/*=========================================================================*/

#endif /* DEFAULT_H_ */
    
    
    //double message test
//    void *tempPtr, *tempPtr1;
//    double tempDbl[2], temp1, temp2;
//    tempDbl[0]= 97.8756;
//    tempDbl[1]= 3.6;
//    tempPtr = &tempDbl;
//
//    ctl_message_queue_post(&sensorValsQ, tempPtr, 0, 0);
//    ctl_message_queue_receive(&sensorValsQ, &tempPtr1, 0 , 0);
//    
//    temp1 = *(double*)tempPtr1;
//    temp2 = *((double*)tempPtr1+1);