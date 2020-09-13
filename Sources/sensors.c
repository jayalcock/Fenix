/****************************************************************************
* File:  		sensors.c
* Author: 		Jay Alcock
* Date: 		20 Dec 12
* Description:          Reading and calculation of sensor data
*
* History:
*
****************************************************************************/

/*=========================================================================*/
/*  DEFINE: Preprocessors                                            	   */
/*=========================================================================*/
#define _sensors_c_

#include "sensors.h"

CTL_EVENT_SET_t sensorEvents;

#include <math.h>


//temporary test task
void test(void *p)
{
    while(1)
    {
	
    }    
}

/*=========================================================================*/
/*  Functions                                                              */
/*=========================================================================*/
//==========================================================================
// Title: 		readSensors
// Author:		Jay Alcock
// Date:		06 Nov 13
// Description:         Schedules and manipulates sensor data. 20ms/50Hz cycle time.
//
// Inputs:		void pointer from task initialization
// Outputs:             None
//==========================================================================
void readSensors(void *p)
{
    char i;
    float accX, accY, gyrX, gyrY, gyrZ, magX, magY, magZ, pitch, roll, yaw;
    
    accX = accY = gyrX = gyrY = gyrZ = magX = magY = magZ = pitch = roll = yaw = 0;

    // initalise system
    cpuSetupHardware();
    
    while(1)
    {
	// toggle output LED
	FIO0PIN ^= GPIO_IO_P13;
      
	// 20ms/50Hz cycle time for task
	ctl_timeout_wait(ctl_get_current_time() + timeStep);
      
	// reading of sensor values. returns scaled engineering units.
	readAcc(&accX, &accY);			//read accelerometers	
	readGyro(&gyrX, &gyrY, &gyrZ);		//read gyros
	readMag(&magX, &magY, &magZ);		//read magnetomometers
	
	// wait for start command to start calculations
//	ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS, &e1, (START && ZERODONE), 0, 0);
      
	// calculation of aircraft attitude though a complimentary filter
	roll = (rollGain * roll + gyrX * timeStep / 1000) + ((1 - rollGain) * accX);
	pitch = (pitchGain * pitch + gyrY * timeStep / 1000) + ((1 - pitchGain) * accY);
//	yaw = (0.98 * yaw + gyrZ * timeStep / 1000) + (0.02 * magZ);
	 
      	// Post values to PID queue 
	ctl_message_queue_post(&PIDQ, (float *)&pitch, 0, 0);
	ctl_message_queue_post(&PIDQ, (float *)&roll, 0, 0);
	
	// enable update_pid task to run
//	ctl_events_set_clear(&e1, UPDATE_PID, NULL);

	// trigger UART write task every two sensor reads
	if(i >= 4)
	{
//	    ctl_message_queue_post(&sensorValsQ, (float *)&pitch, 0, 0);
//	    ctl_message_queue_post(&sensorValsQ, (float *)&roll, 0, 0);
	  
//	    ctl_message_queue_post(&sensorValsQ, (float *)&accX, 0, 0);
//	    ctl_message_queue_post(&sensorValsQ, (float *)&accY, 0, 0);
//	    ctl_message_queue_post(&sensorValsQ, (float *)&accZ, 0, 0);
	
//	    ctl_message_queue_post(&sensorValsQ, (float *)&gyrX, 0, 0);
//	    ctl_message_queue_post(&sensorValsQ, (float *)&gyrY, 0, 0);
//	    ctl_message_queue_post(&sensorValsQ, (float *)&gyrZ, 0, 0);
	    
//	    ctl_message_queue_post(&sensorValsQ, (float *)&magX, 0, 0);
//	    ctl_message_queue_post(&sensorValsQ, (float *)&magY, 0, 0);
//	    ctl_message_queue_post(&sensorValsQ, (float *)&magZ, 0, 0);
	    
	    ctl_message_queue_post(&sensorValsQ, &motor1Output, 0, 0);
	    ctl_message_queue_post(&sensorValsQ, &motor2Output, 0, 0);
	    ctl_message_queue_post(&sensorValsQ, &motor3Output, 0, 0);
	    ctl_message_queue_post(&sensorValsQ, &motor4Output, 0, 0);
//	    
	    //enable uart_write task
	    ctl_events_set_clear(&e1, UART_WRITE, NULL);
	    i = 0;
	}
	i++;

    }

}

//==========================================================================
// Title: 		readAcc
// Author:		Jay Alcock
// Date:		20 Dec 12
// Description:         Reads accelerometer values, calculates & normalises vector magnitude 
//			Raw values must be bit shifted right 4 bits because sensor only has 12bit
//			precision and pads the lower 4 with 0's
// Inputs:		None
// Outputs:             None
//==========================================================================
void readAcc(float *roll, float *pitch)
{
    static BOOL firstReadDone = 0;
    static float xOffset = 0, yOffset = 0, zOffset = 0;
    N16 accXRaw, accYRaw, accZRaw;
    float xScaled, yScaled, zScaled;

    //zeroing function
    if(!firstReadDone)
    {
	zeroAccel(&xOffset, &yOffset, &zOffset);
	firstReadDone = 1;
    }

    // Read X axis
    accXRaw = (I2CReadByte (accelAdrr, OUT_X_H_A) << 8);   
    accXRaw |= (I2CReadByte (accelAdrr, OUT_X_L_A));
    xScaled = ((accXRaw >> 4) * accelGain) - xOffset;
    
     // Read Y axis
    accYRaw = (I2CReadByte (accelAdrr, OUT_Y_H_A) << 8);
    accYRaw |= (I2CReadByte (accelAdrr, OUT_Y_L_A));
    yScaled = ((accYRaw >> 4) * accelGain) - yOffset;
   
    // Read Z axis
    accZRaw = (I2CReadByte (accelAdrr, OUT_Z_H_A) << 8);
    accZRaw |= (I2CReadByte (accelAdrr, OUT_Z_L_A));  
    zScaled = ((accZRaw >> 4) * accelGain) - zOffset; 
	   
    // Calculate absolute value
//    float mag = sqrt(pow(xScaled, 2) + pow(yScaled, 2) + pow(zScaled, 2));  //calculate raw absolute value
//	
////    // Correct/normalise values
//    xScaled /= mag;
//    yScaled /= mag;
//    zScaled /= mag;    

    // Calculate pitch value
    *pitch = sqrt(pow(yScaled, 2) + pow(zScaled, 2));
//    *pitch = sqrt(*pitch);
    *pitch = atan2(xScaled, *pitch);
    *pitch *= (180 / PI);
    
//    *pitch = atan2(xScaled, zScaled);
//    *pitch *= (180 / PI);    

    // Caclulate roll value
    *roll = atan2(-yScaled, -zScaled);
    *roll *= (180 / PI);  
 
}
//==========================================================================
// Title: 		readGyro
// Author:		Jay Alcock
// Date:		20 Dec 12
// Description:         Reads values from gyro and converts into deg/s
//
// Inputs:		None
// Outputs:             None
//==========================================================================
void readGyro(float *gyrX, float *gyrY, float *gyrZ)
{
    static BOOL firstReadDone = 0;
    static float xOffset = 0, yOffset = 0, zOffset = 0; 
    N16 gyrXRaw, gyrYRaw, gyrZRaw;
    float x, y, z;
    
    //zeroing function
    if(!firstReadDone)
    {
	zeroGyro(&xOffset, &yOffset, &zOffset);
	firstReadDone = 1;
//	ctl_message_queue_post(&sensorValsQ, &xOffset, 0, 0);
//	ctl_events_set_clear(&e1, UART_WRITE, NULL);
    }
    
    // read x axis (pitch)
    gyrXRaw = (I2CReadByte (gyroAdrr, OUT_X_H) << 8);	    // gyros
    gyrXRaw |= (I2CReadByte (gyroAdrr, OUT_X_L));
    x = gyrXRaw * gyroGain - xOffset;

    // read y axis (roll)
    gyrYRaw = (I2CReadByte (gyroAdrr, OUT_Y_H) << 8);
    gyrYRaw |= (I2CReadByte (gyroAdrr, OUT_Y_L));
    y = gyrYRaw * gyroGain - yOffset;

    // read z axis (yaw)
    gyrZRaw = (I2CReadByte (gyroAdrr, OUT_Z_H) << 8);
    gyrZRaw |= (I2CReadByte (gyroAdrr, OUT_Z_L));  
    z = gyrZRaw * gyroGain - zOffset;

    *gyrX = x;
    *gyrY = y;
    *gyrZ = z;

}
//==========================================================================
// Title: 		readMag
// Author:		Jay Alcock
// Date:		20 Dec 12
// Description:         Reads values from magnetomometer
//
// Inputs:		pointers to mag axis variables 
// Outputs:             None
//==========================================================================
void readMag(float *magX, float *magY, float *magZ)
{
    N16 magXRaw, magYRaw, magZRaw;
    float mag;
	
    // read x axis
    magXRaw = (I2CReadByte (magAdrr, OUT_X_H_M) << 8);	 
    magXRaw |= (I2CReadByte (magAdrr, OUT_X_L_M) << 0);
    *magX = magXRaw / magGain;

    // read y axis
    magYRaw = (I2CReadByte (magAdrr, OUT_Y_H_M) << 8);	   
    magYRaw |= (I2CReadByte (magAdrr, OUT_Y_L_M)<< 0);
    *magY = magYRaw / magGain;

    // read z axis
    magZRaw = (I2CReadByte (magAdrr, OUT_Z_H_M) << 8);	   
    magZRaw |= (I2CReadByte (magAdrr, OUT_Z_L_M) << 0);   
    *magZ = magZRaw / magZGain;	

    // Calculate mag vector magnitude
//    mag = sqrt(pow(*magX, 2) + pow(*magY, 2) + pow(*magZ, 2));  //calculate raw absolute value
//    
//    // Correct/normalise values
//    *magX /= mag;
//    *magY /= mag;
//    *magZ /= mag;    

    mag = sqrt(pow(*magY, 2) + pow(*magZ, 2));  //calculate raw absolute value
//    float temp = atan2(magX, mag);   

}
//==========================================================================
// Title: 		zeroGyro
// Author:		Jay Alcock
// Date:		20 Dec 12
// Description:         reads gyro values for 5 seconds (250 samples), averages and makes zero point
//
// Inputs:		Pointers to gyro offsets
// Outputs:             None
//==========================================================================
void zeroGyro(float *x, float *y, float *z)
{
    N16 i, temp;
    
    for(i=0; i < samplesNo; i++)
    {    
	ctl_timeout_wait(ctl_get_current_time() + 20);
	// read x axis (pitch)
	temp = (I2CReadByte (gyroAdrr, OUT_X_H) << 8);	    // gyros
	temp |= (I2CReadByte (gyroAdrr, OUT_X_L));
	*x += temp * gyroGain / samplesNo;

	// read y axis (roll)
	temp = (I2CReadByte (gyroAdrr, OUT_Y_H) << 8);
	temp |= (I2CReadByte (gyroAdrr, OUT_Y_L));
	*y += temp * gyroGain / samplesNo;

	// read z axis (yaw)
	temp = (I2CReadByte (gyroAdrr, OUT_Z_H) << 8);
	temp |= (I2CReadByte (gyroAdrr, OUT_Z_L));  
	*z += temp * gyroGain / samplesNo;

    }
}
//==========================================================================
// Title: 		zeroAccel
// Author:		Jay Alcock
// Date:		20 Dec 12
// Description:         reads accelerometer 250 samples, averages and makes zero point
//
// Inputs:		Pointers to accelerometer offsets
// Outputs:             None
//==========================================================================
void zeroAccel(float *xPtr, float *yPtr, float *zPtr)
{
    N16 i, temp; 
    
    for(i=0; i < samplesNo; i++)
    {    
	ctl_timeout_wait(ctl_get_current_time() + 20);
	// Read X axis
	temp = (I2CReadByte (accelAdrr, OUT_X_H_A) << 8);   
	temp |= (I2CReadByte (accelAdrr, OUT_X_L_A));
	*xPtr += ((temp >> 4) * accelGain) / samplesNo;
       
	// Read Y axis
	temp = (I2CReadByte (accelAdrr, OUT_Y_H_A) << 8);
	temp |= (I2CReadByte (accelAdrr, OUT_Y_L_A));
	*yPtr += ((temp >> 4) * accelGain) / samplesNo;
     
	// Read Z axis
	temp = (I2CReadByte (accelAdrr, OUT_Z_H_A) << 8);
	temp |= (I2CReadByte (accelAdrr, OUT_Z_L_A));  
	*zPtr += ((temp >> 4) * accelGain) / samplesNo; 

    }
    
    // calculate absolute value of Z-axis sensor orientation is inverted
    if(*zPtr < 0)
	*zPtr = fabsf(*zPtr);
}