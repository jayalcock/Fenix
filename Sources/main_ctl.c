#include "main.h"
 
// define tasks 
static CTL_TASK_t main_task, readSensors_task, writeUart_task, PWMUpdate_task, hoverPID_task, newtask;  

// define queues
CTL_MESSAGE_QUEUE_t sensorValsQ, motorValsQ, PIDQ;
  
// allocate queues
void *sensorQueue[11], *motorQueue[4], *PIDQueue[3];
  
void *temp;

// allocate stack memory
unsigned sensors_stack[1+80+1]; 
unsigned uart_stack[1+80+1]; 
unsigned pwm_stack[1+STACKSIZE+1]; 
unsigned hoverPID_stack[1+85+1]; 
unsigned new_stack[1+STACKSIZE+1]; 

// error handler
void ctl_handle_error(CTL_ERROR_CODE_t e)
{
    while (1);
}

// initalise stack memory
void initMem(void)
{
    memset(sensors_stack, 0xcd, sizeof(sensors_stack));		// write known values into the stack
    sensors_stack[0]=sensors_stack[1+STACKSIZE]=0xfacefeed;	// put marker values at the words before/after the stack

    memset(uart_stack, 0xcd, sizeof(uart_stack));		// write known values into the stack
    uart_stack[0]=uart_stack[1+STACKSIZE]=0xfacefeed;		// put marker values at the words before/after the stack
      
    memset(pwm_stack, 0xcd, sizeof(pwm_stack));			// write known values into the stack
    pwm_stack[0]=pwm_stack[1+STACKSIZE]=0xfacefeed;		// put marker values at the words before/after the stack
     
    memset(hoverPID_stack, 0xcd, sizeof(hoverPID_stack));	// write known values into the stack
    hoverPID_stack[0]=hoverPID_stack[1+STACKSIZE]=0xfacefeed;	// put marker values at the words before/after the stack
}


int main(void)
{     
    unsigned int v=0;
    
    // create subsequent tasks whilst running at the highest priority.
    ctl_task_init(&main_task, 255, "main");	    
   
    // initialise event set
    ctl_events_init(&e1, 0x01);			    
 
    // initialise message queues
    ctl_message_queue_init(&sensorValsQ, sensorQueue, 10);  
    ctl_message_queue_init(&PIDQ, PIDQueue, 3);
  
    // start the timer
    ctl_start_timer(ctl_increment_tick_from_isr);    

    // initialise stack memory with known data
    initMem();			
    
    //initialise tasks (in order of priority)
    ctl_task_run(&readSensors_task, 200, readSensors, 0, "readSensors_task", sizeof(sensors_stack) / sizeof(unsigned), sensors_stack, 0);
    ctl_task_run(&hoverPID_task, 150, hoverPID, 0, "hoverPID_task", sizeof(hoverPID_stack) / sizeof(unsigned), hoverPID_stack, 0);
    ctl_task_run(&PWMUpdate_task, 15, PWMUpdate, 0, "PWMUpdate_task", sizeof(pwm_stack) / sizeof(unsigned), pwm_stack, 0);
    ctl_task_run(&writeUart_task, 5, writeToUART, 0, "writeUart_task", sizeof(uart_stack) / sizeof(unsigned), uart_stack, 0);

//    ctl_task_run(&newtask, 2, test, 0, "newtask_task", sizeof(new_stack) / sizeof(unsigned), new_stack, 0);

    // drop to lowest priority to start created tasks running.
    ctl_task_set_priority(&main_task, 0);	
    
    while (1)
    {  
	v++;
    }
    return 0;
}
