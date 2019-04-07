// RTOS Framework - Spring 2018
// J Losh

// Student Name: JYOTHI BHAT
//c file: the following code is for uncooperative(preemptive) rtos only
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <hw_nvic.h>
#include <hw_types.h>
#include <math.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define PUSH_BUTTON_P1  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //on board pb4
#define PUSH_BUTTON_P2      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) // off-board PB1
#define PUSH_BUTTON_P3   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board PB2
#define PUSH_BUTTON_P4  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board PB3
#define PUSH_BUTTON_P5   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) // off-board PB4
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) //off-board red led
#define ORANGE_LED  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) //off-orange led
#define YELLOW_LED  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) //off-yellow led
#define GREEN_LED  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4))) //off-green led

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
#define MAX_TASKS 10       // maximum number of valid tasks

struct semaphore
{
  uint16_t count;
  uint16_t max_count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE];    // store task index here
} semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer
#define STATE_RUNNING    5 // is running

uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
  uint8_t skipCount;             //variable which indicates how many times the task has been skipped
  uint8_t number;               //index of tcb entry
  void *store_pid;              //fn value
  float cpu_time;               //time taken by each task
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

int rtosScheduler();

//global varibales created by me
void* systemSP;                         //store system stack pointer
bool pi = true;                        //priority inheritance flag
bool start = false;                     //flag to enable pendsv in systick isr
bool approved = false;                  //to indicate thread creation
struct semaphore *p_semaphore = 0;      //pointer for semaphore
bool preemp = true;

//timer variables
uint32_t insta_time = 0;                //time taken by the function at that particular time
bool first = true;                      //flag to implement iir filter technique
float alpha = 0.99;                     //iir filter technique variable

//for shell command function
#define MAX_CHARS 25                           //static array size
char command[MAX_CHARS];                      //user entered command is stored here
char message[10][MAX_CHARS];                 //transformation to the command entered
uint8_t position[MAX_CHARS];                 //to calculate the position of commands
uint8_t minArgs;                            //the minimum number of arguments the user entered

//functions added by me
void systick_init();                             //to initialize systick isr
void calculate(uint32_t);                       //to calculate and store time taken by function
void setThreadPriority(_fn,uint8_t);    //to set the priority of the function
void setSP(uint8_t *value);                     //to set the value of stack pointer
void* getSP();                                  //to get the value of current sp
void pushContents(uint32_t xpsr, _fn pc);       //to create a stack for unrun state functions
void getsUart0(char*);                          //to get string from uart0
void transform_string();                        //to transform the string entered
void getCommand();                              //to extract the command from the string
void isCommand();                               //to compare the command typed with predefined ones
void putsUart0(char*);                          //put string into uart0
void putcUart0(char);                           //put character into uart0
void putnUart0();                               //put number into uart0
void putfUart0(float);                          //put float value into uart0
char getcUart0();                               //to get a character from uart0
uint32_t hexToDec(uint32_t);                    //to convert hexadecimal to deciaml
uint32_t getvalue(uint32_t);                   //to get the value from the address
uint32_t getR0();                               //get R0 value from the stack
uint32_t getR1();                               //get R1 value from the stack
uint32_t getR2();                               //get R2 value from stack
void reverse(char*, int);                         //to reverse
int intToStr(int x, char str[], int d);          //interger to string conversion
void strcp(char*, char*);                        //to copy two strings
float total_time = 0;
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
  uint8_t i;
  taskCount = 0;                        // no tasks running
  for (i = 0; i < MAX_TASKS; i++)       // clear out tcb records
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
   systick_init();                      // REQUIRED: initialize systick for 1ms system timer
}

void rtosStart()
{
  // REQUIRED: add code to call the first task to be run
  void* value;
  taskCurrent = rtosScheduler();
  tcb[taskCurrent].state = STATE_RUNNING;
  // Add code to initialize the SP with tcb[task_current].sp;
  value = tcb[taskCurrent].sp;
  systemSP = getSP();                   //store system sp
  setSP(value);                         //set function sp
  _fn fn = (_fn)tcb[taskCurrent].pid;
  WTIMER5_TAV_R = 0;                        //resetting the counter
  start = true;                     //ok to pendsv now
  (*fn)();                          //jump to the set pc
}

bool createThread(_fn fn, char name[], int priority)
{
__asm(" SVC #0x00");

  //code moved to svcISR
/*
  bool ok = false;
  uint8_t j,i = 0;
  bool found = false;

  // REQUIRED: store the thread name
  // add task if room in task list
  if (taskCount < MAX_TASKS)
  {
    // make sure fn not already in list (prevent reentrancy)
    while (!found && (i < MAX_TASKS))
    {
      found = (tcb[i++].pid ==  fn);
    }
    if (!found)
    {
      // find first available tcb record
      i = 0;
      while (tcb[i].state != STATE_INVALID) {i++;}
      tcb[i].state = STATE_UNRUN;
      tcb[i].pid = fn;
      tcb[i].sp = &stack[i][255];
      tcb[i].priority = priority;
      tcb[i].currentPriority = priority;
      tcb[i].skipCount = priority;
      tcb[i].number = i;
      for(j=0;j<strlen(name);j++)
          tcb[i].name[j] = name[j];
      tcb[i].store_pid = fn;
      taskCount++;                       // increment task count
      ok = true;
    }
  }
  // REQUIRED: allow tasks switches again

  return ok;
*/
return approved;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm(" SVC #0x01");

    //code moved to svcISR
    /*
        struct semaphore *pSemaphore;
        uint8_t k,j,i = 0;
        while(i<MAX_TASKS)
        {
            if(tcb[i].pid == fn)
            {
                taskCount--;
                tcb[i].pid = NULL;
                pSemaphore = (struct semaphore*)tcb[i].semaphore;

                if(tcb[i].semaphore != NULL)                            //if the task was using semaphore
                {
                    pSemaphore->count++;                                //post operation
                    tcb[taskCurrent].semaphore = NULL;
                    if(pSemaphore->queueSize>0)
                    {
                        tcb[pSemaphore->processQueue[0]].state = STATE_READY;
                        uint8_t i = 0;
                        while(i<pSemaphore->queueSize-1)
                        {
                            pSemaphore->processQueue[i]=pSemaphore->processQueue[i+1];
                            i++;
                        }
                        pSemaphore->count--;
                        pSemaphore->queueSize--;
                    }
                }

                if(tcb[i].state == STATE_BLOCKED)
                {
                    tcb[i].semaphore = NULL;
                    for(k=0;k<pSemaphore->queueSize;k++)
                    {
                        if(tcb[i].number == pSemaphore->processQueue[k])        //removing the thread from semaphore queue
                        {
                            for(j=k; j<pSemaphore->queueSize-1;j++)
                                pSemaphore->processQueue[k] = pSemaphore->processQueue[k+1];
                            pSemaphore->queueSize--;
                        }
                    }
                }

                tcb[i].state = STATE_INVALID;                           //destroying thread
                tcb[i].semaphore = NULL;
                break;
            }
            i++;

        }

     */
}
    // REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
   __asm(" SVC #0x02");

    //code moved to svcISR
 /*
    uint8_t i = 0;
    while(i<MAX_TASKS)
    {
        if(tcb[i].pid == fn)
            tcb[i].currentPriority = priority;
        i++;
    }
 */
}

struct semaphore* createSemaphore(uint8_t count)
{
   __asm(" SVC #0x03");

  //code moved to svcISR
 /*
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
    pSemaphore->max_count = count;
    pSemaphore->queueSize = 0;
  }
  */
  return p_semaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
  __asm(" SVC #0x04");

    //code moved to svcISR
    /*
  // push registers, call scheduler, pop registers, return to new function
    tcb[taskCurrent].state = STATE_READY;
    NVIC_INT_CTRL_R |= 0x10000000;
*/
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm(" SVC #0x05");

    //code moved to scvISR
    /*
  // push registers, set state to delayed, store timeout, call scheduler, pop registers,
  // return to new function (separate unrun or ready processing)
    tcb[taskCurrent].ticks = tick;
    tcb[taskCurrent].state = STATE_DELAYED;
    NVIC_INT_CTRL_R |= 0x10000000;
  */
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC #0x06");
    //code moved to svcISR
    /*
    if(pSemaphore->count>0)
    {
        tcb[taskCurrent].semaphore = pSemaphore;
        pSemaphore->count--;
    }
    else
    {
        pSemaphore->processQueue[pSemaphore->queueSize] = tcb[taskCurrent].number;
        pSemaphore->queueSize += 1;
        tcb[taskCurrent].semaphore = pSemaphore;

        if(pi==true)
        {
            uint8_t i = 0;
            while((i<MAX_TASKS) && (tcb[i].semaphore == pSemaphore))
            {
                if((tcb[i].currentPriority < tcb[taskCurrent].currentPriority) && (tcb[i].state != STATE_BLOCKED))
                    tcb[i].currentPriority = tcb[taskCurrent].currentPriority;
                i++;
            }
        }

        tcb[taskCurrent].state = STATE_BLOCKED;
        NVIC_INT_CTRL_R |= 0x10000000;
    }
  */
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #0x07");

    //code moved to svcISR
    /*
    pSemaphore->count++;
    tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
    tcb[taskCurrent].semaphore = NULL;
    tcb[taskCurrent].state = STATE_READY;
    if(pSemaphore->queueSize>0)
    {
        tcb[pSemaphore->processQueue[0]].state = STATE_READY;
        uint8_t i = 0;
        while(i<pSemaphore->queueSize-1)
        {
            pSemaphore->processQueue[i]=pSemaphore->processQueue[i+1];
            i++;
        }
        pSemaphore->count--;
        pSemaphore->queueSize--;
        NVIC_INT_CTRL_R |= 0x10000000;
     }
  */
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
    ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    if(ok)                                                                                 //if found, see whether it can be skipped
    {
        if(tcb[task].skipCount >= tcb[task].currentPriority)            //if it cannot be skipped
        {
            tcb[task].skipCount = 0;
            goto end;                                                   //schedule it
        }
        else
        {
            tcb[task].skipCount++;
            ok = false;                                             //search for another task to schedule
        }
    }
  }
end:  return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t task = 0;
    while (task<10)
    {
        if(tcb[task].state == STATE_DELAYED)
        {
            tcb[task].ticks--;
            if(tcb[task].ticks == 0)
                tcb[task].state = STATE_READY;
        }
        task++;
    }

    if(preemp == true)
    {
    if(start)
    {
        if(tcb[taskCurrent].state == STATE_RUNNING)         //make it ready only if it running
            tcb[taskCurrent].state = STATE_READY;
        NVIC_INT_CTRL_R |= 0x10000000;
    }
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    __asm(" PUSH {R4-R11}");                //push the contents of the current task
    void *value, *temp;
    _fn fn;
    uint32_t xpsr;
    temp = getSP();
    tcb[taskCurrent].sp = temp;             //store the current stack pointer
    setSP(systemSP);                        //set the system sp
    insta_time = WTIMER5_TAV_R;                          // read counter input
    insta_time /= 40;
    calculate(insta_time);                  //store the value in tcb[taskCurrent].ticks
    taskCurrent = rtosScheduler();
    value = tcb[taskCurrent].sp;
    WTIMER5_TAV_R = 0;                                 //start counter for next task
    setSP(value);                           //next task sp value
//    WTIMER5_TAV_R = 0;                                 //start counter for next task

    if(tcb[taskCurrent].state == STATE_READY)
    {
        __asm(" POP {R4-R11}");
        tcb[taskCurrent].state = STATE_RUNNING;
    }

    if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        fn = (_fn)(tcb[taskCurrent].pid);
        xpsr = 0x41000000;
        pushContents(xpsr,fn);
        tcb[taskCurrent].state = STATE_RUNNING;
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    //this file is uncooperative rtos
    uint32_t R0_value, R1_value, R2_value;
    R0_value = getR0();         //extract values from stack
    R1_value = getR1();
    R2_value = getR2();

    void* ptr_sp;
    uint32_t nptr_sp,val;
    ptr_sp = getSP();           //extract sp value
    nptr_sp = (uint32_t)ptr_sp;
    nptr_sp += 80;              //move it to desired location
    val = getvalue(nptr_sp);
    val = val&0x000000FF;       //extract the svc number

    switch(val)
    {
    case 0:                 //create thread
    {
        _fn fn = (_fn)R0_value;                 //get _fn value passed
        char* name = (char*)R1_value;           //get name[] value passed
        uint8_t priority = R2_value;            //get priority value passed
        //code
        bool ok = false;
        uint8_t j,i = 0;
        bool found = false;

       // REQUIRED: store the thread name
       // add task if room in task list
       if (taskCount < MAX_TASKS)
       {
           // make sure fn not already in list (prevent reentrancy)
           while (!found && (i < MAX_TASKS))
           {
              found = (tcb[i++].pid ==  fn);
           }
           if (!found)
           {
              // find first available tcb record
              i = 0;
              while (tcb[i].state != STATE_INVALID) {i++;}
              tcb[i].state = STATE_UNRUN;
              tcb[i].pid = fn;
              tcb[i].sp = &stack[i][255];
              tcb[i].priority = priority;
              tcb[i].currentPriority = priority;
              tcb[i].skipCount = priority;
              tcb[i].number = i;
              tcb[i].store_pid = fn;
              for(j=0;j<strlen(name);j++)
                  tcb[i].name[j] = *(name+j);
              taskCount++;                       // increment task count
              ok = true;
            }
          }
          // REQUIRED: allow tasks switches again
          approved = ok;                        //for return variable
    }
        break;

    case 1:                         //destroy thread
    {
        _fn fn = (_fn)R0_value;             //extract pid of the function
        struct semaphore *pSemaphore;
        uint8_t k,j,i = 0;
        while(i<MAX_TASKS)
        {
            if(tcb[i].pid == fn)
            {
                taskCount--;
                tcb[i].pid = NULL;
                pSemaphore = (struct semaphore*)tcb[i].semaphore;
                if(tcb[i].semaphore != NULL)                            //if the task was using semaphore
                {
                    pSemaphore->count++;                                //post operation
                    tcb[taskCurrent].semaphore = NULL;
                    if(pSemaphore->queueSize>0)
                    {
                        tcb[pSemaphore->processQueue[0]].state = STATE_READY;
                        uint8_t i = 0;
                        while(i<pSemaphore->queueSize-1)
                        {
                            pSemaphore->processQueue[i]=pSemaphore->processQueue[i+1];
                            i++;
                        }
                        pSemaphore->count--;
                        pSemaphore->queueSize--;
                    }
                }

                if(tcb[i].state == STATE_BLOCKED)
                {
                    tcb[i].semaphore = NULL;
                    for(k=0;k<pSemaphore->queueSize;k++)
                    {
                        if(tcb[i].number == pSemaphore->processQueue[k])        //removing the thread from semaphore queue
                        {
                            for(j=k; j<pSemaphore->queueSize-1;j++)
                                pSemaphore->processQueue[k] = pSemaphore->processQueue[k+1];
                            pSemaphore->queueSize--;
                        }
                    }
                }

                tcb[i].state = STATE_INVALID;                           //destroying thread
                tcb[i].semaphore = NULL;
                break;
            }
            i++;
       }
    }
        break;

    case 2:                                     //set thread priority
    {
        _fn fn = (_fn)R0_value;                 //get the fn value passed
        uint8_t priority = R1_value;            //get the priority value passed
        uint8_t i = 0;
           while(i<MAX_TASKS)
           {
               if(tcb[i].pid == fn)
                   tcb[i].currentPriority = priority;       //set to the required priority
               i++;
           }
    }
        break;

    case 3:                                     //create semaphore
    {
        uint8_t count = R0_value;
        struct semaphore *pSemaphore = 0;
        if (semaphoreCount < MAX_SEMAPHORES)
        {
           pSemaphore = &semaphores[semaphoreCount++];
           pSemaphore->count = count;
           pSemaphore->max_count = count;
           pSemaphore->queueSize = 0;
        }
        p_semaphore = pSemaphore;               //for return variable
    }
        break;

    case 4:                                           //yield code
    {
        tcb[taskCurrent].state = STATE_READY;
        NVIC_INT_CTRL_R |= 0x10000000;             // set pendsv
    }
        break;

    case 5:                                     //sleep code
    {
        uint32_t tick = R0_value;                       //get the ticks value
        tcb[taskCurrent].ticks = tick;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |= 0x10000000;              //pendsv
    }
        break;

    case 6:                                     //wait code
    {
        struct semaphore *pSemaphore = (void*)R0_value;         //get the semaphore pointer
        if(pSemaphore->count>0)
            {
                tcb[taskCurrent].semaphore = pSemaphore;
                pSemaphore->count--;
            }
        else
        {
            pSemaphore->processQueue[pSemaphore->queueSize] = tcb[taskCurrent].number;
            pSemaphore->queueSize += 1;
            tcb[taskCurrent].semaphore = pSemaphore;

            if(pi==true)                    //priority inheritance code.
            {
                uint8_t i = 0;
                while((i<MAX_TASKS) && (tcb[i].semaphore == pSemaphore))
                {
                    if((tcb[i].currentPriority < tcb[taskCurrent].currentPriority) && (tcb[i].state != STATE_BLOCKED))
                        tcb[i].currentPriority = tcb[taskCurrent].currentPriority;
                    i++;
                }
            }

            tcb[taskCurrent].state = STATE_BLOCKED;
            NVIC_INT_CTRL_R |= 0x10000000;
       }
    }
        break;

    case 7:                     //post code
    {
        struct semaphore *pSemaphore = (void*)R0_value;     //get the semaphore pointer
        pSemaphore->count++;
        tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
        tcb[taskCurrent].semaphore = NULL;
        tcb[taskCurrent].state = STATE_READY;
        if(pSemaphore->queueSize>0)
        {
            tcb[pSemaphore->processQueue[0]].state = STATE_READY;
            uint8_t i = 0;
            while(i<pSemaphore->queueSize-1)
            {
                pSemaphore->processQueue[i]=pSemaphore->processQueue[i+1];
                i++;
            }
            pSemaphore->count--;
            pSemaphore->queueSize--;
            NVIC_INT_CTRL_R |= 0x10000000;
        }
    }
        break;

    case 8:                         //ps command in shell operation
    {
        float cpuTime;//total_time;
        uint32_t i=0, pidno;
        while(i<MAX_TASKS-1)
        {
            total_time += tcb[i].cpu_time;
            i++;
        }

        putsUart0("\r\n");
        putsUart0("PIDNO  \t");
        putsUart0("   STATE\t");
        putsUart0("CPU%\t");
        putsUart0("NAME\t");
        putsUart0("\r\n");

        i=0;
        while(i<MAX_TASKS-1)
        {
            putsUart0("\r\n");
            pidno = (uint32_t)tcb[i].pid;
            putnUart0(pidno);
            putsUart0("\t ");

            if(tcb[i].state==0)
                putsUart0("STATE INVALID");
            if(tcb[i].state==1)
                putsUart0("STATE UNRUN  ");
            if(tcb[i].state==2)
                putsUart0("STATE READY  ");
            if(tcb[i].state==3)
                putsUart0("STATE BLOCKED");
            if(tcb[i].state==4)
                putsUart0("STATE DELAYED");
            if(tcb[i].state==5)
                putsUart0("STATE RUNNING");

            putsUart0("\t");
            cpuTime = tcb[i].cpu_time*100/total_time;
            putfUart0(cpuTime);
            putsUart0("\t");

            putsUart0(tcb[i].name);

            i++;
        }
        putsUart0("\r\n\r\n");
        total_time = 0;
    }
        break;

    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, B, F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF;

    // Configure LED and pushbutton pins
    GPIO_PORTA_DIR_R = 0x1C;  // bits 7,6,5 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R = 0x1C; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0xFC;  // enable LEDs and pushbuttons PA 2,3,4
    GPIO_PORTA_PUR_R = 0xE0;  // enable internal pull-up for push button

    GPIO_PORTB_DIR_R = 0x40;  // bit 4 is output, other pins are inputs
    GPIO_PORTB_DR2R_R = 0x40; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R = 0x50;  // enable LEDs and pushbuttons PB6
    GPIO_PORTB_PUR_R = 0x10;  // enable internal pull-up for push button

    GPIO_PORTF_DIR_R = 0x04;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x14;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN ; // enable TX, RX, and module

    //timer configuration as up counter
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;    // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;              // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                            // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;   // configure for edge count mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;          //measure time from positive edge to positive edge
    WTIMER5_IMR_R = 0;                              //turn off interrupts
    WTIMER5_TAV_R = 0;                               //zero the counter
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                //turn on counter
    NVIC_EN3_R &= ~(1 << (INT_WTIMER5A-16-96));      // turn-off interrupt 120 (WTIMER5A)
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t ans=0;
        if(PUSH_BUTTON_P1 == 0)
            ans = 1;
        if(PUSH_BUTTON_P2 == 0)
            ans = 2;
        if(PUSH_BUTTON_P3 == 0)
            ans = 4;
        if(PUSH_BUTTON_P4 == 0)
            ans = 8;
        if(PUSH_BUTTON_P5 == 0)
            ans = 16;
      return ans;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}

void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}

void oneshot()
{
  while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
        createThread(flash4Hz, "Flash4Hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
    }
    if ((buttons & 16) != 0)
    {
      setThreadPriority(lengthyFn, 4);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}

void important()
{
    while(true)
    {
      wait(resource);
      BLUE_LED = 1;
      sleep(1000);
      BLUE_LED = 0;
      post(resource);
    }
}

void shell()
{
  while (true)
  {
    // REQUIRED: add processing for the shell commands through the UART here
    getsUart0(command);
    putsUart0("\n");
    transform_string();
    getCommand();
    isCommand();
  }
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//systick configuration function
void systick_init()
{
    NVIC_ST_CTRL_R = 0;
    NVIC_INT_CTRL_R |= 0x40000000;
    NVIC_ST_RELOAD_R |= 0x93CF;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
}

//to calculate cpu time and store it in tcb variable
void calculate(uint32_t val)
{
    uint32_t d_val;
    d_val = hexToDec(val);
    if(!first)
        tcb[taskCurrent].cpu_time = (alpha*tcb[taskCurrent].cpu_time)+(1-alpha)*d_val;
    else
    {
        first = false;
        tcb[taskCurrent].cpu_time = d_val;
    }
    return;
}

//to change the stack pointer
void setSP(uint8_t *value)
{
    __asm(" ADD R13, #8");
    __asm(" MOV R13,R0");
    __asm(" BX LR");
}

//to get the current stack pointer value
void* getSP()
{
    __asm(" MOV R0, R13");
    __asm(" BX LR");
    return NULL;               //unreachable code, written to avoid a warning
}

//to create stack for unrun thread
void pushContents(uint32_t xpsr, _fn pc)
{
    __asm(" PUSH {R0}");            //xpsr push
    __asm(" PUSH {R1}");            //pc push
    __asm(" MOVW R3, #0xFFF9");     //higher byte
    __asm(" MOVT R3, #0xFFFF");     //lower byte
    __asm(" PUSH {R3}");            //LR push
    __asm(" MOV R3, #1");
    __asm(" PUSH {R3}");            //R12 push
    __asm(" ADD R3, #1");
    __asm(" PUSH {R3}");            //R4 push
    __asm(" ADD R3, #1");
    __asm(" PUSH {R3}");            //R3 push
    __asm(" ADD R3, #1");
    __asm(" PUSH {R3}");            //R2 push
    __asm(" ADD R3, #1");
    __asm(" PUSH {R3}");            //R1 push
    __asm(" BX LR");
}

//to extract the pc value where svc number is passed
uint32_t getvalue(uint32_t num)
{
   __asm(" ADD R13,#8");
   __asm(" LDR R2,[R0]");
   __asm(" SUB R2,#2");
   __asm(" LDR R1,[R2]");
   __asm(" MOV R0,R1");
   __asm(" BX LR");
    return 0;                                   //unreachable code, written to avoid warning
}

// to extract the passed R0 value from the stack
uint32_t getR0()
{
    __asm(" BX LR");
    return 1;                                   //unreachable code, written to avoid warning
}

// to extract the passed R1 value from the stack
uint32_t getR1()
{
    __asm(" MOV R0,R1");
    __asm(" BX LR");
    return 1;                                   //unreachable code, written to avoid warning
}

// to extract the passed R2 value from the stack
uint32_t getR2()
{
    __asm(" MOV R0,R2");
    __asm(" BX LR");
    return 1;                               //unreachable code, written to avoid warning
}

//convert hexadecimal to decimal
uint32_t hexToDec(uint32_t value)
{
    uint8_t digit,power = 0;
    uint32_t result = 0;

    while(value!=0)
    {
        digit = value%10;
        result += digit*pow(16,power);
        power +=1;
        value /= 10;
    }
    return result;
}

// Blocking function that returns character with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        yield();
    return UART0_DR_R & 0xFF;
}

// Blocking function that returns string with serial data once the buffer is not empty
void getsUart0(char *o_string)                      //from my previous project
{
    uint32_t count = 0;
    char c;

getchar :   c = getcUart0();
            if(c==13)               //carriage return
            {
done :          o_string[count++] = 0;
                return;
            }
            if(c==8)            //backspace
            {
                if(count>0)
                    count--;
                goto getchar;
            }
            if((c==32) || (48<=c<=57) || (65<=c<=90) || (97<=c<=122))
            {
                o_string[count] = c;
                count++;
                if(count>= MAX_CHARS)
                    goto done;
                else goto getchar;
            }
            else
                goto getchar;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
    return;
}

// Blocking function that writes a interger when the UART buffer is not full
void putnUart0(uint32_t n)                          //from my previous project
{
    while (UART0_FR_R & UART_FR_TXFF);
    int8_t i,x;
    int8_t count = -1;
    char c, temp[100];
    if(n==0)
        putcUart0(48);
    while(n!=0)
    {
        x = n%10;
        x += 48;
        c = x;
        temp[++count] = c;
        n /= 10;
    }
    for(i=count;i>=0;i--)
        putcUart0(temp[i]);
    return;
}

//Blocking function that reverses a string
//NOTE: this code is extracted from geeksforgeeks website, link is provided below
//LINK: https://www.geeksforgeeks.org/convert-floating-point-number-string/
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

//Blocking function that converts interger to string
//NOTE: this code is extracted from geeksforgeeks website, link is provided below
//LINK: https://www.geeksforgeeks.org/convert-floating-point-number-string/
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

//Blocking function that writes a string when the UART buffer is not full
//NOTE: this code is extracted from geeksforgeeks website, link is provided below
//LINK: https://www.geeksforgeeks.org/convert-floating-point-number-string/
void putfUart0(float num)
{
    char res[10];
    uint8_t ipart,i;
    float fpart;

    ipart = (uint8_t)num;                   // Extract integer part
    fpart = num - (float)ipart;             // Extract floating part
    i = intToStr(ipart, res, 0);            // convert integer part to string
    res[i] = '.';                           // add dot
    fpart = fpart * pow(10, 3);
    intToStr((uint8_t)fpart, res + i + 1, 3);

    for(i=0;i<5;i++)
        putcUart0(res[i]);
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; str[i]!='\0'; i++)
        putcUart0(str[i]);
    return;
}

//Blocking function calculating fields and position vector
void transform_string()                                         //from my previous project
{
    uint8_t slength = strlen(command);
    uint8_t i=0,j=0;
    for(j=0;j<MAX_CHARS;j++)
        position[j] = 0;
    minArgs = 0;
    j=0;

    while(i<slength)
        {
            if(i==0)
            {
                if(command[i]==32)
                    i = i+1;
                else
                {
                    position[j]=0;
                    j++;
                    i++;
                    (minArgs)=(minArgs)+1;
                }
                continue;
            }
            if(command[i]==32)
            {
                command[i]='\0';
                i++;
                while(command[i]==32)
                {
                    command[i]='\0';
                    i = i+1;
                }
                 position[j]=i;
                 j = j+1;
                 minArgs = minArgs + 1;
             }
            i = i+1;
        }
        return;
}

//copy two strings
void strcp(char *s1, char *s2)                  //from my previous project
{
    int i=0;
    while(s2[i]!='\0')
    {
        s1[i]=s2[i];
        i++;
    }
    s1[i]='\0';
    return;
}

//extract command
void getCommand()                           //from my previous project
{
    uint8_t i,j;
    for(i=0;i<10;i++)
    {
        for(j=0;j<MAX_CHARS;j++)
            message[i][j] = '\0';
    }
    for(i=0;i<minArgs;i++)
        strcp(message[i],(command+position[i]));
    return;
}

//compare command
void isCommand()
{
    uint8_t i, k, priority;
    uint32_t pidno;
    bool valid_cmd = true;

    if(strcmp(message[0],"ps")==0)
        if(minArgs == 1)
        {
            __asm(" SVC #0x08");
        }
        else
            valid_cmd = false;

    else if(strcmp(message[0],"pidof")==0)
        if(minArgs == 2)
        {
            i=0;
            while(i<MAX_TASKS)
            {
                if(strcmp(message[1],tcb[i].name)==0)
                {
                    pidno = (uint32_t)tcb[i].pid;
                    putsUart0("PID number of this task = ");
                    putnUart0(pidno);
                    putsUart0("\r\n\r\n");
                    break;
                }
                i++;
                if(i==MAX_TASKS)
                    putsUart0("Sorry, entered task not found \r\n\r\n");
            }
        }
        else
            valid_cmd = false;

    else if(strcmp(message[0],"kill")==0)
        if(minArgs == 2)
        {
            pidno = atoi(message[1]);
            i=0;
            while(i<MAX_TASKS)
            {
                if(pidno==(uint32_t)tcb[i].pid)
                {
                    destroyThread((_fn)tcb[i].pid);
                    putsUart0("Entered task successfully destroyed \r\n\r\n");
                    break;
                }
                if(i==MAX_TASKS)
                    putsUart0("Sorry, entered PID not found \r\n\r\n");
                i++;
            }
        }
        else
           valid_cmd = false;

    else if(strcmp(message[0],"reboot")==0)
        if(minArgs == 1)
        {
           // HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            __asm("    .global _c_int00\n"
                      "    b.w     _c_int00");
        }
        else
            valid_cmd = false;

    else if(strcmp(message[0],"ipcs")==0)
        if(minArgs == 1)
        {
            putsUart0("\r\n");
            putsUart0("NUMBER\t");
            putsUart0("NAME       \t");
            putsUart0("MAX_COUNT\t");
            putsUart0("COUNT \t");
            putsUart0("QUEUE_SIZE\t");
            putsUart0("BLOCKED_TASK \t");
            putsUart0("\r\n");

            i=0;
            while(i<MAX_SEMAPHORES-1)
            {
                putsUart0("\r\n");
                putnUart0(i);
                putsUart0("\t");

                if(i==0)
                    putsUart0("keyPressed \t");
                if(i==1)
                    putsUart0("keyReleased\t");
                if(i==2)
                    putsUart0("Flashreq   \t");
                if(i==3)
                    putsUart0("resource   \t");

                putnUart0(semaphores[i].max_count);
                putsUart0("\t\t");
                putnUart0(semaphores[i].count);
                putsUart0("\t");
                putnUart0(semaphores[i].queueSize);
                putsUart0("\t\t");

                if(semaphores[i].queueSize>0)
                {
                    k=0;
                    while(k<semaphores[i].queueSize)
                    {
                        putsUart0(tcb[semaphores[i].processQueue[k]].name);
                        putsUart0(",");
                        k++;
                    }
                }
                else
                    putsUart0("none");

                i++;
              }
            putsUart0("\r\n\r\n");
        }

        else
            valid_cmd = false;

    else if(strcmp(message[0],"pi")==0)
        if(minArgs == 2)
        {
            if(strcmp(message[1],"on")==0)
            {
                pi = true;
                putsUart0("Priority Inheritance is turned 'ON' \r\n\r\n");
            }
            else if(strcmp(message[1],"off")==0)
            {
                pi = false;
                putsUart0("Priority Inheritance is turned 'OFF' \r\n\r\n");
            }
            else
                valid_cmd = false;
        }
        else
            valid_cmd = false;

    else if(message[0][strlen(message[0])-1]=='&')
        if(minArgs == 1)
        {
            message[0][strlen(message[0])-1] = '\0';
            i=0;
            while(i<MAX_TASKS)
            {
                if(strcmp(message[0],tcb[i].name)==0)
                {
                    priority = tcb[i].priority;
                    _fn fn = (_fn)tcb[i].store_pid;
                    createThread(fn, tcb[i].name, priority);
                    putsUart0("Thread is successfully created \r\n\r\n");
                }
                if(i==MAX_TASKS)
                    putsUart0("Sorry, entered thread not found \r\n\r\n");
                i++;
             }
         }
         else
         valid_cmd = false;

    else if(strcmp(message[0],"preemp")==0)
            if(minArgs == 2)
            {
                if(strcmp(message[1],"on")==0)
                {
                    preemp = true;
                    putsUart0("Preemptive is turned 'ON' \r\n\r\n");
                }
                else if(strcmp(message[1],"off")==0)
                {
                    preemp = false;
                    putsUart0("Preemptive is turned 'OFF' \r\n\r\n");
                }
                else
                    valid_cmd = false;
            }
            else
                valid_cmd = false;

    else
        valid_cmd = false;

    if(valid_cmd==false)
        putsUart0("ERROR: Command entered is incorrect \r\n\r\n");
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  bool ok;

  // Initialize hardware
  initHw();
  rtosInit();

  // Power-up flash
  GREEN_LED = 1;
  waitMicrosecond(250000);
  GREEN_LED = 0;
  waitMicrosecond(250000);


  // Initialize semaphores
  keyPressed = createSemaphore(1);
  keyReleased = createSemaphore(0);
  flashReq = createSemaphore(5);
  resource = createSemaphore(1);


  // Add required idle process
  ok =  createThread(idle, "Idle", 7);

  // Add other processes
  ok &= createThread(lengthyFn, "LengthyFn", 6);
  ok &= createThread(flash4Hz, "Flash4Hz", 2);
  ok &= createThread(oneshot, "OneShot", 2);
  ok &= createThread(readKeys, "ReadKeys", 6);
  ok &= createThread(debounce, "Debounce", 6);
  ok &= createThread(important, "Important", 0);
  ok &= createThread(uncooperative, "Uncoop", 5);
  ok &= createThread(shell, "Shell", 4);

  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
}


