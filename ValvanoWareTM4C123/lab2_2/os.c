// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
// Daniel Valvano
// January 29, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

   Programs 4.4 through 4.12, section 4.2

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include <stdlib.h>

#include "FIFO.h"
#include "LED.h"
#include "os.h"
#include "PLL.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
long StartCritical(void);
void EndCritical(long sr);
void StartOS(void);

#define MAXTHREADS  10        // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
  struct tcb *prev;  // doubly-linked
  uint32_t id;
  int32_t sleep;
};
typedef struct tcb tcbType;
tcbType tcbs[MAXTHREADS];
tcbType *RunPt;
int32_t Stacks[MAXTHREADS][STACKSIZE];
int numThreads = 0;
int currentId = 0;
tcbType *SleepHead = 0;
tcbType *SleepTail = 0;

uint32_t CountTimeSlice = 0; // increments every systick

#define OS_FIFOSIZE   128         // size of the FIFOs (must be power of 2)
#define OS_FIFOSUCCESS 1        // return value on success
#define OS_FIFOFAIL    0         // return value on failure
static Sema4Type fifoEmpty;
static Sema4Type fifoFull;
static Sema4Type fifoLock;
AddIndexFifo(OS, OS_FIFOSIZE, unsigned long, OS_FIFOSUCCESS, OS_FIFOFAIL)

static Sema4Type mailPost;
static Sema4Type mailRecv;
static Sema4Type mailLock;
static unsigned long Mail;

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 50 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 50 MHz
  //LED_Init();
  
  //Initialize PORTF for LEDs and Switches
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  volatile int delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R &= (~0x1F);    // 3) disable analog on PF4-0
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R |= 0x0E;         // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_DIR_R &= (~0x11);
  GPIO_PORTF_AFSEL_R &= (~0x1F);    // 6) disable alt funct on PF4-0
  GPIO_PORTF_PUR_R |= 0x11;         // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R |= 0x1F;         // 7) enable digital I/O on PF4-0
  
  //configure switch interrupts
  GPIO_PORTF_IS_R  &= ~0x11; //PF0,PF4 edge interrupts
  GPIO_PORTF_IBE_R &= ~0x11; //PF0,PF4 single edge interrupt
  GPIO_PORTF_IEV_R &= ~0x11; //PF0,PF4 trigger on falling edge (negative logic)
  GPIO_PORTF_ICR_R = 0x11;      // clear interrupt flag
  GPIO_PORTF_IM_R  |= 0x11;    //PF0,PF4 unmask
  
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  //NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x0000FFFF)|0xE0C00000; //set both systick and pendsv to pri 7
  
  //PF vector 46, interrupt bit 30
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // priority 5 
}

void SetInitialStack(int i){
  Stacks[i][0] = 0xDEADBEEF;
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // 
}

//******** OS_AddThread ***************
// add three foregound threads to the scheduler
// Inputs: three pointers to a void/void foreground tasks
// Outputs: 1 if successful, 0 if this thread can not be added
int OS_AddThreads(void(*task0)(void),
                 void(*task1)(void),
                 void(*task2)(void)){ int32_t status;
  status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[0]; // 2 points to 0
  SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(task0); // PC
  SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(task1); // PC
  SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(task2); // PC
  RunPt = &tcbs[0];       // thread 0 will run first
  numThreads = 3;
  EndCritical(status);
  return 1;               // successful
}

int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority){
  int32_t status;
  status = StartCritical();
  SetInitialStack(numThreads); Stacks[numThreads][STACKSIZE-2] = (int32_t)(task); // set PC
  tcbs[numThreads].id = currentId;
  tcbs[numThreads].sleep = 0;
  
  if(numThreads>0 && numThreads<MAXTHREADS){
    tcbs[numThreads-1].next = &tcbs[numThreads];
    tcbs[numThreads].next = &tcbs[0];
    tcbs[numThreads].prev = &tcbs[numThreads-1];
    tcbs[0].prev = &tcbs[numThreads]; // 2 points to 0
  } else if(numThreads == 0) {
    tcbs[0].next = &tcbs[0];
    tcbs[0].prev = &tcbs[0];
    RunPt = &tcbs[0]; //init runpt
  } else {
    return 0;
  }
  numThreads++;
  currentId++;
  EndCritical(status);
  return 1;               // successful
}

///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}

void OS_Suspend(void){
  //1 give full time slice for next thread
  //NVIC_ST_CURRENT_R = 0;
  //2 trigger systick
  NVIC_INT_CTRL_R |= 0x04000000;
  //2 trigger pendsv
  //NVIC_INT_CTRL_R |= 0x10000000;
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  OS_DisableInterrupts();
  //Remove thread from linked list
  RunPt->prev->next = RunPt->next;
  RunPt->next->prev = RunPt->prev;
  //2 trigger pendsv, context switch
  NVIC_INT_CTRL_R |= 0x10000000;
  //RunPt = RunPt->next;
 
  OS_EnableInterrupts();
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
  int32_t status; status = StartCritical();
  
  //check if sleeptime > 0 and not running single thread
  if(sleepTime > 0 && RunPt->next != RunPt){
    tcbType *thisTcb = RunPt;
    thisTcb->sleep = sleepTime;
    //Remove thread from current run list
    thisTcb->prev->next = thisTcb->next;
    thisTcb->next->prev = thisTcb->prev;
  }
  
  //trigger pendsv, contex switch
  //NVIC_INT_CTRL_R |= 0x10000000;
  EndCritical(status);
}

unsigned long OS_Id(void){
  return RunPt->id;
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads

void(*PeriodicTask)(void);

int OS_AddPeriodicThread(void(*task)(void),unsigned long period, unsigned long priority){
  SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
  PeriodicTask = task;          // user function
  TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAILR_R = period-1;    // 4) reload value
  TIMER3_TAPR_R = 0;            // 5) bus clock resolution
  TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
  TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
//  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x80000000; // 8) priority 4
	NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|(priority<<29); // 0x80000000/4 = 2^29
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
  TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
	return 0;
}

void Timer3A_Handler(void){
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
  (*PeriodicTask)();                // execute user task
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads

void(*SW1Task)(void) = 0;

int OS_AddSW1Task(void(*task)(void), unsigned long priority){
  int32_t status; status = StartCritical();
  //ignore priority for now
  if(SW1Task){
    EndCritical(status);
    return 0;
  }else{
    SW1Task = task;
    NVIC_EN0_R |= 0x40000000; //enable interrupt 30 in NVIC for PortF
    EndCritical(status);
    return 1;
  }
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads

void(*SW2Task)(void) = 0;

int OS_AddSW2Task(void(*task)(void), unsigned long priority){
  int32_t status; status = StartCritical();
  //ignore priority for now
  if(SW2Task){
    EndCritical(status);
    return 0;
  }else{
    SW2Task = task;
    NVIC_EN0_R |= 0x40000000; //enable interrupt 30 in NVIC for PortF
    EndCritical(status);
    return 1;
  }
}

void GPIOPortF_Handler(void){
  GPIO_PORTF_ICR_R = 0x11;      // acknowledge flag4
  int SW1pressed=0; int SW2pressed=0;
  
  int value = GPIO_PORTF_DATA_R; //check switch values
  SW1pressed = (~value) & 0x10; //PF4
  SW2pressed = (~value) & 0x01; //PF0
  
  if(SW1Task && SW1pressed)
    SW1Task();
  
  if(SW2Task && SW2pressed)
    SW2Task();
}

void OS_InitSemaphore(Sema4Type *semaPt, long value) {
    if(semaPt != NULL)
        semaPt->Value = value;
}

void OS_Fifo_Init(unsigned long size) { //size is ignored for lab 2
    OS_InitSemaphore(&fifoFull, OS_FIFOSIZE);
    OS_InitSemaphore(&fifoEmpty, -1);
    OS_InitSemaphore(&fifoLock, 0);
    OSFifo_Init();
}

int OS_Fifo_Put(unsigned long data) {
    int ret;

    OS_Wait(&fifoFull);
    OS_bWait(&fifoLock);
    ret = OSFifo_Put(data);
    OS_bSignal(&fifoLock);
    OS_Signal(&fifoEmpty);

    return ret;
}

unsigned long OS_Fifo_Get(void) {
    unsigned long ret;

    OS_Wait(&fifoEmpty);
    OS_bWait(&fifoLock);
    OSFifo_Get(&ret);
    OS_bSignal(&fifoLock);
    OS_Signal(&fifoFull);

    return ret;
}

long OS_Fifo_Size(void) {
    return OSFifo_Size();
}

void OS_MailBox_Init(void) {
    OS_InitSemaphore(&mailPost, -1);
    OS_InitSemaphore(&mailRecv, 0);
    OS_InitSemaphore(&mailLock, 0);
}

void OS_MailBox_Send(unsigned long data) {
    OS_bWait(&mailRecv);
    OS_bWait(&mailLock);
    Mail = data;
    OS_bSignal(&mailLock);
    OS_bSignal(&mailPost);
}

unsigned long OS_MailBox_Recv(void) {
    unsigned long ret;

    OS_bWait(&mailPost);
    OS_bWait(&mailLock);
    ret = Mail;
    OS_bSignal(&mailLock);
    OS_bSignal(&mailRecv);

    return ret;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
  uint32_t value; int32_t status;
  status = StartCritical();
  value = NVIC_ST_RELOAD_R - NVIC_ST_CURRENT_R + (NVIC_ST_RELOAD_R + 1)*CountTimeSlice;
  EndCritical(status);
  return value;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
  return stop - start;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  int32_t status; status = StartCritical();
  CountTimeSlice = 0;
  EndCritical(status);
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){
  //assuming 80MHz
  return CountTimeSlice*(NVIC_ST_RELOAD_R+1)/TIME_1MS;
}
