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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "../inc/tm4c123gh6pm.h"
#include "../inc/hw_memmap.h"
#include "../driverlib/sysctl.h"
#include "../driverlib/gpio.h"

#include "FIFO.h"
#include "LED.h"
#include "os.h"
#include "PLL.h"
#include "UART.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

#define DEBUG 1

// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
long StartCritical(void);
void EndCritical(long sr);
void StartOS(void);

#define MAXTHREADS  20        // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack
#define SWPRI -8

tcbType tcbs[MAXTHREADS];
tcbType *RunPt;
int32_t Stacks[MAXTHREADS][STACKSIZE];
int numThreads = 0;
int currentId = 0;

#if DEBUG
unsigned static long time11 = 0;  // time at previous ADC sample
unsigned long time21 = 0;         // time at current ADC sample
long MaxJitter1;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize1=JITTERSIZE;
unsigned long JitterHistogram1[JITTERSIZE]={0,};

unsigned static long time12 = 0;  // time at previous ADC sample
unsigned long time22 = 0;         // time at current ADC sample
long MaxJitter2;             // largest time jitter between interrupts in usec
unsigned long const JitterSize2=JITTERSIZE;
unsigned long JitterHistogram2[JITTERSIZE]={0,};
#endif

//"bootstraps" the next pointer for context switch
tcbType dummyTcb;

//similar to dummyTcb
tcbType sleepyHead;
tcbType sleepyTail;

tcbType *runHead = 0;
tcbType *runTail = 0;

volatile uint32_t CountTimeSlice = 0; // increments every systick
volatile uint32_t sysTime = 0;

#define OS_FIFOSIZE    4         // size of the FIFOs (must be power of 2)
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

static void PortB_Init(void);

static void SW1TaskWrapper(void);
static void SW2TaskWrapper(void);

void BlockThread(Sema4Type *sema);
void UnblockThread(Sema4Type *sema);

void InitAllTCBs(void){
  for(int k=0; k<MAXTHREADS; k++){
    tcbs[k].sp = 0;
    tcbs[k].next = 0;
    tcbs[k].prev = 0;
    tcbs[k].id = 666666;
    tcbs[k].sleep = 0;
    tcbs[k].active = 0;
  }
}

void InitSleepTCB(void){
  sleepyHead.id = 0xBEA7BEEF;
  sleepyTail.id = 0xBAADDEED;
  sleepyHead.prev = 0;
  sleepyHead.next = &sleepyTail;
  sleepyTail.prev = &sleepyHead;
  sleepyTail.next = 0;
}

void OS_InitSysTimer(void){
  SYSCTL_RCGCTIMER_R |= 0x10;   // 0) activate TIMER4
  volatile int delay = SYSCTL_RCGCTIMER_R;
  TIMER4_CTL_R = 0x00000000;    // 1) disable TIMER4A during setup
  TIMER4_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER4_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  //TIMER4_TAILR_R = period-1;    // 4) reload value
  TIMER4_TAILR_R = TIME_1MS-1;    // 4) reload value
  TIMER4_TAPR_R = 0;            // 5) bus clock resolution
  TIMER4_ICR_R = 0x00000001;    // 6) clear TIMER4 timeout flag
  TIMER4_IMR_R = 0x00000001;    // 7) arm timeout interrupt
	NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)|(5<<21); // 21 = (70%4)*8+5, 17 = 70/4
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN2_R |= 1<<(70%32);      // 9) enable IRQ 86 in NVIC; 2 = 70/32 = 2, 2*32 = 64; 70 = 86 - 16
  //TIMER4_CTL_R = 0x00000001;    // 10) enable TIMER3A
}

static void PortB_Init(void) {
  SYSCTL_RCGCGPIO_R |= 0x02;       // activate port B
  volatile int delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTB_DIR_R |= 0x0F;    // make PB3-0 output heartbeats
  GPIO_PORTB_AFSEL_R &= ~0x0F;   // disable alt funct on PB3-0
  GPIO_PORTB_DEN_R |= 0x0F;     // enable digital I/O on PB3-0
  GPIO_PORTB_PCTL_R = GPIO_PORTB_PCTL_R & ~0x0000FFFF;
  GPIO_PORTB_AMSEL_R &= ~0x0F;;      // disable analog functionality on PB
}

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 50 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 50 MHz
  InitAllTCBs();
  InitSleepTCB();
  OS_InitSysTimer();
  
  //LED_Init();
  UART_Init();
  
  //Initialize PORTF for LEDs and Switches
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  volatile int delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R &= (~0x1F);    // 3) disable analog on PF4-0
  GPIO_PORTF_PCTL_R = GPIO_PORTF_PCTL_R & ~0x000FFFFF;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R |= 0x0E;         // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_DIR_R &= (~0x11);
  GPIO_PORTF_AFSEL_R &= (~0x1F);    // 6) disable alt funct on PF4-0
  GPIO_PORTF_PUR_R |= 0x11;         // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R |= 0x1F;         // 7) enable digital I/O on PF4-0
  
#if DEBUG
  PortB_Init();
#endif

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
//  SetInitialStack(numThreads); Stacks[numThreads][STACKSIZE-2] = (int32_t)(task); // set PC
//  tcbs[numThreads].id = currentId;
//  tcbs[numThreads].sleep = 0;
  
  if(numThreads>0 && numThreads<MAXTHREADS){
//    tcbs[numThreads-1].next = &tcbs[numThreads];
//    tcbs[numThreads].next = &tcbs[0];
//    tcbs[numThreads].prev = &tcbs[numThreads-1];
//    tcbs[0].prev = &tcbs[numThreads]; // 2 points to 0
    
    //find open tcb spot
    int spot;
    for(int k=0; k<MAXTHREADS; k++){
      if(!(tcbs[k].active)){
        spot = k;
        break;
      }
    }
    
    SetInitialStack(spot);
    Stacks[spot][STACKSIZE-2] = (int32_t)(task);
    //fix linked list
    tcbs[spot].next = runHead;
    tcbs[spot].prev = runTail;
    runTail->next = &tcbs[spot];
    runHead->prev = &tcbs[spot];
    //set runTail to current thread
    runTail = &tcbs[spot];
    //other init
    tcbs[spot].active = 1;
    tcbs[spot].sleep = 0;
    tcbs[spot].id = currentId;
    tcbs[spot].pri = priority;
    tcbs[spot].blocked = 0;
    tcbs[spot].bNext = NULL;
  } else if(numThreads == 0) {
    SetInitialStack(0);
    Stacks[0][STACKSIZE-2] = (int32_t)(task);
    tcbs[0].next = &tcbs[0];
    tcbs[0].prev = &tcbs[0];
    runHead = &tcbs[0];
    runTail = &tcbs[0];
    tcbs[0].active = 1;
    tcbs[0].sleep = 0;
    tcbs[0].id = 0;
    tcbs[0].pri = priority;
    tcbs[0].blocked = 0;
    tcbs[0].bNext = NULL;
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
  TIMER4_CTL_R = 0x00000001;    // 10) enable TIMER4A
  StartOS();                   // start on the first task
}

void OS_Suspend(void){
  //1 give full time slice for next thread
  //NVIC_ST_CURRENT_R = 0;
  //2 trigger systick
  //NVIC_INT_CTRL_R |= 0x04000000;
  //2 trigger pendsv
  NVIC_INT_CTRL_R |= 0x10000000;
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
  //Set tcb to be available to use
  RunPt->active = 0;
  //Fix head and tail
  if(RunPt == runHead)
    runHead = RunPt->next;
  if(RunPt == runTail)
    runTail = RunPt->prev;
  
  numThreads--;
  //2 trigger pendsv, context switch
  NVIC_INT_CTRL_R |= 0x10000000;
 
  OS_EnableInterrupts();
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
  //int32_t status; status = StartCritical();
  OS_DisableInterrupts();
  //check if sleeptime > 0 and not running single thread
  if(sleepTime > 0 && RunPt->next != RunPt){
    tcbType *thisTcb = RunPt;
    thisTcb->sleep = sleepTime;
  }
  
  //trigger pendsv, contex switch
  NVIC_INT_CTRL_R |= 0x10000000;
  OS_EnableInterrupts();
  //EndCritical(status);
}

void Timer4A_Handler(void){
  TIMER4_ICR_R = TIMER_ICR_TATOCINT; //acknowledge interrupt
  int32_t status; status = StartCritical();
  sysTime++;
  
  tcbType *t;
  for(t = RunPt;
      t->next != RunPt;
      t = t->next)
      if(t->sleep > 0)
          t->sleep--;

  if(t->sleep > 0)
      t->sleep--;


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

int PeriodicTaskPeriod;
void(*PeriodicTask)(void);
void(*PeriodicTask2)(void);

#if DEBUG
void PeriodicTaskWrapper() {
  time21 = OS_Time();
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
  (*PeriodicTask)();                // execute user task
  unsigned long deltaT = OS_TimeDifference(time11, time21);
  unsigned long jitter;
  if(deltaT>PeriodicTaskPeriod)
    jitter = (deltaT-PeriodicTaskPeriod+4)/8;  // in 0.1 usec
  else
    jitter = (PeriodicTaskPeriod-deltaT+4)/8;  // in 0.1 usec
  if(jitter > MaxJitter1)
    MaxJitter1 = jitter; // in usec
  if(jitter >= JitterSize1)
    jitter = JITTERSIZE-1;
  JitterHistogram1[jitter]++;
}
#endif

void Timer3_Init(unsigned long period, unsigned long priority){
  SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
  volatile int delay = SYSCTL_RCGCTIMER_R;
  TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAILR_R = period-1;    // 4) reload value
  TIMER3_TAPR_R = 0;            // 5) bus clock resolution
  TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
  TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
	NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|(priority<<29); // 0x80000000/4 = 2^29
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN1_R |= 1<<(35-32);      // 9) enable IRQ 35 in NVIC
}

void Timer5_Init(unsigned long period, unsigned long priority){
  SYSCTL_RCGCTIMER_R |= 0x20;   // 0) activate TIMER5
  volatile int delay = SYSCTL_RCGCTIMER_R;
  TIMER5_CTL_R = 0x00000000;    // 1) disable TIMER4A during setup
  TIMER5_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER5_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER5_TAILR_R = period-1;    // 4) reload value
  TIMER5_TAPR_R = 0;            // 5) bus clock resolution
  TIMER5_ICR_R = 0x00000001;    // 6) clear TIMER4 timeout flag
  TIMER5_IMR_R = 0x00000001;    // 7) arm timeout interrupt
	NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFFFF00)|(priority<<5); //23 = 92/4, 5 = (92%4)*8+5
// interrupts enabled in the main program after all devices initialized
  NVIC_EN2_R |= 1<<(92%32);      // 9) enable IRQ 108/92 in NVIC; 2 = 92/32 = 2, 2*32 = 64; 70 = 86 - 16
}

int OS_AddPeriodicThread(void(*task)(void),unsigned long period, unsigned long priority){
/*
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
  NVIC_EN1_R |= 1<<(35-32);      // 9) enable IRQ 35 in NVIC

  PeriodicTaskPeriod = period;

  TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
	return 0;
*/
  if(PeriodicTask == 0){
    PeriodicTask = task;          // user function
    Timer3_Init(period, priority);
    PeriodicTaskPeriod = period;
    TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
    return 0;
  } else if(PeriodicTask2 == 0){
    PeriodicTask2 = task;
    Timer5_Init(period, priority);
    //PeriodicTaskPeriod = period;
    TIMER5_CTL_R = 0x00000001;    // 10) enable TIMER3A
    return 0;
  } else {
    return 1;
  }
}

void Timer3A_Handler(void){
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
#if DEBUG
  PeriodicTaskWrapper();
#else
  (*PeriodicTask)();                // execute user task
#endif
}

void Timer5A_Handler(void){
  TIMER5_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER5A timeout
// #if DEBUG
//  PeriodicTaskWrapper();
// #else
  (*PeriodicTask2)();                // execute user task
// #endif
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
long SW1TaskPri;

static void SW1TaskWrapper(void) {
    if(SW1Task != NULL)
        SW1Task();
    OS_Kill();
}

int OS_AddSW1Task(void(*task)(void), unsigned long priority){
  int32_t status; status = StartCritical();
  //ignore priority for now
  if(SW1Task){
    EndCritical(status);
    return 0;
  }else{
    SW1Task = task;
    SW1TaskPri = (signed long)(priority) + SWPRI;
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
long SW2TaskPri;

static void SW2TaskWrapper(void) {
    if(SW2Task != NULL)
        SW2Task();
    OS_Kill();
}

int OS_AddSW2Task(void(*task)(void), unsigned long priority){
  int32_t status; status = StartCritical();
  //ignore priority for now
  if(SW2Task){
    EndCritical(status);
    return 0;
  }else{
    SW2Task = task;
    SW2TaskPri = (signed long)(priority) + SWPRI;
    NVIC_EN0_R |= 0x40000000; //enable interrupt 30 in NVIC for PortF
    EndCritical(status);
    return 1;
  }
}

void GPIOPortF_Handler(void){
    if(GPIO_PORTF_MIS_R & 0x10) {
      GPIO_PORTF_ICR_R = 0x10;
      if(SW1Task != NULL) {
          SW1Task();
          //2 trigger pendsv
          NVIC_INT_CTRL_R |= 0x10000000;
      }
    }
        //OS_AddThread(&SW1TaskWrapper, 128 , SW1TaskPri);

    if(GPIO_PORTF_MIS_R & 0x01) {
      GPIO_PORTF_ICR_R = 0x01;
      if(SW2Task != NULL) {
          SW2Task();
          //2 trigger pendsv
          NVIC_INT_CTRL_R |= 0x10000000;
      }
    }
}

void OS_InitSemaphore(Sema4Type *semaPt, long value) {
    if(semaPt != NULL) {
        semaPt->Value = value;
        semaPt->next = NULL;
    }
}

void OS_Fifo_Init(unsigned long size) { //size is ignored for lab 2
    unsigned long fifoSize;

    if(size <= 128 && size > 0)
        fifoSize = size;
    else
        fifoSize = 128;
    OS_InitSemaphore(&fifoFull, fifoSize - 1);
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
  unsigned long value; int32_t status;
  status = StartCritical();
  //value = NVIC_ST_RELOAD_R - NVIC_ST_CURRENT_R + (NVIC_ST_RELOAD_R + 1)*CountTimeSlice;
  value = TIMER4_TAILR_R - TIMER4_TAV_R + sysTime*TIME_1MS;
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
  //CountTimeSlice = 0;
  sysTime = 0;
  EndCritical(status);
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){
  //return CountTimeSlice*(NVIC_ST_RELOAD_R+1)/TIME_1MS;
  //return CountTimeSlice;
  return sysTime;
}

void BlockThread(Sema4Type *sema) {
    OS_DisableInterrupts();

    tcbType *t = sema->next;
    if(t) {
      while(t->bNext)
        t = t->bNext;
      t->bNext = RunPt;
    } else
      sema->next = RunPt;

    RunPt->blocked = 1;
    RunPt->bNext = 0;

    //2 trigger pendsv
    NVIC_INT_CTRL_R |= 0x10000000;
    OS_EnableInterrupts();
}

void OS_bSignal(Sema4Type *semaPt) {
    OS_DisableInterrupts();
    tcbType *t = semaPt->next;
    if(t != NULL) {
        semaPt->next = semaPt->next->bNext;
        t->bNext = NULL;
        t->blocked = 0;
    } else
      semaPt->Value = 0;

    //2 trigger pendsv
    NVIC_INT_CTRL_R |= 0x10000000;
    OS_EnableInterrupts();
}

void OS_Signal(Sema4Type *semaPt) {
    OS_DisableInterrupts();
    tcbType *t = semaPt->next;
    if(t != NULL) {
        semaPt->next = semaPt->next->bNext;
        t->bNext = NULL;
        t->blocked = 0;
    } else
      ++semaPt->Value;

    //2 trigger pendsv
    NVIC_INT_CTRL_R |= 0x10000000;
    OS_EnableInterrupts();
}
