// os.c
// Dung Nguyen & Nico Cortes
// Mar 25 2017

#pragma import(__use_no_semihosting)

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../inc/tm4c123gh6pm.h"

#include "FIFO.h"
#include "LED.h"
#include "os.h"
#include "PLL.h"
#include "UART.h"
#include "ff.h"
#include "loader.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

#define TRIGGER_SYSTICK()       (NVIC_INT_CTRL_R |= 0x04000000)
#define TRIGGER_PENDSV()        (NVIC_INT_CTRL_R |= 0x10000000)

// function definitions in osasm.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCriticalAsm(void);
void EndCriticalAsm(long sr);
void StartOS(void);
void JumpAsm(void);

#define MAXTHREADS  20        // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack
#define SWPRI -8

tcbType tcbs[MAXTHREADS];
tcbType *RunPt;
int32_t Stacks[MAXTHREADS][STACKSIZE];
int numThreads = 0;
int currentId = 0;

#if DEBUG
/* JITTER */
static long MaxJitter;
unsigned static long time11 = 0;  // time at previous ADC sample
unsigned long        time21 = 0;  // time at current ADC sample
long MaxJitter1;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize1=JITTERSIZE;
unsigned long JitterHistogram1[JITTERSIZE]={0,};
unsigned static long time12 = 0;  // time at previous ADC sample
unsigned long        time22 = 0;  // time at current ADC sample
long MaxJitter2;             // largest time jitter between interrupts in usec
unsigned long const JitterSize2=JITTERSIZE;
unsigned long JitterHistogram2[JITTERSIZE]={0,};
Sema4Type jitterLock;

static void Jitter_Init(void);

/* TIMING */
static int IntsEnabled = 0;
static int MasterTime = 0;
static int TmpTimeIntsDisabled;
static int MaxTimeIntsDisabled = 0;
static int TimeIntsDisabled = 0;
static int PercentIntsDisabled = 0;
static unsigned long GetMasterTime(void);

/* THREAD INFO */
#define TINFO_BUF_SIZE 100
struct tinfo tInfoBuf[TINFO_BUF_SIZE];

int tinfoBufI = 0;

void addTInfo(enum tEvent e) {
    if(tinfoBufI >= 100)
        return;
    tInfoBuf[tinfoBufI].tid = OS_Id();
    tInfoBuf[tinfoBufI].e = e;
    tInfoBuf[tinfoBufI].t = GetMasterTime();
    tinfoBufI++;
}
#endif

//"bootstraps" the next pointer for context switch
tcbType dummyTcb;
tcbType *runHead = 0;
tcbType *runTail = 0;

volatile uint32_t CountTimeSlice = 0; // increments every systick
volatile uint32_t sysTime = 0;

/* FIFO */
#define OS_FIFOSIZE    128         // size of the FIFOs (must be power of 2)
#define OS_FIFOSUCCESS 1        // return value on success
#define OS_FIFOFAIL    0         // return value on failure
static Sema4Type fifoEmpty;
AddIndexFifo(OS, OS_FIFOSIZE, unsigned long, OS_FIFOSUCCESS, OS_FIFOFAIL)

/* MAILBOX */
static Sema4Type mailPost;
static Sema4Type mailRecv;
static Sema4Type mailLock;
static unsigned long Mail;

static void PortB_Init(void);

/* SEMAPHORES */
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

static void PortF_Init(void) {
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

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 50 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 50 MHz
  InitAllTCBs();
  OS_InitSysTimer();
  UART_Init();
  PortF_Init();
  #if DEBUG
  Jitter_Init();
  PortB_Init();
  #endif
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
  if(numThreads>0 && numThreads<MAXTHREADS){
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
  OS_DisableInterrupts();
  //check if sleeptime > 0 and not running single thread
  if(sleepTime > 0 && RunPt->next != RunPt){
    tcbType *thisTcb = RunPt;
    thisTcb->sleep = sleepTime;
  }
  
  TRIGGER_PENDSV();
  OS_EnableInterrupts();
}

void Timer4A_Handler(void){
  TIMER4_ICR_R = TIMER_ICR_TATOCINT; //acknowledge interrupt
  int32_t status; status = StartCritical();
  sysTime++;
  #if DEBUG
  MasterTime++;
  #endif
  tcbType *t;
  for(t = RunPt; t->next != RunPt; t = t->next)
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

int PeriodicTaskPeriod1;
int PeriodicTaskPeriod2;
void(*PeriodicTask1)(void);
void(*PeriodicTask2)(void);

#if DEBUG
void PeriodicTaskWrapper1() {
  time21 = OS_Time();
  (*PeriodicTask1)();                // execute user task
  if(time11 != 0) {
      unsigned long deltaT = OS_TimeDifference(time11, time21);
      unsigned long jitter;

      if(deltaT>PeriodicTaskPeriod1)
        jitter = (deltaT-PeriodicTaskPeriod1+4)/8;  // in 0.1 usec
      else
        jitter = (PeriodicTaskPeriod1-deltaT+4)/8;  // in 0.1 usec
      if(jitter > MaxJitter1)
        MaxJitter1 = jitter; // in usec
      if(MaxJitter1 > MaxJitter)
          MaxJitter = MaxJitter1;
      if(jitter >= JitterSize1)
        jitter = JITTERSIZE-1;
      JitterHistogram1[jitter]++;
  }
  time11 = time21;
}
#endif

#if DEBUG
void PeriodicTaskWrapper2() {
  time22 = OS_Time();
  (*PeriodicTask2)();                // execute user task
  if(time12 != 0) {
      unsigned long deltaT = OS_TimeDifference(time12, time22);
      unsigned long jitter;

      if(deltaT>PeriodicTaskPeriod2)
        jitter = (deltaT-PeriodicTaskPeriod2+4)/8;  // in 0.1 usec
      else
        jitter = (PeriodicTaskPeriod2-deltaT+4)/8;  // in 0.1 usec
      if(jitter > MaxJitter2)
        MaxJitter2 = jitter; // in usec
      if(MaxJitter2 > MaxJitter)
        MaxJitter = MaxJitter2; // in usec
      if(jitter >= JitterSize2)
        jitter = JITTERSIZE-1;
      JitterHistogram2[jitter]++;
  }
  time12 = time22;
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

static void Timer5_Init(unsigned long period, unsigned long priority){
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
  if(PeriodicTask1 == 0){
    PeriodicTask1 = task;          // user function
    Timer3_Init(period, priority);
    PeriodicTaskPeriod1 = period;
    TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
    return 0;
  } else if(PeriodicTask2 == 0){
    PeriodicTask2 = task;
    Timer5_Init(period, priority);
    PeriodicTaskPeriod2 = period;
    TIMER5_CTL_R = 0x00000001;    // 10) enable TIMER3A
    return 0;
  } else {
    return 1;
  }
}

void Timer3A_Handler(void){
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
  #if DEBUG
  addTInfo(pStart);
  PeriodicTaskWrapper1();
  addTInfo(pStop);
  #else
  (*PeriodicTask1)();                // execute user task
  #endif
}
/*
void Timer5A_Handler(void){
  TIMER5_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER5A timeout
  #if DEBUG
  addTInfo(pStart);
  PeriodicTaskWrapper2();
  addTInfo(pStop);
  #else
  (*PeriodicTask2)();                // execute user task
  #endif
}
*/

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

void OS_Fifo_Init(unsigned long size){
  OS_InitSemaphore(&fifoEmpty, -1);
  OSFifo_Init();
}

int OS_Fifo_Put(unsigned long data) {
  int ret;
  long sav = StartCritical();
  ret = OSFifo_Put(data);
  EndCritical(sav);
  return ret;
}

unsigned long OS_Fifo_Get(void) {
  unsigned long ret;
  long sav = StartCritical();
  OSFifo_Get(&ret);
  EndCritical(sav);
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
  value = TIMER4_TAILR_R - TIMER4_TAV_R + sysTime*TIME_1MS;
  EndCritical(status);
  return value;
}

#if DEBUG
static unsigned long GetMasterTime(void) {
    return TIMER4_TAILR_R - TIMER4_TAV_R + MasterTime*TIME_1MS;
}
#endif

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
  sysTime = 0;
  EndCritical(status);
}

#if DEBUG
int OS_MaxTimeIntsDisabled(void) {
    return MaxTimeIntsDisabled;
}

int OS_TimeIntsDisabled(void) {
    return TimeIntsDisabled;
}

int OS_PercentIntsDisabled(void) {
    return PercentIntsDisabled;
}
#endif

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){
  return sysTime;
}

void OS_bWait(Sema4Type *sema) {
  long crit = StartCritical();
  //OS_DisableInterrupts();
  if(sema->Value < 0){
    BlockThread(sema);
  }
  sema->Value = -1;
  //OS_EnableInterrupts();
  EndCritical(crit);
}

void OS_Wait(Sema4Type *sema) {
  //OS_DisableInterrupts();
  long crit = StartCritical();
  if(sema->Value < 0){
    BlockThread(sema);
  } else {
    sema->Value--;
  }
  //OS_EnableInterrupts();
  EndCritical(crit);
}

void BlockThread(Sema4Type *sema) {
  //OS_DisableInterrupts();
  tcbType *t = sema->next;
  if(t) {
    while(t->bNext)
      t = t->bNext;
    t->bNext = RunPt;
  } else {
    sema->next = RunPt;
  }
  RunPt->blocked = 1;
  RunPt->bNext = 0;
  TRIGGER_PENDSV();
  //OS_EnableInterrupts();
}

void OS_bSignal(Sema4Type *semaPt) {
  long crit = StartCritical();
  tcbType *t = semaPt->next;
  if(t != NULL) {
    semaPt->next = semaPt->next->bNext;
    t->bNext = 0;
    t->blocked = 0;
    semaPt->Value = -1;
  } else {
    semaPt->Value = 0;
  }
  TRIGGER_PENDSV();
  EndCritical(crit);
}

void OS_Signal(Sema4Type *semaPt) {
  long crit = StartCritical();
  tcbType *t = semaPt->next;
  if(t != NULL) {
    semaPt->next = semaPt->next->bNext;
    t->bNext = 0;
    t->blocked = 0;
  } else {
    semaPt->Value++;
  }
  TRIGGER_PENDSV();
  EndCritical(crit);
}

void UnblockThread(Sema4Type *semaPt) {
  tcbType *t = semaPt->next;
  if(t != NULL) {
    semaPt->next = semaPt->next->bNext;
    t->bNext = 0;
    t->blocked = 0;
  } else {
    semaPt->Value++;
  }
}

#if DEBUG
void Jitter_Init(void) {
  OS_InitSemaphore(&jitterLock, 0);
}

void Jitter(void) {
  OS_bWait(&jitterLock);
  UART_OutStringCRLF("Periodic thread 1 jitter data:");
  UART_OutString("Max jitter: "); UART_OutUDec(MaxJitter1); UART_OutString(" x 0.1 us"); UART_OutCRLF();
  UART_OutStringCRLF("Jitter distribution: ");
  for(int i = 0; i < JITTERSIZE; ++i) {
    UART_OutString("i="); UART_OutUDec(i); UART_OutString(": ");
    for(int j = 0; j < JitterHistogram1[i] >> 2; ++j)
      UART_OutChar('=');
      UART_OutChar(' '); UART_OutUDec(JitterHistogram1[i]); UART_OutCRLF();
  }

  UART_OutStringCRLF("Periodic thread 2 jitter data:");
  UART_OutString("Max jitter: "); UART_OutUDec(MaxJitter2); UART_OutString(" x 0.1 us"); UART_OutCRLF();
  UART_OutStringCRLF("Jitter distribution: ");
  for(int i = 0; i < JITTERSIZE; ++i) {
    UART_OutString("i="); UART_OutUDec(i); UART_OutString(": ");
    for(int j = 0; j < JitterHistogram2[i] >> 2; ++j)
      UART_OutChar('=');
    UART_OutChar(' '); UART_OutUDec(JitterHistogram2[i]); UART_OutCRLF();
  }
  OS_bSignal(&jitterLock);
}
#endif

void OS_DisableInterrupts(void) {
  #if DEBUG
  if(IntsEnabled) {
    TmpTimeIntsDisabled = GetMasterTime();
    IntsEnabled = 0;
  }
  #endif
  DisableInterrupts();
}

void OS_EnableInterrupts(void) {
  #if DEBUG
  unsigned long tmp = GetMasterTime();
  unsigned long t = OS_TimeDifference(TmpTimeIntsDisabled, tmp);
  TimeIntsDisabled += t;
  if(t > MaxTimeIntsDisabled)
    MaxTimeIntsDisabled = t;
  PercentIntsDisabled = (TimeIntsDisabled * 100) / GetMasterTime();
  IntsEnabled = 1;
  #endif
  EnableInterrupts();
}

int OS_AddProcess(void(*entry)(void), uint32_t *text, uint32_t *data, uint32_t stackSize, uint32_t priority){  
  JumpAsm();
  //OS_AddThread(entry,128,1);
  //OS_Launch(TIME_1MS*10);
  return 0;
}

long StartCritical(void) {
  #if DEBUG
  if(IntsEnabled) {
    TmpTimeIntsDisabled = GetMasterTime();
    IntsEnabled = 0;
  }
  #endif
  return StartCriticalAsm();
}

void EndCritical(long sav) {
  #if DEBUG
  unsigned long tmp = GetMasterTime();
  unsigned long t = OS_TimeDifference(TmpTimeIntsDisabled, tmp);
  TimeIntsDisabled += t;
  if(t > MaxTimeIntsDisabled)
     MaxTimeIntsDisabled = t;
  PercentIntsDisabled = (TimeIntsDisabled * 100) / GetMasterTime();
  IntsEnabled = !sav;
  #endif
  EndCriticalAsm(sav);
}
