//*****************************************************************************
// user.c
// Runs on LM4F120/TM4C123
// An example user program that initializes the simple operating system
//   Schedule three independent threads using preemptive round robin  
//   Each thread rapidly toggles a pin on Port D and increments its counter 
//   TIMESLICE is how long each thread runs

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
#include "inc/tm4c123gh6pm.h"
#include "os.h"
#include "adc.h"
#include "st7735.h"

/*
#define TIMESLICE               TIME_2MS    // thread switch time in system time units

uint32_t Count1;   // number of times thread1 loops
uint32_t Count2;   // number of times thread2 loops
uint32_t Count3;   // number of times thread3 loops
*/

#define GPIO_PORTD1             (*((volatile uint32_t *)0x40007008))
#define GPIO_PORTD2             (*((volatile uint32_t *)0x40007010))
#define GPIO_PORTD3             (*((volatile uint32_t *)0x40007020))
#define GPIO_PORTD_DIR_R        (*((volatile uint32_t *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile uint32_t *)0x40007420))
#define GPIO_PORTD_DEN_R        (*((volatile uint32_t *)0x4000751C))
#define GPIO_PORTD_AMSEL_R      (*((volatile uint32_t *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile uint32_t *)0x4000752C))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCGPIO_R3      0x00000008  // GPIO Port D Run Mode Clock
                                            // Gating Control
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_PRGPIO_R3        0x00000008  // GPIO Port D Peripheral Ready

//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
#define FS 400            // producer/consumer sampling
#define RUNLENGTH (20*FS) // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US // DAS 2kHz sampling period in system time units
long x[64],y[64];         // input and output arrays for FFT

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};

#define PB2  (*((volatile unsigned long *)0x40005010))
#define PB3  (*((volatile unsigned long *)0x40005020))
#define PB4  (*((volatile unsigned long *)0x40005040))
#define PB5  (*((volatile unsigned long *)0x40005080))

void PortB_Init(void){ unsigned long volatile delay;
//  SYSCTL_RCGC2_R |= 0x02;       // activate port B
  SYSCTL_RCGCGPIO_R |= 0x02;            // activate clock for Port D
  delay = SYSCTL_RCGC2_R;        
  delay = SYSCTL_RCGC2_R;         
  GPIO_PORTB_DIR_R |= 0x3C;    // make PB5-2 output heartbeats
  GPIO_PORTB_AFSEL_R &= ~0x3C;   // disable alt funct on PB5-2
  GPIO_PORTB_DEN_R |= 0x3C;     // enable digital I/O on PB5-2
  GPIO_PORTB_PCTL_R = ~0x00FFFF00;
  GPIO_PORTB_AMSEL_R &= ~0x3C;      // disable analog functionality on PB
}

/*
void Task1(void){
  Count1 = 0;
  for(;;){
    Count1++;
    GPIO_PORTD1 ^= 0x02;      // toggle PD1
    OS_Suspend();
  }
}
void Task2(void){
  Count2 = 0;
  for(;;){
    Count2++;
    GPIO_PORTD2 ^= 0x04;      // toggle PD2
  }
}
void Task3(void){
  Count3 = 0;
  for(;;){
    Count3++;
    GPIO_PORTD3 ^= 0x08;      // toggle PD3
  }
}
int main(void){
  OS_Init();           // initialize, disable interrupts, 50 MHz
  SYSCTL_RCGCGPIO_R |= 0x08;            // activate clock for Port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){}; // allow time for clock to stabilize
  GPIO_PORTD_DIR_R |= 0x0E;             // make PD3-1 out
  GPIO_PORTD_AFSEL_R &= ~0x0E;          // disable alt funct on PD3-1
  GPIO_PORTD_DEN_R |= 0x0E;             // enable digital I/O on PD3-1
                                        // configure PD3-1 as GPIO
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFF000F)+0x00000000;
  GPIO_PORTD_AMSEL_R &= ~0x0E;          // disable analog functionality on PD3-1
  //OS_AddThreads(&Task1, &Task2, &Task3);
  OS_AddThread(&Task3);
  OS_AddThread(&Task2);
  //OS_AddThread(&Task1);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}
*/

//*******************Initial TEST**********
// This is the simplest configuration, test this first, (Lab 1 part 1)
// run this with 
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops
void Thread1(void){
  Count1 = 0;          
  for(;;){
    PB2 ^= 0x04;       // heartbeat
    Count1++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    PB3 ^= 0x08;       // heartbeat
    Count2++;
    //OS_Suspend();      // cooperative multitasking
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    PB4 ^= 0x10;       // heartbeat
    Count3++;
    //OS_Suspend();      // cooperative multitasking
  }
}

int Testmain1(void){  // Testmain1
  OS_Init();          // initialize, disable interrupts
  PortB_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,2); 
  NumCreated += OS_AddThread(&Thread3,128,3); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 1 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
void Thread1b(void){
  Count1 = 0;          
  for(;;){
    PB2 ^= 0x04;       // heartbeat
    Count1++;
  }
}
void Thread2b(void){
  Count2 = 0;          
  for(;;){
    PB3 ^= 0x08;       // heartbeat
    Count2++;
    if(Count2>1000000)
      OS_Kill();
  }
}
void Thread3b(void){
  Count3 = 0;          
  for(;;){
    PB4 ^= 0x10;       // heartbeat
    Count3++;
  }
}
int Testmain2(void){  // Testmain2
  OS_Init();           // initialize, disable interrupts
  PortB_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,1); 
  NumCreated += OS_AddThread(&Thread2b,128,2); 
  NumCreated += OS_AddThread(&Thread3b,128,3); 
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than testmain1
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//******************* Lab 3 Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// SW1 not needed, 
// SW2 not needed
// logic analyzer on PF1 for systick interrupt (in your OS)
//                on PB2 to measure context switch time
void Thread8(void){       // only thread running
  while(1){
    PB2 ^= 0x04;      // debugging profile  
  }
}
int Testmain7(void){       // Testmain7
  PortB_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread8,128,2); 
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//*******************Third TEST**********
// Once the second test runs, test this (Lab 1 part 2)
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
/*
Sema4Type Readyc;        // set in background
int Lost;
void BackgroundThread1c(void){   // called at 1000 Hz
  Count1++;
  OS_Signal(&Readyc);
}
void Thread5c(void){
  for(;;){
    OS_Wait(&Readyc);
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}
void Thread2c(void){
  OS_InitSemaphore(&Readyc,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5c,128,3); 
  OS_AddPeriodicThread(&BackgroundThread1c,TIME_1MS,0); 
  for(;;){
    OS_Wait(&Readyc);
    Count2++;   // Count2 + Count5 should equal Count1
  }
}

void Thread3c(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4c(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
    OS_Sleep(10);
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5c(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4c,128,3); 
}
      
int Testmain3(void){   // Testmain3
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
// Count2 + Count5 should equal Count1
  NumCreated = 0 ;
  OS_AddSW1Task(&BackgroundThread5c,2);
  NumCreated += OS_AddThread(&Thread2c,128,2); 
  NumCreated += OS_AddThread(&Thread3c,128,3); 
  NumCreated += OS_AddThread(&Thread4c,128,3); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
*/
int main(void){
  Testmain2();
}
