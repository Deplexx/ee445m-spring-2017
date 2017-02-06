// OS.c
// Most code from Timer3.c

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "OS.h"

#define PF1       (*((volatile unsigned long *)0x40025008))
#define PF2       (*((volatile unsigned long *)0x40025010))
#define PF3       (*((volatile unsigned long *)0x40025020))
#define LEDS      (*((volatile unsigned long *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08

// ***** GLOBALS *****
void (*PeriodicTask)(void);   // user function
uint32_t count;

static void dummy(void){
    PF1 ^= RED;
}

void OS_On() {
    SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
    volatile int delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
    GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
    // only PF0 needs to be unlocked, other bits can't be locked
    GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
    GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
    GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
    GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0

    OS_AddPeriodicThread(dummy, 1000, 4);
}

// ***** OS_AddPeriodicThread *****
// Sets thread to run every periodically every ms
// Input: task - function to run
//        period - recurring thread every period in ms
//        priority - NVIC priority
// Output: ??
int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority){
  SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
  PeriodicTask = task;          // user function
  TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
	period = period*80000; // convert to number of cycles needed for 1ms: 1ms * 80MHz = 80000 cycles
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

// ***** OS_ClearPeriodicTime *****
// Resets global counter to 0
// Input: none
// Output: none
void OS_ClearPeriodicTime(void){count = 0;}

// ***** OS_AddPeriodicThread *****
// Returns global counter value in units of period initialized by
// OS_AddPeriodicThread
// Input: none
// Output: counter value
uint32_t OS_ReadPeriodicTime(void){return count;}

// ***** Timer3A_Handler *****
void Timer3A_Handler(void){
  
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
	count++;
  (*PeriodicTask)();                // execute user task
}
