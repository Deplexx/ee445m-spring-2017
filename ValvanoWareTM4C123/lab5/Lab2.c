// Lab2.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3 Testmain4  and main
// Lab3: Testmain5 Testmain6, Testmain7, and main (with SW2)

// Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task, samples PD3
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PD3 Ain3 sampled at 2k, sequencer 3, by DAS software start in ISR
// PD2 Ain5 sampled at 250Hz, sequencer 0, by Producer, timer tigger

#include <stdint.h>
#include <string.h>

#include "../inc/tm4c123gh6pm.h"

#include "ADC.h"
#include "os.h"
#include "ST7735.h"
#include "uart_interp.h"

#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))

static int NumCreated;

void PortE_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x10;       // activate port E
  volatile int delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
}

void ButtonWork(void){
unsigned long myId = OS_Id(); 
  PE1 ^= 0x02;
  ST7735_Message(1,0,"NumCreated =",NumCreated);
  PE1 ^= 0x02;
  OS_Sleep(50);
  PE1 ^= 0x02;
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

void IdleTask(void){
  while(1){;}
}

int main(void){
  OS_Init();
  PortE_Init();
  ST7735_InitR(INITR_REDTAB);

//  OS_AddSW1Task(&SW1Push,2);
//  OS_AddSW2Task(&SW2Push,2);
  
  NumCreated = 0 ;
  
  NumCreated += OS_AddThread(&Interpreter,128,2);
  NumCreated += OS_AddThread(&IdleTask,128,7);
  
  OS_Launch(TIME_2MS);
  return 0;
}
