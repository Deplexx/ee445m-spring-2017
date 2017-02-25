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

#include <stdbool.h>
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "os.h"
#include "ADC.h"
#include "ST7735.h"

void OS_DisableInterrupts(void);
void OS_EnableInterrupts(void);

static void incr(void);

static int c = 0;
Sema4Type semaUser;


int done1 = 0;
int done2 = 0;
int done3 = 0;

static void incr1(void) {
    incr();
    while(true) {}
}

static void incr2(void) {
    incr();
    while(true) {}
}

void incr3(void) {
    incr();
    while(true) {}
}

static void incr(void) {
    for(int i = 0; i < 10000; ++i) {
        OS_bWait(&semaUser);
        ST7735_Message(0, i % 8, "", ++c);
        OS_bSignal(&semaUser);
    }
}

void done(void) {
    while(!(done1 || done2 || done3)) {}

    OS_DisableInterrupts();
    ST7735_Message(0, 1, "", c);
    OS_EnableInterrupts();
}

int realmain(void) {
  OS_Init();           // initialize, disable interrupts
  ST7735_InitR(INITR_REDTAB);
  //ST7735_OutUDec(c);
  OS_InitSemaphore(&semaUser, 0);
  OS_AddThread(&incr1,128,1);
  OS_AddThread(&incr2,128,1);
  //OS_AddThread(&incr3,128,1);
  //NumCreated += OS_AddThread(&Thread2b,128,1);
  //NumCreated += OS_AddThread(&done,128,1);
  OS_Launch(TIME_2MS); // 100us, doesn't return, interrupts enabled in here

  return 0;
}
