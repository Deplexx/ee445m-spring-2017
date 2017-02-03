    // ST7735TestMain.c

#include <stdio.h>
#include <stdint.h>
#include "ST7735_.h"
#include "OS.h"
#include "PLL.h"
#include "../inc/tm4c123gh6pm.h"

#define PF1       (*((volatile unsigned long *)0x40025008))
#define PF2       (*((volatile unsigned long *)0x40025010))
#define PF3       (*((volatile unsigned long *)0x40025020))
#define LEDS      (*((volatile unsigned long *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08

void dummy(void){PF1 ^= RED;}

static int notmain(void){
  PLL_Init(Bus80MHz);                  // set system clock to 80 MHz
  ST7735_InitR(INITR_REDTAB);
  
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
  
	ST7735_Message(0, 0, "dev0 line0", -10);
	ST7735_Message(0, 1, "dev0 line1", 100);
	ST7735_Message(0, 2, "dev0 line2", 999);
	ST7735_Message(0, 3, "dev0 line3", 0);
	
	ST7735_Message(1, 0, "dev1 line0", 0);
	ST7735_Message(1, 1, "dev1 line1", 2);
	ST7735_Message(1, 2, "dev1 line2", 3);
	ST7735_Message(1, 3, "dev1 line3", 4);
  
  OS_AddPeriodicThread(dummy, 1000, 4);
	
	while(1){;};
}

