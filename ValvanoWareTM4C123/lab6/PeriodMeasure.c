// PeriodMeasure.c
// Runs on LM4F120/TM4C123
// Use Timer0A in 24-bit edge time mode to request interrupts on the rising
// edge of PB6 (T0CCP0), and measure period between pulses.
// Daniel Valvano
// May 5, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 7.2, Program 7.2

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

// external signal connected to PB6 (T0CCP0) (trigger on rising edge)
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "os.h"
#include "PLL.h"
#include "Timer3.h"
#include "USSensor.h"

#define NVIC_EN0_INT19          0x00080000  // Interrupt 19 enable
#define TIMER_TAMR_TACMR        0x00000004  // GPTM TimerA Capture Mode
#define TIMER_TAMR_TAMR_CAP     0x00000003  // Capture mode
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_CTL_TAEVENT_POS   0x00000000  // Positive edge
#define TIMER_CTL_TAEVENT_NEG   0x00000004  // Negative edge
#define TIMER_CTL_TAEVENT_BOTH  0x0000000C  // Both edges
#define TIMER_IMR_CAEIM         0x00000004  // GPTM CaptureA Event Interrupt
                                            // Mask
#define TIMER_ICR_CAECINT       0x00000004  // GPTM CaptureA Event Interrupt
                                            // Clear
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void Delay5us(void);
static uint32_t Period;              // (1/clock) units
static uint32_t First;               // Timer0A first edge
static volatile uint32_t Distance;

static volatile int32_t Done = 1;                 // set either edge

uint32_t USSensor_GetDistance(void) {	
	return Distance;
}

// max period is (2^24-1)*12.5ns = 209.7151ms
// min period determined by time to run ISR, which is about 1us
void USSensor_Init(void){
	long sav = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x01;// activate timer0
  SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_DIR_R |= 0x80;        // make PB7 output
  GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
  GPIO_PORTB_AFSEL_R &= ~0x80;     // disable alt funct on PB7
  GPIO_PORTB_DEN_R |= 0xC0;        // enable PB6 as T0CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0x00FFFFFF)+0x07000000;
  GPIO_PORTB_AMSEL_R &= ~0xC0;     // disable analog functionality on PB6
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
  TIMER0_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 24-bit capture         
  TIMER0_CTL_R |= TIMER_CTL_TAEVENT_BOTH;// configure for both edges
  TIMER0_TAILR_R = TIMER_TAILR_M;  // max start value
  TIMER0_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
  TIMER0_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// clear timer0A capture match flag
  TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 16-b, +edge timing, interrupts
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R = NVIC_EN0_INT19;        // enable interrupt 19 in NVIC
	
  //Timer3_Init(&USSensor_SendPulse, TIME_1MS*10);
	
	EndCritical(sav);
}
void Timer0A_Handler(void){
  PF2 = PF2^0x04;  // toggle PF2
  PF2 = PF2^0x04;  // toggle PF2
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
	if(!(GPIO_PORTB_DATA_R & 0x40)) { //ultrasonic sensor echo just arrived; calculate distance
		Period = (First - TIMER0_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
		Distance = (170 * Period) / 800000; //units=cm
		Done = 1;
	} else {First = TIMER0_TAR_R; Done = 0; }            // setup for next
  PF2 = PF2^0x04;  // toggle PF2
}

//debug code
int debug_periodic_measure(void){           
  PLL_Init();              // 80 MHz clock
  USSensor_Init();            // initialize 24-bit timer0A in capture mode
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}

# define US5 400

void up_pulse(void);
void dn_pulse(void);
void set_pb6_gpio_out(void);
void set_pb6_input_capture(void);

void USSensor_SendPulse(void) {
	long sav = StartCritical();
	if(Done) {
		up_pulse();
		Delay5us();
		dn_pulse();
	}
	EndCritical(sav);
}

void up_pulse(void) {	
	set_pb6_gpio_out();
	GPIO_PORTB_DATA_R |= 0x40; //toggle PB6 high
}

void set_pb6_gpio_out(void) {
	GPIO_PORTB_DIR_R |= 0x40;       // make PB6 out
	GPIO_PORTB_AFSEL_R &= ~0x40;      // disable alt funct on PB6/T0CCP0
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x00000000; // disable input capture
}

void dn_pulse(void) {
	GPIO_PORTB_DATA_R &= ~0x40; //toggle PB6 low
	set_pb6_input_capture();
}

void set_pb6_input_capture(void) {
	GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
	GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6/T0CCP0
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000; // enable input capture
	TIMER0_ICR_R = TIMER_ICR_CAECINT;
}

__asm void
delay(uint32_t ulCount)
{
	subs    r0, #1
	bne     delay
	bx      lr
}

void Delay5us(void) {
	delay(133);
}
