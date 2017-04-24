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

#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define NUM_US_SENSORS 3
static uint32_t Period[NUM_US_SENSORS];              // (1/clock) units
static uint32_t First[NUM_US_SENSORS];               // Timer0A first edge
static volatile uint32_t Distance[NUM_US_SENSORS];

static volatile int32_t Done[3] = {1,};                 // set either edge

uint32_t USSensor_GetFrontDistance(void) {	
	return (uint32_t) Distance[FRONT];
}

uint32_t USSensor_GetLeftDistance(void) {	
	return (uint32_t) Distance[LEFT];
}

uint32_t USSensor_GetRightDistance(void) {	
	return (uint32_t) Distance[RIGHT];
}

static void init_front(void);
static void init_left(void);
static void init_right(void);

// max period is (2^24-1)*12.5ns = 209.7151ms
// min period determined by time to run ISR, which is about 1us
void USSensor_Init(void){
	long sav = StartCritical();
	SYSCTL_RCGCTIMER_R |= 0x01;// activate timer0
  SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
	init_front();
	init_left();
	init_right();
	EndCritical(sav);  	
}

static void init_front(void) {
  GPIO_PORTB_DIR_R &= ~0x40;       // make PB6,PB4,PB2 in
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6,PB4,PB2
  GPIO_PORTB_DEN_R |= 0x40;        // enable PB6,PB4,PB2 as T0CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0x00FFFFFF)+0x07000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;     // disable analog functionality on PB6
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
}

static void init_left(void) {
  SYSCTL_RCGCTIMER_R |= 0x02;// activate timer1
  GPIO_PORTB_DIR_R &= ~0x10;       // make PB4 in
  GPIO_PORTB_AFSEL_R |= 0x10;      // enable alt funct on PB4
  GPIO_PORTB_DEN_R |= 0x10;        // enable PB4 as T1CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFF00FFFF)+0x00070000;
  GPIO_PORTB_AMSEL_R &= ~0x10;     // disable analog functionality on PB4
  TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // disable timer1A during setup
  TIMER1_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
  TIMER1_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 24-bit capture         
  TIMER1_CTL_R |= TIMER_CTL_TAEVENT_BOTH;// configure for both edges
  TIMER1_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
  TIMER1_TAILR_R = TIMER_TAILR_M;  // max start value
  TIMER1_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
  TIMER1_ICR_R = TIMER_ICR_CAECINT;// clear timer1A capture match flag
  TIMER1_CTL_R |= TIMER_CTL_TAEN;  // enable timer1A 16-b, +edge timing, interrupts
                                   // Timer0A=priority 2
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00004000; // bits 15-13
  NVIC_EN0_R = 1<<21;           // 9) enable IRQ 21 in NVIC
}

static void init_right(void) {
  SYSCTL_RCGCTIMER_R |= 0x08;// activate timer3
  while((SYSCTL_PRGPIO_R&0x02) == 0){};// ready?
  GPIO_PORTB_DIR_R &= ~0x04;       // make PB2 in
  GPIO_PORTB_AFSEL_R |= 0x04;      // enable alt funct on PB2
  GPIO_PORTB_DEN_R |= 0x04;        // enable PB2 as T3CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00000700;
  GPIO_PORTB_AMSEL_R &= ~0x04;     // disable analog functionality on PB2
  TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // disable timer3A during setup
  TIMER3_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
  TIMER3_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 24-bit capture         
  TIMER3_CTL_R |= TIMER_CTL_TAEVENT_BOTH;// configure for both edges
  TIMER3_TAILR_R = TIMER_TAILR_M;  // max start value
  TIMER3_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
  TIMER3_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
  TIMER3_ICR_R = TIMER_ICR_CAECINT;// clear timer3A capture match flag
  TIMER3_CTL_R |= TIMER_CTL_TAEN;  // enable timer3A 16-b, +edge timing, interrupts
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
}

//front, pb6
void Timer0A_Handler(void){
  PF2 = PF2^0x04;  // toggle PF2
  PF2 = PF2^0x04;  // toggle PF2
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
	if(!(GPIO_PORTB_DATA_R & 0x40)) { //ultrasonic sensor echo just arrived; calculate distance
		Period[FRONT] = (First[FRONT] - TIMER0_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
		Distance[FRONT] = (170 * Period[FRONT]) / 800000; //units=cm
		Done[FRONT] = 1;
	} else {First[FRONT] = TIMER0_TAR_R; Done[FRONT] = 0; }            // setup for next
  PF2 = PF2^0x04;  // toggle PF2
}

//left, pb4
void Timer1A_Handler(void){
  PF2 = PF2^0x04;  // toggle PF2
  PF2 = PF2^0x04;  // toggle PF2
  TIMER1_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
	if(!(GPIO_PORTB_DATA_R & 0x10)) { //ultrasonic sensor echo just arrived; calculate distance
		Period[LEFT] = (First[LEFT] - TIMER0_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
		Distance[LEFT] = (170 * Period[LEFT]) / 800000; //units=cm
		Done[LEFT] = 1;
	} else {First[LEFT] = TIMER0_TAR_R; Done[LEFT] = 0; }            // setup for next
  PF2 = PF2^0x04;  // toggle PF2
}

//right, pb2
void Timer3A_Handler(void){
  PF2 = PF2^0x04;  // toggle PF2
  PF2 = PF2^0x04;  // toggle PF2
  TIMER3_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
	if(!(GPIO_PORTB_DATA_R & 0x04)) { //ultrasonic sensor echo just arrived; calculate distance
		Period[RIGHT] = (First[RIGHT] - TIMER0_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
		Distance[RIGHT] = (170 * Period[RIGHT]) / 800000; //units=cm
		Done[RIGHT] = 1;
	} else {First[RIGHT] = TIMER0_TAR_R; Done[RIGHT] = 0; }            // setup for next
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

void pbn_up_pulse(int n);
void pbn_dn_pulse(int n);
void set_pbn_gpio_out(int n);
void set_pbn_input_capture(int n);

void USSensor_SendFrontPulse(void) {
	long sav = StartCritical();
	if(Done[FRONT]) {
		pbn_up_pulse(6);
		Delay5us();
		pbn_dn_pulse(6);
	}
	EndCritical(sav);
}

void USSensor_SendLeftPulse(void) {
	long sav = StartCritical();
	if(Done[LEFT]) {
		pbn_up_pulse(4);
		Delay5us();
		pbn_dn_pulse(4);
	}
	EndCritical(sav);
}

void USSensor_SendRightPulse(void) {
	long sav = StartCritical();
	if(Done[RIGHT]) {
		pbn_up_pulse(2);
		Delay5us();
		pbn_dn_pulse(2);
	}
	EndCritical(sav);
}

void pbn_up_pulse(int n) {	
	set_pbn_gpio_out(n);
	GPIO_PORTB_DATA_R |= 1 << n; //toggle PB6 high
}

void set_pbn_gpio_out(int n) {
	GPIO_PORTB_DIR_R |= 1 << n;       // make PB6 out
	GPIO_PORTB_AFSEL_R &= 1 << n;      // disable alt funct on PB6/T0CCP0
	switch(n) {
		case 6:
			GPIO_PORTB_PCTL_R = GPIO_PORTB_PCTL_R & 0xF0FFFFFF; // disable input capture
			break;
		case 4:
			GPIO_PORTB_PCTL_R = GPIO_PORTB_PCTL_R & 0xFFF0FFFF; // disable input capture
			break;
		case 2:
			GPIO_PORTB_PCTL_R = GPIO_PORTB_PCTL_R & 0xFFFFF0FF; // disable input capture
			break;
	}
}

void pbn_dn_pulse(int n) {
	GPIO_PORTB_DATA_R &= ~(1 << n); //toggle PB6 low
	set_pbn_input_capture(n);
}

void set_pbn_input_capture(int n) {
	GPIO_PORTB_DIR_R &= ~(1 << n);       // make PB6 in
	GPIO_PORTB_AFSEL_R |= 1 << n;      // enable alt funct on PB6/T0CCP0
	switch(n) {
		case 6:
			GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xF0FFFFFF) + 0x07000000; // disable input capture
			TIMER0_ICR_R = TIMER_ICR_CAECINT;
			break;
		case 4:
			GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFF0FFFF) + 0x00070000; // disable input capture
			TIMER1_ICR_R = TIMER_ICR_CAECINT;
			break;
		case 2:
			GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFF0FF) + 0x00000700; // disable input capture
			TIMER3_ICR_R = TIMER_ICR_CAECINT;
			break;
	}
}

static void send_pulse(int n) {

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
