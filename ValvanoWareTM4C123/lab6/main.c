// Main.c
// Runs on LM4F120/TM4C123
// Main program for the CAN example.  Initialize hardware and software for
// CAN transfers.  Repeatedly send the status of the built-in buttons over
// the CAN and light up the built-in LEDs according to the response.
// Daniel Valvano
// May 3, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
  Program 7.5, example 7.6

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

// MCP2551 Pin1 TXD  ---- CAN0Tx PE5 (8) O TTL CAN module 0 transmit
// MCP2551 Pin2 Vss  ---- ground
// MCP2551 Pin3 VDD  ---- +5V with 0.1uF cap to ground
// MCP2551 Pin4 RXD  ---- CAN0Rx PE4 (8) I TTL CAN module 0 receive
// MCP2551 Pin5 VREF ---- open (it will be 2.5V)
// MCP2551 Pin6 CANL ---- to other CANL on network 
// MCP2551 Pin7 CANH ---- to other CANH on network 
// MCP2551 Pin8 RS   ---- ground, Slope-Control Input (maximum slew rate)
// 120 ohm across CANH, CANL on both ends of network
#include <stdint.h>
#include "PLL.h"
#include "can0.h"
#include "../inc/tm4c123gh6pm.h"
#include "math2.h"
#include "OS.h"
#include "ST7735.h"
#include "US.h"
#include "uart_interp.h"
#include "IR.h"

#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))

#define MIN_SPEED 10
#define MIN_D 50 //mm
#define MIN_IR 70 //ir0 + ir1
#define K_I 1
#define K_P 1

static int base_speed;
static int servo_angle;
static int car_d;

#define R_MIN MIN_SPEED
#define R_SPEEDUP 1
#define R_SLOWDOWN 1
#define L_MIN MIN_SPEED
#define L_SPEEDUP 1
#define L_SLOWDOWN 1
static int l_speed, r_speed;

void StartOS(void);

/*
void canGetThread(void){
  uint8_t data[4];
  CAN0_GetMailNonBlock(data);
  ST7735_Message(0, 0, "IR0: ", data[0]);
  ST7735_Message(0, 1, "IR0: ", data[1]);
  ST7735_Message(0, 2, "IR0: ", data[2]);
  ST7735_Message(0, 3, "IR0: ", data[3]);
}

void canPutThread(void){
  uint8_t data[4];
  uint32_t usdist = USSensor_GetDistance();
  data[0] = usdist & 0x000000FF;
  data[1] = (usdist & 0x0000FF00) >> 8;
  data[2] = (usdist & 0x00FF0000) >> 16;
  data[3] = (usdist & 0xFF000000) >> 24;
  CAN0_SendData(data);
}

void canThread(void){
  static int getPut = 0;
  if(getPut){
    canGetThread();
    getPut = 0;
  } else {
    canPutThread();
    getPut = 1;
  }
}
*/

uint32_t IRdata[4];
uint32_t USdata[3];
static uint8_t data[4];
uint32_t displayFlag;
uint32_t every10ms;
unsigned long calcTime;
void stateMachine(void);
void pid(void);

void lcdDisplay(void){
  uint32_t angleL = get_wall_angle(IRdata[0], IRdata[1]);
  uint32_t angleR = get_wall_angle(IRdata[3], IRdata[2]);
  ST7735_Message(0, 0, "IR0: ", IRdata[0]);
  ST7735_Message(0, 1, "IR1: ", IRdata[1]);
  ST7735_Message(0, 2, "IR2: ", IRdata[2]);
  ST7735_Message(0, 3, "IR3: ", IRdata[3]);
	ST7735_Message(0, 4, "AngleL: ", angleL);
  ST7735_Message(0, 5, "AngleR: ", angleR);
  ST7735_Message(1, 0, "US0: ", USdata[0]);
	ST7735_Message(1, 1, "US1: ", USdata[1]);
	ST7735_Message(1, 2, "US2: ", USdata[2]);
  ST7735_Message(1, 3, "State: ", data[0]); 
  displayFlag = 0;
  OS_Kill();
}

void inputCapture(void){
  IR_In(IRdata);
  if(every10ms==0) {
    US_In(USdata);
		car_d = USdata[1];
    US_StartPing();
    //Timer0_StartPing();
    //Timer1_StartPing();
    //Timer3_StartPing();
	}
  every10ms++;
  if(every10ms==10)
    every10ms = 0;
  if(displayFlag==0){
    displayFlag = 1;
    OS_AddThread(&lcdDisplay,128,1);
  }
  //stateMachine();
	pid();
}

void canSend(int s){
  data[0] = s;
  CAN0_SendData(data);
}

enum STATES {SPEEDUP,LWALL,LTURN};

void stateMachine(void){
  static int diff, avg;
  static int wait = 50;
  static int targetDist = 130;
  static int state = SPEEDUP;
  static int nextState;
  
  diff = IRdata[0] - IRdata[1];
  avg = (IRdata[0] + IRdata[1])/2;

  nextState = state;
  
  switch(state){
    case SPEEDUP:
    canSend(2);
    wait--;
    
    if(wait<=0){
      nextState = LWALL;
    }
    break;
    case LWALL:
     if(IRdata[0]<targetDist && IRdata[1]>targetDist){
      canSend(7); //drift left
    } else if(IRdata[0]>targetDist && IRdata[1]<targetDist){
      canSend(4); //drift right
    } else if(IRdata[0]>targetDist && IRdata[1]>targetDist){
      canSend(7); //drift left
    } else if(IRdata[0]<targetDist && IRdata[1]<targetDist){
      canSend(4); //drift right
    } else {
      canSend(1);//drift straight
    }
    
    if(IRdata[1] > targetDist*2){
      nextState = LTURN;
    }
    break;
    case LTURN:
    canSend(13);
    
    if(IRdata[1] < targetDist){
      nextState = LWALL;
    }
    break;
    default:
    
    break;
  }
  
  state = nextState;
}

void pid(void) {
	int speed_error;
	int wall_angle, d1, d0;
	
	//speed
	speed_error = car_d - MIN_D;
	if(speed_error > 0)
		base_speed = 100;
	else
		base_speed = base_speed * K_I + speed_error * K_P;
		
  if(base_speed < MIN_SPEED)
		base_speed = MIN_SPEED;
	
	//steering
	if((IRdata[0] + IRdata[1]) > (IRdata[2] + IRdata[3])) {
		wall_angle = get_wall_angle(IRdata[3], IRdata[2]); //turn left
		d0 = IRdata[2]; d1 = IRdata[3];
	} else {
		wall_angle = -get_wall_angle(IRdata[0], IRdata[1]); //turn right
		d0 = IRdata[0]; d1 = IRdata[1];
	}
	
	if(d0 + d1 < MIN_IR) { // make turn
		servo_angle = wall_angle * 10 / 9;
		if(wall_angle > 0) {
			r_speed = base_speed + wall_angle * R_SPEEDUP;
			l_speed = base_speed - wall_angle * L_SLOWDOWN;
		} else {
			r_speed = base_speed - wall_angle * R_SLOWDOWN;
			l_speed = base_speed + wall_angle * L_SPEEDUP;
		}
  } else servo_angle = 0;
	
	if(r_speed < R_MIN)
		r_speed = R_MIN;
	
	if(l_speed < L_MIN)
		l_speed = L_MIN;
	
	int8_t data[4] = {10, l_speed, r_speed, servo_angle};
	
	CAN0_SendData((uint8_t*) data);
} 

int main(void){
  OS_Init();
//  Timer0_Init2();
//  Timer1_Init2();
//  Timer3_Init2();
  US_Init();
  CAN0_Open();
  IR_Init();
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0);
  
  displayFlag = 0;
  every10ms = 0;
  base_speed = l_speed = r_speed = 100;
	servo_angle = 0;
	
	OS_AddThread(&Interpreter, 128, 7);
  OS_AddPeriodicThread(&inputCapture,80000,1);
  //OS_AddPeriodicThread(&canThread, 1600000, 0);
	OS_Launch(TIME_1MS*10);
}
