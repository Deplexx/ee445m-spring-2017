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
#include "Timer3.h"
#include "can0.h"
#include "../inc/tm4c123gh6pm.h"
#include "OS.h"
#include "ST7735.h"
#include "USSensor.h"
#include "uart_interp.h"
#include "IR.h"

#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))

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
static uint8_t data[4];
uint32_t displayFlag;
uint32_t every10ms;
void stateMachine(void);

void lcdDisplay(void){
  uint32_t usdist = USSensor_GetFrontDistance();
  ST7735_Message(0, 0, "IR0: ", IRdata[0]);
  ST7735_Message(0, 1, "IR1: ", IRdata[1]);
  ST7735_Message(0, 2, "IR2: ", IRdata[2]);
  ST7735_Message(0, 3, "IR3: ", IRdata[3]);
  ST7735_Message(1, 0, "US0: ", usdist);
  ST7735_Message(1, 3, "State: ", data[0]); 
  displayFlag = 0;
  OS_Kill();
}

void inputCapture(void){
  IR_In(&IRdata[0]);
  if(every10ms==0)
    USSensor_SendFrontPulse(); //5us
  every10ms++;
  if(every10ms==10)
    every10ms = 0;
  if(displayFlag==0){
    displayFlag = 1;
    OS_AddThread(&lcdDisplay,128,1);
  }
  stateMachine();
}

void driveThread(void){
  OS_Sleep(5000);
  static uint8_t data[4];
  //increase speed
  data[0] = 2; data[1] = 0; data[2] = 0; data[3] = 0;
  for(int k=0; k<1000; k++){
    CAN0_SendData(data);
    OS_Sleep(10);
  }
  //stop????
  data[0] = 0; data[1] = 0; data[2] = 0; data[3] = 0;
  while(1){
    CAN0_SendData(data);
    OS_Sleep(10);
  }
}

void driveThread2(void){
  data[0] = 2; data[1] = 0; data[2] = 0; data[3] = 0;
  //accelerate
  for(int k=0; k<40; k++){
    CAN0_SendData(data);
    OS_Sleep(10);
  }
  
  while(1){
    /*
    int diff = IRdata[0] - IRdata[1];
    
    if(diff<0)
      diff = -diff;
    
    if(diff>-10){
      data[0] = 1; //do nothing
      CAN0_SendData(data);
    } else {
      data[0] = 7; //drift left, same speed
      CAN0_SendData(data);
      OS_Sleep(100);
    }
    */
    /*
    if(IRdata[0]-IRdata[1]<-20){
      data[0] = 7; //drift left
      CAN0_SendData(data);
      OS_Sleep(200);
    } else if(IRdata[0]-IRdata[1]>20){
      data[0] = 4; //drift right
      CAN0_SendData(data);
      OS_Sleep(200);
    } else {
      data[0] = 1; //do nothing
      CAN0_SendData(data);
    }
    */
    
    int diff = IRdata[0] - IRdata[1];
    int avg = (IRdata[0] + IRdata[1])/2;
    
    if(avg>250){
      data[0] = 7; //drift left
      CAN0_SendData(data);
    } else if(diff<-25){
      data[0] = 7; //drift left
      CAN0_SendData(data);
    } else if(diff>25){
      data[0] = 4; //drift right
      CAN0_SendData(data);
    } else if(IRdata[0]<100){
      data[0] = 4; //drift right
      CAN0_SendData(data);
    } else if(IRdata[0]>180){
      data[0] = 7; //drift left
      CAN0_SendData(data);
    } else {
      data[0] = 1; //drift straight
      CAN0_SendData(data);
    }
    //OS_Sleep(20);
  }
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

int main(void){
  OS_Init();
	USSensor_Init();
  CAN0_Open();
  IR_Init();
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0);
  
  displayFlag = 0;
  every10ms = 0;
  
	OS_AddThread(&Interpreter, 128, 7);
  //OS_AddThread(&driveThread2, 128, 0);
  OS_AddPeriodicThread(&inputCapture,80000,1);
  //OS_AddPeriodicThread(&canThread, 1600000, 0);
	OS_Launch(TIME_1MS*10);
}
