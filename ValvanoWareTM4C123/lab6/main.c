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
//#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF4       (*((volatile uint32_t *)0x40025040))

//prod
//#define MIN_SPEED 30
//#define MAX_SPEED 50

//#define MAX_ANGLE 100
//#define MIN_ANGLE -100
//#define MIN_D 150 //mm
//#define MIN_IR 700 //ir0 + ir1
//#define SERVO_ANGLE_MULT 15 / 9

//#define K_I 1
//#define K_P 1

//static int base_speed;
//static int servo_angle;
//static int car_d;

//#define R_MIN MIN_SPEED
//#define R_SPEEDUP 1
//#define R_SLOWDOWN 1
//#define L_MIN MIN_SPEED
//#define L_SPEEDUP 1
//#define L_SLOWDOWN 1
//static int l_speed, r_speed;

//testing
#define MIN_SPEED 5
#define MAX_SPEED 95

#define SERVO_ANGLE_MULT 25 / 9

#define minProbDist 300  //min value to during ehich preference can change
#define maxProbDist 1000 //max values to during which preference can change

#define MAX_ANGLE 100
#define MIN_ANGLE -100
#define MIN_D 800 //distance at which US will slow down
//#define SLOWDOWN_FACTOR 20;
#define MIN_US_D 240
#define MAX_IR_D 799  // not used
#define MIN_IR_PAIR_D 1000 //eg. ir0 + ir1; distance to detect a general turn
//#define US_TURN_DETECT 1200 //ultrasonic override to initiate a hard turn;
#define US_TOO_CLOSE 200 //ultrasonice detection of being too close to a wall
#define US_NO_ROOM 300 // will not drift if there is not at least this much US room
#define US_DRIFT 2 // the amount ultrasonic_too_close will attempt to drift if too close to a wall
#define US_DRIFT_ANGLE 33//harshness of a drift on servo (0-100)
#define US_TURN_A 70 //harshness of a hard turn on servo
#define US_TURN_SPEED 60 //harshness of a hard turn on DC motors
#define HARD_TURN_A 75 // angle to detect a hard turn
#define SOFT_TURN_FACTOR 1 // divider on speed during soft turns
#define HARD_TURN_FACTOR 1 // divider on speed during hard turns
#define PROBABLY_DISTANCE 50
#define PREFERENCE_CHANGE_MIN_DIST 320//min front us distance at which prefence will change
#define PREFERENCE_CHANGE_MAX_DIST 1000//max front us distance at whcih preference will change


#define K_I 1
#define K_P 1/4

int countdown_timer = 180000;
static int base_speed;
static int servo_angle;
static int car_fd;
static int car_ld;
static int car_rd;

//harshness of soft turns 
#define R_MIN MIN_SPEED
#define R_SPEEDUP 1
#define R_SLOWDOWN 1
#define L_MIN MIN_SPEED
#define L_SPEEDUP 1
#define L_SLOWDOWN 1
#define SUB_GOD_DIST 400
static int l_speed, r_speed;

void StartOS(void);

int minFrontUS = 10000;
int preference;//0 for left, 1 for right
uint32_t IRdata[4];
uint32_t USdata[3];
static uint8_t data[4];
uint32_t displayFlag;
uint32_t every10ms;
unsigned long calcTime;
void stateMachine(void);
void pid(void);

int turning = 0;

void lcdDisplay(void){
	while(1) {
		uint32_t angleL = get_wall_angle(IRdata[0], IRdata[1]);
		uint32_t angleR = get_wall_angle(IRdata[3], IRdata[2]);
		ST7735_Message(0, 0, "IR0: ", IRdata[0]);
		ST7735_Message(0, 1, "IR1: ", IRdata[1]);
		ST7735_Message(0, 2, "IR2: ", IRdata[2]);
		ST7735_Message(0, 3, "IR3: ", IRdata[3]);
		ST7735_Message(0, 4, "AngleL: ", angleL);
		ST7735_Message(0, 5, "AngleR: ", angleR);
		if(preference){
				ST7735_Message(0, 6, "Right Pref ", preference);
		}else{
			ST7735_Message(0, 6, "Left Pref ", preference);
		}
		ST7735_Message(0, 7, "Turning: ", turning);
		
		ST7735_Message(1, 0, "US0: ", USdata[0]);
		ST7735_Message(1, 1, "US1: ", USdata[1]);
		ST7735_Message(1, 2, "US2: ", USdata[2]);
		ST7735_Message(1, 3, "State: ", data[0]); 
		ST7735_Message(1, 4, "b speed: ", base_speed); 
		ST7735_Message(1, 5, "l speed: ", l_speed); 
		ST7735_Message(1, 6, "r speed: ", r_speed); 
		ST7735_Message(1, 7, "s angle: ", (int8_t) servo_angle); 
		displayFlag = 0;
	}		
}

void inputCapture(void){
  IR_In(IRdata);
  if(every10ms==0) {
    US_StartPing();
	}
	if(every10ms==9){
		US_In(USdata);
		car_rd = USdata[0];
		car_fd = USdata[1];
		car_ld = USdata[2];
	}
  every10ms++;
  if(every10ms==29)
    every10ms = 0;
}

void canSend(int s){
  data[0] = s;
  CAN0_SendData(data);
}

	int otherIR1;
	int otherIR2;
	int godmode;
	int dist_diff;

void pid(void) {

	/**180-sec countdown timer**/
	if(countdown_timer <=0){
		data[0] = 10;
		data[1] = 0;
		data[2] = 0;
		data[3] = 100;
		CAN0_SendData((uint8_t*) data);
		return;
	}
	countdown_timer--;
	
	int speed_error;
	int ir0, ir1, ir2, ir3;
	int wall_angle, d1, d0;
	int dus;
	int wall_angle_l, wall_angle_r;
	
	//log of the smallest front US distance recorded
	if(car_fd < minFrontUS){
		minFrontUS = car_fd;
	}
	
	/****************preference****************/
	//set preference for  left turn or a right turn
	dist_diff = car_ld-car_rd;
	if(dist_diff < 0){
		dist_diff = dist_diff*-1;
	}
	if(car_fd > PREFERENCE_CHANGE_MIN_DIST && car_fd<PREFERENCE_CHANGE_MAX_DIST){//only change prefence when we are between min/max distance
		if(dist_diff > PROBABLY_DISTANCE){
			if(car_ld > car_rd){
				preference = 0;//left
			}else{
				preference = 1;//right
			}
		}
	}

	/****************speed****************/ 
	//pid based on speed and KI/KP
	speed_error = car_fd - MIN_D;
	if(speed_error > 0)
		base_speed = 100;
	else
		base_speed = base_speed * K_I + speed_error * K_P;
  if(base_speed < MIN_SPEED)
		base_speed = MIN_SPEED;
	else if(base_speed > MAX_SPEED)
		base_speed = MAX_SPEED;
	base_speed = MAX_SPEED;
	
	/****************steering****************/
	//get wall distance and angles
	ir0 = IRdata[0]; ir1 = IRdata[1];
	ir2 = IRdata[2]; ir3 = IRdata[3];
	wall_angle_l = get_wall_angle(ir0, ir1);
	wall_angle_r = get_wall_angle(ir3, ir2);
	
	//set the wall_angle to the angle of the closest wall
	if((ir0 + ir1) > (ir2 + ir3)) {
		wall_angle = -get_wall_angle(ir3, ir2); //turn left
		dus = car_rd; d0 = ir2; d1 = ir3;
		otherIR1 = ir0;
		otherIR2 = ir1;
	} else {
		wall_angle = get_wall_angle(ir0, ir1); //turn right
		dus = car_ld; d0 = ir0; d1 = ir1;
		otherIR1 = ir2;
		otherIR2 = ir3;
	}
	int US_d = car_rd + car_ld;
	int US_front = car_fd;
	int WA_right = get_wall_angle(ir3, ir2);
	int WA_left = get_wall_angle(ir0, ir1);
	
	////STATE MACHINE
	if((car_rd<US_TOO_CLOSE || car_ld<US_TOO_CLOSE) && US_front>400){
		/****too close to wall, tilt away****/
		
		PF1 = 0x00;
		turning = 7;
		if(US_d<=US_NO_ROOM){//there's no room to drift	
		}else if (car_rd<US_TOO_CLOSE){																					//right wall too close, drift left
			r_speed = base_speed + US_TURN_SPEED * R_SPEEDUP / HARD_TURN_FACTOR;
			l_speed = base_speed - US_TURN_SPEED * L_SLOWDOWN / HARD_TURN_FACTOR;
//			r_speed = base_speed + US_TURN_SPEED * US_DRIFT;
//			l_speed = base_speed - US_TURN_SPEED * US_DRIFT;
			servo_angle = -US_DRIFT_ANGLE;
		}else if (car_ld<US_TOO_CLOSE){																					//left wall too close, drift right
		r_speed = base_speed - US_TURN_SPEED * R_SLOWDOWN / HARD_TURN_FACTOR;
		l_speed = base_speed + US_TURN_SPEED * L_SPEEDUP / HARD_TURN_FACTOR;
//			r_speed = base_speed - US_TURN_SPEED * US_DRIFT;
//			l_speed = base_speed + US_TURN_SPEED * US_DRIFT;
			servo_angle = US_DRIFT_ANGLE;
		}
	}else if(d0 + d1 < MIN_IR_PAIR_D || US_front < 900) {
		/****make some sort of turn****/
		PF1 = 0x02;
		turning = 1;
		servo_angle = wall_angle * SERVO_ANGLE_MULT;
	
		if(US_front<230){ 
			/**GOD_TURN**/
			turning = 9;
			godmode = 1;
			if(!preference){						//turn left in place
				servo_angle = MIN_ANGLE;
				r_speed = MAX_SPEED;
				l_speed = MIN_SPEED;
			}else if(preference){				//turn right in place
				servo_angle = MAX_ANGLE;
				r_speed = MIN_SPEED;
				l_speed = MAX_SPEED;
			}
		}else if(US_front<340 &&((car_ld > SUB_GOD_DIST)||(car_rd > SUB_GOD_DIST))){
			/**SUB_GOD_TURN**/
			godmode = 1;
						if(!preference) { 		//turn left in place
				servo_angle = MIN_ANGLE;
				r_speed = MAX_SPEED;
				l_speed = MIN_SPEED;
			} else if (preference) { 		//turn right in place
				servo_angle = MAX_ANGLE;
				r_speed = MIN_SPEED;
				l_speed = MAX_SPEED;
			}	
		}else if((wall_angle_l > HARD_TURN_A || wall_angle_r > HARD_TURN_A
			 || wall_angle_l < -HARD_TURN_A || wall_angle_r < -HARD_TURN_A) && 
			 ((car_ld > SUB_GOD_DIST)||(car_rd > SUB_GOD_DIST))) {
				 /**HARD_TURN**/
				 turning = 2;
				 if(godmode){
					 base_speed = MAX_SPEED/2;
					 godmode = 0;
				 }
				if(car_ld > car_rd) { 																									//turn left
					servo_angle = -US_TURN_A * SERVO_ANGLE_MULT;
					r_speed = base_speed + US_TURN_SPEED * R_SPEEDUP / HARD_TURN_FACTOR;
					l_speed = base_speed - US_TURN_SPEED * L_SLOWDOWN / HARD_TURN_FACTOR;
				} else { 																																//turn right
					servo_angle = US_TURN_A * SERVO_ANGLE_MULT;
					r_speed = base_speed - US_TURN_SPEED * R_SLOWDOWN / HARD_TURN_FACTOR;
					l_speed = base_speed + US_TURN_SPEED * L_SPEEDUP / HARD_TURN_FACTOR;
				}	
		} else if((WA_left < 53 && WA_left >37) && (WA_right < 53 && WA_right >37)){
			/**CORNER_CASE**/
			turning = 8;
			if(preference){																													//turn right
				servo_angle = US_TURN_A * SERVO_ANGLE_MULT;
				r_speed = base_speed - US_TURN_SPEED * R_SLOWDOWN / HARD_TURN_FACTOR;
				l_speed = base_speed + US_TURN_SPEED * L_SPEEDUP / HARD_TURN_FACTOR;
			}else{																																	//turn left
				servo_angle = -US_TURN_A * SERVO_ANGLE_MULT;
				r_speed = base_speed + US_TURN_SPEED * R_SPEEDUP / HARD_TURN_FACTOR;
				l_speed = base_speed - US_TURN_SPEED * L_SLOWDOWN / HARD_TURN_FACTOR;
			}
		}else if(wall_angle > 0) {
			/**SOFT_TURN_RIGHT**/
			turning = 3;
			if(godmode){
					 base_speed = MAX_SPEED/2;
					 godmode = 0;
				 }
			if(otherIR1 < 150 || otherIR2 < 150){
				turning = 12;
				servo_angle = -US_TURN_A * SERVO_ANGLE_MULT;													//something is wrong, hard turn left
				r_speed = base_speed + US_TURN_SPEED * R_SPEEDUP / HARD_TURN_FACTOR;
				l_speed = base_speed - US_TURN_SPEED * L_SLOWDOWN / HARD_TURN_FACTOR;
			}else{																																	//systems normal, soft turn right
				r_speed = base_speed - US_TURN_SPEED * R_SLOWDOWN * SOFT_TURN_FACTOR;
				l_speed = base_speed + US_TURN_SPEED * L_SPEEDUP * SOFT_TURN_FACTOR;
			}
			
		} else { 
			/**SOFT_TURN_LEFT**/
			turning = 4;
			if(godmode){
					 base_speed = MAX_SPEED/2;
					 godmode = 0;
				 }
			if((otherIR1 < 150) || (otherIR2 < 150)){
				turning = 12;
				servo_angle = US_TURN_A * SERVO_ANGLE_MULT;														//something is wrong, hard turn right
				r_speed = base_speed - US_TURN_SPEED * R_SLOWDOWN / HARD_TURN_FACTOR;
				l_speed = base_speed + US_TURN_SPEED * L_SPEEDUP / HARD_TURN_FACTOR;
			}else{																																	//systems normal, soft turn left
				r_speed = base_speed + US_TURN_SPEED * R_SPEEDUP * SOFT_TURN_FACTOR;
				l_speed = base_speed - US_TURN_SPEED * L_SLOWDOWN * SOFT_TURN_FACTOR;
			}
		} 
	}else {
		/****go straight****/
		PF1 = 0x00;
		turning = 0;
		servo_angle = 0; l_speed = r_speed = base_speed;
	}
	
	/****************correct under/over flow and send the CAN message****************/
	if(r_speed < R_MIN)
		r_speed = R_MIN;
	if(l_speed < L_MIN)
		l_speed = L_MIN;
	if(servo_angle < MIN_ANGLE)
		servo_angle = MIN_ANGLE;
	else if(servo_angle > MAX_ANGLE)
		servo_angle = MAX_ANGLE;
	
	data[0] = 10;
	data[1] = l_speed;
	data[2] = r_speed;
	data[3] = servo_angle + 100;
	
	CAN0_SendData((uint8_t*) data);
} 

int main(void){
  OS_Init();
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
	OS_AddPeriodicThread(&pid,800000,1);
	OS_AddThread(&lcdDisplay,128,1);
	OS_Launch(TIME_1MS*10);
}
