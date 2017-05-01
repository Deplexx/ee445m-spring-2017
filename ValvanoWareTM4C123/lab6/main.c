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
#define MAX_SPEED 85

#define SERVO_ANGLE_MULT 25 / 9

#define minProbDist 300  //min value to during ehich probably can change
#define maxProbDist 1000 //max values to during which probably can change

#define MAX_ANGLE 100
#define MIN_ANGLE -100
#define MIN_D 600 //distance at which US will slow down
//#define SLOWDOWN_FACTOR 20;
#define MIN_US_D 240
#define MAX_IR_D 799  // not used
#define MIN_IR_PAIR_D 1500 //eg. ir0 + ir1; distance to detect a general turn
//#define US_TURN_DETECT 1200 //ultrasonic override to initiate a hard turn;
#define US_TOO_CLOSE 180 //ultrasonice detection of being too close to a wall
#define US_NO_ROOM 400 // will not drift if there is not at least this much US room
#define US_DRIFT 40 // the amount ultrasonic_too_close will attempt to drift if too close to a wall
#define US_TURN_A 60 //harshness of a hard turn on servo
#define US_TURN_SPEED 60 //harshness of a hard turn on DC motors
#define HARD_TURN_A 70 // angle to detect a hard turn
#define SOFT_TURN_FACTOR 3 // divider on speed during soft turns
#define HARD_TURN_FACTOR 1 // divider on speed during hard turns
#define PROBABLY_DISTANCE 800

//#define D_PERTURB_MULT 1 / 50
//#define LD_THRESH 300
//#define LD_PERTURB_MULT D_PERTURB_MULT
//#define RD_THRESH 300
//#define RD_PERTURB_MULT D_PERTURB_MULT


//#define A_PERTURB_MULT 1
//#define LA_THRESH 70
//#define LA_PERTURB_MULT A_PERTURB_MULT
//#define RA_THRESH 70
//#define RA_PERTURB_MULT A_PERTURB_MULT

#define K_I 1
#define K_P 5/10

static int base_speed;
static int servo_angle;
static int car_fd;
static int car_ld;
static int car_rd;


#define R_MIN MIN_SPEED
#define R_SPEEDUP 1
#define R_SLOWDOWN 1
#define L_MIN MIN_SPEED
#define L_SPEEDUP 1
#define L_SLOWDOWN 1
static int l_speed, r_speed;

void StartOS(void);

int minFrontUS = 10000;
int probably;//0 for left, 1 for right
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
		ST7735_Message(0, 6, "min F US: ", minFrontUS);
		ST7735_Message(0, 7, "Preference: ", probably);
		
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
    US_In(USdata);
		car_rd = USdata[0];
		car_fd = USdata[1];
		car_ld = USdata[2];
    US_StartPing();
	}
  every10ms++;
  if(every10ms==10)
    every10ms = 0;
}

void canSend(int s){
  data[0] = s;
  CAN0_SendData(data);
}

int godmode;

void pid(void) {
	int speed_error;
	int ir0, ir1, ir2, ir3;
	int wall_angle, d1, d0;
	int dus;
	int wall_angle_l, wall_angle_r;
	static int la_hist=0, ra_hist=0;
	static int ld_hist=0, rd_hist=0;
	int la_error, ra_error;
	int ld_error, rd_error;
	int la_perturb, ra_perturb;
  int ld_perturb, rd_perturb;
	
	if(car_fd < minFrontUS){
		minFrontUS = car_fd;
	}
	
	//probably; set preference for  left turn or a right turn
	if(car_fd > 300 && car_fd<1000){
		if((car_ld - car_rd) > PROBABLY_DISTANCE){
			probably = 0;
		}else if ((car_rd - car_ld) > PROBABLY_DISTANCE){
			probably = 1;
		}
	}
	
	//speed
	speed_error = car_fd - MIN_D;
	if(speed_error > 0)
		base_speed = 100;
	else
		base_speed = base_speed * K_I + speed_error * K_P;
		
  if(base_speed < MIN_SPEED)
		base_speed = MIN_SPEED;
	else if(base_speed > MAX_SPEED)
		base_speed = MAX_SPEED;
	
	//steering
	ir0 = IRdata[0]; ir1 = IRdata[1];
	ir2 = IRdata[2]; ir3 = IRdata[3];
	
	wall_angle_l = get_wall_angle(ir0, ir1);
	wall_angle_r = get_wall_angle(ir3, ir2);
//	
//	la_error = wall_angle_l - la_hist;
//	ra_error = wall_angle_r - ra_hist;
//	
//	//integral of wall angles
//	if(la_error > LA_THRESH){
//		PF2 |= 0x04;
//		la_hist = wall_angle_l;
//		la_perturb = la_error * LA_PERTURB_MULT;
//	} else {
//		la_hist = (la_hist*9 + wall_angle_l + ((la_hist >=0) ? 5 : -5))/10;
//		la_perturb = 0;
//	}
//	if(ra_error > RA_THRESH){
//		PF3 |= 0x08;
//		ra_hist = wall_angle_r;
//		ra_perturb = ra_error * RA_PERTURB_MULT;
//	} else {
//		ra_hist = (ra_hist*9 + wall_angle_r + ((ra_hist >=0) ? 5 : -5))/10;	
//		ra_perturb = 0;
//	}

//	la_perturb = la_error * LA_PERTURB_MULT;		
//	la_hist = (la_hist*9 + wall_angle_l + ((la_hist >=0) ? 5 : -5))/10;
//	
//	ra_perturb = ra_error * RA_PERTURB_MULT;
//	ra_hist = (ra_hist*9 + wall_angle_r + ((ra_hist >=0) ? 5 : -5))/10;	

//	if((ir0 + ir1) > (ir2 + ir3)) {
//		wall_angle = -get_wall_angle(ir3, ir2); //turn left
//		d0 = ir2; d1 = ir3;
//	} else {
//		wall_angle = get_wall_angle(ir0, ir1); //turn right
//		d0 = ir0; d1 = ir1;
//	}

	if((ir0 + ir1) > (ir2 + ir3)) {
		wall_angle = -get_wall_angle(ir3, ir2); //turn left
		dus = car_rd; d0 = ir2; d1 = ir3;
	} else {
		wall_angle = get_wall_angle(ir0, ir1); //turn right
		dus = car_ld; d0 = ir0; d1 = ir1;
	}
	
	int US_d = car_rd + car_ld;
	int US_front = car_fd;
	int WA_right = get_wall_angle(ir3, ir2);
	int WA_left = get_wall_angle(ir0, ir1);
	
	if(d0 + d1 < MIN_IR_PAIR_D || US_front < 900) { // make turn
		PF1 = 0x02;
		turning = 1;
//	if(dus < MIN_US_D) { // make turn
		servo_angle = wall_angle * SERVO_ANGLE_MULT;
		
		if((WA_left < 55 && WA_left >35) && (WA_right < 55 && WA_right >35)){
			turning = 8;
			if(probably){//turn right
				servo_angle = US_TURN_A * SERVO_ANGLE_MULT;
				r_speed = base_speed - US_TURN_SPEED * R_SLOWDOWN / HARD_TURN_FACTOR;
				l_speed = base_speed + US_TURN_SPEED * L_SPEEDUP / HARD_TURN_FACTOR;
			}else{//turn left
				servo_angle = -US_TURN_A * SERVO_ANGLE_MULT;
				r_speed = base_speed + US_TURN_SPEED * R_SPEEDUP / HARD_TURN_FACTOR;
				l_speed = base_speed - US_TURN_SPEED * L_SLOWDOWN / HARD_TURN_FACTOR;
			}
		}else if(US_front<190){ // turn like a god
			turning = 9;
			godmode = 1;
			if(!probably){//turn left in place
				servo_angle = MIN_ANGLE;
				r_speed = MIN_SPEED;
				l_speed = MAX_SPEED;
			}else if(probably){//turn right in place
				servo_angle = MAX_ANGLE;
				r_speed = MAX_SPEED;
				l_speed = MIN_SPEED;
			}
		
		}else if(wall_angle_l > HARD_TURN_A || wall_angle_r > HARD_TURN_A
			 || wall_angle_l < -HARD_TURN_A || wall_angle_r < -HARD_TURN_A) { //detected hard turn
				 turning = 2;
				 if(godmode){
					 base_speed = MAX_SPEED/2;
					 godmode = 0;
				 }
			if(car_ld > car_rd) { //turn left
				servo_angle = -US_TURN_A * SERVO_ANGLE_MULT;
				r_speed = base_speed + US_TURN_SPEED * R_SPEEDUP / HARD_TURN_FACTOR;
				l_speed = base_speed - US_TURN_SPEED * L_SLOWDOWN / HARD_TURN_FACTOR;
			} else { //turn right
				servo_angle = US_TURN_A * SERVO_ANGLE_MULT;
				r_speed = base_speed - US_TURN_SPEED * R_SLOWDOWN / HARD_TURN_FACTOR;
				l_speed = base_speed + US_TURN_SPEED * L_SPEEDUP / HARD_TURN_FACTOR;
			}	
		} else if(wall_angle > 0) { //soft turn left
				turning = 3;
			if(godmode){
					 base_speed = MAX_SPEED/2;
					 godmode = 0;
				 }
			r_speed = base_speed - wall_angle * R_SPEEDUP / SOFT_TURN_FACTOR;
			l_speed = base_speed + wall_angle * L_SLOWDOWN / SOFT_TURN_FACTOR;
		} else { //soft turn right
			turning = 4;
			if(godmode){
					 base_speed = MAX_SPEED/2;
					 godmode = 0;
				 }
			r_speed = base_speed - wall_angle * R_SLOWDOWN / SOFT_TURN_FACTOR;
			l_speed = base_speed + wall_angle * L_SPEEDUP / SOFT_TURN_FACTOR;
		} 
  } else if(car_rd<US_TOO_CLOSE || car_ld<US_TOO_CLOSE){//too close to wall, tilt away
		PF1 = 0x00;
		turning = 7;
		if(US_d<=US_NO_ROOM){//there's no room to drift
			
		}else if (car_rd<US_TOO_CLOSE){//right wall too close, drift left
			r_speed = base_speed - US_DRIFT * R_SPEEDUP;
			l_speed = base_speed + US_DRIFT * L_SLOWDOWN;
		}else if (car_ld<US_TOO_CLOSE){//left wall too close, drift right
			r_speed = base_speed - US_DRIFT * R_SLOWDOWN;
			l_speed = base_speed + US_DRIFT * L_SPEEDUP;
		}
	}else {//go straight
		PF1 = 0x00;
		turning = 0;
		servo_angle = 0; l_speed = r_speed = base_speed;
	}
	
	//right-angle turn detected; turn regardless of IR sensor distance measurement
//	servo_angle += ra_perturb - la_perturb; //@TODO: check sign
//	r_speed += la_perturb - ra_perturb;
//	l_speed += ra_perturb - la_perturb;
	
//	servo_angle += rd_perturb - ld_perturb; //@TODO: check sign
//	r_speed += ld_perturb - rd_perturb;
//	l_speed += rd_perturb - ld_perturb;
	
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
