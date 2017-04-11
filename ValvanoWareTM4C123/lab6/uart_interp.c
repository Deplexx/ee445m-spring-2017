// UARTIntsTestMain.c
// Runs on LM4F120/TM4C123
// Tests the UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// September 12, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Program 5.11 Section 5.6, Program 3.10

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

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "os.h"
#include "PLL.h"
#include "ST7735.h"
#include "lcd_cmdLine.h"
#include "uart_interp.h"
#include "sensor_cmdLine.h"
#include "UART.h"
#include "cmdLine.h"

static const char UP_ARROW[4] = {27,91,65, '\0'};

#define CMD_LINE_BUFF_SIZE 10
#define CMD_LINE_SIZE 256
static char cmd_line_buff[CMD_LINE_BUFF_SIZE][CMD_LINE_SIZE];

#define ARGV_SIZE 20
static char argv[ARGV_SIZE][ARGV_TOK_SIZE];
static int argc;

static char *cur_cmd_line_ptr = cmd_line_buff[0];
static int cur_cmd_line_indx = 0;
static int cmd_line_buff_tail = 0;

static void read_cmd_line(void);
static void put_cmd_line(char *cmd_line);
static void show_prev_cmd_line(int *cmd_line_len);
static void show_next_cmd_line(int *cmd_line_len);
static void show_cmd_line(int *cmd_line_len, int prev);

void Interpreter(void) {		
  while(true) {
		read_cmd_line();		
		
    if(strcmp(argv[0], "echo") == 0)
      printf(argv[1]);
    else if(strcmp(argv[0], "quit") == 0) {
      printf("Quitting...");
      break;
    } else if(strcmp(argv[0], "lcd") == 0)
      lcd_runComm(argc, argv);
		else if(strcmp(argv[0], "sensor") == 0)
			sensor_runComm(argc, argv);
    else if(strcmp(argv[0], "") != 0)
      printf("Command not found. Enter \"quit\" to quit.");
  }
}

static void read_cmd_line(void) {
	static char tokenized_cur_cmd_line[CMD_LINE_SIZE];
	int len = argc = 0;
	char c;
	cur_cmd_line_indx = cmd_line_buff_tail++;
	cmd_line_buff_tail %= CMD_LINE_BUFF_SIZE;
	cur_cmd_line_ptr = (char*) &cmd_line_buff[cur_cmd_line_indx];
	for(int i = 0; i < CMD_LINE_SIZE; ++i) 
		cur_cmd_line_ptr[i] = '\0';
	
	strcpy(argv[0], "");	
	
	printf("\r\n> ");
	
  do {
		c = UART_InChar();
		if(c == CR) {
			cur_cmd_line_ptr[len] = '\0';
		} else if(c == BS) {
      if(len > 0) {
				UART_OutChar(BS); 
				cur_cmd_line_ptr[len--] = '\0';
				continue;
			}
		} else if(strncmp(cur_cmd_line_ptr, UP_ARROW, strlen(UP_ARROW)) == 0)
			show_prev_cmd_line(&len);
		else if(len < (CMD_LINE_SIZE - 1)) {
      cur_cmd_line_ptr[len++] = c;
      UART_OutChar(c);
    }
  } while(c != CR);
	
	snprintf(tokenized_cur_cmd_line, CMD_LINE_SIZE, cur_cmd_line_ptr);
	
	for(char *s = strtok(tokenized_cur_cmd_line, " \t"); 
		s != NULL; s = strtok(NULL, " \t")) {
		strncpy(argv[argc++], s, ARGV_TOK_SIZE);
  }
		
	printf("\r\n");
}

static void show_prev_cmd_line(int *cmd_line_len) {
	show_cmd_line(cmd_line_len, 1);
}

static void show_next_cmd_line(int *cmd_line_len) {
	show_cmd_line(cmd_line_len, 0);
}


static void show_cmd_line(int *cmd_line_len, int prev) {
	if(prev == 1) --cur_cmd_line_indx;
	else if(prev == 0) ++cur_cmd_line_indx;			
  for(int i = 0; i < (*cmd_line_len = strlen(cur_cmd_line_ptr)); ++i)
		UART_OutChar(BS);	
	cur_cmd_line_ptr = (char*) &cmd_line_buff[cur_cmd_line_indx];
	UART_OutString(cur_cmd_line_ptr);	
}
