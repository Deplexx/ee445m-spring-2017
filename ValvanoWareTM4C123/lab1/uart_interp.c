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
#include <string.h>

#include "ADCT2ATrigger.h"
#include "PLL.h"
#include "UART.h"

#define IN_BUFF_SIZE 256
char inBuff[IN_BUFF_SIZE];

//debug code
int main(void) {
  char *currTok;

  PLL_Init(Bus50MHz);       // set system clock to 50 MHz
  UART_Init();              // initialize UART
  adc_init();

  UART_OutCRLF();

  while(true) {
    UART_OutString("> ");
    UART_InString(inBuff, IN_BUFF_SIZE);

    UART_OutCRLF();

    if(strlen(inBuff) > IN_BUFF_SIZE) {
      UART_OutStringCRLF("Commands must be <= 256 chars. Resulting buffer overflow may cause errors; exiting...");
      exit(1);
    }

    if(strcmp((currTok = strtok(inBuff, " \t")), "echo") == 0)
      UART_OutStringCRLF(strtok(NULL, " \t"));
    else if(strcmp((const char*) currTok, "quit") == 0) {
      UART_OutStringCRLF("Quitting...");
      break;
    } else if(strcmp((const char*) currTok, "adc") == 0)
      adc_runComm(strtok(NULL, " \t"));
    else if(strcmp((const char*) currTok, "") != 0)
      UART_OutStringCRLF("Command not found. Enter \"quit\" to quit.");
  }
}
