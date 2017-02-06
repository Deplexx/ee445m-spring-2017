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
#include "OS.h"
#include "PLL.h"
#include "ST7735_.h"
#include "UART.h"

#define IN_BUFF_SIZE 256
char inBuff[IN_BUFF_SIZE];

#define TEST_BUFF_SIZE 256
uint16_t adcBuff[TEST_BUFF_SIZE];



static inline int nextInt() {
  return atoi(strtok(NULL, " \t"));
}

static void adc_runComm(const char *comm);
static void os_runComm(const char *comm);
static void lcd_runComm(const char *comm);

int notmain(void) {
    PLL_Init(Bus50MHz);       // set system clock to 50 MHz
    UART_Init();              // initialize UART
    ADC_Collect(0, 5000000, adcBuff, TEST_BUFF_SIZE);

    for(int i = 0; i < TEST_BUFF_SIZE; ++i) {
        UART_OutUDec(adcBuff[i]);
        UART_OutCRLF();
    }

    return 0;
}

int main(void) {
  char *currTok;

  PLL_Init(Bus80MHz);       // set system clock to 50 MHz
  UART_Init();              // initialize UART

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
    else if(strcmp((const char*) currTok, "os") == 0)
      os_runComm(strtok(NULL, " \t"));
    else if(strcmp((const char*) currTok, "lcd") == 0)
      lcd_runComm(strtok(NULL, " \t"));
    else if(strcmp((const char*) currTok, "") != 0)
      UART_OutStringCRLF("Command not found. Enter \"quit\" to quit.");
  }

  return 0;
}

static void adc_runComm(const char *comm) {
    char *currComm = strtok(comm, " \t");

    if(strcmp(currComm, "on") == 0) {
        int channel = nextInt();
        int fs = nextInt();

        if(channel == NULL)
          channel = 0;
        else if(channel < 0) {
          UART_OutStringCRLF("Invalid channel number. Enter a number <= 11 && >= 0");
          return;
        }

        if(fs == NULL)
          fs = 1;
        else if(fs < 1) {
          UART_OutStringCRLF("Invalid frequency. Enter a number <= 11 && >= 0");
          return;
        }

        UART_OutStringCRLF("Turning ADC0 on...");
        ADC_TurnOn(channel, fs);
    } else if(strcmp(currComm, "print") == 0) {
        UART_OutString("ADC Output: ");
        UART_OutUDec(ADC_Val());
        UART_OutCRLF();

        return;
    } else if(strcmp(currComm, "collect")) {
        int channelNum = nextInt();
        int fs = nextInt();
        int numberOfSamples = nextInt();

        if(numberOfSamples == NULL)
            numberOfSamples = 1;
        else if(numberOfSamples > 256) {
          UART_OutStringCRLF("Invalid number of samples; enter a number >= 1.");
          return;
        }

        if(channelNum == NULL)
          channelNum = 0;
        else {
          UART_OutStringCRLF("Invalid channel number; enter a number <= 11 && >= 0");
          return;
        }

        if(fs == NULL)
          fs = 1;
        if(fs < 1) {
          UART_OutStringCRLF("Invalid sample frequency; enter a number <= 125000 && >= 1.");
          return;
        }

        ADC_Collect(channelNum, fs, adcBuff, numberOfSamples);
    } else
      UART_OutStringCRLF("Invalid adc command. Type \"adc -h\" for a list of commands.");
}

static void os_runComm(const char *comm) {
    char *currTok = strtok(comm, " \t");

    if(strcmp((const char*) currTok, "on") == 0)
      OS_On();
    else if(strcmp((const char*) currTok, "count") == 0) {
      UART_OutString("OS task count: "); UART_OutUDec(OS_ReadPeriodicTime()); UART_OutCRLF();
    }
    else if(strcmp((const char*) currTok, "clear") == 0) {
      UART_OutStringCRLF("Clearing os count...");
      OS_ClearPeriodicTime();
    }
    else
        UART_OutStringCRLF("Invalid os command. Type \"os -h\" for a list of commands.");
}

static void lcd_runComm(const char *comm) {
    char *currTok = comm;

    if(strcmp((const char*) currTok, "on") == 0)
        ST7735_InitR(INITR_REDTAB);
    else if(strcmp((const char*) currTok, "echo") == 0) {
        char* str = strtok(NULL, " \t");

        if(str == NULL) {
            UART_OutStringCRLF("Invalid message; enter a valid one.");
            return;
        }

        ST7735_OutString(str);
    } else
      UART_OutStringCRLF("Invalid lcd command. Type \"lcd -h\" for a list of commands.");
}
