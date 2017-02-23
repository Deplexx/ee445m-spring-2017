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

#include "ADC.h"
#include "os.h"
#include "PLL.h"
#include "ST7735.h"
#include "UART.h"
#include "uart_interp.h"

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

void Interpreter(void) {
  char *currTok;

  //PLL_Init(Bus80MHz);       // set system clock to 50 MHz
  //UART_Init();              // initialize UART

  UART_OutCRLF();
  UART_OutString("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  UART_OutString("abcdefghijklmnopqrstuvwxyz");
  UART_OutString("0123456789");
  
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
}

static void adc_runComm(const char *comm) {
    char *currComm = strtok((char*)comm, " \t");

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
        ADC_Init(channel);//, fs);
    } else if(strcmp(currComm, "print") == 0) {
        UART_OutString("ADC Output: Always Nothing");
        //UART_OutUDec(ADC_Val());
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

        ADC_Collect(channelNum, fs, NULL);
    } else
      UART_OutStringCRLF("Invalid adc command. Type \"adc -h\" for a list of commands.");
}

static void os_runComm(const char *comm) {
//    char *currTok = strtok(comm, " \t");
//
//    if(strcmp((const char*) currTok, "on") == 0)
//      OS_On();
//    else if(strcmp((const char*) currTok, "count") == 0) {
//      UART_OutString("OS task count: "); UART_OutUDec(OS_ReadPeriodicTime()); UART_OutCRLF();
//    }
//    else if(strcmp((const char*) currTok, "clear") == 0) {
//      UART_OutStringCRLF("Clearing os count...");
//      OS_ClearPeriodicTime();
//    }
//    else
//        UART_OutStringCRLF("Invalid os command. Type \"os -h\" for a list of commands.");
}

static void lcd_runComm(const char *comm) {
    char *currTok = (char*)comm;

    if(strcmp((const char*) currTok, "on") == 0)
        ST7735_InitR(INITR_REDTAB);
    else if(strcmp((const char*) currTok, "echo") == 0) {
        int device = nextInt();
        int line = nextInt();
        char* string = strtok(NULL, " \t");
        char *valStr = strtok(NULL, " \t");
        uint32_t value = atoi(valStr);

        if(device == NULL)
            device = 0;
        else if(device > 1 || device < 0) {
            UART_OutStringCRLF("Invalid device; enter a number in {0,1}");
            return;
        }

        if(line == NULL)
            line = 0;
        else if(line > 3 || line < 0) {
            UART_OutStringCRLF("Invalid line; enter a number <= 3 && >= 1");
            return;
        }

        if(string == NULL) {
            UART_OutStringCRLF("You must provide a message.");
            return;
        }

        if(valStr == NULL) {
            UART_OutStringCRLF("You must provide a value.");
            return;
        }


        ST7735_Message(device, line, string, value);
    } else if(strcmp(currTok, "clear") == 0)
      Output_Clear();
    else
      UART_OutStringCRLF("Invalid lcd command. Type \"lcd -h\" for a list of commands.");
}
