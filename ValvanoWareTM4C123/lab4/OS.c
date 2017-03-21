// *************os.c**************
// EE445M/EE380L.6 Lab 2, Lab 3 solution
// high level OS functions
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 3/9/17, valvano@mail.utexas.edu


#include "inc/hw_types.h"
#include "PLL.h"
#include "inc/tm4c123gh6pm.h"
#include "UART2.h"
#include "os.h"
#include "adc.h"
#include "ST7735.h"




//******prototypes from startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical(void);     // save previous I bit, disable interrupts, return previous I
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

// prototypes from OSasm.s
void ContextSwitch(void);
void StartOS(void);
void PendSV_Handler(void);



