//IR.c
//
//

#include <stdint.h>
#include "IR.h"
#include "inc/tm4c123gh6pm.h"
/* Calibration data
distance measured from front of the sensor to the wall                
d(cm) 1/d    bL     al     aR   bR  adcSample d (0.01cm)  error
10    0.100  2813  2830  2820  2830  2823.25  1006        0.06
15    0.067  1935  1976  1986  1978  1968.75  1482       -0.18
20    0.050  1520  1500  1520  1550  1522.5   1966       -0.34
30    0.033  1040  1096  1028   933  1024.25  3099        0.99
  
      adcSample = 26813/d+159      2681300    
      d = 26813/(adcSample-159)      -159    
*/
//------------ADC2millimeter------------
// convert 12-bit ADC to distance in 1mm
// it is known the expected range is 100 to 800 mm
// Input:  adcSample 0 to 4095 
// Output: distance in 1mm
uint32_t ADC2millimeter(uint32_t adcSample){
  if(adcSample<494) return 799; // maximum distance 80cm
  return (268130/(adcSample-159));  
}
// Initializes ADC0(PE3), ADC1(PE2), ADC2(PE1) and ADC3(PE0) sampling
// 125k max sampling
// SS2 triggering event: software trigger, busy-wait sampling
// SS2 1st sample source: Ain0 (PE3)
// SS2 2nd sample source: Ain1 (PE2)
// SS2 3rd sample source: Ain2 (PE1)
// SS2 4th sample source: Ain3 (PE0)
// SS2 interrupts: enabled after 4th sample but not promoted to controller
void IR_Init(void){
  volatile uint32_t delay;                         
  SYSCTL_RCGCADC_R |= 0x00000001; // 1) activate ADC0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // 1) activate clock for Port E
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTE_DIR_R &= ~0x0F;      // 3) make PE3-0 input
  GPIO_PORTE_AFSEL_R |= 0x0F;     // 4) enable alternate function on PE3-0
  GPIO_PORTE_DEN_R &= ~0x0F;      // 5) disable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = GPIO_PORTE_PCTL_R&0xFFFF0000;
  GPIO_PORTE_AMSEL_R |= 0x0F;     // 6) enable analog functionality on PE3-0
  ADC0_PC_R &= ~0xF;              // 8) clear max sample rate field
//  ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
	ADC0_PC_R |= 0x7;               //    configure for 1000K samples/sec	
  ADC0_SSPRI_R = 0x3210;          // 9) Sequencer 3 is lowest priority
  ADC0_ACTSS_R &= ~0x0004;        // 10) disable sample sequencer 2
  ADC0_EMUX_R &= ~0x0F00;         // 11) seq2 is software trigger
	ADC0_SAC_R = 0x06;              // 32-point average
	//  ADC0_SAC_R = 0x06;              // 32-point average
  ADC0_SSMUX2_R = 0x0123;         // 12) set channels for SS2
  ADC0_SSCTL2_R = 0x6000;         // 13) no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
  ADC0_IM_R &= ~0x0004;           // 14) disable SS2 interrupts
  ADC0_ACTSS_R |= 0x0004;         // 15) enable sample sequencer 2
}

void IR_In(uint32_t data[4]){
	ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
  data[3] = ADC2millimeter(ADC0_SSFIFO2_R&0xFFF);  // 3A) PE3 result
  data[2] = ADC2millimeter(ADC0_SSFIFO2_R&0xFFF);  // 3A) PE2 result
  data[1] = ADC2millimeter(ADC0_SSFIFO2_R&0xFFF);  // 3A) PE1 result
  data[0] = ADC2millimeter(ADC0_SSFIFO2_R&0xFFF);  // 3B) PE0 result
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}