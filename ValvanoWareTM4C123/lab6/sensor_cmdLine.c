#include "cmdLine.h"
#include "sensor_cmdLine.h"
#include "US.h"
#include "ST7735.h"
#include "can0.h"

#include <stdio.h>
#include <string.h>

void sensor_runComm(int argc, char argv[][ARGV_TOK_SIZE]) {
	static char dist[256];
	static char time[256];
  static char ir0[256];
  static char ir1[256];
  static char ir2[256];
  static char ir3[256];
	if(strcmp(argv[1], "ulso") == 0) {
		snprintf(dist, 256, "US Sensor Distance: %d cm\n\r", Timer0_Read());
		snprintf(time, 256, "US Start Time: %d cm\n\r", Timer0_Read());
		printf(dist, 256, "US Sensor Distance: %d cm\n\r", Timer0_Read());
		printf(time, 256, "US Start Time: %d cm\n\r", Timer0_Read());
		ST7735_OutString(dist);
		ST7735_OutString(time);
	}
  if(strcmp(argv[1], "can") == 0) {
    if(strcmp(argv[2], "get") == 0) {
      uint8_t data[4];
      CAN0_GetMailNonBlock(data);
		  snprintf(ir0, 256, "IR0: %d cm\n\r", data[0]);
		  snprintf(ir1, 256, "IR1: %d cm\n\r", data[1]);
		  snprintf(ir2, 256, "IR2: %d cm\n\r", data[2]);
		  snprintf(ir3, 256, "IR3: %d cm\n\r", data[3]);
		  printf(ir0, 256, "IR0: %d cm\n\r", data[0]);
		  printf(ir1, 256, "IR1: %d cm\n\r", data[1]);
		  printf(ir2, 256, "IR2: %d cm\n\r", data[2]);
		  printf(ir3, 256, "IR3: %d cm\n\r", data[3]);
		  ST7735_OutString(ir0);
		  ST7735_OutString(ir1);
		  ST7735_OutString(ir2);
		  ST7735_OutString(ir3);
    }else
    if(strcmp(argv[2], "put") == 0) {
      uint8_t data[4];
      uint32_t usdist = Timer0_Read();
      data[0] = usdist & 0x000000FF;
      data[1] = (usdist & 0x0000FF00) >> 8;
      data[2] = (usdist & 0x00FF0000) >> 16;
      data[3] = (usdist & 0xFF000000) >> 24;
      CAN0_SendData(data);
    }
	}
  else
		printf("Invalid sensor command.\n\nr");
}
