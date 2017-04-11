#include "cmdLine.h"
#include "sensor_cmdLine.h"
#include "USSensor.h"
#include "ST7735.h"

#include <stdio.h>
#include <string.h>

void sensor_runComm(int argc, char argv[][ARGV_TOK_SIZE]) {
	static char dist[256];
	static char time[256];
	if(strcmp(argv[1], "ulso") == 0) {
		snprintf(dist, 256, "US Sensor Distance: %d cm\n\r", USSensor_GetDistance());
		snprintf(time, 256, "US Start Time: %d cm\n\r", USSensor_GetDistance());
		printf(dist, 256, "US Sensor Distance: %d cm\n\r", USSensor_GetDistance());
		printf(time, 256, "US Start Time: %d cm\n\r", USSensor_GetDistance());
		ST7735_OutString(dist);
		ST7735_OutString(time);
	} else
		printf("Invalid sensor command.\n\nr");
}
