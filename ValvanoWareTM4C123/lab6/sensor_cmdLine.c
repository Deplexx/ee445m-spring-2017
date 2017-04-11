#include "cmdLine.h"
#include "sensor_cmdLine.h"
#include "USSensor.h"

#include <stdio.h>
#include <string.h>

void sensor_runComm(int argc, char argv[][ARGV_TOK_SIZE]) {
	if(strcmp(argv[1], "ulso") == 0) {
		printf("US Sensor Distance: %d\n\r", USSensor_GetDistance());
		printf("US Start Time: %d\n\r", USSensor_GetDistance());
	} else
		printf("Invalid sensor command.\n\nr");
}
