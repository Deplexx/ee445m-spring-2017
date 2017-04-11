#include <stdlib.h>
#include <string.h>

#include "lcd_cmdLine.h"
#include "ST7735.h"

void lcd_runComm(int argc, char argv[][ARGV_TOK_SIZE]) {
	if(strcmp(argv[1], "clear") == 0)
		Output_Clear();
	else if(strcmp(argv[1], "echo") == 0)
		//void ST7735_Message(int device, int line, char *string, int value) 
		ST7735_Message(0, 0, argv[2], atoi(argv[3]));
}
