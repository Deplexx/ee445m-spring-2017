#include "ST7735.h"
#include "lcd_runComm.h"

void lcd_runComm(const char *comm) {
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
