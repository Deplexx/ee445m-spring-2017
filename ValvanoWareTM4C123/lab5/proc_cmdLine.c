#include "ST7735.h"
#include "loader.h"
#include "proc_cmdLine.h"

static ELFSymbol_t symbols[1] = {
	{"ST7735_Message", &ST7735_Message}
};
static ELFEnv_t env ={(const ELFSymbol_t *) &symbols[0], 1};

void proc_runComm(int argc, char argv[][ARGV_TOK_SIZE]) {
	exec_elf(argv[1], &env);
}
	