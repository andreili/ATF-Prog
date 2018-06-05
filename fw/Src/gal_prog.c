#include "gal_prog.h"
#include <string.h>

void prog_init(GALProg_t* prog)
{
	prog->state = PROG_IDLE;
	prog->type = GAL_22V10;
	memset(prog->fuses_map, FUSE_DEFAULT_VAL, FUSES_MAP_SIZE);
}

int prog_proc(GALProg_t* prog)
{
	switch (prog->state)
	{
	case PROG_IDLE:
		return 1;
	case PROG_READ_FUSES:
		return 0;
	case PROG_ERASE_FUSES:
		return 0;
	case PROG_WRITE_FUSES:
		return 0;
	default:
		return 1;
	}
}
