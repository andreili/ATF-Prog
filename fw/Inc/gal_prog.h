#ifndef _GAL_PROG_H_
#define _GAL_PROG_H_

#include "gal_def.h"
#include "stm32f1xx_hal.h"

typedef enum
{
	PROG_IDLE = 0,
	PROG_READ_FUSES = 1,
	PROG_ERASE_FUSES = 2,
	PROG_WRITE_FUSES = 3,
} EState;

typedef struct
{
	EState		state;
	EGALType	type;
	uint8_t 	fuses_map[FUSES_MAP_SIZE];
} GALProg_t;

void prog_init(GALProg_t* prog);
int prog_proc(GALProg_t* prog);

#endif //_GAL_PROG_H_
