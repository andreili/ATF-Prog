#ifndef _GAL_DEF_H_
#define _GAL_DEF_H_

#define FUSES_MAX 7000
#define FUSES_MAP_SIZE (FUSES_MAX / 8)

#define FUSE_DEFAULT_VAL 0xff

typedef enum
{
	GAL_16V8 = 0,
	GAL_20V8 = 1,
	GAL_22V10= 2,
} EGALType;

typedef enum
{
	ROWS_16 = 32,
	ROWS_20 = 40,
	ROWS_22 = 44,
	ROW_ACW = 60,
	ROW_PRT = 61,
	ROW_ERA = 63,
} EGALRows;

typedef enum
{
	COLS_16 = 64,
	COLS_20 = 64,
	COLS_22 = 132,
} EGALCols;

typedef enum
{
	OFFS_16 = 2048,
	OFFS_20 = 2560,
	OFFS_22 = 5808,
} EGALACWOffs;

typedef enum
{
	ACW_16 = 82,
	ACW_20 = 16,
	ACW_22 = 20,
} EGALACWBits;

#define GAL_SIGN_BITS 64

#endif //_GAL_DEF_H_
