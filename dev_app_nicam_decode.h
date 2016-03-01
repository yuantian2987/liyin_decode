
#ifndef _NICAM_DECODE_H
#define _NICAM_DECODE_H
#include "stdio.h"
#define PARITY_BIT  (1<<10)
#define MOST_HIGH_BIT   (1<<9)
#define RANGE_1 (0x7)
#define RANGE_2 (0x6)
#define RANGE_3 (0x5)
#define RANGE_4 (0x3)
#define RANGE_5_0   (0x4)
#define RANGE_5_1   (0x2)
#define RANGE_5_3   (0x1)
#define RANGE_5_4   (0x0)
#define NICAM_STEREO    (0x0)
#define NICAM_DUAL  (0x2)
#define NICAM_MONO  (0x4)

#define NICAM_PK_LEN_U8  (91)
#define NICAM_FAW_U8     (1)

#define APP_INPUT_BUFF_SIZE  0x4000
#define BUF_SIZE_NICAM_FRAME    (91+1)
#define NICAM_LARGE_BUF_SIZE   (APP_INPUT_BUFF_SIZE/4)
#define MAX_SAMPLES_NUM (APP_INPUT_BUFF_SIZE/2) //max sample num 8192

typedef unsigned int               UINT32;
typedef unsigned short              UINT16;
typedef int                        INT32;
typedef short                       INT16;
typedef unsigned char               UINT8;
typedef char                        INT8;
typedef void                        VOID;

#define mon_Event(...)

VOID  dev_app_init_nicam_decode(VOID);
INT32 dev_app_nicam_decode_data(UINT8 *data,UINT32 size,INT16 *out_left,INT16 *out_right);
INT32 dev_app_nicam_get_mon_dual_stereo(VOID);

#endif

