/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Some vectors may be platform dependent such as EINT
 * Move those vectors from irq-vectors to there :g!/EXI/d
 * use #ifdef MY_PLATFORM in case of shared IRQ
 * */

#ifndef _IRQ_VECTORS_PLF_H
#define _IRQ_VECTORS_PLF_H

#if defined (CONFIG_Z170C) || defined (CONFIG_Z170CG)
#define EXI4 0
#define EXI5 41
#define EXI7 56
#define EXI13 55
#define EXI15 61
#define EXI9 40
#define EXI11 0
#define EXI0 0
#define EXI1 57
#define EXI2 58
#define EXI6 60
#define EXI12 0
#define EXI8 118
#define EXI14 0
#define EXI10 122
#define EXI3 123

#endif //end of Z170

#if defined (CONFIG_Z370C) || defined (CONFIG_Z370CG)
#define EXI4 0
#define EXI5 41
#define EXI7 56
#define EXI13 55
#define EXI15 61
#define EXI9 40
#define EXI11 0
#define EXI0 0
#define EXI1 57
#define EXI2 58
#define EXI6 60
#define EXI12 0
#define EXI8 118
#define EXI14 0
#define EXI10 122
#define EXI3 123
#endif //end of Z370

#endif /*_IRQ_VECTORS_PLF_H */
