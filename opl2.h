/******************************************************************************
 * @file            opl2.h
 *****************************************************************************/
#ifndef     _OPL2_H
#define     _OPL2_H

#include    <stdint.h>

void opl2_write (uint8_t reg, uint8_t value);
int opl2_sample (void);
void opl2_reset (void);

#endif      /* _OPL2_H */
