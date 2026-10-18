/******************************************************************************
 * @file            psp.h
 *****************************************************************************/
#ifndef     _PSP_H
#define     _PSP_H

#include    <stdint.h>
#include    "cpu.h"

void init_psp_contents (CPU8086 *cpu, const char *args, uint16_t psp_seg, uint16_t parent_psp, uint16_t env_seg, uint16_t end_seg);

#endif      /* _PSP_H */