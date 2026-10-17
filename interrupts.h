/******************************************************************************
 * @file            interrupts.h
 *****************************************************************************/
#ifndef     _INTERRUPTS_H
#define     _INTERRUPTS_H

#include    "cpu.h"

/* Guest IVT and native DOS entry points. */
void init_ivt (CPU8086 *cpu);

void handle_interrupt (CPU8086 *cpu, uint8_t interrupt_number);
void native_interrupt (CPU8086 *cpu, uint8_t interrupt_number);

#endif      /* _INTERRUPTS_H */
