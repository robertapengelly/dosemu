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

const char *dos_register_initial_program (const char *path);

void bios_timer_refresh (CPU8086 *cpu);
void bios_keyboard_poll (CPU8086 *cpu);
void bios_video_refresh (CPU8086 *cpu);

#if     defined (_WIN32)
void host_windows_keyboard_initialize (void);
void host_windows_keyboard_shutdown (void);
#endif

#endif      /* _INTERRUPTS_H */
