/******************************************************************************
 * @file            video_native.h
 *****************************************************************************/
#ifndef     VIDEO_NATIVE_H
#define     VIDEO_NATIVE_H

#include    "cpu.h"

int video_native_present (CPU8086 *cpu, int mode, int blink_enabled);

uint32_t video_native_cga_pixel (CPU8086 *cpu, unsigned x, unsigned y, unsigned mode);
void video_native_cga_render (CPU8086 *cpu, uint32_t *pixels, unsigned mode);

void video_native_text_render (CPU8086 *cpu, uint32_t *pixels, int blink_enabled);

int video_native_active (void);
int video_native_poll (CPU8086 *cpu);

void video_native_shutdown (void);

#endif
