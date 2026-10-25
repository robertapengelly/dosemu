/******************************************************************************
 * @file            video_native.h
 *****************************************************************************/
#ifndef     VIDEO_NATIVE_H
#define     VIDEO_NATIVE_H

#include    "cpu.h"

int video_native_present (CPU8086 *cpu);
int video_native_active (void);
int video_native_poll (CPU8086 *cpu);

void video_native_shutdown (void);

#endif
