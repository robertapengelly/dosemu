/******************************************************************************
 * @file            video_native_other.c
 *****************************************************************************/
#if     !defined (__linux__) && !defined (_WIN32) && !defined (__APPLE__) || defined (__ANDROID__)
#include    "video_native.h"

int video_native_present (CPU8086 *cpu) {

    (void) cpu;
    return 0;

}

int video_native_active (void) {
    return 0;
}

int video_native_poll (CPU8086 *cpu) {

    (void) cpu;
    return 0;

}

void video_native_shutdown (void) {}
#endif
