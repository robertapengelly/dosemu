/******************************************************************************
 * @file            speaker.h
 *****************************************************************************/
#ifndef     _SPEAKER_H
#define     _SPEAKER_H

#include    <stdint.h>

void speaker_initialize (void);
void speaker_shutdown (void);

void speaker_update (uint8_t port61, uint8_t mode, uint16_t reload, int restart);

#endif      /* _SPEAKER_H */
