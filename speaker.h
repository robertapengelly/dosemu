/******************************************************************************
 * @file            speaker.h
 *****************************************************************************/
#ifndef     _SPEAKER_H
#define     _SPEAKER_H

#include    <stddef.h>
#include    <stdint.h>

void speaker_initialize (void);
void speaker_shutdown (void);

void speaker_update (uint8_t port61, uint8_t mode, uint16_t reload, int restart);

void speaker_sb_write (const uint8_t *samples, size_t count, unsigned rate);
void speaker_sb_reset (void);

#endif      /* _SPEAKER_H */
