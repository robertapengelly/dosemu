/******************************************************************************
 * @file            memory.h
 *****************************************************************************/
#ifndef     _MEMORY_H
#define     _MEMORY_H

#include    <stdint.h>
#include    <stddef.h>

#include    "cpu.h"

extern unsigned mem_video_dirty;
extern int watch_suppress;

uint8_t mem_read8 (const uint8_t *mem, size_t mem_size, uint32_t addr);
uint16_t mem_read16 (uint8_t *mem, size_t mem_size, uint32_t addr);

void mem_write8 (uint8_t *mem, size_t mem_size, uint32_t addr, uint8_t value);
void mem_write16 (uint8_t *mem, size_t mem_size, uint32_t addr, uint32_t value);
void mem_write32 (uint8_t *mem, size_t mem_size, uint32_t addr, uint32_t value);

#endif      /* _MEMORY_H */
