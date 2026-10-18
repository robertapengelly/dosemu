/******************************************************************************
 * @file            loader.h
 *****************************************************************************/
#ifndef     _LOADER_H
#define     _LOADER_H

#include    <stddef.h>
#include    <stdint.h>

#include    "cpu.h"

int dos_executable_memory (const char *path, uint16_t *paragraphs, int *is_mz);
int load_com_file (const char *path, CPU8086 *cpu, uint8_t *mem, size_t mem_size, uint16_t load_addr, size_t *out_size);
int load_mz_file (const char *path, CPU8086 *cpu, uint16_t base_seg, uint16_t available_paragraphs);

#endif      /* _LOADER_H */
