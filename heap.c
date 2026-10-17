/******************************************************************************
 * @file            heap.c
 *****************************************************************************/
#include    <string.h>

#include    "heap.h"
#include    "cpu.h"

void init_dos_heap (CPU8086 *cpu, uint16_t start_seg) {

    uint32_t phys = (uint32_t) start_seg << 4, total_paras;
    DOS_MCB first;
    
    /* 1. Zero out the entire 16-byte paragraph first */
    memset (&first, 0, sizeof (DOS_MCB)); 
    
    first.type = 'Z'; 
    first.owner = 0x0000; 
    
    total_paras = DOS_MEMORY_END_SEG;
    first.size = (uint16_t)(total_paras - start_seg - 1); 
    
    memcpy (&cpu->mem[phys], &first, sizeof (DOS_MCB));

}
