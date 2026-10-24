/******************************************************************************
 * @file            heap.h
 *****************************************************************************/
#ifndef     _HEAP_H
#define     _HEAP_H

#include    <stdint.h>

/* Reserve the last 16 paragraphs before the resident interrupt stubs for DOS system variables. */
#define     DOS_MEMORY_END_SEG          0x9FA0

#define     DOS_LIST_SEG                DOS_MEMORY_END_SEG
#define     DOS_LIST_OFFSET             0x0010

#define     DOS_IVT_STUB_SEG            0x9FB0
#define     DOS_CASE_MAP_SEG            0x9FF0

typedef struct __attribute__ ((packed)) {

    uint8_t type;                       /* 'M' (0x4D) for intermediate blocks, 'Z' (0x5A) for the last block */
    
    uint16_t owner;                     /* PSP segment of the owner (0 means free) */
    uint16_t size;                      /* Size of the block in paragraphs (16-byte chunks) */
    
    uint8_t reserved[3];
    uint8_t name[8];                    /* Optional: Program name */

} DOS_MCB;

#endif      /* _HEAP_H */
