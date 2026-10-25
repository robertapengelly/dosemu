/******************************************************************************
 * @file            memory.c
 *****************************************************************************/
#include    "memory.h"
#include    "cpu.h"

uint8_t mem_read8 (const uint8_t *mem, size_t mem_size, uint32_t addr) {

    if (addr >= mem_size) {
        return 0;
    }
    
    return mem[addr];

}

uint16_t mem_read16 (uint8_t *mem, size_t size, uint32_t addr) {

    uint8_t low = mem_read8 (mem, size, addr);
    uint8_t high = mem_read8 (mem, size, (addr + 1) & 0xFFFFF); 
    
    return (uint16_t) low | ((uint16_t) high << 8);

}

unsigned mem_video_dirty = 0;
int watch_suppress = 0;

void mem_write8 (uint8_t *mem, size_t mem_size, uint32_t addr, uint8_t value) {
    
    if ((size_t) addr < mem_size) {
    
        mem[addr] = value;
        
        if (addr >= 0xA0000 && addr < 0xB0000) {
            mem_video_dirty |= 4;
        } else if (addr >= 0xB0000 && addr < 0xB8000) {
            mem_video_dirty |= 1;
        } else if (addr >= 0xB8000 && addr < 0xC0000) {
            mem_video_dirty |= 2;
        }
    
    }

}

void mem_write16 (uint8_t *mem, size_t mem_size, uint32_t addr, uint32_t value) {

    mem_write8 (mem, mem_size, addr, (uint8_t) (value & 0xff));
    mem_write8 (mem, mem_size, (addr + 1) & 0xFFFFF, (uint8_t) (value >> 8));

}

void mem_write32 (uint8_t *mem, size_t mem_size, uint32_t addr, uint32_t value) {

    mem_write16 (mem, mem_size, addr, (uint16_t) (value & 0xffff));
    mem_write16 (mem, mem_size, (addr + 2) & 0xFFFFF, (uint16_t) (value >> 16) & 0xffff);

}
