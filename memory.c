/******************************************************************************
 * @file            memory.c
 *****************************************************************************/
#include    <string.h>

#include    "memory.h"
#include    "cpu.h"

static CPU8086 *vga_cpu;

static uint8_t vga_planes[4][65536];
static uint8_t vga_latches[4];

void mem_vga_attach (CPU8086 *cpu) {

    vga_cpu = cpu;
    
    memset (vga_planes, 0, sizeof (vga_planes));
    memset (vga_latches, 0, sizeof (vga_latches));

}

void mem_vga_clear (CPU8086 *cpu) {

    memset (vga_planes, 0, sizeof (vga_planes));
    memset (vga_latches, 0, sizeof (vga_latches));
    
    memset (cpu->mem + 0xA0000, 0, 0x10000);
    mem_video_dirty |= 4;

}

uint8_t mem_vga_pixel (CPU8086 *cpu, unsigned x, unsigned y) {

    if (cpu != vga_cpu || x >= 320 || y >= 200) {
        return 0;
    }

    return vga_planes[x & 3][y * 80 + (x >> 2)];

}

static int mem_vga_address (const uint8_t *mem, uint32_t addr) {
    return vga_cpu && vga_cpu->vga_graphics_mode && mem == vga_cpu->mem && addr >= 0xA0000 && addr < 0xB0000;
}

static uint32_t mem_vga_offset (uint32_t addr) {

    uint32_t offset = addr - 0xA0000;
    
    if (vga_cpu->vga_seq_registers[4] & 0x08) {
        return offset >> 2;
    }
    
    return offset;

}

static unsigned mem_vga_read_plane (uint32_t addr) {

    if (vga_cpu->vga_seq_registers[4] & 0x08) {
        return (addr - 0xA0000) & 3;
    }
    
    return vga_cpu->vga_gc_registers[4] & 3;

}

static void mem_vga_write (uint32_t addr, uint8_t value) {

    uint32_t offset = mem_vga_offset (addr);
    
    uint8_t map_mask = vga_cpu->vga_seq_registers[2];
    uint8_t bit_mask = vga_cpu->vga_gc_registers[8];
    uint8_t rotate = vga_cpu->vga_gc_registers[3] & 7;
    
    unsigned plane;
    value = (uint8_t) ((value >> rotate) | (value << ((8 - rotate) & 7)));
    
    if (vga_cpu->vga_seq_registers[4] & 0x08) {
        map_mask &= (uint8_t) (1 << ((addr - 0xA0000) & 3));
    }
    
    for (plane = 0; plane < 4; plane++) {
    
        uint8_t data = value;
        
        if (!(map_mask & (1 << plane))) {
            continue;
        }
        
        if ((vga_cpu->vga_gc_registers[5] & 3) == 1) {
        
            vga_planes[plane][offset] = vga_latches[plane];
            continue;
        
        }
        
        if (vga_cpu->vga_gc_registers[1] & (1 << plane)) {
            data = (vga_cpu->vga_gc_registers[0] & (1 << plane)) ? 0xFF : 0;
        }
        
        switch (vga_cpu->vga_gc_registers[3] & 0x18) {
        
            case 0x08:
            
                data &= vga_latches[plane];
                break;
            
            case 0x10:
            
                data |= vga_latches[plane];
                break;
            
            case 0x18:
            
                data ^= vga_latches[plane];
                break;
        
        }
        
        vga_planes[plane][offset] = (uint8_t) ((data & bit_mask) | (vga_latches[plane] & ~bit_mask));
    
    }
    
    mem_video_dirty |= 4;

}

uint8_t mem_read8 (const uint8_t *mem, size_t mem_size, uint32_t addr) {

    if (addr >= mem_size) {
        return 0;
    }
    
    if (mem_vga_address (mem, addr)) {
    
        uint32_t offset = mem_vga_offset (addr);
        unsigned plane;
        
        for (plane = 0; plane < 4; plane++) {
            vga_latches[plane] = vga_planes[plane][offset];
        }
        
        return vga_latches[mem_vga_read_plane (addr)];
    
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
    
        if (mem_vga_address (mem, addr)) {
            mem_vga_write (addr, value);
        } else {
            mem[addr] = value;
        }
        
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
