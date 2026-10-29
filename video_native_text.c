/******************************************************************************
 * @file            video_native_text.c
 *****************************************************************************/
#include    <stdint.h>

#include    "video_native.h"

void video_native_text_render (CPU8086 *cpu, uint32_t *pixels, int blink_enabled) {

    static const uint8_t vga16[16][3] = {
    
        { 0, 0, 0 }, { 0, 0, 42 }, { 0, 42, 0 }, { 0, 42, 42 },
        { 42, 0, 0 }, { 42, 0, 42 }, { 42, 21, 0 }, { 42, 42, 42 },
        { 21, 21, 21 }, { 21, 21, 63 }, { 21, 63, 21 }, { 21, 63, 63 },
        { 63, 21, 21 }, { 63, 21, 63 }, { 63, 63, 21 }, { 63, 63, 63 }
    
    };
    
    uint32_t colors[16];
    unsigned row, col, x, y, i;
    
    for (i = 0; i < 16; i++) {
    
        colors[i] = ((uint32_t) (vga16[i][0] * 255 + 31) / 63 << 16) |
            ((uint32_t) (vga16[i][1] * 255 + 31) / 63 << 8) | (uint32_t) (vga16[i][2] * 255 + 31) / 63;
    
    }
    
    for (row = 0; row < 100; row++) {
    
        for (col = 0; col < 80; col++) {
        
            unsigned address = 0xB8000 + (row * 80 + col) * 2;
            
            uint8_t ch = cpu->mem[address];
            uint8_t attr = cpu->mem[address + 1];
            uint8_t foreground = attr & 0x0F;
            uint8_t background = (attr >> 4) & (blink_enabled ? 0x07 : 0x0F);
            
            uint32_t left = colors[background];
            uint32_t right = colors[background];
            
            if (ch == 0xDE) {
                right = colors[foreground];
            } else if (ch == 0xDD) {
                left = colors[foreground];
            } else if (ch == 0xDB) {
                left = right = colors[foreground];
            }
            
            for (y = 0; y < 4; y++) {
            
                uint32_t *line = pixels + (row * 4 + y) * 640 + col * 8;
                
                for (x = 0; x < 4; x++) {
                
                    line[x] = left;
                    line[x + 4] = right;
                
                }
            
            }
        
        }
    
    }

}

uint32_t video_native_cga_pixel (CPU8086 *cpu, unsigned x, unsigned y, unsigned mode) {

    static const uint32_t colors[16] = {
    
        0x000000, 0x0000AA, 0x00AA00, 0x00AAAA,
        0xAA0000, 0xAA00AA, 0xAA5500, 0xAAAAAA,
        0x555555, 0x5555FF, 0x55FF55, 0x55FFFF,
        0xFF5555, 0xFF55FF, 0xFFFF55, 0xFFFFFF
    
    };
    
    uint32_t offset = 0xB8000 + (y & 1) * 0x2000 + (y >> 1) * 80;
    
    uint8_t color;
    uint8_t value;
    uint8_t pixel;
    
    if (mode == 0x06) {
    
        value = cpu->mem[offset + (x >> 3)];
        color = (value & (0x80 >> (x & 7))) ? (cpu->cga_color_select & 0x0F) : 0;
    
    } else {
    
        value = cpu->mem[offset + (x >> 2)];
        
        pixel = (value >> (6 - (x & 3) * 2)) & 3;
        color = pixel;
        
        if (pixel == 0) {
            color = cpu->cga_color_select & 0x0F;
        } else if (mode == 0x05) {
            color = (uint8_t) ((const uint8_t[]) { 0, 3, 4, 7 })[color];
        } else if (cpu->cga_color_select & 0x20) {
            color = (uint8_t) ((const uint8_t[]) { 0, 3, 5, 7 })[color];
        } else {
            color = (uint8_t) (color * 2);
        }
        
        if ((cpu->cga_color_select & 0x10) && pixel != 0) {
            color |= 8;
        }
    
    }
    
    return colors[color];

}

void video_native_cga_render (CPU8086 *cpu, uint32_t *pixels, unsigned mode) {

    unsigned x, y;
    
    for (y = 0; y < 200; y++) {
    
        for (x = 0; x < (mode == 0x06 ? 640u : 320u); x++) {
        
            uint32_t color = video_native_cga_pixel (cpu, x, y, mode);
            
            unsigned output_x = mode == 0x06 ? x : x * 2;
            unsigned width = mode == 0x06 ? 1 : 2;
            
            unsigned i;
            
            for (i = 0; i < width; i++) {
            
                pixels[y * 2 * 640 + output_x + i] = color;
                pixels[(y * 2 + 1) * 640 + output_x + i] = color;
            
            }
        
        }
    
    }

}
