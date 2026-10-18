/******************************************************************************
 * @file            psp.c
 *****************************************************************************/
#include    <stdint.h>
#include    <string.h>

#include    "cpu.h"
#include    "psp.h"
#include    "memory.h"

static void psp_clear_fcb (CPU8086 *cpu, uint32_t psp_base, uint16_t off) {

    int i;
    mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + 0, 0x00);
    
    for (i = 1; i < 16; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + i, 0x00);
    }

}

static const char *psp_parse_fcb (CPU8086 *cpu, uint32_t psp_base, uint16_t off, const char *args) {

    const char *p = args;
    uint8_t drive = 0;
    
    int i;
    
    if (!p) {
        return NULL;
    }
    
    while (*p == ' ' || *p == '\t' || *p == ',' || *p == ';' || *p == '=') {
        p++;
    }
    
    if (!*p || *p == '\r') {
        return p;
    }
    
    if (((p[0] >= 'A' && p[0] <= 'Z') || (p[0] >= 'a' && p[0] <= 'z')) && p[1] == ':') {
    
        drive = (uint8_t) ((p[0] & ~0x20) - 'A' + 1);
        p += 2;
    
    }
    
    mem_write8 (cpu->mem, cpu->mem_size, psp_base + off, drive);
    
    for (i = 1; i <= 11; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + i, ' ');
    }
    
    for (i = 0; i < 8 && *p && *p != '.' && *p != ' ' && *p != '\t' && *p != ',' && *p != ';' && *p != '=' && *p != '\r'; i++) {
    
        uint8_t c = (uint8_t) *p++;
        
        if (c >= 'a' && c <= 'z') {
            c -= 32;
        }
        
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + 1 + i, c);
    
    }
    
    while (*p && *p != '.' && *p != ' ' && *p != '\t' && *p != ',' && *p != ';' && *p != '=' && *p != '\r') {
        p++;
    }
    
    if (*p == '.') {
    
        p++;
        
        for (i = 0; i < 3 && *p && *p != ' ' && *p != '\t' && *p != ',' && *p != ';' && *p != '=' && *p != '\r'; i++) {
        
            uint8_t c = (uint8_t) *p++;
            
            if (c >= 'a' && c <= 'z') {
                c -= 32;
            }
            
            mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + 9 + i, c);
        
        }
    
    }
    
    while (*p && *p != ' ' && *p != '\t' && *p != ',' && *p != ';' && *p != '=' && *p != '\r') {
        p++;
    }
    
    return p;

}

void init_psp_contents (CPU8086 *cpu, const char *args, uint16_t psp_seg, uint16_t parent_psp, uint16_t env_seg, uint16_t end_seg) {

    uint32_t psp_base = (uint32_t) psp_seg << 4;
    
    /* PSP:0000 = INT 20h */
    mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x00, 0xCD);
    mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x01, 0x20);
    
    /* PSP:0002 = segment beyond allocation */
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x02, end_seg);
    
    /* PSP:0005 - CP/M CALL 5 compatibility and .COM segment size. */
    mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x05, 0x9A);
    
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x06, 0xFFF0);
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x08, 0xF00D);
    
    /* PSP:0016 = parent PSP */
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x16, parent_psp);
    
    /* PSP:0018 = DOS Job File Table (JFT). */
    for (int i = 0; i < 20; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x18 + i, (uint8_t) ((i < 5) ? i : 0xFF));
    }
    
    /* PSP:0032/0034 = JFT length and pointer (DOS 3+). */
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x32, 20);
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x34, 0x0018);
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x36, psp_seg);
    
    /* PSP:002C = environment segment */
    mem_write16 (cpu->mem, cpu->mem_size, psp_base + 0x2C, env_seg);
    
    /* Command tail */
    if (args) {
    
        uint8_t len = (uint8_t) strlen (args);
        
        if (len > 126) {
            len = 126;
        }
        
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x80, len);
        
        for (uint8_t i = 0; i < len; i++) {
            mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x81 + i, (uint8_t) args[i]);
        }
        
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x81 + len, 0x0D);
    
    } else {
    
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x80, 0);
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + 0x81, 0x0D);
    
    }
    
    psp_clear_fcb (cpu, psp_base, 0x5C);
    psp_clear_fcb (cpu, psp_base, 0x6C);
    
    {
    
        const char *next = psp_parse_fcb (cpu, psp_base, 0x5C, args);
        psp_parse_fcb (cpu, psp_base, 0x6C, next);
    
    }
    
}
