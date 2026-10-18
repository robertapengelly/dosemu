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

static void psp_make_drive_fcb (CPU8086 *cpu, uint32_t psp_base, uint16_t off, char drive) {

    uint8_t d;
    int i;
    
    if (drive >= 'a' && drive <= 'z') {
        drive -= 32;
    }
    
    d = (uint8_t) (drive - 'A' + 1);
    mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + 0, d);
    
    for (i = 1; i <= 11; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + i, ' ');
    }
    
    for (i = 12; i < 16; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, psp_base + off + i, 0x00);
    }

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
    
        const char *a = args;
        
        while (a && (*a == ' ' || *a == '\t')) {
            a++;
        }
        
        if (a && a[0] && ((a[0] >= 'A' && a[0] <= 'Z') || (a[0] >= 'a' && a[0] <= 'z')) && a[1] == ':') {
            psp_make_drive_fcb (cpu, psp_base, 0x5C, a[0]);
        }
    
    }
    
}
