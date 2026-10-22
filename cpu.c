/******************************************************************************
 * @file            cpu.c
 *****************************************************************************/
#if     !defined (_WIN32) && !defined (_POSIX_C_SOURCE)
# define    _POSIX_C_SOURCE             200809L
#endif

#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <time.h>

#if     defined (_WIN32)
# include       <windows.h>
#endif

#include    "cpu.h"
#include    "heap.h"
#include    "interrupts.h"
#include    "opcodes.h"
#include    "memory.h"
#include    "speaker.h"

#define     FLAG_CF     0x0001
#define     FLAG_PF     0x0004          /* Parity Flag (bit 2) */
#define     FLAG_AF     0x0010          /* Auxiliary Carry Flag (bit 4) */
#define     FLAG_ZF     0x0040
#define     FLAG_SF     0x0080
#define     FLAG_TF     0x0100
#define     FLAG_IF     0x0200
#define     FLAG_DF     0x0400          /* Direction Flag (bit 10) */
#define     FLAG_OF     0x0800          /* Overflow Flag (bit 11) */

static uint8_t get_al (CPU8086 *cpu) {
    return (uint8_t) (cpu->regs[REG_AX] & 0x00FF);
}

static void set_al (CPU8086 *cpu, uint8_t v) {
    cpu->regs[REG_AX] = (uint16_t) ((cpu->regs[REG_AX] & 0xFF00) | v);
}

static int rep_active (const CPU8086 *cpu) {
    return cpu->rep_type == 1 || cpu->rep_type == 2;
}

static uint16_t rep_initial_count (const CPU8086 *cpu) {
    return rep_active (cpu) ? cpu->regs[REG_CX] : 1;
}

uint32_t get_phys (uint16_t seg, uint16_t off) {

    /* Standard 8086 behavior: wrap the offset at 64KB (16-bit) */
    /* BEFORE adding it to the segment base.*/
    return (((uint32_t) seg << 4) + (uint16_t) off) & 0xFFFFF;

}

uint8_t fetch8 (CPU8086 *cpu) {

    uint32_t phys = get_phys (cpu->cs, cpu->ip);
    
    uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, phys);
    cpu->ip = (uint16_t) (cpu->ip + 1); 
    
    return v;

}

uint16_t fetch16 (CPU8086 *cpu) {

    /* Byte 1 at CS:IP */
    uint8_t low = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->cs, cpu->ip));
    
    /* Byte 2 at CS:(IP + 1). The +1 wraps 16-bit because IP is uint16_t. */
    uint8_t high = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->cs, cpu->ip + 1));
    
    cpu->ip = (uint16_t) (cpu->ip + 2);
    return (uint16_t) low | ((uint16_t) high << 8);

}

uint32_t fetch32 (CPU8086 *cpu) {

    uint32_t low = fetch16 (cpu);
    uint32_t high = fetch16 (cpu);
    
    return (high << 16) | low;

}

uint8_t get_reg8 (const CPU8086 *cpu, int reg) {

    reg &= 7;
    
    if (reg < 4) {
        return (uint8_t) (cpu->regs[reg] & 0x00ffu);
    }
    
    return (uint8_t) ((cpu->regs[reg - 4] >> 8) & 0x00ffu);

}

void set_reg8 (CPU8086 *cpu, int reg, uint8_t value) {

    reg &= 7;
    
    if (reg < 4) {
        cpu->regs[reg] = (uint16_t)((cpu->regs[reg] & 0xff00u) | value);
    } else {
    
        int wreg = reg - 4;
        cpu->regs[wreg] = (uint16_t)((cpu->regs[wreg] & 0x00ffu) | ((uint16_t)value << 8));
    
    }

}

static void set_zf_sf_8 (CPU8086 *cpu, uint8_t value) {

    if (value == 0) {
        cpu->flags |= FLAG_ZF;
    } else {
        cpu->flags &= (uint16_t) ~FLAG_ZF;
    }
    
    if ((value & 0x80u) != 0) {
        cpu->flags |= FLAG_SF;
    } else {
        cpu->flags &= (uint16_t) ~FLAG_SF;
    }

}

static void update_common_flags8 (CPU8086 *cpu, uint8_t result) {

    uint8_t p;
    
    /* Zero Flag */
    if (result == 0) {
        cpu->flags |= FLAG_ZF;
    } else {
        cpu->flags &= ~FLAG_ZF;
    }
    
    /* Sign Flag */
    if (result & 0x80) {
        cpu->flags |= FLAG_SF;
    } else {
        cpu->flags &= ~FLAG_SF;
    }
    
    /* Parity Flag: set if low byte has even number of 1s */
    p = result;
    
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    
    if (!(p & 1)) {
        cpu->flags |= FLAG_PF;
    } else {
        cpu->flags &= ~FLAG_PF;
    }

}

static void update_common_flags16 (CPU8086 *cpu, uint16_t result) {

    /* 1. Zero Flag (Full 16 bits) */
    if (result == 0) {
        cpu->flags |= FLAG_ZF;
    } else {
        cpu->flags &= ~FLAG_ZF;
    }
    
    /* 2. Sign Flag (Bit 15) */
    if (result & 0x8000) {
        cpu->flags |= FLAG_SF;
    } else {
        cpu->flags &= ~FLAG_SF;
    }
    
    /* 3. Parity Flag (Only low 8 bits) */
    uint8_t p = (uint8_t) (result & 0xFF);
    p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    
    if (!(p & 1)) {
        cpu->flags |= FLAG_PF;
    } else {
        cpu->flags &= ~FLAG_PF;
    }

}

static uint8_t dec8 (CPU8086 *cpu, uint8_t val) {

    uint8_t res = val - 1;
    uint16_t old_cf = cpu->flags & FLAG_CF;
    
    update_common_flags8 (cpu, res);
    
    if (val == 0x80) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    cpu->flags = (cpu->flags & ~FLAG_CF) | old_cf;          /* Preserve Carry */
    return res;

}

static uint16_t dec16 (CPU8086 *cpu, uint16_t val) {

    uint16_t res = val - 1;
    uint16_t old_cf = cpu->flags & FLAG_CF;
    
    update_common_flags16 (cpu, res);
    
    if (val == 0x8000) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    cpu->flags = (cpu->flags & ~FLAG_CF) | old_cf;          /* Preserve Carry */
    return res;

}

static uint8_t inc8 (CPU8086 *cpu, uint8_t val) {

    uint8_t res = val + 1;
    uint16_t old_cf = cpu->flags & FLAG_CF;                 /* 1. Save existing Carry */
    
    update_common_flags8 (cpu, res);                        /* 2. Update ZF, SF, PF */
    
    /* 3. Manually handle Overflow (OF) for 0x7F -> 0x80 */
    if (val == 0x7F) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    /* 4. Restore the original Carry Flag */
    cpu->flags = (cpu->flags & ~FLAG_CF) | old_cf; 
    return res;

}

static uint8_t add8 (CPU8086 *cpu, uint8_t a, uint8_t b) {

    uint16_t res = (uint16_t) a + (uint16_t) b;
    
    /* Carry */
    if (res & 0x100) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    /* Auxiliary Carry */
    if ((a ^ b ^ res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    /* Overflow: (both inputs same sign) AND (result sign is different) */
    if (!((a ^ b) & 0x80) && ((a ^ res) & 0x80)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }

    update_common_flags8 (cpu, (uint8_t) res);
    return (uint8_t) res;

}

static uint16_t add16 (CPU8086 *cpu, uint16_t a, uint16_t b) {

    uint32_t res = (uint32_t) a + (uint32_t) b;
    
    if (res & 0x10000) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    if ((a ^ b ^ res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    if (!((a ^ b) & 0x8000) && ((a ^ res) & 0x8000)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    update_common_flags16 (cpu, (uint16_t) res);
    return (uint16_t) res;

}

static uint8_t sub8 (CPU8086 *cpu, uint8_t a, uint8_t b) {

    uint16_t res = (uint16_t) a - (uint16_t) b;
    
    if (a < b) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    if ((a ^ b ^ res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    /* Overflow: (inputs have different signs) AND (result sign doesn't match minuend) */
    if (((a ^ b) & 0x80) && ((a ^ res) & 0x80)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    update_common_flags8 (cpu, (uint8_t) res);
    return (uint8_t) res;

}

static uint16_t sub16 (CPU8086 *cpu, uint16_t a, uint16_t b) {

    uint32_t res = (uint32_t)a - (uint32_t)b;
    
    if (a < b) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    if ((a ^ b ^ res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    if (((a ^ b) & 0x8000) && ((a ^ res) & 0x8000)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }

    update_common_flags16 (cpu, (uint16_t) res);
    return (uint16_t) res;

}

static uint8_t xor8 (CPU8086 *cpu, uint8_t a, uint8_t b) {

    uint8_t result = (uint8_t) (a ^ b);
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
    update_common_flags8 (cpu, result);
    
    return result;

}

static uint16_t xor16 (CPU8086 *cpu, uint16_t a, uint16_t b) {

    uint16_t result = (uint16_t) (a ^ b);
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
    update_common_flags16 (cpu, result);
    
    return result;

}

static uint16_t inc16 (CPU8086 *cpu, uint16_t val) {

    uint16_t res = val + 1;
    uint16_t old_cf = cpu->flags & FLAG_CF;                 /* Save current CF */
    
    update_common_flags16 (cpu, res);                       /* Updates ZF, SF, PF */
    
    /* Manually handle OF (Overflow) */
    if (val == 0x7FFF) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    cpu->flags = (cpu->flags & ~FLAG_CF) | old_cf;      /* Restore saved CF */
    return res;

}

void push16 (CPU8086 *cpu, uint16_t value) {

    cpu->regs[REG_SP] -= 2;
    
    mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP]), value & 0xFF);
    mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP] + 1), value >> 8);

}

uint16_t pop16 (CPU8086 *cpu) {

    uint16_t sp = cpu->regs[REG_SP];
    
    uint8_t lo = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, sp));
    uint8_t hi = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, sp + 1));
    
    uint16_t value = lo | (hi << 8);
    
    cpu->regs[REG_SP] += 2;
    return value;

}

static uint8_t and8 (CPU8086 *cpu, uint8_t a, uint8_t b) {

    uint8_t result = a & b;
    
    /* Clear BOTH CF and OF */
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
    
    update_common_flags8 (cpu, result);
    return result;

}

static uint16_t and16 (CPU8086 *cpu, uint16_t a, uint16_t b) {

    uint16_t result = a & b;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
    update_common_flags16 (cpu, result);
    
    return result;

}

static uint8_t or8 (CPU8086 *cpu, uint8_t a, uint8_t b) {

    uint8_t result = a | b;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);          /* MUST clear both */
    update_common_flags8 (cpu, result);
    
    return result;

}

static uint16_t or16 (CPU8086 *cpu, uint16_t a, uint16_t b) {

    uint16_t result = a | b;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);      /* MUST clear both */
    update_common_flags16 (cpu, result);
    
    return result;

}

static uint8_t adc8 (CPU8086 *cpu, uint8_t a, uint8_t b) {

    uint16_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;
    uint32_t res = (uint32_t) a + (uint32_t) b + carry;
    
    /* Carry Flag */
    if (res & 0x100) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    /* Auxiliary Carry Flag */
    if ((a ^ b ^ res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    /* Overflow Flag */
    if (!((a ^ b) & 0x80) && ((a ^ res) & 0x80)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    update_common_flags8 (cpu, (uint8_t) res);
    return (uint8_t) res;

}

static uint16_t adc16 (CPU8086 *cpu, uint16_t a, uint16_t b) {

    uint16_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;
    uint32_t res = (uint32_t) a + (uint32_t) b + carry;
    
    if (res & 0x10000) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    if ((a ^ b ^ res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    if (!((a ^ b) & 0x8000) && ((a ^ res) & 0x8000)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    update_common_flags16 (cpu, (uint16_t) res);
    return (uint16_t) res;

}

static uint8_t sbb8 (CPU8086 *cpu, uint8_t a, uint8_t b) {

    uint16_t borrow = (cpu->flags & FLAG_CF) ? 1 : 0;
    uint16_t res = (uint16_t) a - (uint16_t) b - borrow;
    
    /* 1. Carry Flag: Borrow required from next higher bit */
    if (a < (uint16_t) b + borrow) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    /* 2. Auxiliary Carry: Borrow from bit 4 (nibble boundary) */
    if ((a ^ b ^ res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    /* 3. Overflow: Set if signed overflow occurs */
    if (((a ^ b) & 0x80) && ((a ^ res) & 0x80)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    /* 4. Updates ZF, SF, and PF using your existing helper */
    update_common_flags8 (cpu, (uint8_t) res);
    
    return (uint8_t) res;

}

static uint16_t sbb16 (CPU8086 *cpu, uint16_t a, uint16_t b) {

    uint16_t borrow = (cpu->flags & FLAG_CF) ? 1 : 0;
    uint32_t res = (uint32_t) a - (uint32_t) b - borrow;
    
    /* Carry Flag */
    if (a < (uint32_t) b + borrow) {
        cpu->flags |= FLAG_CF;
    } else {
        cpu->flags &= ~FLAG_CF;
    }
    
    /* Auxiliary Carry Flag */
    if ((a ^ b ^ (uint16_t) res) & 0x10) {
        cpu->flags |= FLAG_AF;
    } else {
        cpu->flags &= ~FLAG_AF;
    }
    
    /* Overflow Flag */
    if (((a ^ b) & 0x8000) && ((a ^ (uint16_t) res) & 0x8000)) {
        cpu->flags |= FLAG_OF;
    } else {
        cpu->flags &= ~FLAG_OF;
    }
    
    /* Updates ZF, SF, and PF */
    update_common_flags16 (cpu, (uint16_t) res);
    
    return (uint16_t) res;

}

static void daa (CPU8086 *cpu) {

    uint8_t al = get_reg8 (cpu, REG_AX);
    uint8_t old_al = al;
    uint8_t old_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
    
    /* Adjust low nibble */
    if ((al & 0x0F) > 9 || (cpu->flags & FLAG_AF)) {
    
        al += 6;
        cpu->flags |= FLAG_AF;
    
    } else {
        cpu->flags &= (uint16_t) ~FLAG_AF;
    }
    
    /* Adjust high nibble */
    if (old_al > 0x99 || old_cf) {
    
        al += 0x60;
        cpu->flags |= FLAG_CF;
    
    } else {
        cpu->flags &= (uint16_t) ~FLAG_CF;
    }
    
    set_reg8 (cpu, REG_AX, al);
    set_zf_sf_8 (cpu, al);

}

static void das (CPU8086 *cpu) {

    uint8_t al = get_reg8 (cpu, REG_AX);
    uint8_t old_al = al;
    uint8_t old_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
    
    if ((al & 0x0F) > 9 || (cpu->flags & FLAG_AF)) {
    
        al -= 6;
        cpu->flags |= FLAG_AF;
    
    } else {
        cpu->flags &= (uint16_t) ~FLAG_AF;
    }
    
    if (old_al > 0x99 || old_cf) {
    
        al -= 0x60;
        cpu->flags |= FLAG_CF;
    
    } else {
        cpu->flags &= (uint16_t) ~FLAG_CF;
    }
    
    set_reg8 (cpu, REG_AX, al);
    set_zf_sf_8 (cpu, al);

}

static void aaa (CPU8086 *cpu) {

    uint8_t al = get_reg8 (cpu, REG_AX);
    
    /* Check if the low nibble is > 9 or if AF is set */
    if ((al & 0x0F) > 9 || (cpu->flags & FLAG_AF)) {
    
        set_reg8 (cpu, REG_AX, (al + 6) & 0x0F);
        
        /* Increment AH (High byte of AX) */
        uint8_t ah = (uint8_t) (cpu->regs[REG_AX] >> 8);
        ah++;
        
        cpu->regs[REG_AX] = (cpu->regs[REG_AX] & 0x00FF) | ((uint16_t) ah << 8);
        
        cpu->flags |= FLAG_AF;
        cpu->flags |= FLAG_CF;
    
    } else {
    
        set_reg8 (cpu, REG_AX, al & 0x0F);
        
        cpu->flags &= (uint16_t) ~FLAG_AF;
        cpu->flags &= (uint16_t) ~FLAG_CF;
    
    }

}

static uint64_t pit_clock_ns (void) {

#if     defined (_WIN32)
    LARGE_INTEGER counter, frequency;
    
    if (!QueryPerformanceCounter (&counter) || !QueryPerformanceFrequency (&frequency) || frequency.QuadPart <= 0) {
        return (uint64_t) GetTickCount () * 1000000ULL;
    }
    
    return ((uint64_t) counter.QuadPart / (uint64_t) frequency.QuadPart) * 1000000000ULL +
        ((uint64_t) counter.QuadPart % (uint64_t) frequency.QuadPart) * 1000000000ULL / (uint64_t) frequency.QuadPart;
#else
    struct timespec now;
    
    if (clock_gettime (CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }
    
    return (uint64_t) now.tv_sec * 1000000000ULL + (uint64_t) now.tv_nsec;
#endif

}

static uint64_t pit2_elapsed (CPU8086 *cpu) {

    uint64_t now = pit_clock_ns ();
    
    uint64_t ns = now >= cpu->pit2_started_ns ? now - cpu->pit2_started_ns : 0;
    return (ns / 1000000000ULL) * 1193182ULL + (ns % 1000000000ULL) * 1193182ULL / 1000000000ULL;

}

static uint16_t pit2_count (CPU8086 *cpu) {

    uint32_t reload = cpu->pit2_reload ? cpu->pit2_reload : 65536;
    uint64_t elapsed = (cpu->port61 & 1) ? pit2_elapsed (cpu) : 0;
    uint32_t remaining;
    
    if (cpu->pit2_mode == 0 || cpu->pit2_mode == 1 || cpu->pit2_mode == 4 || cpu->pit2_mode == 5) {
        remaining = elapsed >= reload ? 0 : reload - (uint32_t) elapsed;
    } else {
        remaining = reload - (uint32_t) (elapsed % reload);
    }
    
    return (uint16_t) remaining;

}

static uint8_t pit2_output (CPU8086 *cpu) {

    uint32_t reload = cpu->pit2_reload ? cpu->pit2_reload : 65536;
    uint64_t elapsed;
    
    if (!(cpu->port61 & 1)) {
        return 1;
    }
    
    elapsed = pit2_elapsed (cpu);
    
    switch (cpu->pit2_mode) {
    
        case 0:     case 1:
        
            return elapsed >= reload;
        
        case 2:
        
            return (elapsed % reload) != reload - 1;
        
        case 3:
        
            return (elapsed % reload) < (reload + 1) / 2;
        
        case 4:     case 5:
        
            return elapsed != reload;
        
        default:
        
            return 1;
    
    }

}

static uint8_t io_read8 (CPU8086 *cpu, uint16_t port) {

    switch (port) {
        
        case 0x42: {                    /* PIT channel 2 count */
        
            uint16_t count = cpu->pit2_latch_active ? cpu->pit2_latch : pit2_count (cpu);
            uint8_t result;
            
            if (cpu->pit2_access == 2) {
                result = (uint8_t) (count >> 8);
            } else if (cpu->pit2_access == 3) {
            
                result = cpu->pit2_read_high ? (uint8_t) (count >> 8) : (uint8_t) count;
                cpu->pit2_read_high ^= 1;
            
            } else {
                result = (uint8_t) count;
            }
            
            if (cpu->pit2_latch_active && (cpu->pit2_access != 3 || !cpu->pit2_read_high)) {
                cpu->pit2_latch_active = 0;
            }
            
            return result;
        
        }
        
        case 0x60:                      /* keyboard data port */
        
            if (cpu->kbd_ack_pending) {
            
                cpu->kbd_ack_pending--;
                return 0xFA;            /* keyboard command acknowledged */
            
            }
            
            return 0x00;                /* no scan code pending */
        
        case 0x61:                      /* keyboard controller / speaker control */
        
            return (uint8_t) ((cpu->port61 & (uint8_t) ~0x20) | (pit2_output (cpu) ? 0x20 : 0));

        case 0x64:                      /* keyboard status port */
        
            return cpu->kbd_ack_pending ? 0x01 : 0x00;
        
        case 0x3B5:     case 0x3D5: {                       /* 6845 CRT controller data */
        
            uint8_t index = cpu->crtc_index;
            
            if (port != (uint16_t) (cpu->crtc_base + 1)) {
                return 0xFF;            /* adapter not installed */
            }
            
            if (index >= 0x0E && index <= 0x11) {
                return cpu->crtc_registers[index];
            }
            
            return 0x00;                /* write-only / undefined register */
        
        }
         
        case 0x3DA:                     /* CGA/EGA/VGA status */
        
            cpu->vga_status_flip++;
            return (uint8_t) ((cpu->vga_status_flip & 0x01) | ((cpu->vga_status_flip & 0xE0) == 0xE0 ? 0x08 : 0x00));
        
        default:
        
            fprintf (stderr, "Unhandled IO read8 port %04X at %04X:%04X\n", port, cpu->cs, cpu->ip);
            exit (1);
    
    }

}

static uint16_t io_read16 (CPU8086 *cpu, uint16_t port) {

    uint8_t lo = io_read8 (cpu, port);
    uint8_t hi = io_read8 (cpu, (uint16_t) (port + 1));
    
    return (uint16_t) (lo | ((uint16_t) hi << 8));

}

static void io_write8 (CPU8086 *cpu, uint16_t port, uint8_t value) {

    switch (port) {
    
        case 0x42:                      /* PIT channel 2 divisor */
        
            if (cpu->pit2_access == 2) {
                cpu->pit2_reload = (cpu->pit2_reload & 0x00FF) | ((uint16_t) value << 8);
            } else if (cpu->pit2_access == 3 && !cpu->pit2_write_high) {
            
                cpu->pit2_write_low = value;
                cpu->pit2_write_high = 1;
                
                break;
            
            } else if (cpu->pit2_access == 3) {
            
                cpu->pit2_reload = (uint16_t) ((uint16_t) value << 8) | cpu->pit2_write_low;
                cpu->pit2_write_high = 0;
            
            } else {
                cpu->pit2_reload = (cpu->pit2_reload & 0xFF00) | value;
            }
            
            cpu->pit2_started_ns = pit_clock_ns ();
            
            speaker_update (cpu->port61, cpu->pit2_mode, cpu->pit2_reload, 1);
            break;
        
        case 0x43: {                    /* PIT control word */
        
            uint8_t channel = (value >> 6) & 3;
            uint8_t access = (value >> 4) & 3;
            uint8_t mode = (value >> 1) & 7;
            
            if (channel != 2) {
            
                fprintf (stderr, "Unsupported PIT channel %u at %04X:%04X\n", channel, cpu->cs, cpu->ip);
                exit (1);
            
            }
            
            if (access == 0) {          /* latch current count */
            
                cpu->pit2_latch = pit2_count (cpu);
                
                cpu->pit2_latch_active = 1;
                cpu->pit2_read_high = 0;
                
                break;
            
            }
            
            cpu->pit2_access = access;
            cpu->pit2_mode = mode >= 6 ? mode - 4 : mode;
            
            cpu->pit2_write_high = 0;
            cpu->pit2_read_high = 0;
            
            cpu->pit2_latch_active = 0;
            
            speaker_update (cpu->port61, cpu->pit2_mode, cpu->pit2_reload, 0);
            break;
        
        }
        
        case 0x60:                      /* AT keyboard data port */
        
            if (cpu->kbd_typematic_pending) {
            
                cpu->kbd_typematic_rate = value & 0x7F;
                cpu->kbd_typematic_pending = 0;
            
            } else if (value == 0xF3) { /* set typematic rate/delay */
            
                cpu->kbd_typematic_pending = 1;
            
            } else {
            
                fprintf (stderr, "Unhandled keyboard command %02X at %04X:%04X\n", value, cpu->cs, cpu->ip);
                exit (1);
            
            }
            
            if (cpu->kbd_ack_pending < 2) {
                cpu->kbd_ack_pending++;
            }
            
            break;
        
        case 0x61: {
        
            int restart = !(cpu->port61 & 1) && (value & 1);
            
            if (restart) {
                cpu->pit2_started_ns = pit_clock_ns ();
            }
            
            cpu->port61 = value & (uint8_t) ~0x20;
            
            speaker_update (cpu->port61, cpu->pit2_mode, cpu->pit2_reload, restart);
            break;
        
        }
        
        case 0x3B4:     case 0x3D4:                         /* 6845 register select */
        
            if (port == cpu->crtc_base) {
                cpu->crtc_index = value & 0x1F;
            }
            
            break;
        
        case 0x3B5:     case 0x3D5:                         /* 6845 register data */
        
            if (port == (uint16_t) (cpu->crtc_base + 1) && cpu->crtc_index < 0x10) {
            
                if (cpu->crtc_index == 0x0E) {
                    value &= 0x3F;
                }
                
                cpu->crtc_registers[cpu->crtc_index] = value;
            
            }
            
            break;
        
        case 0x3D8:     case 0x3D9:
        
            /* CGA mode control / colour select - not yet modelled */
            break;
        
        default:
        
            fprintf (stderr, "Unhandled IO write8 port %04X value %02X at %04X:%04X\n", port, value, cpu->cs, cpu->ip);
            exit (1);
    
    }

}

static void io_write16 (CPU8086 *cpu, uint16_t port, uint16_t value) {

    io_write8 (cpu, port, (uint8_t) (value & 0xFF));
    io_write8 (cpu, (uint16_t) (port + 1), (uint8_t) (value >> 8));

}

typedef struct OperandRef {

    int is_mem;
    int reg;
    
    uint16_t addr;
    uint16_t default_seg;
    
    uint16_t seg;

} OperandRef;

static OperandRef decode_rm_operand (CPU8086 *cpu, uint8_t modrm) {

    OperandRef out;
    
    uint8_t mod = (modrm >> 6) & 3;
    uint8_t rm  = modrm & 7;
    
    out.reg = rm;
    out.is_mem = (mod != 3);
    out.addr = 0;
    out.default_seg = cpu->ds;
    out.seg = cpu->ds;
    
    if (mod == 3) {
    
        out.seg = 0;
        return out;
    
    }
    
    {
    
        uint16_t base = 0;
        int uses_bp = 0;
        
        switch (rm) {
        
            case 0:
            
                base = (uint16_t) (cpu->regs[REG_BX] + cpu->regs[REG_SI]);
                break;
            
            case 1:
            
                base = (uint16_t) (cpu->regs[REG_BX] + cpu->regs[REG_DI]);
                break;
            
            case 2:
            
                base = (uint16_t) (cpu->regs[REG_BP] + cpu->regs[REG_SI]); uses_bp = 1;
                break;
            
            case 3:
            
                base = (uint16_t) (cpu->regs[REG_BP] + cpu->regs[REG_DI]); uses_bp = 1;
                break;
            
            case 4:
            
                base = cpu->regs[REG_SI];
                break;
            
            case 5:
            
                base = cpu->regs[REG_DI];
                break;
            
            case 6:
            
                if (mod == 0) {
                
                    base = fetch16 (cpu);                   /* [disp16] */
                    uses_bp = 0;
                
                } else {
                
                    base = cpu->regs[REG_BP];               /* [BP+disp] */
                    uses_bp = 1;
                
                }
                
                break;
            
            case 7:
            
                base = cpu->regs[REG_BX];
                break;
        
        }
        
        if (mod == 1) {
        
            int8_t disp8 = (int8_t) fetch8 (cpu);
            base = (uint16_t) (base + disp8);
        
        } else if (mod == 2) {
        
            uint16_t disp16 = fetch16 (cpu);
            base = (uint16_t) (base + disp16);
        
        }
        
        out.addr = base;
        out.default_seg = uses_bp ? cpu->ss : cpu->ds;
        out.seg = cpu->has_seg_override ? cpu->seg_override : out.default_seg;
    
    }
    
    return out;

}

static uint8_t operand_read8 (CPU8086 *cpu, const OperandRef *op) {

    if (op->is_mem) {
    
        uint16_t seg = op->seg;
        return mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, op->addr));
    
    }
    
    return get_reg8 (cpu, op->reg);

}

static void operand_write8 (CPU8086 *cpu, const OperandRef *op, uint8_t value) {

    if (op->is_mem) {
    
        uint16_t seg = op->seg;;
        mem_write8 (cpu->mem, cpu->mem_size, get_phys (seg, op->addr), value);
    
    } else {
        set_reg8 (cpu, op->reg, value);
    }

}

static uint16_t operand_read16 (CPU8086 *cpu, const OperandRef *op) {

    if (op->is_mem) {
    
        uint16_t seg = op->seg;
        uint16_t off = (uint16_t) op->addr;
        
        uint32_t phys_lo = get_phys (seg, off);
        uint32_t phys_hi = get_phys (seg, (uint16_t) (off + 1));
        
        uint8_t low = mem_read8 (cpu->mem, cpu->mem_size, phys_lo);
        uint8_t high = mem_read8 (cpu->mem, cpu->mem_size, phys_hi);
        
        uint16_t val = (uint16_t) low | ((uint16_t) high << 8);
        return val;
    
    }
    
    return cpu->regs[op->reg & 7];

}

static void operand_write16 (CPU8086 *cpu, const OperandRef *op, uint16_t value) {

    if (op->is_mem) {
    
        uint16_t seg = op->seg;
        uint16_t off = (uint16_t) op->addr;
        
        mem_write8 (cpu->mem, cpu->mem_size, get_phys (seg, off), (uint8_t) (value & 0xFF));
        mem_write8 (cpu->mem, cpu->mem_size, get_phys (seg, (uint16_t) (off + 1)), (uint8_t) (value >> 8));
        
        return;
    
    }
    
    cpu->regs[op->reg & 7] = value;

}

void cpu_crtc_set_cursor (CPU8086 *cpu, unsigned row, unsigned col, unsigned columns) {

    unsigned address = row * columns + col;
    
    cpu->crtc_registers[0x0E] = (uint8_t) ((address >> 8) & 0x3F);
    cpu->crtc_registers[0x0F] = (uint8_t) address;

}

void cpu_reset (CPU8086 *cpu, uint8_t *mem, size_t mem_size) {

    memset (cpu, 0, sizeof (*cpu));
    
    cpu->crtc_base = 0x3D4;
    cpu->mem = mem;
    cpu->mem_size = mem_size;
    
    /* Initial flags (Interrupts disabled, etc.) */
    cpu->flags = 0x0002;

}

uint16_t cpu_get_reg (const CPU8086 *cpu, int reg) {
    return cpu->regs[reg & 7];
}

void cpu_set_reg (CPU8086 *cpu, int reg, uint16_t value) {
    cpu->regs[reg & 7] = value;
}

void cpu_dump (const CPU8086 *cpu) {

    printf ("AX=%04X BX=%04X CX=%04X DX=%04X\n", cpu->regs[REG_AX], cpu->regs[REG_BX], cpu->regs[REG_CX], cpu->regs[REG_DX]);
    printf ("SP=%04X BP=%04X SI=%04X DI=%04X IP=%04X FL=%04X\n", cpu->regs[REG_SP], cpu->regs[REG_BP], cpu->regs[REG_SI], cpu->regs[REG_DI], cpu->ip, cpu->flags);

}

static void dump_guest_bytes (CPU8086 *cpu, uint16_t seg, uint16_t off, uint16_t n, const char *tag) {

    uint32_t phys = get_phys (seg, off);
    uint16_t i;
    
    printf ("\n[%s] %04X%04X phys=%05X\n", tag, seg, off, (unsigned) phys);
    
    printf ("[BYTES] ");
    
    for (i = 0; i < n; i++) {
        printf ("%02X ", mem_read8 (cpu->mem, cpu->mem_size, phys + i));
    }
    
    printf ("\n[TEXT ] ");
    
    for (i = 0; i < n; i++) {
    
        unsigned char c = mem_read8 (cpu->mem, cpu->mem_size, phys + i);
        putchar ((c >= 32 && c <= 126) ? c : '.');
    
    }
    
    printf ("\n");

}

int cpu_step (CPU8086 *cpu) {

    int trap_inhibit = 0;
    int trap_pending;
    
    uint8_t op;
    int old_ip;
    
    if (cpu == NULL || cpu->halted) {
        return 0;
    }
    
    trap_pending = (cpu->flags & FLAG_TF) != 0;
    
    cpu->seg_override = 0;
    cpu->has_seg_override = 0;
    
    cpu->has_addr_override = 0;
    cpu->has_operand_override = 0;
    cpu->rep_type = 0;                  /* 0=None, 1=REPE, 2=REPNZ */

fetch_instruction:
    
    //printf ("%s: CS:IP: %04X:%04X\n", __FUNCTION__, cpu->cs, cpu->ip);
    
    old_ip = cpu->ip;
    op = fetch8 (cpu);
    
    switch (op) {
    
        /* Arithmetic operations */
        /* Arithmetic and Logic (ADD, OR, AND, SUB, XOR, CMP) */
        case 0x00: case 0x01: case 0x02: case 0x03:         /* ADD */
        case 0x08: case 0x09: case 0x0A: case 0x0B:         /* OR */
        case 0x20: case 0x21: case 0x22: case 0x23:         /* AND */
        case 0x28: case 0x29: case 0x2A: case 0x2B:         /* SUB */
        case 0x30: case 0x31: case 0x32: case 0x33:         /* XOR */
        case 0x38: case 0x39: case 0x3A: case 0x3B:         /* CMP */ {
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint16_t dst, src;
            
            if (op == 0x39) {
            
                dst = operand_read16 (cpu, &rm);
                src = cpu->regs[reg];
            
            } else {
            
                dst = cpu->regs[reg];
                src = operand_read16 (cpu, &rm);
            
            }
            
            if (op == 0x00) {
                operand_write8 (cpu, &rm, add8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            } else if (op == 0x01) {
                operand_write16 (cpu, &rm, add16 (cpu, operand_read16 (cpu, &rm), cpu->regs[reg]));
            } else if (op == 0x02) {
                set_reg8 (cpu, reg, add8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            } else if (op == 0x03) {
                cpu->regs[reg] = add16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            } else if (op == 0x08) {
                operand_write8 (cpu, &rm, or8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            } else if (op == 0x09) {
                operand_write16 (cpu, &rm, or16 (cpu, operand_read16 (cpu, &rm), cpu->regs[reg]));
            } else if (op == 0x0A) {
                set_reg8 (cpu, reg, or8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            } else if (op == 0x0B) {
                cpu->regs[reg] = or16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            } else if (op == 0x20) {
                operand_write8 (cpu, &rm, and8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            } else if (op == 0x21) {
                operand_write16 (cpu, &rm, and16 (cpu, operand_read16 (cpu, &rm), cpu->regs[reg]));
            } else if (op == 0x22) {
                set_reg8 (cpu, reg, and8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            } else if (op == 0x23) {
                cpu->regs[reg] = and16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            } else if (op == 0x28) {
                
                /* when handling prefix 0x26 */
                operand_write8 (cpu, &rm, sub8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            
            } else if (op == 0x29) {
                operand_write16 (cpu, &rm, sub16 (cpu, operand_read16 (cpu, &rm), cpu->regs[reg]));
            } else if (op == 0x2A) {
                set_reg8 (cpu, reg, sub8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            } else if (op == 0x2B) {
                cpu->regs[reg] = sub16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            } else if (op == 0x30) {
                operand_write8 (cpu, &rm, xor8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            } else if (op == 0x31) {
                
                uint16_t left = operand_read16 (cpu, &rm);
                uint16_t right = cpu->regs[reg];
                
                operand_write16 (cpu, &rm, xor16 (cpu, left, right));
            
            } else if (op == 0x32) {
                set_reg8 (cpu, reg, xor8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            } else if (op == 0x33) {
                cpu->regs[reg] = xor16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            } else if (op == 0x3B) {
                
                uint16_t rhs = operand_read16 (cpu, &rm);
                uint16_t lhs = cpu->regs[reg];
                
                sub16 (cpu, lhs, rhs);
            
            } else if (op == 0x38) {
                sub8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg));
            } else if (op == 0x39) {
                sub16 (cpu, dst, src);
            } else if (op == 0x3A) {
                sub8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm));
            } else if (op == 0x3B) {
                
                uint16_t rhs = operand_read16 (cpu, &rm);
                sub16 (cpu, cpu->regs[reg], rhs);
            
            }
            
            break;
        
        }
        
        case 0x0C: {           /* OR AL, imm8 */
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, REG_AX);
            
            set_reg8 (cpu, REG_AX, or8 (cpu, al, imm8));
            break;
        
        }
        
        case 0x0D: {           /* OR AX, imm16 */
        
            uint16_t imm16 = fetch16 (cpu);                 /* Fetch the 16-bit immediate value */
            
            cpu->regs[REG_AX] = or16 (cpu, cpu->regs[REG_AX], imm16);           /* Perform OR and update flags */
            break;
        
        }
        
        case 0x10:  case 0x11:  case 0x12:  case 0x13: {    /* ADC */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0x10) {
                operand_write8 (cpu, &rm, adc8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            } else if (op == 0x11) {
                operand_write16 (cpu, &rm, adc16 (cpu, operand_read16 (cpu, &rm), cpu->regs[reg]));
            } else if (op == 0x12) {
                set_reg8 (cpu, reg, adc8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            } else if (op == 0x13) {
                cpu->regs[reg] = adc16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            }
            
            break;
        
        }
        
        case 0x14: {
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, REG_AX);
            
            set_reg8 (cpu, REG_AX, adc8 (cpu, al, imm8));
            break;
        
        }
        
        case 0x18: {
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            operand_write8 (cpu, &rm, sbb8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            
            break;
        
        }
        
        case 0x19: {           /* SBB r/m16, r16 */
        
            uint8_t modrm = fetch8 (cpu);
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            /* The 'reg' field of ModRM (bits 5-3) selects the source register */
            uint16_t src = cpu->regs[(modrm >> 3) & 7];
            uint16_t dst = operand_read16 (cpu, &rm);
            
            /* Perform subtraction with borrow and update flags */
            /* (Assuming you have a helper like sbb16 for flag logic) */
            uint16_t result = sbb16 (cpu, dst, src);
            
            operand_write16 (cpu, &rm, result);
            break;
        
        }
        
        case 0x1A: {           /* SBB r8, r/m8 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            set_reg8 (cpu, reg, sbb8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            
            break;
        
        }
        
        case 0x1B: {           /* SBB r16, r/m16 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            cpu->regs[reg] = sbb16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            
            break;
        
        }
        
        case 0x1C: {           /* SBB AL, imm8 */
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, 0);
            
            set_reg8 (cpu, 0, sbb8 (cpu, al, imm8));
            break;
        
        }
        
        case 0x1D: {           /* SBB AX, imm16 */
        
            uint16_t imm16 = fetch16 (cpu);
            
            cpu->regs[REG_AX] = sbb16 (cpu, cpu->regs[REG_AX], imm16);
            break;
        
        }
        
        /* Additional Arithmetic and Logic Operations */
        case 0x04:
        
            set_reg8 (cpu, 0, add8 (cpu, get_reg8 (cpu, 0), fetch8 (cpu)));
            break;
        
        case 0x05:
        
            cpu->regs[REG_AX] = add16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            break;
        
        /* --- Segment Registers --- */
        case 0x06:             /* PUSH ES */
        
            push16 (cpu, cpu->es);
            break;
        
        case 0x07: {           /* POP ES */
        
            cpu->es = pop16 (cpu);
            break;
        
        }
        
        case 0x0E:             /* PUSH CS */
        
            push16 (cpu, cpu->cs);
            break;
        
        case 0x0F: {
        
            uint8_t op2 = fetch8 (cpu);
            
            switch (op2) {
            
                case 0xA0:
                
                    push16 (cpu, cpu->fs);
                    break;
                
                case 0xA1:
                
                    cpu->fs = pop16 (cpu);
                    break;
                
                case 0xA8:
                
                    push16 (cpu, cpu->gs);
                    break;
                
                case 0xA9:
                
                    cpu->gs = pop16 (cpu);
                    break;
                
                default:
                
                    printf ("Unhandled 0F opcode 0x%02X at %04X%04X\n", op2, cpu->cs, old_ip);
                    return -1;
            
            }
            
            break;
        
        }
        
        case 0x15: {           /* ADC AX, imm16 */
        
            uint16_t imm16 = fetch16 (cpu);
            
            cpu->regs[REG_AX] = adc16 (cpu, cpu->regs[REG_AX], imm16);
            break;
        
        }
        
        case 0x16:             /* PUSH SS */
        
            push16 (cpu, cpu->ss);
            break;
        
        case 0x17:             /* POP SS */
        
            cpu->ss = pop16 (cpu);
            
            trap_inhibit = 1;
            break;
        
        case 0x1E:             /* PUSH DS */
        
            push16 (cpu, cpu->ds);
            break;
        
        case 0x1F:             /* POP DS */
        
            cpu->ds = pop16 (cpu);
            break;
        
        /* AND AL, imm8 */
        case 0x24: {
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, REG_AX);            /* AL is the low byte of AX */
            
            set_reg8 (cpu, REG_AX, and8 (cpu, al, imm8));
            break;
        
        }
        
        /* AND AX, imm16 */
        case 0x25: {
        
            uint16_t imm16 = fetch16 (cpu);
            
            cpu->regs[REG_AX] = and16 (cpu, cpu->regs[REG_AX], imm16);
            break;
        
        }
        
        case 0x26:
            
            cpu->seg_override = cpu->es;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x2E:
            
            cpu->seg_override = cpu->cs;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x36:
            
            cpu->seg_override = cpu->ss;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x3E:
            
            cpu->seg_override = cpu->ds;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x27:
        
            daa (cpu);
            break;
        
        case 0x2F:
        
            das (cpu);
            break;
        
        case 0x2C:
        
            set_reg8 (cpu, 0, sub8 (cpu, get_reg8 (cpu, 0), fetch8 (cpu)));
            break;
        
        case 0x2D:
        
            cpu->regs[REG_AX] = sub16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            break;
        
        case 0x34:
        
            set_reg8 (cpu, 0, xor8 (cpu, get_reg8 (cpu, 0), fetch8 (cpu)));
            break;
        
        case 0x35:
        
            cpu->regs[REG_AX] = xor16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            break;
        
        case 0x37:
        
            aaa (cpu);
            break;
        
        case 0x3C: {           /* CMP AL, imm8 */
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, REG_AX);
            
            sub8 (cpu, al, imm8);       /* CMP performs subtraction but discards the result */
            break;
        
        }
        
        case 0x3D: {
        
            uint16_t imm16 = fetch16 (cpu);
            sub16 (cpu, cpu->regs[REG_AX], imm16);
            
            break;
        
        }
        
        /* Control Flow Operations */
        case 0x40:  case 0x41:  case 0x42:  case 0x43:
        case 0x44:  case 0x45:  case 0x46:  case 0x47: {
        
            int reg = op - 0x40;
            
            cpu->regs[reg] = inc16 (cpu, cpu->regs[reg]);   /* Use helper that preserves CF */
            break;
        
        }
        
        case 0x48:  case 0x49:  case 0x4A:  case 0x4B:
        case 0x4C:  case 0x4D:  case 0x4E:  case 0x4F: {
        
            int reg = op - 0x48;
            
            cpu->regs[reg] = dec16 (cpu, cpu->regs[reg]);
            break;
        
        }
        
        case 0x50:  case 0x51:  case 0x52:  case 0x53:  case 0x54:  case 0x55:  case 0x57:
        
            push16 (cpu, cpu->regs[op - 0x50]);
            break;
        
        case 0x56: {
            
            push16 (cpu, cpu->regs[REG_SI]);
            break;
        
        }
        
        case 0x58:  case 0x59:  case 0x5A:  case 0x5B:
        case 0x5C:  case 0x5D:  case 0x5E:  case 0x5F:
        
            cpu->regs[op - 0x58] = pop16 (cpu);
            break;
        
        case 0x61: {           /* POPA (Pop All - 80186+) */
        
            cpu->regs[REG_DI] = pop16 (cpu);
            cpu->regs[REG_SI] = pop16 (cpu);
            cpu->regs[REG_BP] = pop16 (cpu);
            
            /* Increment SP past the stored SP value without loading it */
            pop16 (cpu);
            
            cpu->regs[REG_BX] = pop16 (cpu);
            cpu->regs[REG_DX] = pop16 (cpu);
            cpu->regs[REG_CX] = pop16 (cpu);
            cpu->regs[REG_AX] = pop16 (cpu);
            
            break;
        
        }
        
        case 0x60: {           /* PUSHA (80186+) */
        
            uint16_t temp_sp = cpu->regs[REG_SP];
            
            push16 (cpu, cpu->regs[REG_AX]);
            push16 (cpu, cpu->regs[REG_CX]);
            push16 (cpu, cpu->regs[REG_DX]);
            push16 (cpu, cpu->regs[REG_BX]);
            push16 (cpu, temp_sp);
            push16 (cpu, cpu->regs[REG_BP]);
            push16 (cpu, cpu->regs[REG_SI]);
            push16 (cpu, cpu->regs[REG_DI]);
            
            break;
        
        }
        
        case 0x62: {           /* BOUND r16, m16&16 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg_idx = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (!rm.is_mem) {
            
                /* BOUND requires a memory operand for the bounds pair */
                return -1;
            
            }
            
            {
            
                uint16_t index = cpu->regs[reg_idx];
                uint16_t seg = rm.seg;
                
                /* Read the bounds from memory */
                int16_t low_bound  = (int16_t) mem_read16 (cpu->mem, cpu->mem_size, get_phys (seg, rm.addr));
                int16_t upper_bound = (int16_t) mem_read16 (cpu->mem, cpu->mem_size, get_phys (seg, rm.addr + 2));
                
                /* Perform signed comparison */
                if ((int16_t) index < low_bound || (int16_t) index > upper_bound) {
                
                    /* Trigger Interrupt 5 (Bound Range Exceeded) */
                    /* Note: The saved IP should point to the START of the instruction (old_ip) */
                    cpu->ip = old_ip;
                    handle_interrupt (cpu, 5);
                
                }
            
            }
            
            break;
        
        }
        
        case 0x63: {           /* ARPL r/m16, r16 (80286+ Protected Mode instruction) */
        
            uint8_t modrm = fetch8 (cpu);
            int reg_src = (modrm >> 3) & 7;                 /* The source register */
            
            OperandRef rm_dest = decode_rm_operand (cpu, modrm);
            
            uint16_t dest_val = operand_read16 (cpu, &rm_dest);
            uint16_t src_val = cpu->regs[reg_src];
            
            /* Check the RPL (bits 0-1) */
            if ((dest_val & 0x0003) < (src_val & 0x0003)) {
            
                /* If dest RPL < src RPL, set dest RPL to src RPL and set ZF */
                dest_val = (dest_val & 0xFFFC) | (src_val & 0x0003);
                
                operand_write16 (cpu, &rm_dest, dest_val);
                cpu->flags |= FLAG_ZF;
            
            } else {
            
                /* Otherwise, just clear ZF */
                cpu->flags &= (uint16_t) ~FLAG_ZF;
            
            }
            
            break;
        
        }
        
        /* Change lines 654-655 to: */
        case 0x64:
            
            cpu->seg_override = cpu->fs;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x65:
            
            cpu->seg_override = cpu->gs;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x66:
        
            cpu->has_operand_override = 1;
            goto fetch_instruction;
        
        case 0x67:
        
            cpu->has_addr_override = 1;
            goto fetch_instruction;
        
        case 0x68: {           /* PUSH imm16 (80186+) */
        
            uint16_t imm16 = fetch16 (cpu);
            
            push16 (cpu, imm16);
            break;
        
        }
        
        case 0x6A: {           /* PUSH imm8 (80186+) */
        
            /* Sign-extend the 8-bit immediate to 16-bits before pushing */
            int16_t imm8 = (int8_t) fetch8 (cpu);
            
            push16 (cpu, (uint16_t) imm8);
            break;
        
        }
        
        case 0x69: /* IMUL r16, r/m16, imm16 */     case 0x6B: /* IMUL r16, r/m16, imm8 (sign-extended) */ {
        
            uint8_t modrm = fetch8 (cpu);
            int reg_dest = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            int16_t src_val = (int16_t) operand_read16 (cpu, &rm);
            int16_t imm;
            
            if (op == 0x69) {
                imm = (int16_t) fetch16 (cpu);
            } else {
            
                /* Sign-extend the 8-bit immediate to 16 bits */
                imm = (int16_t) (int8_t) fetch8 (cpu);
            
            }
            
            {
            
                /* Perform 32-bit signed multiplication to check for overflow */
                int32_t result = (int32_t) src_val * (int32_t) imm;
                
                /* Store the truncated 16-bit result in the destination register */
                cpu->regs[reg_dest] = (uint16_t) result;
                
                /* Update Flags: CF and OF are set if the result doesn't fit in 16 bits */
                if (result < -32768 || result > 32767) {
                    cpu->flags |= (FLAG_CF | FLAG_OF);
                } else {
                    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                }
            
            }
            
            /* SF, ZF, AF, and PF are technically undefined for IMUL, */
            /* but many emulators leave them unchanged or clear them. */
            break;
        }
        
        case 0x6C: /* INSB */   case 0x6D: /* INSW */ {
        
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            uint16_t orig = cpu->regs[REG_DI];
            
            while (1) {
            
                uint32_t phys;
                
                if (cpu->rep_type != 0 && cpu->regs[REG_CX] == 0) {
                    break;
                }
                
                phys = get_phys (cpu->es, cpu->regs[REG_DI]);
                
                if (op == 0x6C) {
                
                    uint8_t val = io_read8 (cpu, cpu->regs[REG_DX]);
                    
                    mem_write8 (cpu->mem, cpu->mem_size, phys, val);
                    cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                
                } else {
                
                    uint16_t val = io_read16 (cpu, cpu->regs[REG_DX]);
                    
                    mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]), (uint8_t) (val & 0xFF));
                    mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_DI] + 1)), (uint8_t) (val >> 8));
                    
                    cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + (step * 2));
                
                }
                
                if (cpu->rep_type != 0) {
                    cpu->regs[REG_CX]--;
                } else {
                    break;
                }
            
            }
            
            dump_guest_bytes (cpu, cpu->es, orig, 96, "POST-IN BUFFER");
            break;
        
        }
        
        case 0x6E: /* OUTSB */      case 0x6F: /* OUTSW */ {
        
            uint16_t seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            
            uint16_t orig = cpu->regs[REG_SI];
            
            while (1) {
            
                uint32_t phys;
                
                if (cpu->rep_type != 0 && cpu->regs[REG_CX] == 0) {
                    break;
                }
                
                phys = get_phys (seg, cpu->regs[REG_SI]);
                
                if (op == 0x6E) {
                
                    uint8_t val = mem_read8 (cpu->mem, cpu->mem_size, phys);
                    
                    io_write8 (cpu, cpu->regs[REG_DX], val);
                    cpu->regs[REG_SI] += step;
                
                } else {
                
                    /* Explicitly read bytes to enforce segment wrapping */
                    uint8_t low = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, cpu->regs[REG_SI]));
                    uint8_t high = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, (uint16_t) (cpu->regs[REG_SI] + 1)));
                    uint16_t val = (uint16_t) low | ((uint16_t) high << 8);
                    
                    io_write16 (cpu, cpu->regs[REG_DX], val);
                    cpu->regs[REG_SI] += (step * 2);
                
                }
                
                if (cpu->rep_type != 0) {
                    cpu->regs[REG_CX]--;
                } else {
                    break;
                }
            
            }
            
            dump_guest_bytes (cpu, seg, orig, 96, "POST-OUT BUFFER");
            break;
        
        }
        
        case 0x70:  case 0x71:  case 0x72:  case 0x73:
        case 0x74:  case 0x75:  case 0x76:  case 0x77:
        case 0x78:  case 0x79:  case 0x7A:  case 0x7B:
        case 0x7C:  case 0x7D:  case 0x7E:  case 0x7F: {
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint16_t next_ip = cpu->ip;
            uint16_t target = (uint16_t) (next_ip + rel);
            
            int take = 0;
            
            switch (op) {
            
                case 0x70:
                
                    take = (cpu->flags & FLAG_OF) != 0;
                    break;
                
                case 0x71:
                
                    take = (cpu->flags & FLAG_OF) == 0;
                    break;
                
                case 0x72:
                
                    take = (cpu->flags & FLAG_CF) != 0;
                    break;
                
                case 0x73:
                
                    take = (cpu->flags & FLAG_CF) == 0;
                    break;
                
                case 0x74:
                
                    take = (cpu->flags & FLAG_ZF) != 0;
                    break;
                
                case 0x75:
                
                    take = (cpu->flags & FLAG_ZF) == 0;
                    break;
                
                case 0x76:
                
                    take = ((cpu->flags & FLAG_CF) != 0) || ((cpu->flags & FLAG_ZF) != 0);
                    break;
                
                case 0x77:
                
                    take = ((cpu->flags & FLAG_CF) == 0) && ((cpu->flags & FLAG_ZF) == 0);
                    break;
                
                case 0x78:
                
                    take = (cpu->flags & FLAG_SF) != 0;
                    break;
                
                case 0x79:
                
                    take = (cpu->flags & FLAG_SF) == 0;
                    break;
                
                case 0x7A:
                
                    take = (cpu->flags & FLAG_PF) != 0;
                    break;
                
                case 0x7B:
                
                    take = (cpu->flags & FLAG_PF) == 0;
                    break;
                
                case 0x7C:
                
                    take = (((cpu->flags & FLAG_SF) != 0) != ((cpu->flags & FLAG_OF) != 0));
                    break;
                
                case 0x7D:
                
                    take = (((cpu->flags & FLAG_SF) != 0) == ((cpu->flags & FLAG_OF) != 0));
                    break;
                
                case 0x7E:
                
                    take = ((cpu->flags & FLAG_ZF) != 0) ||
                        (((cpu->flags & FLAG_SF) != 0) != ((cpu->flags & FLAG_OF) != 0));
                    break;
                
                case 0x7F:
                
                    take = ((cpu->flags & FLAG_ZF) == 0) &&
                        (((cpu->flags & FLAG_SF) != 0) == ((cpu->flags & FLAG_OF) != 0));
                    break;
            
            }
            
            if (take) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        /* Bitwise and Immediate Operations */
        case 0x80:  case 0x81:  case 0x82:  case 0x83: {
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0x80 || op == 0x82) {
            
                uint8_t imm8 = fetch8 (cpu);
                uint8_t left = operand_read8 (cpu, &rm);
                
                switch (subop) {
                
                    case 0:
                    
                        operand_write8 (cpu, &rm, add8 (cpu, left, imm8));
                        break;
                    
                    case 1:
                    
                        operand_write8 (cpu, &rm, or8 (cpu, left, imm8));
                        break;
                    
                    case 2:
                    
                        operand_write8 (cpu, &rm, adc8 (cpu, left, imm8));
                        break;
                    
                    case 3:
                    
                        operand_write8 (cpu, &rm, sbb8 (cpu, left, imm8));
                        break;
                    
                    case 4:
                    
                        operand_write8 (cpu, &rm, and8 (cpu, left, imm8));
                        break;
                    
                    case 5:
                    
                        operand_write8 (cpu, &rm, sub8 (cpu, left, imm8));
                        break;
                    
                    case 6:
                    
                        operand_write8 (cpu, &rm, xor8 (cpu, left, imm8));
                        break;
                    
                    case 7:
                    
                        (void) sub8 (cpu, left, imm8);
                        break;
                
                }
            
            } else {
            
                uint16_t imm16;
                uint16_t left = operand_read16 (cpu, &rm);
                
                if (op == 0x81) {
                    imm16 = fetch16 (cpu);
                } else {
                    imm16 = (uint16_t) (int16_t) (int8_t) fetch8 (cpu);
                }
                
                switch (subop) {
                
                    case 0: {
                    
                        uint16_t result = add16 (cpu, left, imm16);
                        operand_write16 (cpu, &rm, result);
                        
                        break;
                    }
                    
                    case 1:
                    
                        operand_write16 (cpu, &rm, or16 (cpu, left, imm16));
                        break;
                    
                    case 2:
                    
                        operand_write16 (cpu, &rm, adc16 (cpu, left, imm16));
                        break;
                    
                    case 3:
                    
                        operand_write16 (cpu, &rm, sbb16 (cpu, left, imm16));
                        break;
                    
                    case 4:
                    
                        operand_write16 (cpu, &rm, and16 (cpu, left, imm16));
                        break;
                    
                    case 5:
                    
                        operand_write16 (cpu, &rm, sub16 (cpu, left, imm16));
                        break;
                    
                    case 6:
                    
                        operand_write16 (cpu, &rm, xor16 (cpu, left, imm16));
                        break;
                    
                    /* opcode 83 /7: CMP r/m16, imm8 */
                    case 7: {
                    
                        uint16_t lhs = left;
                        int16_t rhs = imm16;
                        
                        sub16 (cpu, lhs, (uint16_t) rhs);
                        break;
                    
                    }
                
                }
            
            }
            
            break;
        
        }
        
        case 0x84: {           /* TEST r/m8, r8: */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0x84) {
            
                uint8_t val1 = operand_read8 (cpu, &rm);
                uint8_t val2 = get_reg8 (cpu, reg);
                
                and8 (cpu, val1, val2);                     /* Result is discarded, only flags are set */
            
            } else {
            
                uint16_t val1 = operand_read16 (cpu, &rm);
                uint16_t val2 = cpu->regs[reg & 7];
                
                and16 (cpu, val1, val2); /* Result is discarded */
            
            }
            
            break;
        
        }
        
        case 0x85: {
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint16_t val1 = operand_read16 (cpu, &rm);
            uint16_t val2 = cpu->regs[reg & 7];
            
            and16 (cpu, val1, val2);
            break;
        
        }
        
        case 0x86: /* XCHG r/m8, r8 */      case 0x87: /* XCHG r/m16, r16 */ {
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0x86) {
            
                uint8_t val1 = get_reg8 (cpu, reg);
                uint8_t val2 = operand_read8 (cpu, &rm);
                
                set_reg8 (cpu, reg, val2);
                operand_write8 (cpu, &rm, val1);
            
            } else {
                uint16_t val1 = cpu->regs[reg];
                uint16_t val2 = operand_read16 (cpu, &rm);
                
                cpu->regs[reg] = val2;
                operand_write16 (cpu, &rm, val1);
            
            }
            
            break;
        
        }
        
        /* Register-Memory and Immediate Operations */
        case 0x88: {           /* MOV r/m8, r8 */
        
            uint8_t modrm = fetch8 (cpu);
            int src = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint8_t v = get_reg8 (cpu, src);
            
            if (rm.is_mem) {
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (rm.seg, (uint16_t) rm.addr), v);
            } else {
                set_reg8 (cpu, rm.reg, v);
            }
            
            break;
        
        }
        
        case 0x8A: {           /* MOV r8, r/m8 */
        
            uint8_t modrm = fetch8 (cpu);
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint8_t value;
            
            if (rm.is_mem) {
                value = mem_read8 (cpu->mem, cpu->mem_size, get_phys (rm.seg, rm.addr));
            } else {
                value = get_reg8 (cpu, rm.reg);
            }
            
            set_reg8 (cpu, (modrm >> 3) & 7, value);
            break;
        
        }
        
        case 0x89:      case 0x8B: {
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0x89) {
            
                uint16_t val = cpu->regs[reg];
                operand_write16 (cpu, &rm, val);
            
            } else {
            
                uint16_t val = operand_read16 (cpu, &rm);
                cpu->regs[reg] = val;
            
            }
            
            break;
        
        }
        
        case 0x8C: {           /* MOV r/m16, Sreg */
        
            uint8_t modrm = fetch8 (cpu);
            uint8_t sreg_idx = (modrm >> 3) & 0x07;         /* ONLY 0–3 */
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint16_t val = 0;
            
            switch (sreg_idx) {
            
                case 0:
                
                    val = cpu->es;
                    break;
                
                case 1:
                
                    val = cpu->cs;
                    break;
                
                case 2:
                
                    val = cpu->ss;
                    break;
                
                case 3:
                
                    val = cpu->ds;
                    break;
                
                case 4:
                
                    val = cpu->fs;
                    break;
                
                case 5:
                
                    val = cpu->gs;
                    break;
            
            }
            
            operand_write16 (cpu, &rm, val);
            break;
        
        }
        
        case 0x8D: {           /* LEA r16, m */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (!rm.is_mem) {
            
                /* LEA technically requires a memory operand to calculate an address. */
                /* On a real CPU, if mod=3 (register), the behavior is often undefined */
                /* or results in an invalid opcode exception. */
                return -1;
            
            }
            
            /* Simply load the calculated offset into the register. */
            /* Note: LEA ignores segment registers; it only cares about the offset. */
            cpu->regs[reg & 7] = (uint16_t) rm.addr;
            break;
        
        }
        
        case 0x8E: {           /* MOV Sreg, r/m16 */
        
            uint8_t modrm = fetch8 (cpu);
            int sreg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint16_t value = operand_read16 (cpu, &rm);
            
            switch (sreg) {
            
                case 0:
                
                    cpu->es = value;
                    break;
                
                case 2:
                
                    cpu->ss = value;
                    
                    trap_inhibit = 1;
                    break;
                
                case 3:
                
                    cpu->ds = value;
                    break;
                
                case 4:
                
                    cpu->fs = value;
                    break;
                
                case 5:
                
                    cpu->gs = value;
                    break;
                
                case 1:
                
                    printf ("[INVALID MOV CS] CS:IP=%04X%04X modrm=%02X value=%04X\n", cpu->cs, old_ip, modrm, value);
                    return -1;
            
            }
            
            break;
        
        }
        
        case 0x8F: {
        
            uint8_t modrm = fetch8 (cpu);
            
            OperandRef rm;
            uint16_t value;
            
            /* only /0 is valid for POP r/m16 on 8086 */
            if (((modrm >> 3) & 7) != 0) {
            
                printf ("Unhandled 0x8F /%d at %04X\n", (modrm >> 3) & 7, old_ip);
                return -1;
            
            }
            
            rm = decode_rm_operand (cpu, modrm);
            value = pop16 (cpu);
            
            operand_write16 (cpu, &rm, value);
            break;
        
        }
        
        case 0x90:             /* NOP (XCHG AX, AX) */
        
            break;
        
        case 0x91:  case 0x92:  case 0x93:  case 0x94:  case 0x95:  case 0x96:  case 0x97: {
        
            /* XCHG AX, reg16 */
            int reg = op - 0x90;
            uint16_t temp = cpu->regs[REG_AX];
            
            cpu->regs[REG_AX] = cpu->regs[reg];
            cpu->regs[reg] = temp;
            
            break;
        
        }
        
        case 0x98: {           /* CBW - Convert Byte to Word */
        
            /* Sign-extend AL into AX */
            if (cpu->regs[REG_AX] & 0x80) {
                cpu->regs[REG_AX] |= 0xFF00;
            } else {
                cpu->regs[REG_AX] &= 0x00FF;
            }
            
            break;
        
        }
        
        case 0x99:             /* CWD - Convert Word to Doubleword */
        
            if (cpu->regs[REG_AX] & 0x8000) {
                cpu->regs[REG_DX] = 0xFFFF;
            } else {
                cpu->regs[REG_DX] = 0x0000;
            }
            
            break;
        
        case 0x9A: {
        
            uint16_t new_ip = fetch16 (cpu);
            uint16_t new_cs = fetch16 (cpu);
            
            push16 (cpu, cpu->cs);
            push16 (cpu, cpu->ip);
            
            cpu->cs = new_cs;
            cpu->ip = new_ip;
            
            break;
        
        }
        
        case 0x9C:             /* PUSHF (Push Flags onto Stack) */
        
            push16 (cpu, cpu->flags);
            break;
        
        case 0x9D: {           /* POPF */
        
            uint16_t f = pop16 (cpu);
            
            cpu->flags = (f & (FLAG_CF | FLAG_PF | FLAG_AF | FLAG_ZF | FLAG_SF | FLAG_TF | FLAG_IF | FLAG_DF | FLAG_OF)) | 0x0002;
            break;
        
        }
        
        case 0x9E: {           /* SAHF */
        
            uint8_t ah = get_reg8 (cpu, 4);
            cpu->flags &= ~(FLAG_CF | FLAG_PF | FLAG_AF | FLAG_ZF | FLAG_SF);
            
            if (ah & 0x01) {
                cpu->flags |= FLAG_CF;
            }
            
            if (ah & 0x04) {
                cpu->flags |= FLAG_PF;
            }
            
            if (ah & 0x10) {
                cpu->flags |= FLAG_AF;
            }
            
            if (ah & 0x40) {
                cpu->flags |= FLAG_ZF;
            }
            
            if (ah & 0x80) {
                cpu->flags |= FLAG_SF;
            }
            
            cpu->flags |= 0x0002; /* bit 1 always set */
            break;
        
        }
        
        case 0x9F: {           /* LAHF */
        
            uint8_t ah = 0x02; /* bit 1 always set */
            
            if (cpu->flags & FLAG_CF) {
                ah |= 0x01;
            }
            
            if (cpu->flags & FLAG_PF) {
                ah |= 0x04;
            }
            
            if (cpu->flags & FLAG_AF) {
                ah |= 0x10;
            }
            
            if (cpu->flags & FLAG_ZF) {
                ah |= 0x40;
            }
            
            if (cpu->flags & FLAG_SF) {
                ah |= 0x80;
            }
            
            set_reg8 (cpu, 4, ah); /* AH */
            break;
        
        }
        
        /* Inside cpu_step switch (op) in cpu.c */
        case 0xA0: /* MOV AL, moffs8 */     case 0xA1: /* MOV AX, moffs16 */ {
        
            uint16_t offset = fetch16 (cpu);
            uint16_t seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            uint32_t phys = get_phys (seg, offset);
            
            if (op == 0xA0) {
                set_reg8 (cpu, REG_AX, mem_read8 (cpu->mem, cpu->mem_size, phys));
            } else {
            
                /* Read the word as two separate bytes to enforce segment wrapping */
                uint8_t low = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, offset));
                uint8_t high = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, offset + 1));
                
                cpu->regs[REG_AX] = (uint16_t) low | ((uint16_t) high << 8);
            
            }
            
            break;
        
        }
        
        case 0xA2: /* MOV moffs8, AL */     case 0xA3: /* MOV moffs16, AX */ {
        
            uint16_t offset = fetch16 (cpu);
            uint16_t seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            uint32_t phys = get_phys (seg, offset);
            
            if (op == 0xA2) {
                mem_write8 (cpu->mem, cpu->mem_size, phys, get_reg8 (cpu, REG_AX));
            } else {
            
                /* Write the word as two separate bytes to enforce segment wrapping */
                uint16_t val = cpu->regs[REG_AX];
                
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (seg, offset), (uint8_t) (val & 0xFF));
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (seg, offset + 1), (uint8_t) (val >> 8));
            
            }
            
            break;
        
        }
        
        case 0xA4: {           /* MOVSB */
        
            uint16_t src_seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            
            uint16_t count = rep_initial_count (cpu);
            uint16_t i;
            
            for (i = 0; i < count; i++) {
            
                uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, cpu->regs[REG_SI]));
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]), v);
                
                cpu->regs[REG_SI] = (uint16_t) (cpu->regs[REG_SI] + step);
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
            
            }
            
            if (rep_active (cpu)) {
                cpu->regs[REG_CX] = 0;
            }
            
            break;
        
        }
        
        case 0xA5: {           /* MOVSW */
        
            uint16_t src_seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            int step = (cpu->flags & FLAG_DF) ? -2 : 2;
            
            uint16_t count = rep_initial_count (cpu);
            uint16_t i;
            
            for (i = 0; i < count; i++) {
            
                uint8_t lo = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, cpu->regs[REG_SI]));
                uint8_t hi = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, (uint16_t) (cpu->regs[REG_SI] + 1)));
                
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]), lo);
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_DI] + 1)), hi);
                
                cpu->regs[REG_SI] = (uint16_t) (cpu->regs[REG_SI] + step);
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
            
            }
            
            if (rep_active (cpu)) {
                cpu->regs[REG_CX] = 0;
            }
            
            break;
        
        }
        
        case 0xA6: {           /* CMPSB */
        
            uint16_t src_seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            
            uint16_t count = rep_initial_count (cpu);
            
            while (count--) {
            
                uint8_t lhs = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, cpu->regs[REG_SI]));
                uint8_t rhs = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]));
                
                sub8 (cpu, lhs, rhs);
                
                cpu->regs[REG_SI] = (uint16_t) (cpu->regs[REG_SI] + step);
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                
                if (rep_active (cpu)) {
                
                    cpu->regs[REG_CX]--;
                    
                    if (cpu->rep_type == 1) {               /* REPE/REPZ */
                    
                        if (!(cpu->flags & FLAG_ZF)) {
                            break;
                        }
                    
                    } else if (cpu->rep_type == 2) {        /* REPNE/REPNZ */
                    
                        if (cpu->flags & FLAG_ZF) {
                            break;
                        }
                    
                    }
                    
                    if (cpu->regs[REG_CX] == 0) {
                        break;
                    }
                
                } else {
                    break;
                }
            
            }
            
            break;
        
        }
        
        case 0xA7: {           /* CMPSW */
        
            uint16_t src_seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            int step = (cpu->flags & FLAG_DF) ? -2 : 2;
            
            uint16_t count = rep_initial_count (cpu);
            
            while (count--) {
            
                uint8_t llo = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, cpu->regs[REG_SI]));
                uint8_t lhi = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, (uint16_t) (cpu->regs[REG_SI] + 1)));
                uint8_t rlo = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]));
                uint8_t rhi = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_DI] + 1)));
                
                uint16_t lhs = (uint16_t) llo | ((uint16_t) lhi << 8);
                uint16_t rhs = (uint16_t) rlo | ((uint16_t) rhi << 8);
                
                sub16 (cpu, lhs, rhs);
                
                cpu->regs[REG_SI] = (uint16_t) (cpu->regs[REG_SI] + step);
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                
                if (rep_active (cpu)) {
                
                    cpu->regs[REG_CX]--;
                    
                    if (cpu->rep_type == 1) {
                    
                        if (!(cpu->flags & FLAG_ZF)) {
                            break;
                        }
                    
                    } else if (cpu->rep_type == 2) {
                    
                        if (cpu->flags & FLAG_ZF) {
                            break;
                        }
                    
                    }
                    
                    if (cpu->regs[REG_CX] == 0) {
                        break;
                    }
                
                } else {
                    break;
                }
            
            }
            
            break;
        
        }
        
        case 0xA8: {           /* TEST AL, imm8 */
        
            uint8_t imm8 = fetch8 (cpu);
            
            and8 (cpu, get_reg8 (cpu, REG_AX), imm8);
            break;
        
        }
        
        case 0xA9: {           /* TEST AX, imm16 */
        
            uint16_t imm16 = fetch16 (cpu);
            
            and16 (cpu, cpu->regs[REG_AX], imm16);
            break;
        
        }
        
        case 0xAA: {           /* STOSB */
        
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            uint16_t val = cpu->regs[REG_AX];
            
            if (cpu->rep_type && cpu->regs[REG_CX] == 0) {
                break;
            }
            
            while (1) {
            
                if (cpu->rep_type != 0 && cpu->regs[REG_CX] == 0) {
                    break;
                }
                
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]), (uint8_t) (val & 0xFF));
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                
                if (cpu->rep_type != 0) {
                    cpu->regs[REG_CX]--;
                } else {
                    break;
                }
            
            }
            
            break;
        
        }
        
        case 0xAB: {           /* STOSW */
        
            int step = (cpu->flags & FLAG_DF) ? -2 : 2;
            uint16_t val = cpu->regs[REG_AX];
            
            if (cpu->rep_type && cpu->regs[REG_CX] == 0) {
                break;
            }
            
            while (1) {
            
                if (cpu->rep_type != 0 && cpu->regs[REG_CX] == 0) {
                    break;
                }
                
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]), (uint8_t) (val & 0xFF));
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_DI] + 1)), (uint8_t) (val >> 8));
                
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                
                if (cpu->rep_type != 0) {
                    cpu->regs[REG_CX]--;
                } else {
                    break;
                }
            
            }
            
            break;
        
        }
        
        case 0xAC: /* LODSB */      case 0xAD: /* LODSW */ {
        
            uint16_t seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            int step = (cpu->flags & FLAG_DF) ? ((op == 0xAC) ? -1 : -2) : ((op == 0xAC) ?  1 :  2);
            
            uint16_t count = rep_initial_count (cpu);
            
            while (count--) {
            
                if (op == 0xAC) {
                
                    uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, cpu->regs[REG_SI]));
                    set_al (cpu, v);
                
                } else {
                
                    uint8_t lo = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, cpu->regs[REG_SI]));
                    uint8_t hi = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, (uint16_t) (cpu->regs[REG_SI] + 1)));
                    
                    cpu->regs[REG_AX] = (uint16_t) lo | ((uint16_t) hi << 8);
                
                }
                
                cpu->regs[REG_SI] = (uint16_t) (cpu->regs[REG_SI] + step);
                
                if (rep_active (cpu)) {
                
                    cpu->regs[REG_CX]--;
                    
                    if (cpu->regs[REG_CX] == 0) {
                        break;
                    }
                
                } else {
                    break;
                }
            
            }
            
            break;
        
        }
        
        case 0xAE: {           /* SCASB */
        
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            uint8_t al = get_al (cpu);
            
            if (!rep_active (cpu)) {
            
                uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]));
                sub8 (cpu, al, v);
                
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                break;
            
            }
            
            while (cpu->regs[REG_CX] != 0) {
            
                uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_DI]));
                sub8 (cpu, al, v);
                
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                cpu->regs[REG_CX]--;
                
                if (cpu->rep_type == 1) {                   /* REPE/REPZ */
                
                    if (!(cpu->flags & FLAG_ZF)) {
                        break;
                    }
                
                } else if (cpu->rep_type == 2) {            /* REPNE/REPNZ */
                
                    if (cpu->flags & FLAG_ZF) {
                        break;
                    }
                
                }
            
            }
            
            break;
        
        }
        
        case 0xAF: {           /* SCASW */
        
            int step = (cpu->flags & FLAG_DF) ? -2 : 2;
            
            if (cpu->rep_type) {
            
                while (cpu->regs[REG_CX] != 0) {
                
                    uint16_t off = cpu->regs[REG_DI];
                    uint16_t v = (uint16_t) mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, off)) | ((uint16_t) mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (off + 1))) << 8);
                    
                    sub16 (cpu, cpu->regs[REG_AX], v);
                    
                    cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
                    cpu->regs[REG_CX]--;
                    
                    if (cpu->rep_type == 1 && !(cpu->flags & FLAG_ZF)) {
                        break;
                    }
                    
                    if (cpu->rep_type == 2 &&  (cpu->flags & FLAG_ZF)) {
                        break;
                    }
                
                }
            
            } else {
            
                uint16_t off = cpu->regs[REG_DI];
                uint16_t v = (uint16_t) mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, off)) | ((uint16_t) mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (off + 1))) << 8);
                
                sub16 (cpu, cpu->regs[REG_AX], v);
                cpu->regs[REG_DI] = (uint16_t) (cpu->regs[REG_DI] + step);
            
            }
            
            break;
        
        }
        
        /* Data Move Immediate to Register */
        case 0xB0:  case 0xB1:  case 0xB2:  case 0xB3:  case 0xB5:  case 0xB6:  case 0xB7:
        
            set_reg8 (cpu, op - 0xB0, fetch8 (cpu));
            break;
        
        case 0xB4: {
            
            uint8_t imm8 = fetch8 (cpu);
            set_reg8 (cpu, 4, imm8);
            
            break;
        
        }
        
        case 0xB8: case 0xB9: case 0xBA: case 0xBB:
        case 0xBC: case 0xBD: case 0xBE: case 0xBF:
        
            cpu->regs[op - 0xB8] = fetch16 (cpu);
            break;
        
        case 0xC0: {           /*8-bit Shift/Rotate by imm8 (80186+) */
        
            uint8_t modrm = fetch8 (cpu), result;
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint8_t val = operand_read8 (cpu, &rm);
            uint8_t count = fetch8 (cpu) & 0x1F;            /* Real CPUs mask count to 5 bits (0-31) */
            
            if (count == 0) {
                break;
            }
            
            result = val;
            
            switch (subop) {
            
                case 0:             /* ROL */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint8_t bit7 = (result & 0x80) >> 7;
                        result = (result << 1) | bit7;
                        
                        if (bit7) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {                    /* OF only defined for single-step rotations */
                    
                        if (((result & 0x80) >> 7) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= ~FLAG_OF;
                        }
                    
                    }
                    
                    break;
                
                case 4:             /* SHL / SAL */
                
                    if (count > 8) {
                    
                        cpu->flags &= ~FLAG_CF;
                        result = 0;
                    
                    } else {
                    
                        if ((val << (count - 1)) & 0x80) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = val << count;
                    
                    }
                    
                    if (count == 1) {
                    
                        if ((result ^ val) & 0x80) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= ~FLAG_OF;
                        }
                    
                    }
                    
                    update_common_flags8 (cpu, result);
                    break;
                
                case 5:             /* SHR */
                
                    if (count > 8) {
                    
                        cpu->flags &= ~FLAG_CF;
                        result = 0;
                    
                    } else {
                    
                        if ((val >> (count - 1)) & 0x01) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = val >> count;
                    
                    }
                    
                    if (count == 1) {
                    
                        if (val & 0x80) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= ~FLAG_OF;
                        }
                    
                    }
                    
                    update_common_flags8 (cpu, result);
                    break;
                
                default:
                
                    fprintf (stderr, "Opcode 0xC0 subop /%d not fully implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write8 (cpu, &rm, result);
            break;
        
        }
        
        case 0xC1: {           /*16-bit Shift/Rotate by imm8 (80186+) */
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            uint16_t result;
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint16_t val = operand_read16 (cpu, &rm);
            uint8_t count = fetch8 (cpu) & 0x1F; /* 186+ masks count */
            
            if (count == 0) {
                break;
            }
            
            result = val;
            
            switch (subop) {
            
                case 0: {           /* ROL */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint16_t bit15 = (result >> 15) & 1;
                        result = (uint16_t) ((result << 1) | bit15);
                        
                        if (bit15) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {
                    
                        if ((((result >> 15) & 1) ^ ((cpu->flags & FLAG_CF) ? 1 : 0))) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 1: {           /* ROR */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint16_t bit0 = result & 1;
                        result = (uint16_t) ((result >> 1) | (bit0 << 15));
                        
                        if (bit0) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {
                    
                        if ((((result >> 15) & 1) ^ ((result >> 14) & 1))) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 2: {           /* RCL */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint16_t curr_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                        uint16_t new_cf = (result >> 15) & 1;
                        
                        result = (uint16_t) ((result << 1) | curr_cf);
                        
                        if (new_cf) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {
                    
                        if ((((result >> 15) & 1) ^ ((cpu->flags & FLAG_CF) ? 1 : 0))) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 3: {           /* RCR */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint16_t curr_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                        uint16_t new_cf = result & 1;
                        
                        result = (uint16_t) ((result >> 1) | (curr_cf << 15));
                        
                        if (new_cf) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {
                    
                        if ((((result >> 15) & 1) ^ ((result >> 14) & 1))) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 4: {           /* SHL / SAL */
                
                    if (count > 16) {
                    
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                        result = 0;
                    
                    } else {
                    
                        if ((val << (count - 1)) & 0x8000) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        result = (uint16_t) (val << count);
                    
                    }
                    
                    if (count == 1) {
                    
                        if ((result ^ val) & 0x8000) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                    
                    update_common_flags16 (cpu, result);
                    break;
                
                }
                
                case 5: {           /* SHR */
                
                    if (count > 16) {
                    
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                        result = 0;
                    
                    } else {
                    
                        if ((val >> (count - 1)) & 0x0001) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        result = (uint16_t) (val >> count);
                    
                    }
                    
                    if (count == 1) {
                    
                        if (val & 0x8000) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                    
                    update_common_flags16 (cpu, result);
                    break;
                
                }
                
                case 7: {           /* SAR */
                
                    if (count > 16) {
                    
                        result = (uint16_t) ((val & 0x8000) ? 0xFFFF : 0x0000);
                        
                        if (val & 0x8000) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    } else {
                    
                        if (((int16_t) val >> (count - 1)) & 0x0001) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        result = (uint16_t) ((int16_t) val >> count);
                    
                    }
                    
                    if (count == 1) {
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                    }
                    
                    update_common_flags16 (cpu, result);
                    break;
                
                }
                
                default:
                
                    fprintf (stderr, "Opcode 0xC1 subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write16 (cpu, &rm, result);
            break;
        
        }
        
        case 0xC2: {           /* RET imm16 */
        
            uint16_t add_sp = fetch16 (cpu);
            uint16_t new_ip = pop16 (cpu);
            
            cpu->regs[REG_SP] = (uint16_t) (cpu->regs[REG_SP] + add_sp);
            cpu->ip = new_ip;
            
            break;
        
        }
        
        case 0xC3: {           /* RET near */
        
            cpu->ip = pop16 (cpu);
            break;
        
        }
        
        case 0xC4: {           /* LES r16, m16:16 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint16_t ea_seg;
            uint16_t off_addr;
            uint16_t off;
            uint16_t es_val;
            
            if (!rm.is_mem) {
            
                printf ("LES requires memory operand at %04X%04X modrm=%02X\n", cpu->cs, old_ip, modrm);
                exit (1);
            
            }
            
            ea_seg   = rm.seg;
            off_addr = (uint16_t) rm.addr;
            
            off    = mem_read16 (cpu->mem, cpu->mem_size, get_phys (ea_seg, off_addr));
            es_val = mem_read16 (cpu->mem, cpu->mem_size, get_phys (ea_seg, (uint16_t) (off_addr + 2)));
            
            cpu->regs[reg] = off;
            cpu->es = es_val;
            
            break;
        
        }
        
        case 0xC5: {           /* LDS r16, m16:16 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint16_t src_seg, ptr_offset, seg_low, seg_high, ptr_segment;
            
            if (!rm.is_mem) {
            
                /* LES/LDS requires a memory operand (mod != 3) */
                return -1;
            
            }
            
            /* Determine the segment to read the pointer FROM (respects overrides) */
            src_seg = rm.seg;
            
            /* Read the 4-byte far pointer: Offset is at [addr], Segment is at [addr+2] */
            ptr_offset = operand_read16 (cpu, &rm);                    /* Already wrapped in your code */
            seg_low = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, rm.addr + 2));
            seg_high = mem_read8 (cpu->mem, cpu->mem_size, get_phys (src_seg, rm.addr + 3));
            ptr_segment = (uint16_t) seg_low | ((uint16_t) seg_high << 8);
            
            /* Load the registers */
            cpu->regs[reg] = ptr_offset;
            cpu->ds = ptr_segment;
            
            break;
        
        }
        
        case 0xC6: /* MOV r/m8, imm8 */     case 0xC7: /* MOV r/m16, imm16 */ {
        
            uint8_t modrm = fetch8 (cpu);
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0xC6) {
            
                uint8_t imm8 = fetch8 (cpu);                /* Correctly fetch 8 bits */
                operand_write8 (cpu, &rm, imm8);
            
            } else {
            
                uint16_t imm16 = fetch16 (cpu);             /* Correctly fetch 16 bits */
                operand_write16 (cpu, &rm, imm16);
            
            }
            
            break;
        
        }
        
        case 0xC8: {                    /* ENTER imm16, imm8 (80186+) */
        
            uint16_t size = fetch16 (cpu);
            unsigned level = fetch8 (cpu) & 0x1F;
            
            uint16_t frame, parent = cpu->regs[REG_BP];
            unsigned i;
            
            push16 (cpu, parent);
            frame = cpu->regs[REG_SP];
            
            if (level) {
            
                for (i = 1; i < level; i++) {
                
                    uint16_t low, high;
                    parent = (uint16_t) (parent - 2);
                    
                    low = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, parent));
                    high = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, (uint16_t) (parent + 1)));
                    
                    push16 (cpu, (uint16_t) (low | (high << 8)));
                
                }
                
                push16 (cpu, frame);
            
            }
            
            cpu->regs[REG_BP] = frame;
            cpu->regs[REG_SP] = (uint16_t) (cpu->regs[REG_SP] - size);
            
            break;
        
        }
        
        case 0xC9:             /* LEAVE */
        
            cpu->regs[REG_SP] = cpu->regs[REG_BP];
            cpu->regs[REG_BP] = pop16 (cpu);
            
            break;
        
        case 0xCA: {           /* RETF imm16 */
        
            uint16_t add_sp = fetch16 (cpu);
            uint16_t new_ip = pop16 (cpu);
            uint16_t new_cs = pop16 (cpu);
            
            cpu->regs[REG_SP] = (uint16_t) (cpu->regs[REG_SP] + add_sp);
            cpu->ip = new_ip;
            cpu->cs = new_cs;
            
            break;
        
        }
        
        case 0xCB: {
        
            uint16_t new_ip = pop16 (cpu);
            uint16_t new_cs = pop16 (cpu);
            
            cpu->ip = new_ip;
            cpu->cs = new_cs;
            
            break;
        
        }
        
        case 0xCD: {           /* INT imm8 */
        
            uint8_t inum = fetch8 (cpu);
            
            /* Native DOS bridge executes only from our reserved IVT stubs. */
            if (inum == 0xFF && cpu->cs == DOS_IVT_STUB_SEG &&
                old_ip >= 0 && old_ip < 1024 && (old_ip & 3) == 0 &&
                    mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->cs, (uint16_t) old_ip)) == 0xCD &&
                        mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->cs, (uint16_t) (old_ip + 1))) == 0xFF) {
                native_interrupt (cpu, (uint8_t) (old_ip >> 2));
            } else {
                handle_interrupt (cpu, inum);
            }
            
            break;
        
        }
        
        case 0xCC: {           /* INT 3 (Breakpoint) */
        
            handle_interrupt (cpu, 3);
            break;
        
        }
        
        case 0xCE: {           /* INTO (Interrupt on Overflow) */
        
            if (cpu->flags & FLAG_OF) {
                handle_interrupt (cpu, 4);
            }
            
            break;
        
        }
        
        case 0xCF: {
        
            uint16_t new_ip = pop16 (cpu);
            uint16_t new_cs = pop16 (cpu);
            uint16_t new_flags = pop16 (cpu);
            
            cpu->ip = new_ip;
            cpu->cs = new_cs;
            cpu->flags = new_flags;
            
            break;
        
        }
        
        case 0xD4: {           /* AAM imm8 */
        
            uint8_t base = fetch8 (cpu);
            uint8_t al = get_al (cpu);
            
            if (base == 0) {
            
                handle_interrupt (cpu, 0);
                break;
            
            }
            
            cpu->regs[REG_AX] = (uint16_t) (((uint16_t) (al / base) << 8) | (al % base));
            
            update_common_flags8 (cpu, get_al (cpu));
            break;
        
        }
        
        case 0xFF: {
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            switch (subop) {
            
                case 0:             /* INC rm16 */
                
                    operand_write16 (cpu, &rm, inc16 (cpu, operand_read16 (cpu, &rm)));
                    break;
                
                case 1:             /* DEC rm16 */
                
                    operand_write16 (cpu, &rm, dec16 (cpu, operand_read16 (cpu, &rm)));
                    break;
                
                case 2: {           /* CALL near indirect */
                
                    uint16_t target = operand_read16 (cpu, &rm);
                    push16 (cpu, cpu->ip);
                    
                    cpu->ip = target;
                    break;
                
                }
                
                case 3:             /* CALL far indirect */
                
                    if (!rm.is_mem) {
                        return -1;
                    }
                    
                    {
                    
                        uint16_t ptr_seg = rm.seg;
                        uint16_t new_ip = operand_read16 (cpu, &rm);
                        
                        uint8_t cs_low  = mem_read8 (cpu->mem, cpu->mem_size, get_phys (ptr_seg, rm.addr + 2));
                        uint8_t cs_high = mem_read8 (cpu->mem, cpu->mem_size, get_phys (ptr_seg, rm.addr + 3));
                        
                        uint16_t new_cs = (uint16_t) cs_low | ((uint16_t) cs_high << 8);
                        
                        push16 (cpu, cpu->cs);
                        push16 (cpu, cpu->ip);
                        
                        cpu->cs = new_cs;
                        cpu->ip = new_ip;
                    
                    }
                    
                    break;
                
                case 4:             /* JMP near indirect */
                
                    cpu->ip = operand_read16 (cpu, &rm);
                    break;
                
                case 5:             /* JMP far indirect (m16:16) */
                
                    if (!rm.is_mem) {
                        return -1;
                    }
                    
                    {
                    
                        uint16_t ptr_seg = rm.seg;
                        uint16_t new_ip = operand_read16 (cpu, &rm);
                        
                        uint8_t cs_low  = mem_read8 (cpu->mem, cpu->mem_size, get_phys (ptr_seg, rm.addr + 2));
                        uint8_t cs_high = mem_read8 (cpu->mem, cpu->mem_size, get_phys (ptr_seg, rm.addr + 3));
                        
                        uint16_t new_cs = (uint16_t) cs_low | ((uint16_t) cs_high << 8);
                        
                        cpu->cs = new_cs;
                        cpu->ip = new_ip;
                    
                    }
                    
                    break;
                
                case 6:             /* PUSH rm16 */
                
                    push16 (cpu, operand_read16 (cpu, &rm));
                    break;
                
                default:
                
                    fprintf (stderr, "Unhandled 0xFF extension /%d (Undefined) at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            break;
        
        }
        
        case 0xD0: {           /*8-bit Shift/Rotate by 1 */
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint8_t val = operand_read8 (cpu, &rm);
            uint8_t result = val;
            uint8_t old_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
            
            switch (subop) {
            
                case 0:             /* ROL (Rotate Left) */
                
                    result = (val << 1) | (val >> 7);
                    
                    if (val & 0x80) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    /* OF = bit 7 (after) XOR CF */
                    if (((result & 0x80) >> 7) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 1:             /* ROR (Rotate Right) */
                
                    result = (val >> 1) | (val << 7);
                    
                    if (val & 0x01) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    /* OF = bit 7 XOR bit 6 (after) */
                    if (((result & 0x80) >> 7) ^ ((result & 0x40) >> 6)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 2:             /* RCL (Rotate through Carry Left) */
                
                    result = (val << 1) | old_cf;
                    
                    if (val & 0x80) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if (((result & 0x80) >> 7) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 3:             /* RCR (Rotate through Carry Right) */
                
                    result = (val >> 1) | (old_cf << 7);
                    
                    if (val & 0x01) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if (((result & 0x80) >> 7) ^ ((result & 0x40) >> 6)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 4:             /* SHL / SAL */
                
                    result = val << 1;
                    
                    if (val & 0x80) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if ((result ^ val) & 0x80) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    update_common_flags8 (cpu, result);     /* Shifts DO affect ZF/SF/PF */
                    break;
                
                case 5:             /* SHR */
                
                    result = val >> 1;
                    
                    if (val & 0x01) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if (val & 0x80) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    update_common_flags8 (cpu, result);
                    break;
                
                case 7:             /* SAR */
                
                    result = (uint8_t) ((int8_t) val >> 1);
                    
                    if (val & 0x01) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    cpu->flags &= ~FLAG_OF;
                    update_common_flags8 (cpu, result);
                    
                    break;
                
                default:
                
                    fprintf (stderr, "Opcode 0xD0 subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write8 (cpu, &rm, result);
            break;
        
        }
        
        case 0xD1: {           /*16-bit Shift/Rotate by 1 */
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint16_t val = operand_read16 (cpu, &rm);
            uint16_t result = val;
            
            uint8_t old_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
            
            switch (subop) {
            
                case 0:             /* ROL (Rotate Left) */
                
                    result = (val << 1) | (val >> 15);
                    
                    if (val & 0x8000) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    /* OF = bit 15 (after) XOR CF */
                    if (((result & 0x8000) >> 15) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 1:             /* ROR (Rotate Right) */
                
                    result = (val >> 1) | (val << 15);
                    
                    if (val & 0x0001) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    /* OF = bit 15 XOR bit 14 (after) */
                    if (((result & 0x8000) >> 15) ^ ((result & 0x4000) >> 14)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 2:             /* RCL (Rotate through Carry Left) */
                
                    result = (val << 1) | old_cf;
                    
                    if (val & 0x8000) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if (((result & 0x8000) >> 15) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 3:             /* RCR (Rotate through Carry Right) */
                
                    result = (val >> 1) | ((uint16_t) old_cf << 15);
                    
                    if (val & 0x0001) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if (((result & 0x8000) >> 15) ^ ((result & 0x4000) >> 14)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    break;
                
                case 4:             /* SHL / SAL */
                
                    result = val << 1;
                    
                    if (val & 0x8000) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if ((result ^ val) & 0x8000) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    update_common_flags16 (cpu, result);
                    break;
                
                case 5:             /* SHR */
                
                    result = val >> 1;
                    
                    if (val & 0x0001) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    if (val & 0x8000) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= ~FLAG_OF;
                    }
                    
                    update_common_flags16 (cpu, result);
                    break;
                
                case 7:             /* SAR */
                
                    result = (uint16_t) ((int16_t) val >> 1);
                    
                    if (val & 0x0001) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= ~FLAG_CF;
                    }
                    
                    cpu->flags &= ~FLAG_OF;
                    update_common_flags16 (cpu, result);
                    
                    break;
                
                default:
                
                    fprintf (stderr, "Opcode 0xD1 subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write16 (cpu, &rm, result);
            break;
        
        }
        
        case 0xD2: {           /*8-bit Shift/Rotate by CL */
        
            uint8_t modrm = fetch8 (cpu), result;
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint8_t val = operand_read8 (cpu, &rm);
            uint8_t count = get_reg8 (cpu, REG_CX);         /* Use CL register value */
            
            if (count == 0) {
                break;
            }
            
            /* For 8086, count is not masked. For 186+, count = count & 0x1F. */
            /* Standard practice is to mask it to prevent long hangs. */
            count &= 0x1F;
            result = val;
            
            switch (subop) {
            
                case 0:             /* ROL */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint8_t bit7 = (result & 0x80) >> 7;
                        result = (result << 1) | bit7;
                        
                        if (bit7) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                    
                    }
                    
                    break;
                
                case 1:             /* ROR */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint8_t bit0 = result & 0x01;
                        result = (result >> 1) | (bit0 << 7);
                        
                        if (bit0) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                    
                    }
                    
                    break;
                
                case 2:             /* RCL */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint8_t curr_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                        
                        if (result & 0x80) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = (result << 1) | curr_cf;
                    
                    }
                    
                    break;
                
                case 3:             /* RCR */
                
                    for (int i = 0; i < count; i++) {
                    
                        uint8_t curr_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                        
                        if (result & 0x01) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = (result >> 1) | (curr_cf << 7);
                    
                    }
                    
                    break;
                
                case 4:             /* SHL / SAL */
                
                    if (count >= 8) {
                    
                        if ((val << (count - 1)) & 0x80) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = 0;
                    
                    } else {
                    
                        if ((val << (count - 1)) & 0x80) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = val << count;
                    
                    }
                    
                    update_common_flags8 (cpu, result);
                    break;
                
                case 5:             /* SHR */
                
                    if (count >= 8) {
                    
                        if ((val >> (count - 1)) & 0x01) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = 0;
                    
                    } else {
                    
                        if ((val >> (count - 1)) & 0x01) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = val >> count;
                    
                    }
                    
                    update_common_flags8 (cpu, result);
                    break;
                
                case 7:             /* SAR */
                
                    if (count >= 8) {
                    
                        result = (val & 0x80) ? 0xFF : 0;
                        
                        if (val & 0x80) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                    
                    } else {
                    
                        if (((int8_t) val >> (count - 1)) & 0x01) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= ~FLAG_CF;
                        }
                        
                        result = (uint8_t) ((int8_t) val >> count);
                    
                    }
                    
                    update_common_flags8 (cpu, result);
                    break;
                
                default:
                
                    fprintf (stderr, "Opcode 0xD2 subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write8 (cpu, &rm, result);
            break;
        
        }
        
        case 0xD3: {
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint16_t val = operand_read16 (cpu, &rm), result;
            uint8_t count = get_reg8 (cpu, REG_CX);         /* Use CL register */
            
            /* For 8086, count is not masked. For 186+, count = count & 0x1F. */
            /* Standard practice is to mask it to prevent long hangs. */
            count &= 0x1F;
            
            if (count == 0) {
                break;
            }
            
            result = val;
            
            if (subop == 0) {           /* ROL */
            
                for (int i = 0; i < count; i++) {
                
                    uint16_t bit15 = (result >> 15) & 1;
                    result = (uint16_t) ((result << 1) | bit15);
                    
                    if (bit15) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                
                }
                
                if (count == 1) {
                
                    if (((result >> 15) & 1) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                    }
                
                }
            
            } else if (subop == 1) {    /* ROR */
            
                for (int i = 0; i < count; i++) {
                
                    uint16_t bit0 = result & 1;
                    result = (uint16_t) ((result >> 1) | (bit0 << 15));
                    
                    if (bit0) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                
                }
                
                if (count == 1) {
                
                    if (((result >> 15) & 1) ^ ((result >> 14) & 1)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                    }
                
                }
            
            } else if (subop == 2) {    /* RCL */
            
                for (int i = 0; i < count; i++) {
                
                    uint16_t curr_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                    uint16_t new_cf = (result >> 15) & 1;
                    
                    result = (uint16_t) ((result << 1) | curr_cf);
                    
                    if (new_cf) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                
                }
                
                if (count == 1) {
                
                    if (((result >> 15) & 1) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                    }
                
                }
            
            } else if (subop == 3) {    /* RCR */
            
                for (int i = 0; i < count; i++) {
                
                    uint16_t curr_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                    uint16_t new_cf = result & 1;
                    
                    result = (uint16_t) ((result >> 1) | (curr_cf << 15));
                    
                    if (new_cf) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                
                }
                
                if (count == 1) {
                
                    if (((result >> 15) & 1) ^ ((result >> 14) & 1)) {
                        cpu->flags |= FLAG_OF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                    }
                
                }
            
            } else if (subop == 4) {    /* SHL / SAL */
            
                result = val << count;
                
                /* Set Carry Flag (CF) to the last bit shifted out */
                if ((val << (count - 1)) & 0x8000) {
                    cpu->flags |= FLAG_CF;
                } else {
                    cpu->flags &= (uint16_t) ~FLAG_CF;
                }
            
            } else if (subop == 5) {    /* SHR */
            
                result = val >> count;
                
                if ((val >> (count - 1)) & 0x0001) {
                    cpu->flags |= FLAG_CF;
                } else {
                    cpu->flags &= (uint16_t) ~FLAG_CF;
                }
            
            } else if (subop == 7) {    /* SAR (Arithmetic shift - preserves sign) */
            
                result = (uint16_t) ((int16_t) val >> count);
                
                if (((int16_t) val >> (count - 1)) & 0x0001) {
                    cpu->flags |= FLAG_CF;
                } else {
                    cpu->flags &= (uint16_t) ~FLAG_CF;
                }
            
            } else {
            
                fprintf (stderr, "Shift subop /%d not implemented at %04X\n", subop, old_ip);
                return -1;
            
            }
            
            operand_write16 (cpu, &rm, result);
            
            if (subop == 4 || subop == 5 || subop == 7) {
                update_common_flags16 (cpu, result);
            }
            
            break;
        
        }
        
        case 0xD7: {           /* XLAT */
        
            uint16_t index = (uint16_t) (cpu->regs[REG_BX] + get_reg8 (cpu, REG_AX));
            uint16_t seg = cpu->has_seg_override ? cpu->seg_override : cpu->ds;
            
            uint8_t value = mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, index));
            set_reg8 (cpu, REG_AX, value);                  /* AL */
            
            break;
        
        }
        
        case 0xD8:  case 0xD9:  case 0xDA:  case 0xDB:
        case 0xDC:  case 0xDD:  case 0xDE:  case 0xDF: {
        
            uint8_t modrm = fetch8 (cpu);
            
            /* Even if FPU is not implemented, we MUST decode the ModRM */
            /* to ensure the IP correctly skips any displacement bytes. */
            decode_rm_operand (cpu, modrm);
            break;
        
        }
        
        case 0xE0: {           /* LOOPNE / LOOPNZ */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint16_t next_ip = cpu->ip, new_cx, target;
            int zf, take;
            
            cpu->regs[REG_CX] = (uint16_t) (cpu->regs[REG_CX] - 1);
            
            new_cx = cpu->regs[REG_CX];
            target = (uint16_t) (next_ip + rel);
            
            zf = (cpu->flags & FLAG_ZF) != 0;
            take = (new_cx != 0) && !zf;
            
            if (take) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE1: {           /* LOOPE / LOOPZ */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint16_t next_ip = cpu->ip, new_cx, target;
            int zf, take;
            
            cpu->regs[REG_CX] = (uint16_t) (cpu->regs[REG_CX] - 1);
            
            new_cx = cpu->regs[REG_CX];
            target = (uint16_t) (next_ip + rel);
            
            zf = (cpu->flags & FLAG_ZF) != 0;
            take = (new_cx != 0) && zf;
            
            if (take) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE2: {           /* LOOP */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint16_t next_ip = cpu->ip, new_cx, target;
            int take;
            
            cpu->regs[REG_CX] = (uint16_t) (cpu->regs[REG_CX] - 1);
            
            new_cx = cpu->regs[REG_CX];
            target = (uint16_t) (next_ip + rel);
            
            if ((take = (new_cx != 0))) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE3: {           /* JCXZ */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint16_t next_ip = cpu->ip;
            uint16_t target = (uint16_t) (next_ip + rel);
            
            int take = (cpu->regs[REG_CX] == 0);
            
            if (take) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE4: {           /* IN AL, imm8 */
        
            uint8_t port = fetch8 (cpu);
            set_reg8 (cpu, 0, io_read8 (cpu, port));
            
            break;
        
        }
        
        case 0xE5: {           /* IN AX, imm8 */
        
            uint8_t port = fetch8 (cpu);
            cpu->regs[REG_AX] = io_read16 (cpu, port);
            
            break;
        
        }
        
        case 0xE6: {           /* OUT imm8, AL */
        
            uint8_t port = fetch8 (cpu);
            io_write8 (cpu, port, get_reg8 (cpu, 0));
            
            break;
        
        }
        
        case 0xE7: {           /* OUT imm8, AX */
        
            uint8_t port = fetch8 (cpu);
            io_write16 (cpu, port, cpu->regs[REG_AX]);
            
            break;
        
        }
        
        case 0xE8: {
        
            int16_t rel = (int16_t) fetch16 (cpu);
            
            uint16_t ret = cpu->ip;
            uint16_t target = (uint16_t) (cpu->ip + rel);
            
            push16 (cpu, ret);
            cpu->ip = target;
            
            break;
        
        }
        
        case 0xE9: {
        
            int16_t rel = (int16_t) fetch16 (cpu);
            uint16_t target = (uint16_t) (cpu->ip + rel);
            
            cpu->ip = target;
            break;
        
        }
        
        case 0xEA: {           /* JMP far ptr16:16 */
        
            uint16_t new_ip = fetch16 (cpu);
            uint16_t new_cs = fetch16 (cpu);
            
            cpu->cs = new_cs;
            cpu->ip = new_ip;
            
            break;
        
        }
        
        case 0xEB: {
        
            int8_t rel = (int8_t) fetch8 (cpu);
            uint16_t target = (uint16_t) (cpu->ip + rel);
            
            cpu->ip = target;
            break;
        
        }
        
        case 0xEC: {           /* IN AL, DX */
        
            uint16_t port = cpu->regs[REG_DX];
            set_reg8 (cpu, 0, io_read8 (cpu, port));
            
            break;
        
        }
        
        case 0xED: {           /* IN AX, DX */
        
            uint16_t port = cpu->regs[REG_DX];
            cpu->regs[REG_AX] = io_read16 (cpu, port);
            
            break;
        
        }
        
        case 0xEE: {           /* OUT DX, AL */
        
            io_write8 (cpu, cpu->regs[REG_DX], get_reg8 (cpu, 0));
            break;
        
        }
        
        case 0xEF: {           /* OUT DX, AX */
        
            io_write16 (cpu, cpu->regs[REG_DX], cpu->regs[REG_AX]);
            break;
        
        }
        
        case 0xF0:             /* LOCK prefix */
        
            goto fetch_instruction;
        
        case 0xF2:             /* REPNZ */
        
            cpu->rep_type = 2;
            goto fetch_instruction;
        
        case 0xF3:             /* REP / REPE */
        
            cpu->rep_type = 1;
            goto fetch_instruction;
        
        case 0xF4:
        
            cpu->halted = 1;
            break;
        
        case 0xF6: /* Unary rm8 */      case 0xF7: {
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0xF6) {                    /*8-bit Operations */
            
                switch (subop) {
                
                    case 0: case 1: {
                    
                        uint8_t imm = fetch8 (cpu);
                        uint8_t val = operand_read8 (cpu, &rm);
                        
                        and8 (cpu, val, imm);
                        break;
                    }
                    
                    case 2:             /* NOT rm8 */
                    
                        operand_write8 (cpu, &rm, ~operand_read8 (cpu, &rm));
                        break;
                    
                    case 3:             /* NEG rm8 */
                    
                        operand_write8 (cpu, &rm, sub8 (cpu, 0, operand_read8 (cpu, &rm)));
                        break;
                    
                    case 4: {           /* MUL rm8 (AL * rm8 -> AX) */
                    
                        uint8_t src = operand_read8 (cpu, &rm);
                        uint16_t res = (uint16_t) get_reg8 (cpu, 0) * (uint16_t) src;               /* 0 = AL */
                        
                        cpu->regs[REG_AX] = res;
                        
                        if ((res >> 8) != 0) {
                            cpu->flags |= (FLAG_CF | FLAG_OF);
                        } else {
                            cpu->flags &= ~(FLAG_CF | FLAG_OF);
                        }
                        
                        break;
                    
                    }
                    
                    case 5: {           /* IMUL rm8 (AL * rm8 -> AX) */
                    
                        int8_t src = (int8_t) operand_read8 (cpu, &rm);
                        int16_t res = (int16_t) (int8_t) get_reg8 (cpu, 0) * (int16_t) src;
                        
                        cpu->regs[REG_AX] = (uint16_t) res;
                        
                        if (res < -128 || res > 127) {
                            cpu->flags |= (FLAG_CF | FLAG_OF);
                        } else {
                            cpu->flags &= ~(FLAG_CF | FLAG_OF);
                        }
                        
                        break;
                    
                    }
                    
                    case 6: {           /* DIV rm8 (AX / rm8) */
                    
                        uint16_t dividend, quotient, remainder;
                        uint8_t divisor = operand_read8 (cpu, &rm);
                        
                        if (divisor == 0) {
                            return -1;
                        }
                        
                        dividend = cpu->regs[REG_AX];
                        quotient = dividend / divisor;
                        remainder = dividend % divisor;
                        
                        if (quotient > 0xFF) {
                            return -1;
                        }
                        
                        set_reg8 (cpu, 0, (uint8_t) quotient);  /* AL */
                        set_reg8 (cpu, 4, (uint8_t) remainder); /* AH */
                        
                        break;
                    
                    }
                    
                    default:
                    
                        fprintf (stderr, "F6 Subop /%d not implemented\n", subop);
                        return -1;
                
                }
            
            } else {                    /*0xF7: 16-bit Operations */
            
                switch (subop) {
                
                    case 0:     case 1:                     /* TEST rm16, imm16 */
                    
                        and16 (cpu, operand_read16 (cpu, &rm), fetch16 (cpu));
                        break;
                    
                    case 2:             /* NOT rm16 */
                    
                        operand_write16 (cpu, &rm, ~operand_read16 (cpu, &rm));
                        break;
                    
                    case 3:             /* NEG rm16 */
                    
                        operand_write16 (cpu, &rm, sub16 (cpu, 0, operand_read16 (cpu, &rm)));
                        break;
                    
                    case 4: {           /* MUL rm16 (AX * rm16 -> DX:AX) */
                    
                        uint16_t src = operand_read16 (cpu, &rm);
                        uint32_t res = (uint32_t) cpu->regs[REG_AX] * (uint32_t) src;
                        
                        cpu->regs[REG_AX] = (uint16_t) (res & 0xFFFF);
                        cpu->regs[REG_DX] = (uint16_t) (res >> 16);
                        
                        if (cpu->regs[REG_DX] != 0) {
                            cpu->flags |= (FLAG_CF | FLAG_OF);
                        } else {
                            cpu->flags &= ~(FLAG_CF | FLAG_OF);
                        }
                        
                        break;
                    
                    }
                    
                    case 5: {           /* IMUL rm16 (AX * rm16 -> DX:AX) */
                    
                        int16_t src = (int16_t) operand_read16 (cpu, &rm);
                        int32_t res = (int32_t) (int16_t) cpu->regs[REG_AX] * (int32_t) src;
                        
                        cpu->regs[REG_AX] = (uint16_t) (res & 0xFFFF);
                        cpu->regs[REG_DX] = (uint16_t) ((uint32_t) res >> 16);
                        
                        if (res < -32768 || res > 32767) {
                            cpu->flags |= (FLAG_CF | FLAG_OF);
                        } else {
                            cpu->flags &= ~(FLAG_CF | FLAG_OF);
                        }
                        
                        break;
                    
                    }
                    
                    case 6: {           /* DIV rm16 (Unsigned DX:AX / rm16) */
                    
                        uint32_t dividend, quotient, remainder;
                        uint16_t divisor = operand_read16 (cpu, &rm);
                        
                        if (divisor == 0) {
                            handle_interrupt (cpu, 0); break;
                        }
                        
                        dividend = ((uint32_t) cpu->regs[REG_DX] << 16) | cpu->regs[REG_AX];
                        quotient = dividend / divisor;
                        remainder = dividend % divisor;
                        
                        if (quotient > 0xFFFF) {
                            handle_interrupt (cpu, 0); break;
                        }
                        
                        cpu->regs[REG_AX] = (uint16_t) quotient;
                        cpu->regs[REG_DX] = (uint16_t) remainder;
                        
                        break;
                    
                    }
                    
                    case 7: {           /* IDIV rm16 (Signed DX:AX / rm16) */
                    
                        int32_t dividend, quotient, remainder;
                        int16_t divisor = (int16_t) operand_read16 (cpu, &rm);
                        
                        if (divisor == 0) {
                            handle_interrupt (cpu, 0); break;
                        }
                        
                        dividend = (int32_t) (((uint32_t) cpu->regs[REG_DX] << 16) | cpu->regs[REG_AX]);
                        quotient = dividend / divisor;
                        remainder = dividend % divisor;
                        
                        if (quotient > 32767 || quotient < -32768) {
                            handle_interrupt (cpu, 0); break;
                        }
                        
                        cpu->regs[REG_AX] = (uint16_t) quotient;
                        cpu->regs[REG_DX] = (uint16_t) remainder;
                        
                        break;
                    
                    }
                    
                    default:
                    
                        fprintf (stderr, "F7 Subop /%d not implemented\n", subop);
                        return -1;
                
                }
            
            }
            
            break;
        
        }
        
        case 0xF8:             /* CLC (Clear Carry Flag) */
        
            cpu->flags &= (uint16_t) ~FLAG_CF;
            break;
        
        case 0xF9:             /* STC (Set Carry Flag) */
        
            cpu->flags |= FLAG_CF;
            break;
        
        case 0xF5:             /* CMC (Complement Carry Flag) */
        
            cpu->flags ^= FLAG_CF;
            break;
        
        /* Handle CLI (Clear Interrupt Flag - 0xFA) */
        case 0xFA: {
        
            cpu->flags &= ~FLAG_IF;     /* Clear the interrupt flag (IF) */
            break;
        
        }
        
        /* Handle STI (Set Interrupt Flag - 0xFB) */
        case 0xFB: {
        
            cpu->flags |= FLAG_IF;      /* Set the interrupt flag (IF) */
            break;
        
        }
        
        case 0xFC:             /* CLD (Clear Direction Flag) */
        
            cpu->flags &= (uint16_t) ~FLAG_DF;
            break;
        
        case 0xFD:             /* STD (Set Direction Flag) */
        
            cpu->flags |= FLAG_DF;
            break;
        
        case 0xFE: {
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint8_t val = operand_read8 (cpu, &rm);
            
            if (subop == 0) {           /* INC rm8 */
                operand_write8 (cpu, &rm, inc8 (cpu, val));
            } else if (subop == 1) {    /* DEC rm8 */
                operand_write8 (cpu, &rm, dec8 (cpu, val));
            } else {
            
                fprintf (stderr, "Unhandled 0xFE extension /%d at %04X\n", subop, old_ip);
                return -1;
            
            }
            
            break;
        
        }
        
        default:
        
            fprintf (stderr, "Unhandled opcode 0x%02X (%s) at %04X\n", op, opcode_name (op), old_ip);
            return -1;
    
    }
    
    if (trap_pending && !trap_inhibit && (cpu->flags & FLAG_TF) && !cpu->halted) {
        handle_interrupt (cpu, 1);
    }
    
    return 1;

}
