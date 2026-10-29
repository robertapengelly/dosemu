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

uint32_t get_phys (uint16_t seg, uint16_t off) {

    /* Standard 8086 behavior: wrap the offset at 64KB (16-bit) */
    /* BEFORE adding it to the segment base.*/
    return (((uint32_t) seg << 4) + (uint16_t) off) & 0xFFFFF;

}

static uint32_t cpu_segment_phys (CPU8086 *cpu, unsigned index, uint32_t off) {

    SegmentCache *cache = &cpu->segments[index];
    uint16_t selector;
    
    switch (index) {
        
        case 0:
        
            selector = cpu->es;
            break;
        
        case 1:
        
            selector = cpu->cs;
            break;
        
        case 2:
        
            selector = cpu->ss;
            break;
        
        case 3:
        
            selector = cpu->ds;
            break;
        
        case 4:
        
            selector = cpu->fs;
            break;
        
        default:
        
            selector = cpu->gs;
            break;
    
    }
    
    if (cache->protected && cache->selector == selector) {
    
        uint32_t addr = cache->base + off;
        return cpu->a20_enabled ? addr : addr & ~0x100000UL;
    
    }
    
    if (cpu->a20_enabled) {
        return ((uint32_t) selector << 4) + (uint16_t) off;
    }
    
    return get_phys (selector, (uint16_t) off);

}

static int cpu_code_32 (const CPU8086 *cpu) {
    return cpu->segments[1].protected && cpu->segments[1].selector == cpu->cs && (cpu->segments[1].flags & 0x40);
}

static void cpu_cache_real_segment (CPU8086 *cpu, unsigned index, uint16_t selector) {

    SegmentCache *cache = &cpu->segments[index];
    
    cache->selector = selector;
    cache->base = (uint32_t) selector << 4;
    cache->limit = 0xFFFFUL;
    cache->access = index == 1 ? 0x9B : 0x93;
    cache->flags = 0;
    cache->valid = 1;
    cache->protected = 0;

}

static void cpu_cache_real_segments (CPU8086 *cpu) {

    uint16_t selectors[6];
    unsigned i;
    
    selectors[0] = cpu->es;
    selectors[1] = cpu->cs;
    selectors[2] = cpu->ss;
    selectors[3] = cpu->ds;
    selectors[4] = cpu->fs;
    selectors[5] = cpu->gs;
    
    for (i = 0; i < 6; i++) {
    
        if (!cpu->segments[i].valid || (!cpu->segments[i].protected && cpu->segments[i].selector != selectors[i])) {
            cpu_cache_real_segment (cpu, i, selectors[i]);
        }
    
    }

}

static int cpu_load_segment (CPU8086 *cpu, unsigned index, uint16_t selector) {

    SegmentCache *cache = &cpu->segments[index];
    
    uint32_t desc;
    uint32_t base, limit;
    
    uint8_t access, flags;
    unsigned cpl, dpl, rpl;
    
    if (!(cpu->cr0 & 1)) {
        cpu_cache_real_segment (cpu, index, selector);
    } else {
    
        cpl = cpu->segments[1].protected ? (cpu->cs & 3) : 0;
        
        if (!(selector & 0xFFFC) && index != 1 && index != 2) {
        
            memset (cache, 0, sizeof (*cache));
            cache->selector = selector;
            
            switch (index) {
                
                case 0:
                
                    cpu->es = selector;
                    break;
                
                case 3:
                
                    cpu->ds = selector;
                    break;
                
                case 4:
                
                    cpu->fs = selector;
                    break;
                
                case 5:
                
                    cpu->gs = selector;
                    break;
            
            }
            
            return 0;
        
        }
        
        if (selector & 4) {
        
            if (!cpu->ldtr_valid || (uint32_t) (selector & 0xFFF8) + 7 > cpu->ldtr_limit) {
            
                fprintf (stderr, "invalid protected-mode LDT selector %04X (LDT limit %08X)\n", selector, cpu->ldtr_limit);
                return -1;
            
            }
            
            desc = cpu->ldtr_base + (selector & 0xFFF8);
        
        } else {
        
            if ((uint32_t) (selector & 0xFFF8) + 7 > cpu->gdtr_limit) {
            
                fprintf (stderr, "unsupported protected-mode selector %04X (GDT limit %04X)\n", selector, cpu->gdtr_limit);
                return -1;
            
            }
            
            desc = cpu->gdtr_base + (selector & 0xFFF8);
        
        }
        
        if ((uint64_t) desc + 7 >= cpu->mem_size) {
        
            fprintf (stderr, "protected-mode descriptor outside emulated memory: %08X\n", desc);
            return -1;
        
        }
        
        limit = (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, desc) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, desc + 1) << 8);
        base = (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, desc + 2) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, desc + 3) << 8) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, desc + 4) << 16) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, desc + 7) << 24);
        
        access = mem_read8 (cpu->mem, cpu->mem_size, desc + 5);
        flags = mem_read8 (cpu->mem, cpu->mem_size, desc + 6);
        
        limit |= ((uint32_t) flags & 0x0F) << 16;
        
        if (flags & 0x80) {
            limit = (limit << 12) | 0xFFF;
        }
        
        if (!(access & 0x80) || !(access & 0x10) || (index == 1 && !(access & 8)) || (index != 1 && (access & 8) && !(access & 2)) || (index == 2 && ((access & 8) || !(access & 2)))) {
        
            fprintf (stderr, "invalid protected-mode segment descriptor for selector %04X\n", selector);
            return -1;
        
        }
        
        dpl = (access >> 5) & 3;
        rpl = selector & 3;
        
        if (index == 1) {
        
            if ((access & 4) ? dpl > cpl : (dpl != cpl || rpl > cpl)) {
            
                fprintf (stderr, "invalid protected-mode code segment privilege for selector %04X\n", selector);
                return -1;
            
            }
            
            /* Conforming code retains the caller's CPL. */
            selector = (uint16_t) ((selector & ~3) | cpl);
        
        } else if (index == 2) {
        
            if (rpl != cpl || dpl != cpl) {
            
                fprintf (stderr, "invalid protected-mode stack segment privilege for selector %04X\n", selector);
                return -1;
            
            }
        
        } else if (!(access & 8) || !(access & 4)) {
        
            if (dpl < cpl || dpl < rpl) {
            
                fprintf (stderr, "invalid protected-mode data segment privilege for selector %04X\n", selector);
                return -1;
            
            }
        
        }
        
        cache->base = base;
        cache->limit = limit;
        cache->selector = selector;
        cache->access = access;
        cache->flags = flags;
        cache->valid = 1;
        cache->protected = 1;
    
    }
    
    switch (index) {
        
        case 0:
        
            cpu->es = selector;
            break;
        
        case 1:
        
            cpu->cs = selector;
            break;
        
        case 2:
        
            cpu->ss = selector;
            break;
        
        case 3:
        
            cpu->ds = selector;
            break;
        
        case 4:
        
            cpu->fs = selector;
            break;
        
        case 5:
        
            cpu->gs = selector;
            break;
    
    }
    
    return 0;

}

uint8_t fetch8 (CPU8086 *cpu) {

    uint32_t phys = cpu_segment_phys (cpu, 1, cpu->ip);
    
    uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, phys);
    cpu->ip = cpu_code_32 (cpu) ? cpu->ip + 1 : (uint16_t) (cpu->ip + 1); 
    
    return v;

}

uint16_t fetch16 (CPU8086 *cpu) {

    uint16_t low = fetch8 (cpu);
    uint16_t high = fetch8 (cpu);
    
    return low | (high << 8);

}

uint32_t fetch32 (CPU8086 *cpu) {

    uint32_t low = fetch16 (cpu);
    uint32_t high = fetch16 (cpu);
    
    return (high << 16) | low;

}

static uint32_t get_reg32 (const CPU8086 *cpu, int reg) {

    reg &= 7;
    return ((uint32_t) cpu->regs_high[reg] << 16) | cpu->regs[reg];

}

static void set_reg32 (CPU8086 *cpu, int reg, uint32_t value) {

    reg &= 7;
    
    cpu->regs[reg] = (uint16_t) value;
    cpu->regs_high[reg] = (uint16_t) (value >> 16);

}

static int cpu_stack_32 (CPU8086 *cpu) {
    return cpu->segments[2].protected && cpu->segments[2].selector == cpu->ss && (cpu->segments[2].flags & 0x40);
}

static uint32_t cpu_stack_pointer (CPU8086 *cpu) {
    return cpu_stack_32 (cpu) ? get_reg32 (cpu, REG_SP) : cpu->regs[REG_SP];
}

static void cpu_set_stack_pointer (CPU8086 *cpu, uint32_t value) {

    if (cpu_stack_32 (cpu)) {
        set_reg32 (cpu, REG_SP, value);
    } else {
        cpu->regs[REG_SP] = (uint16_t) value;
    }

}

static void push32 (CPU8086 *cpu, uint32_t value) {

    uint32_t off = cpu_stack_pointer (cpu) - 4;
    unsigned i;
    
    cpu_set_stack_pointer (cpu, off);
    
    for (i = 0; i < 4; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 2, off + i), (uint8_t) (value >> (8 * i)));
    }

}

static uint32_t pop32 (CPU8086 *cpu) {

    uint32_t off = cpu_stack_pointer (cpu);
    uint32_t value = 0;
    
    unsigned i;
    
    for (i = 0; i < 4; i++) {
        value |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 2, off + i)) << (8 * i);
    }
    
    cpu_set_stack_pointer (cpu, off + 4);
    return value;

}

static int cpu_protected_interrupt (CPU8086 *cpu, uint8_t vector) {

    uint32_t addr = cpu->idtr_base + (uint32_t) vector * 8;
    uint32_t offset;
    
    uint16_t selector;
    uint8_t type, access;
    
    uint32_t flags;
    
    if ((uint32_t) vector * 8 + 7 > cpu->idtr_limit || (uint64_t) addr + 7 >= cpu->mem_size) {
    
        fprintf (stderr, "protected-mode INT %02X outside IDT limit %04X\n", vector, cpu->idtr_limit);
        return -1;
    
    }
    
    offset = (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 1) << 8);
    selector = (uint16_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 2) | ((uint16_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 3) << 8);
    
    access = mem_read8 (cpu->mem, cpu->mem_size, addr + 5);
    type = access & 0x0F;
    
    offset |= ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 6) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 7) << 8)) << 16;
    
    if (!(access & 0x80) || (type != 6 && type != 7 && type != 14 && type != 15) || (access & 0x10) || ((access >> 5) & 3)) {
    
        fprintf (stderr, "protected-mode INT %02X has no supported present IDT gate at %08X (access %02X)\n", vector, addr, access);
        return -1;
    
    }
    
    flags = cpu->eflags_high | cpu->flags;
    
    if (type == 14 || type == 15) {
    
        push32 (cpu, flags);
        push32 (cpu, cpu->cs);
        push32 (cpu, cpu->ip);
    
    } else {
    
        push16 (cpu, cpu->flags);
        push16 (cpu, cpu->cs);
        push16 (cpu, cpu->ip);
        
        offset &= 0xFFFF;
    
    }
    
    if (cpu_load_segment (cpu, 1, selector) != 0) {
        return -1;
    }
    
    if (offset > cpu->segments[1].limit) {
    
        fprintf (stderr, "protected interrupt target %08X exceeds segment limit\n", offset);
        return -1;
    
    }
    
    cpu->ip = offset;
    cpu->flags &= (uint16_t) ~FLAG_TF;
    
    if (type == 6 || type == 14) {
        cpu->flags &= (uint16_t) ~FLAG_IF;
    }
    
    return 0;

}

static int cpu_divide_error (CPU8086 *cpu, uint32_t ip) {

    cpu->ip = ip;
    
    if (cpu->cr0 & 1) {
        return cpu_protected_interrupt (cpu, 0);
    }
    
    handle_interrupt (cpu, 0);
    return 0;

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

    uint32_t off = cpu_stack_pointer (cpu) - 2;
    cpu_set_stack_pointer (cpu, off);
    
    mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 2, off), (uint8_t) value);
    mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 2, off + 1), (uint8_t) (value >> 8));

}

uint16_t pop16 (CPU8086 *cpu) {

    uint32_t off = cpu_stack_pointer (cpu);
    
    uint8_t lo = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 2, off));
    uint8_t hi = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 2, off + 1));
    
    uint16_t value = lo | ((uint16_t) hi << 8);
    
    cpu_set_stack_pointer (cpu, off + 2);
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

static uint32_t pit_reload (PITCounter *counter) {

    if (!counter->bcd) {
        return counter->reload ? counter->reload : 65536;
    }
    
    {
    
        uint16_t raw = counter->reload;
        uint32_t count = (raw & 15) + ((raw >> 4) & 15) * 10 + ((raw >> 8) & 15) * 100 + ((raw >> 12) & 15) * 1000;
        
        return count ? count : 10000;
    
    }

}

static uint16_t pit_count_bcd (PITCounter *counter, uint32_t count) {

    if (!counter->bcd) {
        return (uint16_t) count;
    }
    
    count %= 10000;
    return (uint16_t) ((count % 10) | ((count / 10 % 10) << 4) | ((count / 100 % 10) << 8) | ((count / 1000 % 10) << 12));

}

static uint64_t pit_elapsed (CPU8086 *cpu, unsigned channel) {

    PITCounter *counter = &cpu->pit[channel];
    
    uint64_t now = pit_clock_ns ();
    uint64_t ns = now >= counter->started_ns ? now - counter->started_ns : 0;
    
    if (channel == 2 && !(cpu->port61 & 1)) {
        return 0;
    }
    
    return (ns / 1000000000ULL) * 1193182ULL + (ns % 1000000000ULL) * 1193182ULL / 1000000000ULL;

}

static uint16_t pit_count (CPU8086 *cpu, unsigned channel) {

    PITCounter *counter = &cpu->pit[channel];
    
    uint32_t reload = pit_reload (counter);
    uint64_t elapsed = pit_elapsed (cpu, channel);
    
    uint32_t remaining;
    
    switch (counter->mode) {
    
        case 2:
        
            remaining = reload - (uint32_t) (elapsed % reload);
            break;
        
        case 3: {
        
            uint32_t phase = (uint32_t) (elapsed % reload);
            uint32_t high = (reload + 1) / 2;
            
            remaining = phase < high ? reload - 2 * phase : (reload & ~1U) - 2 * (phase - high);
            break;
        
        }
        
        default:
        
            remaining = elapsed >= reload ? 0 : reload - (uint32_t) elapsed;
            break;
    
    }
    
    return pit_count_bcd (counter, remaining);

}

static uint8_t pit_output (CPU8086 *cpu, unsigned channel) {

    PITCounter *counter = &cpu->pit[channel];
    
    uint32_t reload = pit_reload (counter);
    uint64_t elapsed;
    
    if (channel == 2 && !(cpu->port61 & 1)) {
        return 1;
    }
    
    elapsed = pit_elapsed (cpu, channel);
    
    switch (counter->mode) {
    
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

static void pit_latch (CPU8086 *cpu, unsigned channel) {

    PITCounter *counter = &cpu->pit[channel];
    
    if (!counter->latch_active) {
    
        counter->latch = pit_count (cpu, channel);
        
        counter->latch_active = 1;
        counter->read_high = 0;
    
    }

}

static uint8_t pit_read (CPU8086 *cpu, unsigned channel) {

    PITCounter *counter = &cpu->pit[channel];
    
    uint16_t count;
    uint8_t result;
    
    if (counter->status_latch_active) {
    
        counter->status_latch_active = 0;
        return counter->status;
    
    }
    
    count = counter->latch_active ? counter->latch : pit_count (cpu, channel);
    
    if (counter->access == 2) {
        result = (uint8_t) (count >> 8);
    } else if (counter->access == 3) {
    
        result = counter->read_high ? (uint8_t) (count >> 8) : (uint8_t) count;
        counter->read_high ^= 1;
    
    } else {
        result = (uint8_t) count;
    }
    
    if (counter->latch_active && (counter->access != 3 || !counter->read_high)) {
        counter->latch_active = 0;
    }
    
    return result;

}

static void pit_write (CPU8086 *cpu, unsigned channel, uint8_t value) {

    PITCounter *counter = &cpu->pit[channel];
    
    if (counter->access == 2) {
        counter->reload = (uint16_t) value << 8;
    } else if (counter->access == 3 && !counter->write_high) {
    
        counter->write_low = value;
        counter->write_high = 1;
        
        return;
    
    } else if (counter->access == 3) {
    
        counter->reload = (uint16_t) ((uint16_t) value << 8) | counter->write_low;
        counter->write_high = 0;
    
    } else {
        counter->reload = value;
    }
    
    counter->null_count = 0;
    counter->started_ns = pit_clock_ns ();
    
    if (channel == 0) {
        cpu->pit_irq_last = 0;
    }
    
    if (channel == 2) {
        speaker_update (cpu->port61, counter->mode, counter->reload, 1);
    }

}

static void pit_control (CPU8086 *cpu, uint8_t value) {

    unsigned channel = (value >> 6) & 3;
    unsigned access = (value >> 4) & 3;
    unsigned mode = (value >> 1) & 7;
    
    PITCounter *counter;
    
    if (channel == 3) {                 /* 8254 read-back command */
    
        for (unsigned i = 0; i < 3; i++) {
        
            if (!(value & (1U << (i + 1)))) {
                continue;
            }
            
            counter = &cpu->pit[i];
            
            if (!(value & 0x10) && !counter->status_latch_active) {
            
                counter->status = (uint8_t) ((pit_output (cpu, i) << 7) |
                    (counter->null_count << 6) | counter->control);
                counter->status_latch_active = 1;
            
            }
            
            if (!(value & 0x20)) {
                pit_latch (cpu, i);
            }
        
        }
        
        return;
    
    }
    
    counter = &cpu->pit[channel];
    
    if (access == 0) {                  /* latch current count */
    
        pit_latch (cpu, channel);
        return;
    
    }
    
    counter->access = (uint8_t) access;
    counter->mode = (uint8_t) (mode >= 6 ? mode - 4 : mode);
    counter->bcd = value & 1;
    counter->control = value & 0x3F;
    counter->write_high = 0;
    counter->read_high = 0;
    counter->latch_active = 0;
    counter->status_latch_active = 0;
    counter->null_count = 1;
    
    if (channel == 2) {
        speaker_update (cpu->port61, counter->mode, counter->reload, 0);
    
    }

}

static void pic_write_command (PIC8259 *pic, uint8_t value) {

    if (value & 0x10) {
    
        pic->init_step = 1;
        
        pic->icw4_required = value & 1;
        pic->single = (value & 2) != 0;
        
        pic->irr = 0;
        pic->isr = 0;
        
        pic->read_isr = 0;
    
    } else if (value & 8) {             /* OCW3 */
    
        if (value & 2) {
            pic->read_isr = value & 1;
        }
    
    } else if (value & 0x20) {          /* OCW2: end of interrupt */
    
        unsigned i;
        
        if (value & 0x40) {             /* specific EOI */
            pic->isr &= (uint8_t) ~(1U << (value & 7));
        } else {
        
            for (i = 0; i < 8; i++) {
            
                if (pic->isr & (1U << i)) {
                
                    pic->isr &= (uint8_t) ~(1U << i);
                    break;
                
                }
            
            }
        
        }
    
    }

}

static void pic_write_data (PIC8259 *pic, uint8_t value) {

    switch (pic->init_step) {
        
        case 1:                         /* ICW2 */
        
            pic->offset = value & 0xF8;
            pic->init_step = pic->single ? (pic->icw4_required ? 3 : 0) : 2;
            
            break;
        
        case 2:                         /* ICW3 */
        
            pic->init_step = pic->icw4_required ? 3 : 0;
            break;
        
        case 3:                         /* ICW4 */
        
            pic->init_step = 0;
            break;
        
        default:                        /* OCW1: interrupt mask */
        
            pic->mask = value;
            break;
    
    }

}

static void sb_dma_update (CPU8086 *cpu) {

    DMA8237 *dma = &cpu->dma[0];
    
    uint64_t now;
    unsigned bytes;
    
    if (!cpu->sb_dsp_dma_active || cpu->sb_dsp_dma_paused || !cpu->sb_dsp_block_length || (dma->mask & 2) || (dma->mode[1] & 0x0F) != 0x09) {
        return;
    }
    
    now = pit_clock_ns ();
    
    if (now < cpu->sb_dsp_next_block_ns) {
        return;
    }
    
    bytes = cpu->sb_dsp_block_length;
    
    while (bytes) {
    
        unsigned remaining = (unsigned) dma->count[1] + 1;
        unsigned transferred = bytes < remaining ? bytes : remaining;
        
        if (dma->mode[1] & 0x20) {
            dma->address[1] = (uint16_t) (dma->address[1] - transferred);
        } else {
            dma->address[1] = (uint16_t) (dma->address[1] + transferred);
        }
        
        dma->count[1] = (uint16_t) (dma->count[1] - transferred);
        bytes -= transferred;
        
        if (transferred == remaining) {
        
            if (dma->mode[1] & 0x10) {
            
                dma->address[1] = dma->base_address[1];
                dma->count[1] = dma->base_count[1];
            
            } else {
            
                dma->mask |= 2;
                
                cpu->sb_dsp_dma_active = 0;
                break;
            
            }
        
        }
    
    }
    
    cpu->sb_dsp_irq_pending = 1;
    cpu->pic[0].irr |= 0x80;
    
    if (!cpu->sb_dsp_dma_auto) {
        cpu->sb_dsp_dma_active = 0;
    }
    
    cpu->sb_dsp_next_block_ns = now + (uint64_t) cpu->sb_dsp_block_length * (256 - cpu->sb_dsp_time_constant) * 1000ULL;

}

static int cpu_service_timer_irq (CPU8086 *cpu) {

    uint64_t elapsed;
    uint64_t now;
    
    sb_dma_update (cpu);
    
    if (cpu->pit[0].null_count) {
        return 0;
    }
    
    elapsed = pit_elapsed (cpu, 0);
    now = cpu->pit[0].mode == 2 || cpu->pit[0].mode == 3 ? elapsed / pit_reload (&cpu->pit[0]) : (elapsed >= pit_reload (&cpu->pit[0]) ? 1 : 0);
    
    if (now != cpu->pit_irq_last) {
    
        cpu->pit_irq_last = now;
        cpu->pic[0].irr |= 1;
    
    }
    
    if (cpu->kbd_release_scancode && !cpu->kbd_scancode_pending && pit_clock_ns () >= cpu->kbd_release_time) {
    
        cpu->kbd_scancode = cpu->kbd_release_scancode;
        cpu->kbd_scancode_pending = 1;
        cpu->kbd_release_scancode = 0;
        cpu->pic[0].irr |= 2;
    
    }
    
    if (!(cpu->flags & FLAG_IF)) {
        return 0;
    }
    
    if (!(cpu->pic[0].mask & 1) && !(cpu->pic[0].isr & 1) && (cpu->pic[0].irr & 1)) {
    
        cpu->pic[0].irr &= (uint8_t) ~1;
        cpu->pic[0].isr |= 1;
        
        if (cpu->cr0 & 1) {
            return cpu_protected_interrupt (cpu, cpu->pic[0].offset) == 0 ? 1 : -1;
        }
        
        handle_interrupt (cpu, cpu->pic[0].offset);
        return 1;
    
    }
    
    if (!(cpu->pic[0].mask & 2) && !(cpu->pic[0].isr & 3) && (cpu->pic[0].irr & 2)) {
    
        cpu->pic[0].irr &= (uint8_t) ~2;
        cpu->pic[0].isr |= 2;
        
        if (cpu->cr0 & 1) {
            return cpu_protected_interrupt (cpu, (uint8_t) (cpu->pic[0].offset + 1)) == 0 ? 1 : -1;
        }
        
        handle_interrupt (cpu, (uint8_t) (cpu->pic[0].offset + 1));
        return 1;
    
    }
    
    if (!(cpu->pic[0].mask & 0x80) && !cpu->pic[0].isr && (cpu->pic[0].irr & 0x80)) {
    
        cpu->pic[0].irr &= (uint8_t) ~0x80;
        cpu->pic[0].isr |= 0x80;
        
        if (cpu->cr0 & 1) {
            return cpu_protected_interrupt (cpu, (uint8_t) (cpu->pic[0].offset + 7)) == 0 ? 1 : -1;
        }
        
        handle_interrupt (cpu, (uint8_t) (cpu->pic[0].offset + 7));
        return 1;
    
    }
    
    return 0;

}

static void sb_dsp_reply (CPU8086 *cpu, uint8_t value) {

    if (cpu->sb_dsp_reply_count < sizeof (cpu->sb_dsp_reply)) {
    
        unsigned tail = (cpu->sb_dsp_reply_head + cpu->sb_dsp_reply_count) % sizeof (cpu->sb_dsp_reply);
        
        cpu->sb_dsp_reply[tail] = value;
        cpu->sb_dsp_reply_count++;
    
    }

}

static uint64_t opl_clock_advance (CPU8086 *cpu) {

    uint64_t now = pit_clock_ns ();
    
    if (cpu->opl_clock_ns && now < cpu->opl_clock_ns + 1250) {
        now = cpu->opl_clock_ns + 1250;
    }
    
    cpu->opl_clock_ns = now;
    return now;

}

static uint8_t opl_status (CPU8086 *cpu) {

    uint64_t now = opl_clock_advance (cpu);
    
    if ((cpu->opl_control & 1) && now - cpu->opl_timer1_started_ns >= (uint64_t) (256 - cpu->opl_timer1) * 80000ULL) {
        cpu->opl_status |= 0x40;
    }
    
    if ((cpu->opl_control & 2) && now - cpu->opl_timer2_started_ns >= (uint64_t) (256 - cpu->opl_timer2) * 320000ULL) {
        cpu->opl_status |= 0x20;
    }
    
    if (((cpu->opl_status & 0x40) && !(cpu->opl_control & 0x40)) || ((cpu->opl_status & 0x20) && !(cpu->opl_control & 0x20))) {
        cpu->opl_status |= 0x80;
    }
    
    return cpu->opl_status;

}

static void opl_write (CPU8086 *cpu, uint8_t value) {

    uint8_t old_control = cpu->opl_control;
    
    switch (cpu->opl_index) {
    
        case 0x02:
        
            cpu->opl_timer1 = value;
            break;
        
        case 0x03:
        
            cpu->opl_timer2 = value;
            break;
        
        case 0x04:
        
            if (value & 0x80) {
                cpu->opl_status = 0;
                break;
            }
            
            cpu->opl_control = value;
            
            if ((value & 1) && !(old_control & 1)) {
                cpu->opl_timer1_started_ns = cpu->opl_clock_ns;
            }
            
            if ((value & 2) && !(old_control & 2)) {
                cpu->opl_timer2_started_ns = cpu->opl_clock_ns;
            }
            
            break;
        
        default:
        
            /* Other OPL registers control audio synthesis, not its timers. */
            break;
    
    }

}

static int io_absent_sound_port (uint16_t port) {

    if ((port >= 0x210 && port <= 0x293) || (port >= 0x300 && port <= 0x301) ||
            (port >= 0x320 && port <= 0x34F) || (port >= 0x384 && port <= 0x38F)) {
        return 1;
    }
    
    if ((port >= 0x620 && port <= 0x623) || (port >= 0x640 && port <= 0x643) ||
            (port >= 0x660 && port <= 0x663) || (port >= 0x680 && port <= 0x683) ||
            (port >= 0xA20 && port <= 0xA23) || (port >= 0xA40 && port <= 0xA43) ||
            (port >= 0xA60 && port <= 0xA63) || (port >= 0xA80 && port <= 0xA83) ||
            (port >= 0xE20 && port <= 0xE23) || (port >= 0xE40 && port <= 0xE43) ||
            (port >= 0xE60 && port <= 0xE63) || (port >= 0xE80 && port <= 0xE83)) {
        return 1;
    }
    
    if ((port >= 0x788 && port <= 0x78B) || (port >= 0xB88 && port <= 0xB8B) ||
            (port >= 0xF88 && port <= 0xF8B) || (port >= 0x1388 && port <= 0x138B) ||
            (port >= 0x1788 && port <= 0x178B) || (port >= 0x1B88 && port <= 0x1B8B)) {
        return 1;
    }
    
    return 0;

}

static int dma_page_channel (uint16_t port) {

    switch (port) {
    
        case 0x81:
        
            return 2;
        
        case 0x82:
        
            return 3;
        
        case 0x83:
        
            return 1;
        
        case 0x87:
        
            return 0;
        
        case 0x89:
        
            return 6;
        
        case 0x8A:
        
            return 7;
        
        case 0x8B:
        
            return 5;
        
        case 0x8F:
        
            return 4;
    
    }
    
    return -1;

}

static int dma_data_access (CPU8086 *cpu, uint16_t port, uint8_t *value, int write) {

    unsigned controller, channel, count;
    
    DMA8237 *dma;
    uint16_t reg;
    
    if (port <= 0x07) {
    
        controller = 0;
        channel = port >> 1;
        count = port & 1;
    
    } else if (port >= 0xC0 && port <= 0xCE && !(port & 1)) {
    
        controller = 1;
        channel = (port - 0xC0) >> 2;
        count = (port & 2) != 0;
    
    } else {
        return 0;
    }
    
    dma = &cpu->dma[controller];
    reg = count ? dma->count[channel] : dma->address[channel];
    
    if (write) {
    
        if (dma->flipflop) {
            reg = (uint16_t) ((reg & 0x00FF) | ((uint16_t) *value << 8));
        } else {
            reg = (uint16_t) ((reg & 0xFF00) | *value);
        }
        
        if (count) {
        
            dma->count[channel] = reg;
            dma->base_count[channel] = reg;
        
        } else {
        
            dma->address[channel] = reg;
            dma->base_address[channel] = reg;
        
        }
    
    } else {
        *value = dma->flipflop ? (uint8_t) (reg >> 8) : (uint8_t) reg;
    }
    
    dma->flipflop ^= 1;
    return 1;

}

static int dma_control_access (CPU8086 *cpu, uint16_t port, uint8_t *value, int write) {

    unsigned controller, reg;
    DMA8237 *dma;
    
    if (port >= 0x08 && port <= 0x0F) {
    
        controller = 0;
        reg = port - 0x08;
    
    } else if (port >= 0xD0 && port <= 0xDE && !(port & 1)) {
    
        controller = 1;
        reg = (port - 0xD0) >> 1;
    
    } else {
        return 0;
    }
    
    dma = &cpu->dma[controller];
    
    if (!write) {
    
        if (reg == 0) {
            *value = (uint8_t) (dma->request << 4);
        } else if (reg == 7) {
            *value = dma->mask;
        } else {
            *value = 0xFF;
        }
        
        return 1;
    
    }
    
    switch (reg) {
    
        case 0:
        
            dma->command = *value;
            break;
        
        case 1:
        
            if (*value & 4) {
                dma->request |= (uint8_t) (1U << (*value & 3));
            } else {
                dma->request &= (uint8_t) ~(1U << (*value & 3));
            }
            
            break;
        
        case 2:
        
            if (*value & 4) {
                dma->mask |= (uint8_t) (1U << (*value & 3));
            } else {
                dma->mask &= (uint8_t) ~(1U << (*value & 3));
            }
            
            break;
        
        case 3:
        
            dma->mode[*value & 3] = *value;
            break;
        
        case 4:
        
            dma->flipflop = 0;
            break;
        
        case 5:
        
            dma->command = 0;
            dma->request = 0;
            
            dma->mask = 0x0F;
            dma->flipflop = 0;
            
            break;
        
        case 6:
        
            dma->mask = 0;
            break;
        
        case 7:
        
            dma->mask = *value & 0x0F;
            break;
    
    }
    
    return 1;

}

static uint8_t io_read8 (CPU8086 *cpu, uint16_t port) {

    uint8_t value;
    int channel;
    
    if (dma_data_access (cpu, port, &value, 0) || dma_control_access (cpu, port, &value, 0)) {
        return value;
    }
    
    if (port >= 0x80 && port <= 0x8F) {
    
        channel = dma_page_channel (port);
        return channel < 0 ? 0xFF : cpu->dma[channel >> 2].page[channel & 3];
    
    }
    
    if (port >= 0xC0 && port <= 0xDF) {
        return 0xFF;                    /* Unused 8237 ports */
    }
    
    switch (port) {
        
        case 0x20:     case 0xA0:       /* 8259 command/status */
        
            return port == 0x20 ? (cpu->pic[0].read_isr ? cpu->pic[0].isr : cpu->pic[0].irr) : (cpu->pic[1].read_isr ? cpu->pic[1].isr : cpu->pic[1].irr);
        
        case 0x21:     case 0xA1:       /* 8259 interrupt mask */
        
            return cpu->pic[port == 0xA1].mask;
        
        case 0x40:     case 0x41:     case 0x42:  /* PIT counters */
        
            return pit_read (cpu, port - 0x40);
        
        case 0x60:                      /* keyboard data port */
        
            if (cpu->kbd_output_port_read_pending) {
            
                cpu->kbd_output_port_read_pending = 0;
                return cpu->kbd_output_port;
            
            }
            
            if (cpu->kbd_ack_pending) {
            
                cpu->kbd_ack_pending--;
                return 0xFA;            /* keyboard command acknowledged */
            
            }
            
            if (cpu->kbd_scancode_pending) {
            
                cpu->kbd_scancode_pending = 0;
                
                if (cpu->kbd_release_scancode) {
                    cpu->kbd_release_time = pit_clock_ns () + 25000000ULL;
                }
                
                return cpu->kbd_scancode;
            
            }
            
            return 0x00;                /* no scan code pending */
        
        case 0x61:                      /* keyboard controller / speaker control */
        
            return (uint8_t) ((cpu->port61 & (uint8_t) ~0x20) | (pit_output (cpu, 2) ? 0x20 : 0));

        case 0x64:                      /* keyboard status port */
        
            return cpu->kbd_ack_pending || cpu->kbd_output_port_read_pending || cpu->kbd_scancode_pending ? 0x01 : 0x00;
        
        case 0x71:                      /* CMOS/RTC data register */
        
            return cpu->cmos_regs[cpu->cmos_index];
        
        case 0x92:                      /* system control port A */
        
            return cpu->port92;
        
        case 0x201:                     /* game port: no joysticks connected */
        
            return 0xFF;
        
        case 0x226:                     /* Sound Blaster DSP reset (write-only) */
        
            return 0xFF;
        
        case 0x22A:                     /* Sound Blaster DSP read data */
        
            if (cpu->sb_dsp_reply_count) {
            
                uint8_t result = cpu->sb_dsp_reply[cpu->sb_dsp_reply_head];
                
                cpu->sb_dsp_reply_head = (uint8_t) ((cpu->sb_dsp_reply_head + 1) % sizeof (cpu->sb_dsp_reply));
                cpu->sb_dsp_reply_count--;
                
                return result;
            
            }
            
            return 0xFF;
        
        case 0x22C:                     /* Sound Blaster DSP write-buffer status */
        
            return 0x7F;                /* DSP command port ready */
        
        case 0x22E:                     /* Sound Blaster DSP read-buffer status / IRQ acknowledge */
        
            cpu->sb_dsp_irq_pending = 0;
            return cpu->sb_dsp_reply_count ? 0x80 : 0x00;
        
        case 0x330:                     /* MPU-401 data port */
        
            if (cpu->mpu_ack_pending) {
            
                cpu->mpu_ack_pending = 0;
                return 0xFE;
            
            }
            
            return 0xFF;
        
        case 0x331:                     /* MPU-401 status: bit 7 input empty */

            return cpu->mpu_ack_pending ? 0x00 : 0x80;
        
        case 0x388:                     /* OPL2 status / timer flags */
        
            return opl_status (cpu);
        
        case 0x389:                     /* OPL2 data port is write-only */
        
            return 0xFF;
        
        case 0x3B4:     case 0x3D4:     /* VGA CRT controller index */
        
            return port == cpu->crtc_base ? cpu->crtc_index : 0xFF;
        
        case 0x3B5:     case 0x3D5: {                       /* 6845 CRT controller data */
        
            uint8_t index = cpu->crtc_index;
            
            if (port != (uint16_t) (cpu->crtc_base + 1)) {
                return 0xFF;            /* adapter not installed */
            }
            
            if (index < 25) {
                return cpu->crtc_registers[index];
            }
            
            return 0xFF;                /* undefined register */
        
        }
        
        case 0x379:     case 0x3BD:     case 0x279:         /* Parallel port status */
        
            return 0xFF;
        
        case 0x3BA:     case 0x3DA:      /* VGA input status 1 */
        
            cpu->vga_attr_flipflop = 0;
            cpu->vga_status_flip++;
            
            return (uint8_t) (((cpu->vga_status_flip & 0x1F) >= 24 ? 0x01 : 0x00) | ((cpu->vga_status_flip & 0xE0) == 0xE0 ? 0x08 : 0x00));
        
        case 0x3C0:                     /* VGA attribute controller index */
        
            return cpu->vga_attr_index;
        
        case 0x3C1:                     /* VGA attribute controller data */
        
            return (cpu->vga_attr_index & 0x1F) < 21 ? cpu->vga_attr_registers[cpu->vga_attr_index & 0x1F] : 0xFF;
        
        case 0x3C2:                     /* VGA input status 0 */
        
            return 0;
        
        case 0x3C3:                     /* VGA video subsystem enable */
        
            return cpu->vga_enable;
        
        case 0x3C4:                     /* VGA sequencer index */
        
            return cpu->vga_seq_index;
        
        case 0x3C5:                     /* VGA sequencer data */
        
            return cpu->vga_seq_index < 5 ? cpu->vga_seq_registers[cpu->vga_seq_index] : 0xFF;
        
        case 0x3C6:                     /* VGA DAC pixel mask */
        
            return cpu->vga_dac_mask;
        
        case 0x3C7:                     /* VGA DAC state */
        
            return cpu->vga_dac_state;
        
        case 0x3C8:                     /* VGA DAC write index */
        
            return cpu->vga_dac_write_index;
        
        case 0x3C9: {                   /* VGA DAC palette data */
        
            uint8_t value = cpu->vga_dac_palette[cpu->vga_dac_read_index][cpu->vga_dac_read_component];
            
            if (++cpu->vga_dac_read_component == 3) {
            
                cpu->vga_dac_read_component = 0;
                cpu->vga_dac_read_index++;
            
            }
            
            return value;
        
        }
        
        case 0x3CC:                     /* VGA miscellaneous output */
        
            return cpu->vga_misc_output;
        
        case 0x3CE:                     /* VGA graphics controller index */
        
            return cpu->vga_gc_index;
        
        case 0x3CF:                     /* VGA graphics controller data */
        
            return cpu->vga_gc_index < 9 ? cpu->vga_gc_registers[cpu->vga_gc_index] : 0xFF;
         
        default:
        
            if (io_absent_sound_port (port)) {
                return 0xFF;
            }
            
            fprintf (stderr, "unhandled IO read8 port %04X at %04X:%04X\n", port, cpu->cs, cpu->ip);
            exit (1);
    
    }

}

static uint16_t io_read16 (CPU8086 *cpu, uint16_t port) {

    uint8_t lo = io_read8 (cpu, port);
    uint8_t hi = io_read8 (cpu, (uint16_t) (port + 1));
    
    return (uint16_t) (lo | ((uint16_t) hi << 8));

}

static void io_write8 (CPU8086 *cpu, uint16_t port, uint8_t value) {

    int channel;
    
    if (dma_data_access (cpu, port, &value, 1) || dma_control_access (cpu, port, &value, 1)) {
        return;
    }
    
    if (port >= 0x80 && port <= 0x8F) {
    
        channel = dma_page_channel (port);
        
        if (channel >= 0) {
            cpu->dma[channel >> 2].page[channel & 3] = value;
        }
        
        return;
    
    }
    
    if (port >= 0xC0 && port <= 0xDF) {
        return;                         /* Unused 8237 ports */
    }
    
    switch (port) {
    
        case 0x20:     case 0xA0:       /* 8259 control words */
        
            pic_write_command (&cpu->pic[port == 0xA0], value);
            break;
        
        case 0x21:     case 0xA1:       /* 8259 mask/initialisation */
        
            pic_write_data (&cpu->pic[port == 0xA1], value);
            break;
        
        case 0x40:     case 0x41:     case 0x42:  /* PIT counters */
        
            pit_write (cpu, port - 0x40, value);
            break;
        
        case 0x43:                      /* PIT control word */
        
            pit_control (cpu, value);
            break;
        
        case 0x60:                      /* AT keyboard data port */
        
            if (cpu->kbd_output_port_pending) {
            
                cpu->kbd_output_port_pending = 0;
                
                if (!(value & 1)) {
                    fprintf (stderr, "8042 keyboard controller reset is not implemented\n");
                    exit (1);
                }
                
                cpu->kbd_output_port = value;
                cpu->a20_enabled = ((cpu->kbd_output_port | cpu->port92) & 2) != 0;
                
                break;
            
            }
            
            if (cpu->kbd_typematic_pending) {
            
                cpu->kbd_typematic_rate = value & 0x7F;
                cpu->kbd_typematic_pending = 0;
            
            } else if (value == 0xF3) { /* set typematic rate/delay */
            
                cpu->kbd_typematic_pending = 1;
            
            } else {
            
                fprintf (stderr, "unhandled keyboard command %02X at %04X:%04X\n", value, cpu->cs, cpu->ip);
                exit (1);
            
            }
            
            if (cpu->kbd_ack_pending < 2) {
                cpu->kbd_ack_pending++;
            }
            
            break;
        
        case 0x61: {
        
            int restart = !(cpu->port61 & 1) && (value & 1);
            
            if (restart) {
                cpu->pit[2].started_ns = pit_clock_ns ();
            }
            
            cpu->port61 = value & (uint8_t) ~0x20;
            
            speaker_update (cpu->port61, cpu->pit[2].mode, cpu->pit[2].reload, restart);
            break;
        
        }
        
        case 0x64:                      /* 8042 controller command port */
        
            if (value == 0xD0) {
                cpu->kbd_output_port_read_pending = 1;
            } else if (value == 0xD1) {
                cpu->kbd_output_port_pending = 1;
            } else if (value >= 0xF0 && (value & 1)) {
                /* Pulse output bits 0-3; no reset pulse requested. */
            } else {
            
                fprintf (stderr, "unhandled keyboard controller command %02X at %04X:%04X\n", value, cpu->cs, cpu->ip);
                exit (1);
            
            }
            
            break;
        
        case 0x70:                      /* CMOS/RTC register select */
        
            cpu->cmos_index = value & 0x7F;
            break;
        
        case 0x71:                      /* CMOS/RTC data register */
        
            cpu->cmos_regs[cpu->cmos_index] = value;
            break;
        
        case 0x92:                      /* system control port A */
        
            if (value & 1) {
            
                fprintf (stderr, "80386 fast reset through port 92h is not implemented\n");
                exit (1);
            
            }
            
            cpu->port92 = value;
            
            cpu->a20_enabled = ((cpu->kbd_output_port | value) & 2) != 0;
            break;
        
        case 0x201:                     /* game port: reset joystick timing */
        
            /* No joysticks are connected, so there are no timers to reset. */
            break;
        
        case 0x226:                     /* Sound Blaster DSP reset */
        
            if (value & 1) {
            
                cpu->sb_dsp_reset = 1;
                cpu->sb_dsp_reply_head = 0;
                cpu->sb_dsp_reply_count = 0;
                cpu->sb_dsp_speaker = 0;
                cpu->sb_dsp_dma_active = 0;
                cpu->sb_dsp_dma_auto = 0;
                cpu->sb_dsp_dma_paused = 0;
                cpu->sb_dsp_irq_pending = 0;
                cpu->sb_dsp_param_count = 0;
            
            } else if (cpu->sb_dsp_reset) {
            
                cpu->sb_dsp_reset = 0;
                sb_dsp_reply (cpu, 0xAA);
            
            }
            
            break;
        
        case 0x22C:                     /* Sound Blaster DSP command */
        
            if (cpu->sb_dsp_param_count) {
            
                cpu->sb_dsp_params[cpu->sb_dsp_param_index++] = value;
                
                if (--cpu->sb_dsp_param_count == 0) {
                
                    if (cpu->sb_dsp_command == 0x40) {
                        cpu->sb_dsp_time_constant = cpu->sb_dsp_params[0];
                    } else if (cpu->sb_dsp_command == 0x48) {
                        cpu->sb_dsp_block_length = (uint32_t) (cpu->sb_dsp_params[0] | ((uint16_t) cpu->sb_dsp_params[1] << 8)) + 1;
                    } else if (cpu->sb_dsp_command == 0x14) {
                    
                        cpu->sb_dsp_block_length = (uint32_t) (cpu->sb_dsp_params[0] | ((uint16_t) cpu->sb_dsp_params[1] << 8)) + 1;
                        cpu->sb_dsp_dma_active = 1;
                        cpu->sb_dsp_dma_auto = 0;
                        cpu->sb_dsp_dma_paused = 0;
                        cpu->sb_dsp_next_block_ns = pit_clock_ns () + (uint64_t) cpu->sb_dsp_block_length * (256 - cpu->sb_dsp_time_constant) * 1000ULL;
                    
                    }
                
                }
                
                break;
            
            }
            
            switch (value) {
            
                case 0x40:              /* Set DSP time constant */
                case 0x48:              /* Set DMA transfer block length */
                case 0x14:              /* Single-cycle 8-bit DMA output */
                
                    cpu->sb_dsp_command = value;
                    cpu->sb_dsp_param_index = 0;
                    cpu->sb_dsp_param_count = value == 0x40 ? 1 : 2;
                    
                    break;
                
                case 0x1C:              /* Auto-initialize 8-bit DMA output */
                
                    cpu->sb_dsp_dma_active = 1;
                    cpu->sb_dsp_dma_auto = 1;
                    cpu->sb_dsp_dma_paused = 0;
                    cpu->sb_dsp_next_block_ns = pit_clock_ns () + (uint64_t) cpu->sb_dsp_block_length * (256 - cpu->sb_dsp_time_constant) * 1000ULL;
                    
                    break;
                
                case 0xD0:              /* Halt 8-bit DMA */
                
                    cpu->sb_dsp_dma_paused = 1;
                    break;
                
                case 0xD1:              /* Enable DSP speaker */
                
                    cpu->sb_dsp_speaker = 1;
                    break;
                
                case 0xD3:              /* Disable DSP speaker */
                
                    cpu->sb_dsp_speaker = 0;
                    break;
                
                case 0xD4:              /* Continue 8-bit DMA */
                
                    cpu->sb_dsp_dma_paused = 0;
                    cpu->sb_dsp_next_block_ns = pit_clock_ns () + (uint64_t) cpu->sb_dsp_block_length * (256 - cpu->sb_dsp_time_constant) * 1000ULL;
                    
                    break;
                
                case 0xD8:              /* Read DSP speaker status */
                
                    sb_dsp_reply (cpu, cpu->sb_dsp_speaker ? 0xFF : 0x00);
                    break;
                
                case 0xDA:              /* Exit auto-initialize after current block */
                
                    cpu->sb_dsp_dma_auto = 0;
                    break;
                
                case 0xE1:              /* Get DSP version: Sound Blaster 2.01 */
                
                    sb_dsp_reply (cpu, 2);
                    sb_dsp_reply (cpu, 1);
                    
                    break;
                
                default:
                
                    /* Unsupported DSP commands have no audio effect. */
                    break;
            
            }
            
            break;
        
        case 0x330:                     /* MPU-401 MIDI data in UART mode */
        
            /* MIDI output requires a synthesizer, not just a UART. */
            break;
        
        case 0x331:                     /* MPU-401 command port */
        
            if (value == 0xFF) {       /* Reset */
            
                cpu->mpu_uart = 0;
                cpu->mpu_ack_pending = 1;
            
            } else if (value == 0x3F) { /* Enter UART mode */
            
                cpu->mpu_uart = 1;
                cpu->mpu_ack_pending = 1;
            
            } else {
                /* No MPU-401 intelligent-mode emulation is installed. */
            }
            
            break;
        
        case 0x388:                     /* OPL2 register index */
        
            opl_clock_advance (cpu);
            
            cpu->opl_index = value;
            break;
        
        case 0x389:                     /* OPL2 register data */
        
            opl_clock_advance (cpu);
            
            opl_write (cpu, value);
            break;
        
        case 0x3B4:     case 0x3D4:                         /* 6845 register select */
        
            if (port == cpu->crtc_base) {
                cpu->crtc_index = value & 0x1F;
            }
            
            break;
        
        case 0x3B5:     case 0x3D5:                         /* 6845 register data */
        
            if (port == (uint16_t) (cpu->crtc_base + 1) && cpu->crtc_index < 25) {
            
                if (cpu->crtc_index == 0x0E) {
                    value &= 0x3F;
                }
                
                cpu->crtc_registers[cpu->crtc_index] = value;
            
            }
            
            break;
        
        case 0x3C0:                     /* VGA attribute controller */
        
            if (cpu->vga_attr_flipflop) {
            
                if ((cpu->vga_attr_index & 0x1F) < 21) {
                    cpu->vga_attr_registers[cpu->vga_attr_index & 0x1F] = value;
                }
            
            } else {
                cpu->vga_attr_index = value;
            }
            
            cpu->vga_attr_flipflop ^= 1;
            break;
        
        case 0x3C2:                     /* VGA miscellaneous output */
        
            cpu->vga_misc_output = value;
            break;
        
        case 0x3C3:                     /* VGA video subsystem enable */
        
            cpu->vga_enable = value & 1;
            break;
        
        case 0x3C4:                     /* VGA sequencer index */
        
            cpu->vga_seq_index = value & 7;
            break;
        
        case 0x3C5:                     /* VGA sequencer data */
        
            if (cpu->vga_seq_index < 5) {
                cpu->vga_seq_registers[cpu->vga_seq_index] = value;
            }
            
            break;
        
        case 0x3C6:                     /* VGA DAC pixel mask */
        
            cpu->vga_dac_mask = value;
            mem_video_dirty |= 4;
            
            break;
        
        case 0x3C7:                     /* VGA DAC read index */
        
            cpu->vga_dac_read_index = value;
            cpu->vga_dac_read_component = 0;
            cpu->vga_dac_state = 0;
            
            break;
        
        case 0x3C8:                     /* VGA DAC write index */
        
            cpu->vga_dac_write_index = value;
            cpu->vga_dac_write_component = 0;
            cpu->vga_dac_state = 3;
            
            break;
        
        case 0x3C9:                     /* VGA DAC palette data */
        
            cpu->vga_dac_palette[cpu->vga_dac_write_index][cpu->vga_dac_write_component] = value & 0x3F;
            mem_video_dirty |= 4;
            
            if (++cpu->vga_dac_write_component == 3) {
            
                cpu->vga_dac_write_component = 0;
                cpu->vga_dac_write_index++;
            
            }
            
            break;
        
        case 0x3CE:                     /* VGA graphics controller index */
        
            cpu->vga_gc_index = value & 0x0F;
            break;
        
        case 0x3CF:                     /* VGA graphics controller data */
        
            if (cpu->vga_gc_index < 9) {
                cpu->vga_gc_registers[cpu->vga_gc_index] = value;
            }
            
            break;
        
        case 0x37A:     case 0x3BE:     case 0x27A:         /* Parallel port controls */
        
            break;
        
        case 0x3D8:     case 0x3D9:
        
            /* CGA mode control / colour select - not yet modelled */
            break;
        
        default:
        
            if (io_absent_sound_port (port)) {
                break;
            }
            
            fprintf (stderr, "unhandled IO write8 port %04X value %02X at %04X:%04X\n", port, value, cpu->cs, cpu->ip);
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
    
    uint32_t addr;
    uint16_t default_seg;
    
    uint16_t seg;
    unsigned seg_index;

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
    out.seg_index = 3;
    
    if (mod == 3) {
    
        out.seg = 0;
        return out;
    
    }
    
    if (cpu->has_addr_override) {
    
        uint32_t base = 0;
        int uses_bp = 0;
        
        if (rm == 4) {                  /* SIB addressing */
        
            uint8_t sib = fetch8 (cpu);
            
            unsigned scale = sib >> 6;
            unsigned index = (sib >> 3) & 7;
            unsigned base_reg = sib & 7;
            
            if (index != 4) {
                base = get_reg32 (cpu, index) << scale;
            }
            
            if (mod == 0 && base_reg == 5) {
                base += fetch32 (cpu);
            } else {
            
                base += get_reg32 (cpu, base_reg);
                uses_bp = base_reg == REG_BP || base_reg == REG_SP;
            
            }
        
        } else if (mod == 0 && rm == 5) {
            base = fetch32 (cpu);
        } else {
        
            base = get_reg32 (cpu, rm);
            uses_bp = rm == REG_BP || rm == REG_SP;
        
        }
        
        if (mod == 1) {
            base += (int8_t) fetch8 (cpu);
        } else if (mod == 2) {
            base += fetch32 (cpu);
        }
        
        out.addr = base;
        
        out.default_seg = uses_bp ? cpu->ss : cpu->ds;
        out.seg = cpu->has_seg_override ? cpu->seg_override : out.default_seg;
        out.seg_index = cpu->has_seg_override ? cpu->seg_override_index : uses_bp ? 2 : 3;
        
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
        out.seg_index = cpu->has_seg_override ? cpu->seg_override_index : uses_bp ? 2 : 3;
    
    }
    
    return out;

}

static uint8_t operand_read8 (CPU8086 *cpu, const OperandRef *op) {

    if (op->is_mem) {
        return mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, op->seg_index, op->addr));
    }
    
    return get_reg8 (cpu, op->reg);

}

static void operand_write8 (CPU8086 *cpu, const OperandRef *op, uint8_t value) {

    if (op->is_mem) {
        mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, op->seg_index, op->addr), value);
    } else {
        set_reg8 (cpu, op->reg, value);
    }

}

static uint16_t operand_read16 (CPU8086 *cpu, const OperandRef *op) {

    if (op->is_mem) {
    
        uint32_t off = op->addr;
        
        uint32_t phys_lo = cpu_segment_phys (cpu, op->seg_index, off);
        uint32_t phys_hi = cpu_segment_phys (cpu, op->seg_index, off + 1);
        
        uint8_t low = mem_read8 (cpu->mem, cpu->mem_size, phys_lo);
        uint8_t high = mem_read8 (cpu->mem, cpu->mem_size, phys_hi);
        
        uint16_t val = (uint16_t) low | ((uint16_t) high << 8);
        return val;
    
    }
    
    return cpu->regs[op->reg & 7];

}

static void operand_write16 (CPU8086 *cpu, const OperandRef *op, uint16_t value) {

    if (op->is_mem) {
    
        uint32_t off = op->addr;
        
        mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, op->seg_index, off), (uint8_t) (value & 0xFF));
        mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, op->seg_index, off + 1), (uint8_t) (value >> 8));
        
        return;
    
    }
    
    cpu->regs[op->reg & 7] = value;

}

static uint32_t operand_read32 (CPU8086 *cpu, const OperandRef *op) {

    uint32_t value = 0;
    unsigned i;
    
    if (!op->is_mem) {
        return get_reg32 (cpu, op->reg);
    }
    
    for (i = 0; i < 4; i++) {
        value |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, op->seg_index, op->addr + i)) << (8 * i);
    }
    
    return value;

}

static void operand_write32 (CPU8086 *cpu, const OperandRef *op, uint32_t value) {

    unsigned i;
    
    if (!op->is_mem) {
        
        set_reg32 (cpu, op->reg, value);
        return;
    
    }
    
    for (i = 0; i < 4; i++) {    
        mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, op->seg_index, op->addr + i), (uint8_t) (value >> (8 * i)));
    }

}

static void update_common_flags32 (CPU8086 *cpu, uint32_t value) {

    uint8_t p = (uint8_t) value;
    cpu->flags &= (uint16_t) ~(FLAG_ZF | FLAG_SF | FLAG_PF);
    
    if (value == 0) {
        cpu->flags |= FLAG_ZF;
    }
    
    if (value & 0x80000000UL) {
        cpu->flags |= FLAG_SF;
    }
    
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    
    if (!(p & 1)) {
        cpu->flags |= FLAG_PF;
    }

}

static uint32_t add32 (CPU8086 *cpu, uint32_t a, uint32_t b) {

    uint64_t full = (uint64_t) a + b;
    uint32_t result = (uint32_t) full;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_AF | FLAG_OF);
    
    if (full >> 32) {
        cpu->flags |= FLAG_CF;
    }
    
    if ((a ^ b ^ result) & 0x10) {
        cpu->flags |= FLAG_AF;
    }
    
    if ((~(a ^ b) & (a ^ result)) & 0x80000000UL) {
        cpu->flags |= FLAG_OF;
    }
    
    update_common_flags32 (cpu, result);
    return result;

}

static uint32_t adc32 (CPU8086 *cpu, uint32_t a, uint32_t b) {

    uint32_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;
    uint64_t full = (uint64_t) a + b + carry;
    uint32_t result = (uint32_t) full;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_AF | FLAG_OF);
    
    if (full >> 32) {
        cpu->flags |= FLAG_CF;
    }
    
    if ((a ^ b ^ result) & 0x10) {
        cpu->flags |= FLAG_AF;
    }
    
    if ((~(a ^ b) & (a ^ result)) & 0x80000000UL) {
        cpu->flags |= FLAG_OF;
    }
    
    update_common_flags32 (cpu, result);
    return result;

}

static uint32_t sub32 (CPU8086 *cpu, uint32_t a, uint32_t b) {

    uint32_t result = a - b;
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_AF | FLAG_OF);
    
    if (a < b) {
        cpu->flags |= FLAG_CF;
    }
    
    if ((a ^ b ^ result) & 0x10) {
        cpu->flags |= FLAG_AF;
    }
    
    if (((a ^ b) & (a ^ result)) & 0x80000000UL) {
        cpu->flags |= FLAG_OF;
    }
    
    update_common_flags32 (cpu, result);
    return result;

}

static uint32_t sbb32 (CPU8086 *cpu, uint32_t a, uint32_t b) {

    uint32_t borrow = (cpu->flags & FLAG_CF) ? 1 : 0;
    uint32_t result = a - b - borrow;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_AF | FLAG_OF);
    
    if ((uint64_t) a < (uint64_t) b + borrow) {
        cpu->flags |= FLAG_CF;
    }
    
    if ((a ^ b ^ result) & 0x10) {
        cpu->flags |= FLAG_AF;
    }
    
    if (((a ^ b) & (a ^ result)) & 0x80000000UL) {
        cpu->flags |= FLAG_OF;
    }
    
    update_common_flags32 (cpu, result);
    return result;

}

static uint32_t or32 (CPU8086 *cpu, uint32_t a, uint32_t b) {
    
    uint32_t result = a | b;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
    update_common_flags32 (cpu, result);
    
    return result;
    
}

static uint32_t and32 (CPU8086 *cpu, uint32_t a, uint32_t b) {

    uint32_t result = a & b;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
    update_common_flags32 (cpu, result);
    
    return result;

}

static uint32_t xor32 (CPU8086 *cpu, uint32_t a, uint32_t b) {

    uint32_t result = a ^ b;
    
    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
    update_common_flags32 (cpu, result);
    
    return result;

}

void cpu_crtc_set_cursor (CPU8086 *cpu, unsigned row, unsigned col, unsigned columns) {

    unsigned address = row * columns + col;
    
    cpu->crtc_registers[0x0E] = (uint8_t) ((address >> 8) & 0x3F);
    cpu->crtc_registers[0x0F] = (uint8_t) address;

}

void cpu_reset (CPU8086 *cpu, uint8_t *mem, size_t mem_size) {

    memset (cpu, 0, sizeof (*cpu));
    
    cpu->crtc_base = 0x3D4;
    cpu->kbd_output_port = 0x01;
    
    cpu->cmos_regs[0x0A] = 0x26;
    cpu->cmos_regs[0x0B] = 0x02;
    cpu->cmos_regs[0x0D] = 0x80;
    cpu->cmos_regs[0x15] = 0x80;
    cpu->cmos_regs[0x16] = 0x02;
    
    cpu->cmos_regs[0x18] = (uint8_t) (((mem_size - 0x100000) / 1024) >> 8);
    cpu->cmos_regs[0x17] = (uint8_t) (((mem_size - 0x100000) / 1024) & 0xff);
    
    cpu->cmos_regs[0x30] = cpu->cmos_regs[0x17];
    cpu->cmos_regs[0x31] = cpu->cmos_regs[0x18];
    
    cpu->cr0 = 0x00000010UL;            /* 80386 ET bit is set after reset */
    
    cpu->gdtr_limit = 0xFFFF;
    cpu->idtr_limit = 0x03FF;
    
    cpu->mem_size = mem_size;
    cpu->mem = mem;
    mem_vga_attach (cpu);
    
    cpu->pic[0].mask = 0xF8;
    cpu->pic[0].offset = 0x08;
    
    cpu->pic[1].mask = 0xFF;
    cpu->pic[1].offset = 0x70;
    
    cpu->dma[0].mask = 0x0F;
    cpu->dma[1].mask = 0x0F;
    
    cpu->pit[0].access = 3;
    cpu->pit[0].mode = 3;
    cpu->pit[0].control = 0x36;
    cpu->pit[0].started_ns = pit_clock_ns ();
    
    cpu->pit[1].access = 3;
    cpu->pit[1].mode = 2;
    cpu->pit[1].control = 0x74;
    cpu->pit[1].started_ns = cpu->pit[0].started_ns;
    
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

int cpu_step (CPU8086 *cpu) {

    int trap_inhibit = 0;
    int trap_pending;
    
    uint8_t op;
    int old_ip;
    
    if (cpu == NULL || cpu->halted) {
        return 0;
    }
    
    if (++cpu->irq_poll_count == 0) {
    
        int irq = cpu_service_timer_irq (cpu);
        
        if (irq) {
            return irq;
        }
    
    }
    
    trap_pending = (cpu->flags & FLAG_TF) != 0;
    
    cpu->seg_override = 0;
    cpu->has_seg_override = 0;
    
    cpu->has_addr_override = cpu_code_32 (cpu);
    cpu->has_operand_override = cpu_code_32 (cpu);
    
    cpu->rep_type = 0;                  /* 0=None, 1=REPE, 2=REPNZ */

fetch_instruction:
    
    if ((cpu->cr0 & 1) && cpu->segments[1].protected && cpu->ip > cpu->segments[1].limit) {
    
        fprintf (stderr, "protected-mode instruction pointer %08X exceeds CS limit %08X\n", cpu->ip, cpu->segments[1].limit);
        return -1;
    
    }
    
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
            
            if (cpu->has_operand_override && (op & 1)) {
            
                if (op == 0x01) {
                    operand_write32 (cpu, &rm, add32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg)));
                } else if (op == 0x03) {
                    set_reg32 (cpu, reg, add32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm)));
                } else if (op == 0x09) {
                    operand_write32 (cpu, &rm, or32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg)));
                } else if (op == 0x0B) {
                   set_reg32 (cpu, reg, or32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm)));
                } else if (op == 0x21) {
                    operand_write32 (cpu, &rm, and32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg)));
                } else if (op == 0x23) {
                    set_reg32 (cpu, reg, and32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm)));
                } else if (op == 0x29) {
                    operand_write32 (cpu, &rm, sub32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg)));
                } else if (op == 0x2B) {
                    set_reg32 (cpu, reg, sub32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm)));
                } else if (op == 0x31) {
                    operand_write32 (cpu, &rm, xor32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg)));
                } else if (op == 0x33) {
                    set_reg32 (cpu, reg, xor32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm)));
                } else if (op == 0x39) {
                    sub32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg));
                } else if (op == 0x3B) {
                    sub32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm));
                } else {
                
                    fprintf (stderr, "unsupported 80386 32-bit arithmetic opcode %02X at %04X:%04X\n", op, cpu->cs, old_ip);
                    return -1;
                
                }
                
                break;
            
            }
            
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
        
        case 0x0C: {                    /* OR AL, imm8 */
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, REG_AX);
            
            set_reg8 (cpu, REG_AX, or8 (cpu, al, imm8));
            break;
        
        }
        
        case 0x0D: {                    /* OR AX/EAX, imm16/imm32 */
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, or32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu)));
            } else {
                cpu->regs[REG_AX] = or16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            }
            
            break;
        
        }
        
        case 0x10:  case 0x11:  case 0x12:  case 0x13: {    /* ADC */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0x10) {
                operand_write8 (cpu, &rm, adc8 (cpu, operand_read8 (cpu, &rm), get_reg8 (cpu, reg)));
            } else if (op == 0x11) {
            
                if (cpu->has_operand_override) {
                    operand_write32 (cpu, &rm, adc32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg)));
                } else {
                    operand_write16 (cpu, &rm, adc16 (cpu, operand_read16 (cpu, &rm), cpu->regs[reg]));
                }
            
            } else if (op == 0x12) {
                set_reg8 (cpu, reg, adc8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            } else if (op == 0x13) {
            
                if (cpu->has_operand_override) {
                    set_reg32 (cpu, reg, adc32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm)));
                } else {
                    cpu->regs[reg] = adc16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
                }
            
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
        
        case 0x19: {                    /* SBB r/m16, r16 */
        
            uint8_t modrm = fetch8 (cpu);
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (cpu->has_operand_override) {
            
                operand_write32 (cpu, &rm, sbb32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, (modrm >> 3) & 7)));
                break;
            
            }
            
            {
            
                /* The 'reg' field of ModRM (bits 5-3) selects the source register */
                uint16_t src = cpu->regs[(modrm >> 3) & 7];
                uint16_t dst = operand_read16 (cpu, &rm);
                
                /* Perform subtraction with borrow and update flags */
                /* (Assuming you have a helper like sbb16 for flag logic) */
                uint16_t result = sbb16 (cpu, dst, src);
                
                operand_write16 (cpu, &rm, result);
            
            }
            
            break;
        
        }
        
        case 0x1A: {                    /* SBB r8, r/m8 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            set_reg8 (cpu, reg, sbb8 (cpu, get_reg8 (cpu, reg), operand_read8 (cpu, &rm)));
            
            break;
        
        }
        
        case 0x1B: {                    /* SBB r16, r/m16 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (cpu->has_operand_override) {
                set_reg32 (cpu, reg, sbb32 (cpu, get_reg32 (cpu, reg), operand_read32 (cpu, &rm)));
            } else {
                cpu->regs[reg] = sbb16 (cpu, cpu->regs[reg], operand_read16 (cpu, &rm));
            }
            
            break;
        
        }
        
        case 0x1C: {                    /* SBB AL, imm8 */
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, sbb32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu)));
            } else {
            
                uint16_t imm16 = fetch16 (cpu);
                cpu->regs[REG_AX] = sbb16 (cpu, cpu->regs[REG_AX], imm16);
            
            }
            
            break;
        
        }
        
        case 0x1D: {                    /* SBB AX, imm16 */
        
            uint16_t imm16 = fetch16 (cpu);
            
            cpu->regs[REG_AX] = sbb16 (cpu, cpu->regs[REG_AX], imm16);
            break;
        
        }
        
        /* Additional Arithmetic and Logic Operations */
        case 0x04:
        
            set_reg8 (cpu, 0, add8 (cpu, get_reg8 (cpu, 0), fetch8 (cpu)));
            break;
        
        case 0x05:
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, add32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu)));
            } else {
                cpu->regs[REG_AX] = add16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            }
            
            break;
        
        /* --- Segment Registers --- */
        case 0x06:                      /* PUSH ES */
        
            if (cpu->has_operand_override) {
                push32 (cpu, cpu->es);
            } else {
                push16 (cpu, cpu->es);
            
            }
            break;
        
        case 0x07: {                    /* POP ES */
        
            if (cpu_load_segment (cpu, 0, (uint16_t) (cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu))) != 0) {
                return -1;
            }
            
            break;
        
        }
        
        case 0x0E:                      /* PUSH CS */
        
            if (cpu->has_operand_override) {
                push32 (cpu, cpu->cs);
            } else {
                push16 (cpu, cpu->cs);
            }
            
            break;
        
        case 0x0F: {
        
            uint8_t op2 = fetch8 (cpu);
            
            switch (op2) {
            
                case 0x00: {            /* 80386 segment selector and task register operations */
                
                    uint8_t modrm = fetch8 (cpu);
                    unsigned subop = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    
                    uint16_t selector;
                    uint32_t addr, base, limit;
                    
                    uint8_t access, flags;
                    
                    if (subop > 3) {
                    
                        fprintf (stderr, "unsupported 80386 0F 00 /%u at %04X:%04X\n", subop, cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    if (!(cpu->cr0 & 1)) {
                    
                        fprintf (stderr, "80386 0F 00 requires protected mode at %04X:%04X\n", cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    if (subop == 0 || subop == 1) {      /* SLDT/STR */
                    
                        uint16_t value = subop == 0 ? cpu->ldtr_selector : cpu->tr_selector;
                        
                        if (cpu->has_operand_override && !rm.is_mem) {
                            operand_write32 (cpu, &rm, value);
                        } else {
                            operand_write16 (cpu, &rm, value);
                        }
                        
                        break;
                    
                    }
                    
                    selector = operand_read16 (cpu, &rm);
                    
                    if (subop == 2 && !(selector & 0xFFFC)) {     /* LLDT with null selector */
                    
                        cpu->ldtr_selector = selector;
                        cpu->ldtr_base = 0;
                        cpu->ldtr_limit = 0;
                        cpu->ldtr_valid = 0;
                        
                        break;
                    
                    }
                    
                    if (!(selector & 0xFFF8) || (selector & 4) || (uint32_t) (selector & 0xFFF8) + 7 > cpu->gdtr_limit) {
                    
                        fprintf (stderr, "invalid system descriptor selector %04X at %04X:%04X\n", selector, cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    addr = cpu->gdtr_base + (selector & 0xFFF8);
                    
                    if ((uint64_t) addr + 7 >= cpu->mem_size) {
                    
                        fprintf (stderr, "system descriptor outside emulated memory at %04X:%04X\n", cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    access = mem_read8 (cpu->mem, cpu->mem_size, addr + 5);
                    flags = mem_read8 (cpu->mem, cpu->mem_size, addr + 6);
                    
                    if (!(access & 0x80) || (access & 0x10) || (subop == 2 ? (access & 0x0F) != 2 : ((access & 0x0F) != 1 && (access & 0x0F) != 9))) {
                    
                        fprintf (stderr, "invalid system descriptor type %02X for selector %04X at %04X:%04X\n", access, selector, cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    limit = (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 1) << 8) | ((uint32_t) (flags & 0x0F) << 16);
                    base = (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 2) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 3) << 8) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 4) << 16) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 7) << 24);
                    
                    if (flags & 0x80) {
                        limit = (limit << 12) | 0xFFF;
                    }
                    
                    if (subop == 2) {                   /* LLDT */
                    
                        cpu->ldtr_selector = selector;
                        cpu->ldtr_base = base;
                        cpu->ldtr_limit = limit;
                        cpu->ldtr_valid = 1;
                    
                    } else {                            /* LTR */
                    
                        cpu->tr_selector = selector;
                        cpu->tr_base = base;
                        cpu->tr_limit = limit;
                        cpu->tr_valid = 1;
                        
                        mem_write8 (cpu->mem, cpu->mem_size, addr + 5, access | 2);   /* TSS busy */
                    
                    }
                    
                    break;
                
                }
                
                case 0x01: {            /* 80386 descriptor table and machine status operations */
                
                    uint8_t modrm = fetch8 (cpu);
                    unsigned subop = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    
                    if (subop == 4) {                       /* SMSW r/m16 */
                        operand_write16 (cpu, &rm, (uint16_t) cpu->cr0);
                    } else if (subop == 6) {                /* LMSW r/m16 */
                        
                        uint16_t value = operand_read16 (cpu, &rm);
                        uint32_t old_cr0 = cpu->cr0;
                        
                        cpu->cr0 = (cpu->cr0 & ~0xFUL) | (value & 0xFUL) | (cpu->cr0 & 1);
                        
                        if (!(old_cr0 & 1) && (cpu->cr0 & 1)) {
                            cpu_cache_real_segments (cpu);
                        }
                    
                    } else if (subop <= 3 && rm.is_mem) {
                    
                        OperandRef base_rm = rm;
                        base_rm.addr = cpu->has_addr_override ? rm.addr + 2 : (uint16_t) (rm.addr + 2);
                        
                        if (subop == 0 || subop == 1) {     /* SGDT/SIDT */
                        
                            uint32_t base = subop == 0 ? cpu->gdtr_base : cpu->idtr_base;
                            uint16_t limit = subop == 0 ? cpu->gdtr_limit : cpu->idtr_limit;
                            
                            operand_write16 (cpu, &rm, limit);
                            operand_write32 (cpu, &base_rm, base);
                        
                        } else {        /* LGDT/LIDT */
                        
                            uint16_t limit = operand_read16 (cpu, &rm);
                            uint32_t base = operand_read32 (cpu, &base_rm);
                            
                            if (!cpu->has_operand_override) {
                                base &= 0x00FFFFFFUL;
                            }
                            
                            if (subop == 2) {
                            
                                cpu->gdtr_base = base;
                                cpu->gdtr_limit = limit;
                            
                            } else {
                            
                                cpu->idtr_base = base;
                                cpu->idtr_limit = limit;
                            
                            }
                        
                        }
                    
                    } else {
                    
                        fprintf (stderr, "unsupported 80386 0F 01 /%u at %04X:%04X\n", subop, cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    break;
                
                }
                
                case 0x02:      case 0x03: {                /* LAR / LSL */
                
                    uint8_t modrm = fetch8 (cpu);
                    
                    int reg = (modrm >> 3) & 7;
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    
                    uint16_t selector = operand_read16 (cpu, &rm);
                    
                    uint32_t addr;
                    uint32_t value;
                    
                    uint8_t access, flags;
                    unsigned type;
                    
                    int valid;
                    
                    if (!(cpu->cr0 & 1)) {
                    
                        fprintf (stderr, "LAR/LSL requires protected mode at %04X:%04X\n", cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    valid = (selector & 0xFFF8) != 0 && !(selector & 4) && (uint32_t) (selector & 0xFFF8) + 7 <= cpu->gdtr_limit;
                    addr = cpu->gdtr_base + (selector & 0xFFF8);
                    
                    if (valid && (uint64_t) addr + 7 >= cpu->mem_size) {
                        valid = 0;
                    }
                    
                    if (valid) {
                    
                        access = mem_read8 (cpu->mem, cpu->mem_size, addr + 5);
                        flags = mem_read8 (cpu->mem, cpu->mem_size, addr + 6);
                        type = access & 0x0F;
                        
                        if (!(access & 0x10)) {
                        
                            if (op2 == 0x02) {
                                valid = type == 1 || type == 2 || type == 3 || type == 4 || type == 5 || type == 9 || type == 11 || type == 12;
                            } else {
                                valid = type == 1 || type == 2 || type == 3 || type == 9 || type == 11;
                            }
                        
                        }
                        
                        if (valid && (selector & 3) > ((access >> 5) & 3) && !((access & 0x1C) == 0x1C)) {
                            valid = 0;
                        }
                        
                        if (valid) {
                        
                            if (op2 == 0x02) {
                                value = ((uint32_t) access << 8) | ((uint32_t) (flags & 0xF0) << 16);
                            } else {
                            
                                value = (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr) | ((uint32_t) mem_read8 (cpu->mem, cpu->mem_size, addr + 1) << 8) | ((uint32_t) (flags & 0x0F) << 16);
                                
                                if (flags & 0x80) {
                                    value = (value << 12) | 0xFFF;
                                }
                            
                            }
                            
                            if (cpu->has_operand_override) {
                                set_reg32 (cpu, reg, value);
                            } else {
                                cpu->regs[reg] = (uint16_t) value;
                            }
                        
                        }
                    
                    }
                    
                    if (valid) {
                        cpu->flags |= FLAG_ZF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_ZF;
                    }
                    
                    break;
                
                }
                
                case 0x20:      case 0x22: {                /* MOV r32, CRn / MOV CRn, r32 */
                
                    uint8_t modrm = fetch8 (cpu);
                    
                    unsigned cr = (modrm >> 3) & 7;
                    unsigned reg = modrm & 7;
                    
                    uint32_t value;
                    
                    if ((modrm & 0xC0) != 0xC0 || (cr != 0 && cr != 2 && cr != 3)) {
                    
                        fprintf (stderr, "invalid 80386 control-register instruction at %04X:%04X\n", cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    if (op2 == 0x20) {
                    
                        value = cr == 0 ? cpu->cr0 : cr == 2 ? cpu->cr2 : cpu->cr3;
                        set_reg32 (cpu, reg, value);
                    
                    } else {
                    
                        value = get_reg32 (cpu, reg);
                        
                        if (cr == 0 && (value & 0x80000000UL)) {
                        
                            fprintf (stderr, "80386 paging is not implemented at %04X:%04X\n", cpu->cs, old_ip);
                            return -1;
                        
                        }
                        
                        if (cr == 0) {
                        
                            if (!(cpu->cr0 & 1) && (value & 1)) {
                                cpu_cache_real_segments (cpu);
                            }
                            
                            cpu->cr0 = value;
                        
                        } else if (cr == 2) {
                            cpu->cr2 = value;
                        } else {
                            cpu->cr3 = value;
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 0x80:  case 0x81:  case 0x82:  case 0x83:
                case 0x84:  case 0x85:  case 0x86:  case 0x87:
                case 0x88:  case 0x89:  case 0x8A:  case 0x8B:
                case 0x8C:  case 0x8D:  case 0x8E:  case 0x8F: {
                
                    int32_t displacement;
                    
                    int cf = (cpu->flags & FLAG_CF) != 0;
                    int zf = (cpu->flags & FLAG_ZF) != 0;
                    int sf = (cpu->flags & FLAG_SF) != 0;
                    int of = (cpu->flags & FLAG_OF) != 0;
                    int pf = (cpu->flags & FLAG_PF) != 0;
                    
                    int take;
                    displacement = cpu->has_operand_override ? (int32_t) fetch32 (cpu) : (int16_t) fetch16 (cpu);
                    
                    switch ((op2 & 0x0F) >> 1) {
                    
                        case 0:
                        
                            take = of;
                            break;
                        
                        case 1:
                        
                            take = cf;
                            break;
                        
                        case 2:
                        
                            take = zf;
                            break;
                        
                        case 3:
                        
                            take = cf || zf;
                            break;
                        
                        case 4:
                        
                            take = sf;
                            break;
                        
                        case 5:
                        
                            take = pf;
                            break;
                        
                        case 6:
                        
                            take = sf != of;
                            break;
                        
                        default:
                        
                            take = zf || (sf != of);
                            break;
                    
                    }
                    
                    if (op2 & 1) {
                        take = !take;
                    }
                    
                    if (take) {
                        cpu->ip = cpu->has_operand_override ? cpu->ip + displacement : (uint16_t) (cpu->ip + displacement);
                    }
                    
                    break;
                
                }
                
                case 0x90:  case 0x91:  case 0x92:  case 0x93:
                case 0x94:  case 0x95:  case 0x96:  case 0x97:
                case 0x98:  case 0x99:  case 0x9A:  case 0x9B:
                case 0x9C:  case 0x9D:  case 0x9E:  case 0x9F: {
                
                    uint8_t modrm = fetch8 (cpu);
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    
                    int cf = (cpu->flags & FLAG_CF) != 0;
                    int zf = (cpu->flags & FLAG_ZF) != 0;
                    int sf = (cpu->flags & FLAG_SF) != 0;
                    int of = (cpu->flags & FLAG_OF) != 0;
                    int pf = (cpu->flags & FLAG_PF) != 0;
                    
                    int take;
                    
                    switch ((op2 & 0x0F) >> 1) {
                    
                        case 0:
                        
                            take = of;
                            break;
                        
                        case 1:
                        
                            take = cf;
                            break;
                        
                        case 2:
                        
                            take = zf;
                            break;
                        
                        case 3:
                        
                            take = cf || zf;
                            break;
                        
                        case 4:
                        
                            take = sf;
                            break;
                        
                        case 5:
                        
                            take = pf;
                            break;
                        
                        case 6:
                        
                            take = sf != of;
                            break;
                        
                        default:
                        
                            take = zf || (sf != of);
                            break;
                    
                    }
                    
                    if (op2 & 1) {
                        take = !take;
                    }
                    
                    operand_write8 (cpu, &rm, (uint8_t) take);
                    break;
                
                }
                
                case 0xB2: {            /* LSS r16/32, m16:16/32 */
                
                    uint8_t modrm = fetch8 (cpu);
                    int reg = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    OperandRef selector_ref = rm;
                    
                    uint32_t offset;
                    uint16_t selector;
                    
                    if (!rm.is_mem) {
                    
                        fprintf (stderr, "LSS requires memory operand at %04X:%04X\n", cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    offset = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                    
                    selector_ref.addr = cpu->has_addr_override ? rm.addr + (cpu->has_operand_override ? 4 : 2) : (uint16_t) (rm.addr + (cpu->has_operand_override ? 4 : 2));
                    selector = operand_read16 (cpu, &selector_ref);
                    
                    if (cpu_load_segment (cpu, 2, selector) != 0) {
                        return -1;
                    }
                    
                    trap_inhibit = 1;
                    
                    if (cpu->has_operand_override) {
                        set_reg32 (cpu, reg, offset);
                    } else {
                        cpu->regs[reg] = (uint16_t) offset;
                    }
                    
                    break;
                
                }
                
                case 0xB6:  case 0xB7:  case 0xBE:  case 0xBF: {
                
                    uint8_t modrm = fetch8 (cpu);
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    uint32_t v;
                    
                    if (op2 == 0xB6 || op2 == 0xBE) {
                    
                        uint8_t raw = operand_read8 (cpu, &rm);
                        v = op2 == 0xBE ? (uint32_t) (int32_t) (int8_t) raw : raw;
                    
                    } else {
                    
                        uint16_t raw = operand_read16 (cpu, &rm);
                        v = op2 == 0xBF ? (uint32_t) (int32_t) (int16_t) raw : raw;
                    
                    }
                    
                    if (cpu->has_operand_override) {
                        set_reg32 (cpu, (modrm >> 3) & 7, v);
                    } else {
                        cpu->regs[(modrm >> 3) & 7] = (uint16_t) v;
                    }
                    
                    break;
                
                }
                
                case 0xA0:
                
                    if (cpu->has_operand_override) {
                        push32 (cpu, cpu->fs);
                    } else {
                        push16 (cpu, cpu->fs);
                    }
                    
                    break;
                
                case 0xA1:
                
                    if (cpu_load_segment (cpu, 4, (uint16_t) (cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu))) != 0) {
                        return -1;
                    }
                    
                    break;
                
                case 0xA4:  case 0xA5: {                    /* SHLD r/m16/32, r16/32, imm8/CL */
                
                    uint8_t modrm = fetch8 (cpu);
                    int reg = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    unsigned count = (op2 == 0xA4 ? fetch8 (cpu) : get_reg8 (cpu, REG_CX)) & 31;
                    
                    if (count != 0) {
                    
                        uint32_t old_value, value;
                        uint64_t pair;
                        
                        uint32_t mask = cpu->has_operand_override ? 0xFFFFFFFFUL : 0xFFFFUL;
                        unsigned bits = cpu->has_operand_override ? 32 : 16;
                        
                        if (count > bits) {
                        
                            fprintf (stderr, "undefined 80386 SHLD shift count at %04X:%04X\n", cpu->cs, old_ip);
                            return -1;
                        
                        }
                        
                        old_value = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                        
                        pair = ((uint64_t) old_value << bits) | (cpu->has_operand_override ? get_reg32 (cpu, reg) : cpu->regs[reg]);
                        value = (uint32_t) ((pair << count) >> bits) & mask;
                        
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                        
                        if ((old_value >> (bits - count)) & 1) {
                            cpu->flags |= FLAG_CF;
                        }
                        
                        if (count == 1) {
                        
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                            
                            if ((old_value ^ value) & (1UL << (bits - 1))) {
                                cpu->flags |= FLAG_OF;
                            }
                        
                        }
                        
                        if (cpu->has_operand_override) {
                        
                            operand_write32 (cpu, &rm, value);
                            update_common_flags32 (cpu, value);
                        
                        } else {
                        
                            operand_write16 (cpu, &rm, (uint16_t) value);
                            update_common_flags16 (cpu, (uint16_t) value);
                        
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 0xA8:
                
                    if (cpu->has_operand_override) {
                        push32 (cpu, cpu->gs);
                    } else {
                        push16 (cpu, cpu->gs);
                    }
                    
                    break;
                
                case 0xA9:
                
                    if (cpu_load_segment (cpu, 5, (uint16_t) (cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu))) != 0) {
                        return -1;
                    }
                    
                    break;
                
                case 0xAC:  case 0xAD: {                    /* SHRD r/m16/32, r16/32, imm8/CL */
                
                    uint8_t modrm = fetch8 (cpu);
                    int reg = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    unsigned count = (op2 == 0xAC ? fetch8 (cpu) : get_reg8 (cpu, REG_CX)) & 31;
                    
                    if (count != 0) {
                    
                        uint32_t old_value, value;
                        uint64_t pair;
                        
                        uint32_t mask = cpu->has_operand_override ? 0xFFFFFFFFUL : 0xFFFFUL;
                        unsigned bits = cpu->has_operand_override ? 32 : 16;
                        
                        if (count > bits) {
                        
                            fprintf (stderr, "undefined 80386 SHRD shift count at %04X:%04X\n", cpu->cs, old_ip);
                            return -1;
                        
                        }
                        
                        old_value = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                        pair = ((uint64_t) (cpu->has_operand_override ? get_reg32 (cpu, reg) : cpu->regs[reg]) << bits) | old_value;
                        value = (uint32_t) (pair >> count) & mask;
                        
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                        
                        if ((pair >> (count - 1)) & 1) {
                            cpu->flags |= FLAG_CF;
                        }
                        
                        if (count == 1) {
                        
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                            
                            if ((old_value ^ value) & (1UL << (bits - 1))) {
                                cpu->flags |= FLAG_OF;
                            }
                        
                        }
                        
                        if (cpu->has_operand_override) {
                        
                            operand_write32 (cpu, &rm, value);
                            update_common_flags32 (cpu, value);
                        
                        } else {
                        
                            operand_write16 (cpu, &rm, (uint16_t) value);
                            update_common_flags16 (cpu, (uint16_t) value);
                        
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 0xAF: {
                
                    uint8_t modrm = fetch8 (cpu);
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    
                    int reg = (modrm >> 3) & 7;
                    
                    if (cpu->has_operand_override) {
                    
                        int64_t result = (int64_t) (int32_t) get_reg32 (cpu, reg) * (int32_t) operand_read32 (cpu, &rm);
                        
                        set_reg32 (cpu, reg, (uint32_t) result);
                        
                        if (result < INT32_MIN || result > INT32_MAX) {
                            cpu->flags |= FLAG_CF | FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                        }
                        
                        break;
                    
                    }
                    
                    {
                    
                        int32_t result = (int32_t) (int16_t) cpu->regs[reg] * (int32_t) (int16_t) operand_read16 (cpu, &rm);
                        cpu->regs[reg] = (uint16_t) result;
                        
                        if (result < -32768 || result > 32767) {
                            cpu->flags |= FLAG_CF | FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 0xA3:  case 0xAB:  case 0xB3:  case 0xBB: {    /* BT/BTS/BTR/BTC r/m16/32, r16/32 */
                
                    uint8_t modrm = fetch8 (cpu);
                    unsigned reg = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    unsigned bits = cpu->has_operand_override ? 32 : 16;
                    
                    int32_t index = cpu->has_operand_override ? (int32_t) get_reg32 (cpu, reg) : (int16_t) cpu->regs[reg];
                    unsigned bit = (unsigned) index & (bits - 1);
                    
                    uint32_t value, mask;
                    
                    if (rm.is_mem) {
                    
                        int32_t word_index = index / (int32_t) bits;
                        
                        if (index % (int32_t) bits < 0) {
                            --word_index;
                        }
                        
                        rm.addr = cpu->has_addr_override ? rm.addr + (uint32_t) (word_index * (int32_t) (bits / 8)) :
                            (uint16_t) (rm.addr + (uint32_t) (word_index * (int32_t) (bits / 8)));
                    
                    }
                    
                    value = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                    mask = 1UL << bit;
                    
                    cpu->flags &= (uint16_t) ~FLAG_CF;
                    
                    if (value & mask) {
                        cpu->flags |= FLAG_CF;
                    }
                    
                    if (op2 == 0xAB) {
                        value |= mask;
                    } else if (op2 == 0xB3) {
                        value &= ~mask;
                    } else if (op2 == 0xBB) {
                        value ^= mask;
                    }
                    
                    if (op2 != 0xA3) {
                    
                        if (cpu->has_operand_override) {
                            operand_write32 (cpu, &rm, value);
                        } else {
                            operand_write16 (cpu, &rm, (uint16_t) value);
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 0xBA: {            /* BT/BTS/BTR/BTC r/m16/32, imm8 */
                
                    uint8_t modrm = fetch8 (cpu);
                    unsigned subop = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    
                    unsigned bit = fetch8 (cpu);
                    unsigned bits = cpu->has_operand_override ? 32 : 16;
                    
                    uint32_t value;
                    uint32_t mask;
                    
                    if (subop < 4) {
                    
                        fprintf (stderr, "invalid 80386 bit-test instruction at %04X:%04X\n", cpu->cs, old_ip);
                        return -1;
                    
                    }
                    
                    if (rm.is_mem) {
                    
                        rm.addr = cpu->has_addr_override ? rm.addr + (bit / bits) * (bits / 8) :
                            (uint16_t) (rm.addr + (bit / bits) * (bits / 8));
                    
                    }
                    
                    bit %= bits;
                    
                    value = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                    mask = 1UL << bit;
                    
                    cpu->flags &= (uint16_t) ~FLAG_CF;
                    
                    if (value & mask) {
                        cpu->flags |= FLAG_CF;
                    }
                    
                    if (subop == 5) {
                        value |= mask;
                    } else if (subop == 6) {
                        value &= ~mask;
                    } else if (subop == 7) {
                        value ^= mask;
                    }
                    
                    if (subop != 4) {
                    
                        if (cpu->has_operand_override) {
                            operand_write32 (cpu, &rm, value);
                        } else {
                            operand_write16 (cpu, &rm, (uint16_t) value);
                        }
                    
                    }
                    
                    break;
                
                }
                
                case 0xBC:  case 0xBD: {                    /* BSF/BSR r16/32, r/m16/32 */
                
                    uint8_t modrm = fetch8 (cpu);
                    int reg = (modrm >> 3) & 7;
                    
                    OperandRef rm = decode_rm_operand (cpu, modrm);
                    uint32_t value = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                    
                    if (value == 0) {
                        cpu->flags |= FLAG_ZF;
                    } else {
                    
                        unsigned bit = op2 == 0xBC ? 0 : cpu->has_operand_override ? 31 : 15;
                        
                        if (op2 == 0xBC) {
                        
                            while (!(value & (1UL << bit))) {
                                ++bit;
                            }
                        
                        } else {
                        
                            while (!(value & (1UL << bit))) {
                                --bit;
                            }
                        
                        }
                        
                        if (cpu->has_operand_override) {
                            set_reg32 (cpu, reg, bit);
                        } else {
                            cpu->regs[reg] = (uint16_t) bit;
                        }
                        
                        cpu->flags &= (uint16_t) ~FLAG_ZF;
                    
                    }
                    
                    break;
                
                }
                
                case 0xC8:  case 0xC9:  case 0xCA:  case 0xCB:
                case 0xCC:  case 0xCD:  case 0xCE:  case 0xCF: {                /* BSWAP r32 */
                
                    uint32_t value = get_reg32 (cpu, op2 - 0xC8);
                    
                    set_reg32 (cpu, op2 - 0xC8, (value << 24) | ((value & 0x0000FF00UL) << 8) |
                        ((value & 0x00FF0000UL) >> 8) | (value >> 24));
                    
                    break;
                
                }
                
                default:
                
                    printf ("unhandled 0F opcode 0x%02X at %04X%04X\n", op2, cpu->cs, old_ip);
                    return -1;
            
            }
            
            break;
        
        }
        
        case 0x15: {                    /* ADC AX/EAX, imm16/imm32 */
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, adc32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu)));
            } else {
                cpu->regs[REG_AX] = adc16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            }
            
            break;
        
        }
        
        case 0x16:                      /* PUSH SS */
        
            if (cpu->has_operand_override) {
                push32 (cpu, cpu->ss);
            } else {
                push16 (cpu, cpu->ss);
            }
            
            break;
        
        case 0x17:                      /* POP SS */
        
            if (cpu_load_segment (cpu, 2, (uint16_t) (cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu))) != 0) {
                return -1;
            }
            
            trap_inhibit = 1;
            break;
        
        case 0x1E:                      /* PUSH DS */
        
            if (cpu->has_operand_override) {
                push32 (cpu, cpu->ds);
            } else {
                push16 (cpu, cpu->ds);
            }
            
            break;
        
        case 0x1F:                      /* POP DS */
        
            if (cpu_load_segment (cpu, 3, (uint16_t) (cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu))) != 0) {
                return -1;
            }
            
            break;
        
        /* AND AL, imm8 */
        case 0x24: {
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, REG_AX);            /* AL is the low byte of AX */
            
            set_reg8 (cpu, REG_AX, and8 (cpu, al, imm8));
            break;
        
        }
        
        /* AND AX/EAX, imm16/imm32 */
        case 0x25: {
        
            if (cpu->has_operand_override) {
            
                uint32_t value = get_reg32 (cpu, REG_AX) & fetch32 (cpu);
                set_reg32 (cpu, REG_AX, value);
                
                cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                update_common_flags32 (cpu, value);
            
            } else {
            
                uint16_t imm16 = fetch16 (cpu);
                cpu->regs[REG_AX] = and16 (cpu, cpu->regs[REG_AX], imm16);
            
            }
            
            break;
        
        }
        
        case 0x26:
            
            cpu->seg_override = cpu->es;
            cpu->seg_override_index = 0;
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
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, sub32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu)));
            } else {
                cpu->regs[REG_AX] = sub16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            }
            
            break;
        
        case 0x2E:
            
            cpu->seg_override = cpu->cs;
            cpu->seg_override_index = 1;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x34:
        
            set_reg8 (cpu, 0, xor8 (cpu, get_reg8 (cpu, 0), fetch8 (cpu)));
            break;
        
        case 0x35:
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, xor32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu)));
            } else {
                cpu->regs[REG_AX] = xor16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            }
            
            break;
        
        case 0x36:
            
            cpu->seg_override = cpu->ss;
            cpu->seg_override_index = 2;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x37:
        
            aaa (cpu);
            break;
        
        case 0x3C: {                    /* CMP AL, imm8 */
        
            uint8_t imm8 = fetch8 (cpu);
            uint8_t al = get_reg8 (cpu, REG_AX);
            
            sub8 (cpu, al, imm8);       /* CMP performs subtraction but discards the result */
            break;
        
        }
        
        case 0x3D: {
        
            if (cpu->has_operand_override) {
                sub32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu));
            } else {
                sub16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            }
            
            break;
        
        }
        
        case 0x3E:
            
            cpu->seg_override = cpu->ds;
            cpu->seg_override_index = 3;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        /* Control Flow Operations */
        case 0x40:  case 0x41:  case 0x42:  case 0x43:
        case 0x44:  case 0x45:  case 0x46:  case 0x47: {
        
            int reg = op - 0x40;
            
            if (cpu->has_operand_override) {
            
                uint16_t cf = cpu->flags & FLAG_CF;
                
                set_reg32 (cpu, reg, add32 (cpu, get_reg32 (cpu, reg), 1));
                cpu->flags = (cpu->flags & (uint16_t) ~FLAG_CF) | cf;
            
            } else {
                cpu->regs[reg] = inc16 (cpu, cpu->regs[reg]);
            }
            
            break;
        
        }
        
        case 0x48:  case 0x49:  case 0x4A:  case 0x4B:
        case 0x4C:  case 0x4D:  case 0x4E:  case 0x4F: {
        
            int reg = op - 0x48;
            
            if (cpu->has_operand_override) {
            
                uint16_t cf = cpu->flags & FLAG_CF;
                
                set_reg32 (cpu, reg, sub32 (cpu, get_reg32 (cpu, reg), 1));
                cpu->flags = (cpu->flags & (uint16_t) ~FLAG_CF) | cf;
            
            } else {
                cpu->regs[reg] = dec16 (cpu, cpu->regs[reg]);
            }
            
            break;
        
        }
        
        case 0x50:  case 0x51:  case 0x52:  case 0x53:  case 0x54:  case 0x55:  case 0x57:
        
            if (cpu->has_operand_override) {
                push32 (cpu, get_reg32 (cpu, op - 0x50));
            } else {
                push16 (cpu, cpu->regs[op - 0x50]);
            }
            
            break;
        
        case 0x56: {
            
            if (cpu->has_operand_override) {
                push32 (cpu, get_reg32 (cpu, REG_SI));
            } else {
                push16 (cpu, cpu->regs[REG_SI]);
            }
            
            break;
        
        }
        
        case 0x58:  case 0x59:  case 0x5A:  case 0x5B:
        case 0x5C:  case 0x5D:  case 0x5E:  case 0x5F:
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, op - 0x58, pop32 (cpu));
            } else {
                cpu->regs[op - 0x58] = pop16 (cpu);
            }
            
            break;
        
        case 0x60: {                    /* PUSHA (80186+) */
        
            if (cpu->has_operand_override) {
            
                uint32_t temp_esp = get_reg32 (cpu, REG_SP);
                
                push32 (cpu, get_reg32 (cpu, REG_AX));
                push32 (cpu, get_reg32 (cpu, REG_CX));
                push32 (cpu, get_reg32 (cpu, REG_DX));
                push32 (cpu, get_reg32 (cpu, REG_BX));
                
                push32 (cpu, temp_esp);
                
                push32 (cpu, get_reg32 (cpu, REG_BP));
                push32 (cpu, get_reg32 (cpu, REG_SI));
                push32 (cpu, get_reg32 (cpu, REG_DI));
                
                break;
            
            }
            
            {
            
                uint16_t temp_sp = cpu->regs[REG_SP];
                
                push16 (cpu, cpu->regs[REG_AX]);
                push16 (cpu, cpu->regs[REG_CX]);
                push16 (cpu, cpu->regs[REG_DX]);
                push16 (cpu, cpu->regs[REG_BX]);
                
                push16 (cpu, temp_sp);
                
                push16 (cpu, cpu->regs[REG_BP]);
                push16 (cpu, cpu->regs[REG_SI]);
                push16 (cpu, cpu->regs[REG_DI]);
            
            }
            
            break;
        
        }
        
        case 0x61: {                    /* POPA (Pop All - 80186+) */
        
            if (cpu->has_operand_override) {
            
                set_reg32 (cpu, REG_DI, pop32 (cpu));
                set_reg32 (cpu, REG_SI, pop32 (cpu));
                set_reg32 (cpu, REG_BP, pop32 (cpu));
                
                pop32 (cpu);
                
                set_reg32 (cpu, REG_BX, pop32 (cpu));
                set_reg32 (cpu, REG_DX, pop32 (cpu));
                set_reg32 (cpu, REG_CX, pop32 (cpu));
                set_reg32 (cpu, REG_AX, pop32 (cpu));
                
                break;
            
            }
            
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
        
        case 0x62: {                    /* BOUND r16, m16&16 */
        
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
        
        case 0x63: {                    /* ARPL r/m16, r16 (80286+ Protected Mode instruction) */
        
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
            cpu->seg_override_index = 4;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x65:
            
            cpu->seg_override = cpu->gs;
            cpu->seg_override_index = 5;
            cpu->has_seg_override = 1;
            
            goto fetch_instruction;
        
        case 0x66:
        
            cpu->has_operand_override = !cpu_code_32 (cpu);
            goto fetch_instruction;
        
        case 0x67:
        
            cpu->has_addr_override = !cpu_code_32 (cpu);
            goto fetch_instruction;
        
        case 0x68: {               /* PUSH imm16/imm32 (80186+) */
        
            if (cpu->has_operand_override) {
                push32 (cpu, fetch32 (cpu));
            } else {
                push16 (cpu, fetch16 (cpu));
            }
            
            break;
        
        }
        
        case 0x6A: {                    /* PUSH imm8 (80186+) */
        
            /* Sign-extend the 8-bit immediate to 16-bits before pushing */
            int16_t imm8 = (int8_t) fetch8 (cpu);
            
            if (cpu->has_operand_override) {
                push32 (cpu, (uint32_t) (int32_t) imm8);
            } else {
                push16 (cpu, (uint16_t) imm8);
            }
            
            break;
        
        }
        
        case 0x69: /* IMUL r16/r32, r/m16/r/m32, imm16/imm32 */     case 0x6B: /* IMUL r16/r32, r/m16/r/m32, imm8 */    {
        
            uint8_t modrm = fetch8 (cpu);
            int reg_dest = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (cpu->has_operand_override) {
            
                int32_t src_val = (int32_t) operand_read32 (cpu, &rm);
                int32_t imm;
                int64_t result;
                
                if (op == 0x69) {
                    imm = (int32_t) fetch32 (cpu);
                } else {
                    imm = (int8_t) fetch8 (cpu);
                }
                
                result = (int64_t) src_val * (int64_t) imm;
                set_reg32 (cpu, reg_dest, (uint32_t) result);
                
                if (result < INT32_MIN || result > INT32_MAX) {
                    cpu->flags |= (FLAG_CF | FLAG_OF);
                } else {
                    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                }
            
            } else {
            
                int16_t src_val = (int16_t) operand_read16 (cpu, &rm);
                int16_t imm;
                int32_t result;
                
                if (op == 0x69) {
                    imm = (int16_t) fetch16 (cpu);
                } else {
                    imm = (int8_t) fetch8 (cpu);
                }
                
                result = (int32_t) src_val * (int32_t) imm;
                cpu->regs[reg_dest] = (uint16_t) result;
                
                if (result < INT16_MIN || result > INT16_MAX) {
                    cpu->flags |= (FLAG_CF | FLAG_OF);
                } else {
                    cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                }
            
            }
            
            break;
        
        }
        
        case 0x6C: /* INSB */   case 0x6D: /* INSW / INSD */ {
        
            unsigned size = op == 0x6C ? 1 : (cpu->has_operand_override ? 4 : 2);
            int step = (cpu->flags & FLAG_DF) ? -(int) size : (int) size;
            
            uint32_t off = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            
            while (count) {
            
                uint32_t value = 0;
                unsigned i;
                
                for (i = 0; i < size; i++) {
                    value |= (uint32_t) io_read8 (cpu, (uint16_t) (cpu->regs[REG_DX] + i)) << (i * 8);
                }
                
                for (i = 0; i < size; i++) {
                
                    uint32_t address = cpu->has_addr_override ? off + i : (uint16_t) (off + i);
                    mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, address), (uint8_t) (value >> (i * 8)));
                
                }
                
                off = cpu->has_addr_override ? off + step : (uint16_t) (off + step);
                count--;
            
            }
            
            if (cpu->has_addr_override) {
            
                set_reg32 (cpu, REG_DI, off);
                
                if (rep_active (cpu)) {
                    set_reg32 (cpu, REG_CX, count);
                }
            
            } else {
            
                cpu->regs[REG_DI] = (uint16_t) off;
                
                if (rep_active (cpu)) {
                    cpu->regs[REG_CX] = (uint16_t) count;
                }
            
            }
            
            break;
        
        }
        
        case 0x6E: /* OUTSB */      case 0x6F: /* OUTSW / OUTSD */ {
        
            unsigned index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            unsigned size = op == 0x6E ? 1 : (cpu->has_operand_override ? 4 : 2);
            
            int step = (cpu->flags & FLAG_DF) ? -(int) size : (int) size;
            
            uint32_t off = cpu->has_addr_override ? get_reg32 (cpu, REG_SI) : cpu->regs[REG_SI];
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            
            while (count) {
            
                unsigned i;
                
                for (i = 0; i < size; i++) {
                
                    uint32_t address = cpu->has_addr_override ? off + i : (uint16_t) (off + i);
                    uint8_t value = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, index, address));
                    
                    io_write8 (cpu, (uint16_t) (cpu->regs[REG_DX] + i), value);
                
                }
                
                off = cpu->has_addr_override ? off + step : (uint16_t) (off + step);
                count--;
            
            }
            
            if (cpu->has_addr_override) {
            
                set_reg32 (cpu, REG_SI, off);
                
                if (rep_active (cpu)) {
                    set_reg32 (cpu, REG_CX, count);
                }
            
            } else {
            
                cpu->regs[REG_SI] = (uint16_t) off;
                
                if (rep_active (cpu)) {
                    cpu->regs[REG_CX] = (uint16_t) count;
                }
            
            }
            
            break;
        
        }
        
        case 0x70:  case 0x71:  case 0x72:  case 0x73:
        case 0x74:  case 0x75:  case 0x76:  case 0x77:
        case 0x78:  case 0x79:  case 0x7A:  case 0x7B:
        case 0x7C:  case 0x7D:  case 0x7E:  case 0x7F: {
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint32_t target = cpu_code_32 (cpu) ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
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
            
            } else if (cpu->has_operand_override) {
            
                uint32_t imm32 = op == 0x81 ? fetch32 (cpu) : (uint32_t) (int32_t) (int8_t) fetch8 (cpu);
                uint32_t left = operand_read32 (cpu, &rm);
                uint32_t result = 0;
                
                switch (subop) {
                
                    case 0:
                    
                        result = add32 (cpu, left, imm32);
                        break;
                    
                    case 1:
                    
                        result = or32 (cpu, left, imm32);
                        break;
                    
                    case 2: {
                    
                        uint32_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;
                        uint64_t full = (uint64_t) left + imm32 + carry;
                        
                        result = (uint32_t) full;
                        cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_AF | FLAG_OF);
                        
                        if (full >> 32) {
                            cpu->flags |= FLAG_CF;
                        }
                        
                        if ((left ^ imm32 ^ result) & 0x10) {
                            cpu->flags |= FLAG_AF;
                        }
                        
                        if ((~(left ^ imm32) & (left ^ result)) & 0x80000000UL) {
                            cpu->flags |= FLAG_OF;
                        }
                        
                        update_common_flags32 (cpu, result);
                        break;
                    
                    }
                    
                    case 3: {
                    
                        uint32_t borrow = (cpu->flags & FLAG_CF) ? 1 : 0;
                        
                        result = left - imm32 - borrow;
                        cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_AF | FLAG_OF);
                        
                        if ((uint64_t) left < (uint64_t) imm32 + borrow) {
                            cpu->flags |= FLAG_CF;
                        }
                        
                        if ((left ^ imm32 ^ result) & 0x10) {
                            cpu->flags |= FLAG_AF;
                        }
                        
                        if (((left ^ imm32) & (left ^ result)) & 0x80000000UL) {
                            cpu->flags |= FLAG_OF;
                        }
                        
                        update_common_flags32 (cpu, result);
                        break;
                    
                    }
                    
                    case 4:
                    
                        result = left & imm32;
                        
                        cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                        update_common_flags32 (cpu, result);
                        
                        break;
                    
                    case 5:
                    
                        result = sub32 (cpu, left, imm32);
                        break;
                    
                    case 6:
                    
                        result = xor32 (cpu, left, imm32);
                        break;
                    
                    case 7:
                    
                        sub32 (cpu, left, imm32);
                        break;
                
                }
                
                if (subop != 7) {
                    operand_write32 (cpu, &rm, result);
                }
            
            } else {
            
                uint16_t left = operand_read16 (cpu, &rm), imm16;
                
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
        
        case 0x84: {                    /* TEST r/m8, r8: */
        
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
            
            if (cpu->has_operand_override) {
                and32 (cpu, operand_read32 (cpu, &rm), get_reg32 (cpu, reg));
            } else {
                and16 (cpu, operand_read16 (cpu, &rm), cpu->regs[reg]);
            }
            
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
            
            } else if (cpu->has_operand_override) {
            
                uint32_t val1 = get_reg32 (cpu, reg);
                uint32_t val2 = operand_read32 (cpu, &rm);
                
                set_reg32 (cpu, reg, val2);
                operand_write32 (cpu, &rm, val1);
            
            } else {
            
                uint16_t val1 = cpu->regs[reg];
                uint16_t val2 = operand_read16 (cpu, &rm);
                
                cpu->regs[reg] = val2;
                operand_write16 (cpu, &rm, val1);
            
            }
            
            break;
        
        }
        
        /* Register-Memory and Immediate Operations */
        case 0x88: {               /* MOV r/m8, r8 */
        
            uint8_t modrm = fetch8 (cpu);
            int src = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint8_t v = get_reg8 (cpu, src);
            
            operand_write8 (cpu, &rm, v);
            break;
        
        }
        
        case 0x8A: {                    /* MOV r8, r/m8 */
        
            uint8_t modrm = fetch8 (cpu);
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint8_t value = operand_read8 (cpu, &rm);
            
            set_reg8 (cpu, (modrm >> 3) & 7, value);
            break;
        
        }
        
        case 0x89:      case 0x8B: {
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (cpu->has_operand_override) {
            
                if (op == 0x89) {
                    operand_write32 (cpu, &rm, get_reg32 (cpu, reg));
                } else {
                    set_reg32 (cpu, reg, operand_read32 (cpu, &rm));
                }
                
                break;
            
            }
            
            if (op == 0x89) {
            
                uint16_t val = cpu->regs[reg];
                operand_write16 (cpu, &rm, val);
            
            } else {
            
                uint16_t val = operand_read16 (cpu, &rm);
                cpu->regs[reg] = val;
            
            }
            
            break;
        
        }
        
        case 0x8C: {                    /* MOV r/m16, Sreg */
        
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
        
        case 0x8D: {                    /* LEA r16, m */
        
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
            if (cpu->has_operand_override) {
                set_reg32 (cpu, reg, rm.addr);
            } else {
                cpu->regs[reg & 7] = (uint16_t) rm.addr;
            }
            
            break;
        
        }
        
        case 0x8E: {                    /* MOV Sreg, r/m16 */
        
            uint8_t modrm = fetch8 (cpu);
            int sreg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            uint16_t value = operand_read16 (cpu, &rm);
            
            if (sreg == 1 || sreg > 5) {
            
                printf ("[INVALID MOV CS] CS:IP=%04X%04X modrm=%02X value=%04X\n", cpu->cs, old_ip, modrm, value);
                return -1;
            
            }
            
            if (cpu_load_segment (cpu, (unsigned) sreg, value) != 0) {
                return -1;
            }
            
            if (sreg == 2) {
                trap_inhibit = 1;
            }
            
            break;
        
        }
        
        case 0x8F: {
        
            uint8_t modrm = fetch8 (cpu);
            
            OperandRef rm;
            uint16_t value;
            
            /* only /0 is valid for POP r/m16 on 8086 */
            if (((modrm >> 3) & 7) != 0) {
            
                printf ("unhandled 0x8F /%d at %04X\n", (modrm >> 3) & 7, old_ip);
                return -1;
            
            }
            
            rm = decode_rm_operand (cpu, modrm);
            
            if (cpu->has_operand_override) {
                operand_write32 (cpu, &rm, pop32 (cpu));
            } else {
            
                value = pop16 (cpu);
                operand_write16 (cpu, &rm, value);
            
            }
            
            break;
        
        }
        
        case 0x90:                      /* NOP (XCHG AX, AX) */
        
            break;
        
        case 0x91:  case 0x92:  case 0x93:  case 0x94:  case 0x95:  case 0x96:  case 0x97: {
        
            int reg = op - 0x90;
            
            if (cpu->has_operand_override) {
            
                uint32_t temp = get_reg32 (cpu, REG_AX);
                
                set_reg32 (cpu, REG_AX, get_reg32 (cpu, reg));
                set_reg32 (cpu, reg, temp);
            
            } else {
            
                uint16_t temp = cpu->regs[REG_AX];
                
                cpu->regs[REG_AX] = cpu->regs[reg];
                cpu->regs[reg] = temp;
            
            }
            
            break;
        
        }
        
        case 0x98: {                    /* CBW / CWDE */

            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, (uint32_t) (int32_t) (int16_t) cpu->regs[REG_AX]);
            } else {
            
                if (cpu->regs[REG_AX] & 0x80) {
                    cpu->regs[REG_AX] |= 0xFF00;
                } else {
                    cpu->regs[REG_AX] &= 0x00FF;
                }
            
            }
            
            break;
        
        }
        
        case 0x99:                      /* CWD / CDQ */

            if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_DX, (get_reg32 (cpu, REG_AX) & 0x80000000UL) ? 0xFFFFFFFFUL : 0);
            } else {
                cpu->regs[REG_DX] = (cpu->regs[REG_AX] & 0x8000) ? 0xFFFF : 0;
            }
            
            break;
        
        case 0x9A: {
        
            uint32_t new_ip = cpu->has_operand_override ? fetch32 (cpu) : fetch16 (cpu);
            uint16_t new_cs = fetch16 (cpu);
            
            if (cpu->has_operand_override) {
            
                push32 (cpu, cpu->cs);
                push32 (cpu, cpu->ip);
            
            } else {
            
                push16 (cpu, cpu->cs);
                push16 (cpu, (uint16_t) cpu->ip);
            
            }
            
            if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                return -1;
            }
            
            cpu->ip = new_ip;
            break;
        
        }
        
        case 0x9B:                      /* WAIT / FWAIT */
        
            if ((cpu->cr0 & 0x0000000AUL) == 0x0000000AUL) {
            
                cpu->ip = (uint32_t) old_ip;
                
                if (cpu->cr0 & 1) {
                
                    if (cpu_protected_interrupt (cpu, 7) != 0) {
                        return -1;
                    }
                
                } else {
                    handle_interrupt (cpu, 7);
                }
            
            }
            
            break;
        
        case 0x9C:                      /* PUSHF (Push Flags onto Stack) */
        
            if (cpu->has_operand_override) {
                push32 (cpu, (cpu->eflags_high & 0x003F0000UL) | cpu->flags);
            } else {
                push16 (cpu, cpu->flags);
            }
            
            break;
        
        case 0x9D: {                    /* POPF */
        
            uint32_t f = cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu);
            
            cpu->flags = (uint16_t) ((f & (FLAG_CF | FLAG_PF | FLAG_AF | FLAG_ZF | FLAG_SF | FLAG_TF | FLAG_IF | FLAG_DF | FLAG_OF | 0x7000)) | 0x0002);
            /* On the 80386, POPFD cannot change RF or VM; higher bits are reserved. */
            
            break;
        
        }
        
        case 0x9E: {                    /* SAHF */
        
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
            
            cpu->flags |= 0x0002;       /* bit 1 always set */
            break;
        
        }
        
        case 0x9F: {                    /* LAHF */
        
            uint8_t ah = 0x02;          /* bit 1 always set */
            
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
        
            uint32_t offset = cpu->has_addr_override ? fetch32 (cpu) : fetch16 (cpu);
            
            unsigned index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            unsigned size = op == 0xA0 ? 1 : (cpu->has_operand_override ? 4 : 2);
            
            uint32_t value = 0;
            unsigned i;
            
            for (i = 0; i < size; i++) {
            
                uint32_t addr = cpu->has_addr_override ? offset + i : (uint16_t) (offset + i);
                value |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, index, addr)) << (8 * i);
            
            }
            
            if (op == 0xA0) {
                set_reg8 (cpu, REG_AX, (uint8_t) value);
            } else if (cpu->has_operand_override) {
                set_reg32 (cpu, REG_AX, value);
            } else {
                cpu->regs[REG_AX] = (uint16_t) value;
            }
            
            break;
        
        }
        
        case 0xA2: /* MOV moffs8, AL */     case 0xA3: /* MOV moffs16, AX */ {
        
            uint32_t offset = cpu->has_addr_override ? fetch32 (cpu) : fetch16 (cpu);
            
            unsigned index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            unsigned size = op == 0xA2 ? 1 : (cpu->has_operand_override ? 4 : 2);
            
            uint32_t value = op == 0xA2 ? get_reg8 (cpu, REG_AX) : (cpu->has_operand_override ? get_reg32 (cpu, REG_AX) : cpu->regs[REG_AX]);
            unsigned i;
            
            for (i = 0; i < size; i++) {
            
                uint32_t addr = cpu->has_addr_override ? offset + i : (uint16_t) (offset + i);
                mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, index, addr), (uint8_t) (value >> (8 * i)));
            
            }
            
            break;
        
        }
        
        case 0xA4: {                    /* MOVSB */
        
            unsigned src_index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            
            uint32_t src = cpu->has_addr_override ? get_reg32 (cpu, REG_SI) : cpu->regs[REG_SI];
            uint32_t dst = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
            
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            
            uint32_t i;
            
            for (i = 0; i < count; i++) {
            
                uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, src_index, src));
                mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, dst), v);
                
                src = cpu->has_addr_override ? src + step : (uint16_t) (src + step);
                dst = cpu->has_addr_override ? dst + step : (uint16_t) (dst + step);
            
            }
            
            if (cpu->has_addr_override) {
            
                set_reg32 (cpu, REG_SI, src);
                set_reg32 (cpu, REG_DI, dst);
                
                if (rep_active (cpu)) {
                    set_reg32 (cpu, REG_CX, 0);
                }
            
            } else {
            
                cpu->regs[REG_SI] = (uint16_t) src;
                cpu->regs[REG_DI] = (uint16_t) dst;
                
                if (rep_active (cpu)) {
                    cpu->regs[REG_CX] = 0;
                }
            
            }
            
            break;
        
        }
        
        case 0xA5: {                    /* MOVSW / MOVSD */
        
            unsigned src_index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            
            uint32_t src = cpu->has_addr_override ? get_reg32 (cpu, REG_SI) : cpu->regs[REG_SI];
            uint32_t dst = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            
            unsigned width = cpu->has_operand_override ? 4 : 2;
            int step = (cpu->flags & FLAG_DF) ? -(int) width : (int) width;
            
            uint32_t i;
            
            for (i = 0; i < count; i++) {
            
                unsigned j;
                
                for (j = 0; j < width; j++) {
                
                    uint32_t source = cpu->has_addr_override ? src + j : (uint16_t) (src + j);
                    uint32_t target = cpu->has_addr_override ? dst + j : (uint16_t) (dst + j);
                    uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, src_index, source));
                    
                    mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, target), v);
                
                }
                
                src = cpu->has_addr_override ? src + step : (uint16_t) (src + step);
                dst = cpu->has_addr_override ? dst + step : (uint16_t) (dst + step);
            
            }
            
            if (cpu->has_addr_override) {
            
                set_reg32 (cpu, REG_SI, src);
                set_reg32 (cpu, REG_DI, dst);
                
                if (rep_active (cpu)) {
                    set_reg32 (cpu, REG_CX, 0);
                }
            
            } else {
            
                cpu->regs[REG_SI] = (uint16_t) src;
                cpu->regs[REG_DI] = (uint16_t) dst;
                
                if (rep_active (cpu)) {
                    cpu->regs[REG_CX] = 0;
                }
            
            }
            
            break;
        
        }
        
        case 0xA6: {                    /* CMPSB */
        
            unsigned src_index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            
            uint32_t src = cpu->has_addr_override ? get_reg32 (cpu, REG_SI) : cpu->regs[REG_SI];
            uint32_t dst = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            
            while (count) {
            
                uint8_t lhs = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, src_index, src));
                uint8_t rhs = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, dst));
                
                sub8 (cpu, lhs, rhs);
                
                src = cpu->has_addr_override ? src + step : (uint16_t) (src + step);
                dst = cpu->has_addr_override ? dst + step : (uint16_t) (dst + step);
                
                count--;
                
                if (cpu->rep_type == 1 && !(cpu->flags & FLAG_ZF)) {
                    break;
                }
                
                if (cpu->rep_type == 2 && (cpu->flags & FLAG_ZF)) {
                    break;
                }
            
            }
            
            if (cpu->has_addr_override) {
            
                set_reg32 (cpu, REG_SI, src);
                set_reg32 (cpu, REG_DI, dst);
                
                if (rep_active (cpu)) {
                    set_reg32 (cpu, REG_CX, count);
                }
            
            } else {
            
                cpu->regs[REG_SI] = (uint16_t) src;
                cpu->regs[REG_DI] = (uint16_t) dst;
                
                if (rep_active (cpu)) {
                    cpu->regs[REG_CX] = (uint16_t) count;
                }
            
            }
            
            break;
        
        }
        
        case 0xA7: {                    /* CMPSW / CMPSD */
        
            unsigned src_index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            unsigned width = cpu->has_operand_override ? 4 : 2;
            
            int step = (cpu->flags & FLAG_DF) ? -(int) width : (int) width;
            
            uint32_t src = cpu->has_addr_override ? get_reg32 (cpu, REG_SI) : cpu->regs[REG_SI];
            uint32_t dst = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
            
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            
            while (count) {
            
                uint32_t lhs = 0, rhs = 0;
                unsigned i;
                
                for (i = 0; i < width; i++) {
                
                    uint32_t source = cpu->has_addr_override ? src + i : (uint16_t) (src + i);
                    uint32_t target = cpu->has_addr_override ? dst + i : (uint16_t) (dst + i);
                    
                    lhs |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, src_index, source)) << (i * 8);
                    rhs |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, target)) << (i * 8);
                
                }
                
                if (width == 4) {
                    sub32 (cpu, lhs, rhs);
                } else {
                    sub16 (cpu, (uint16_t) lhs, (uint16_t) rhs);
                }
                
                src = cpu->has_addr_override ? src + step : (uint16_t) (src + step);
                dst = cpu->has_addr_override ? dst + step : (uint16_t) (dst + step);
                
                count--;
                
                if (cpu->rep_type == 1 && !(cpu->flags & FLAG_ZF)) {
                    break;
               }
               
                if (cpu->rep_type == 2 && (cpu->flags & FLAG_ZF)) {
                    break;
                }
            
            }
            
            if (cpu->has_addr_override) {
            
                set_reg32 (cpu, REG_SI, src);
                set_reg32 (cpu, REG_DI, dst);
                
                if (rep_active (cpu)) {
                    set_reg32 (cpu, REG_CX, count);
                }
            
            } else {
            
                cpu->regs[REG_SI] = (uint16_t) src;
                cpu->regs[REG_DI] = (uint16_t) dst;
                
                if (rep_active (cpu)) {
                    cpu->regs[REG_CX] = (uint16_t) count;
                }
            
            }
            
            break;
        
        }
        
        case 0xA8: {                    /* TEST AL, imm8 */
        
            uint8_t imm8 = fetch8 (cpu);
            
            and8 (cpu, get_reg8 (cpu, REG_AX), imm8);
            break;
        
        }
        
        case 0xA9: {                    /* TEST AX/EAX, imm16/32 */
        
            if (cpu->has_operand_override) {
                and32 (cpu, get_reg32 (cpu, REG_AX), fetch32 (cpu));
            } else {
                and16 (cpu, cpu->regs[REG_AX], fetch16 (cpu));
            }
            
            break;
        
        }
        
        case 0xAA: {                    /* STOSB */
        
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            uint8_t value = get_al (cpu);
            
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            uint32_t off = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
            
            while (count) {
            
                mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, off), value);
                
                off = cpu->has_addr_override ? off + step : (uint16_t) (off + step);
                count--;
                
                if (rep_active (cpu)) {
                
                    if (cpu->has_addr_override) {
                        set_reg32 (cpu, REG_CX, count);
                    } else {
                        cpu->regs[REG_CX] = (uint16_t) count;
                    }
                
                }
            
            }
                
            if (cpu->has_addr_override) {
                set_reg32 (cpu, REG_DI, off);
            } else {
                cpu->regs[REG_DI] = (uint16_t) off;
            }
            
            break;
        
        }
        
        case 0xAB: {                    /* STOSW / STOSD */
        
            unsigned size = cpu->has_operand_override ? 4 : 2;
            int step = (cpu->flags & FLAG_DF) ? -(int) size : (int) size;
            
            uint32_t val = cpu->has_operand_override ? get_reg32 (cpu, REG_AX) : cpu->regs[REG_AX];
            uint32_t count = cpu->rep_type ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            
            while (count) {
            
                uint32_t off = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
                unsigned i;
                
                for (i = 0; i < size; i++) {
                
                    uint32_t address = cpu->has_addr_override ? off + i : (uint16_t) (off + i);
                    mem_write8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, address), (uint8_t) (val >> (i * 8)));
                
                }
                
                if (cpu->has_addr_override) {
                    set_reg32 (cpu, REG_DI, off + step);
                } else {
                    cpu->regs[REG_DI] = (uint16_t) (off + step);
                }
                
                count--;
                
                if (cpu->rep_type) {
                
                    if (cpu->has_addr_override) {
                        set_reg32 (cpu, REG_CX, count);
                    } else {
                        cpu->regs[REG_CX] = (uint16_t) count;
                    }
                
                }
            
            }
            
            break;
        
        }
        
        case 0xAC: /* LODSB */      case 0xAD: /* LODSW / LODSD */ {
        
            unsigned index = cpu->has_seg_override ? cpu->seg_override_index : 3;
            unsigned size = op == 0xAC ? 1 : (cpu->has_operand_override ? 4 : 2);
            
            int step = (cpu->flags & FLAG_DF) ? -(int) size : (int) size;
            
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            uint32_t off = cpu->has_addr_override ? get_reg32 (cpu, REG_SI) : cpu->regs[REG_SI];
            
            while (count) {
            
                uint32_t value = 0;
                unsigned i;
                
                for (i = 0; i < size; i++) {
                
                    uint32_t addr = cpu->has_addr_override ? off + i : (uint16_t) (off + i);
                    value |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, index, addr)) << (i * 8);
                
                }
            
                if (op == 0xAC) {
                    set_al (cpu, (uint8_t) value);
                } else if (cpu->has_operand_override) {
                    set_reg32 (cpu, REG_AX, value);
                } else {
                    cpu->regs[REG_AX] = (uint16_t) value;
                }
                
                off = cpu->has_addr_override ? off + step : (uint16_t) (off + step);
                count--;
                
                if (rep_active (cpu)) {
                
                    if (cpu->has_addr_override) {
                        set_reg32 (cpu, REG_CX, count);
                    } else {
                        cpu->regs[REG_CX] = (uint16_t) count;
                    }
                
                }
            
            }
            
            if (cpu->has_addr_override) {
                set_reg32 (cpu, REG_SI, off);
            } else {
                cpu->regs[REG_SI] = (uint16_t) off;
            }
            
            break;
        
        }
        
        case 0xAE: {                    /* SCASB */
        
            int step = (cpu->flags & FLAG_DF) ? -1 : 1;
            uint8_t al = get_al (cpu);
            
            uint32_t off = cpu->has_addr_override ? get_reg32 (cpu, REG_DI) : cpu->regs[REG_DI];
            uint32_t count = rep_active (cpu) ? (cpu->has_addr_override ? get_reg32 (cpu, REG_CX) : cpu->regs[REG_CX]) : 1;
            
            while (count) {
            
                uint8_t v = mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 0, off));
                sub8 (cpu, al, v);
                
                off = cpu->has_addr_override ? off + step : (uint16_t) (off + step);
                count--;
                
                if (cpu->rep_type == 1 && !(cpu->flags & FLAG_ZF)) {
                    break;
                }
                
                if (cpu->rep_type == 2 && (cpu->flags & FLAG_ZF)) {
                    break;
                }
            
            }
            
            if (cpu->has_addr_override) {
            
                set_reg32 (cpu, REG_DI, off);
                
                if (rep_active (cpu)) {
                    set_reg32 (cpu, REG_CX, count);
                }
            
            } else {
            
                cpu->regs[REG_DI] = (uint16_t) off;
                
                if (rep_active (cpu)) {
                    cpu->regs[REG_CX] = (uint16_t) count;
                }
            
            }
            
            break;
        
        }
        
        case 0xAF: {                    /* SCASW */
        
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
        
        case 0xB8:  case 0xB9:  case 0xBA:  case 0xBB:
        case 0xBC:  case 0xBD:  case 0xBE:  case 0xBF:
        
            if (cpu->has_operand_override) {
                set_reg32 (cpu, op - 0xB8, fetch32 (cpu));
            } else {
                cpu->regs[op - 0xB8] = fetch16 (cpu);
            }
            
            break;
        
        case 0xC0: {                    /*8-bit shift/Rotate by imm8 (80186+) */
        
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
            
                case 0:                 /* ROL */
                
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
                
                case 3:                 /* RCR */
                
                    for (int i = 0; i < count % 9; i++) {
                    
                        uint8_t curr_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                        uint8_t next_cf = result & 1;
                        
                        result = (uint8_t) ((result >> 1) | (curr_cf << 7));
                        
                        if (next_cf) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {
                    
                        if ((result ^ (result << 1)) & 0x80) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                    
                    break;
                
                case 4:                 /* SHL / SAL */
                
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
                
                case 5:                 /* SHR */
                
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
                
                    fprintf (stderr, "opcode 0xC0 subop /%d not fully implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write8 (cpu, &rm, result);
            break;
        
        }
        
        case 0xC1: {                    /* 16-bit shift/Rotate by imm8 (80186+) */
        
            if (cpu->has_operand_override) {
            
                uint8_t modrm = fetch8 (cpu);
                
                unsigned subop = (modrm >> 3) & 7;
                OperandRef rm = decode_rm_operand (cpu, modrm);
                
                uint8_t count = fetch8 (cpu) & 0x1F;
                
                uint32_t value = operand_read32 (cpu, &rm);
                uint32_t result;
                
                if (!count) {
                    break;
                }
                
                if (subop == 0) {                           /* ROL r/m32, imm8 */
                
                    result = (value << count) | (value >> (32 - count));
                    
                    if (result & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if (((result >> 31) ^ (result & 1)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
               
                } else if (subop == 1) {                    /* ROR r/m32, imm8 */
                
                    result = (value >> count) | (value << (32 - count));
                    
                    if (result & 0x80000000UL) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if (((result >> 31) ^ (result >> 30)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 4) {                    /* SHL r/m32, imm8 */
                
                    result = value << count;
                    
                    if ((value >> (32 - count)) & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if ((result ^ value) & 0x80000000UL) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 5) {                    /* SHR r/m32, imm8 */
                
                    result = value >> count;
                    
                    if ((value >> (count - 1)) & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if (value & 0x80000000UL) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 7) {                    /* SAR r/m32, imm8 */
                
                    result = (uint32_t) ((int32_t) value >> count);
                    
                    if ((value >> (count - 1)) & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                    }
                
                } else {
                
                    fprintf (stderr, "unsupported 80386 C1 /%u at %04X:%04X\n", subop, cpu->cs, old_ip);
                    return -1;
                
                }
                
                if (subop != 0) {
                    update_common_flags32 (cpu, result);
                }
                
                operand_write32 (cpu, &rm, result);
                break;
            
            }
            
            {
            
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
                    
                        fprintf (stderr, "opcode 0xC1 subop /%d not implemented at %04X\n", subop, old_ip);
                        return -1;
                
                }
                
                operand_write16 (cpu, &rm, result);
            
            }
            
            break;
        
        }
        
        case 0xC2: {                    /* RET imm16 */
        
            uint16_t add_sp = fetch16 (cpu);
            uint32_t new_ip = cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu);
            
            cpu_set_stack_pointer (cpu, cpu_stack_pointer (cpu) + add_sp);
            cpu->ip = new_ip;
            
            break;
        
        }
        
        case 0xC3: {                    /* RET near */
        
            cpu->ip = cpu->has_operand_override ? pop32 (cpu) : pop16 (cpu);
            break;
        
        }
        
        case 0xC4: {                    /* LES r16, m16:16 / m16:32 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            OperandRef selector_ref = rm;
            
            uint32_t offset;
            uint16_t selector;
            
            if (!rm.is_mem) {
                return -1;
            }
            
            offset = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
            
            selector_ref.addr = cpu->has_addr_override ? rm.addr + (cpu->has_operand_override ? 4 : 2) : (uint16_t) (rm.addr + (cpu->has_operand_override ? 4 : 2));
            selector = operand_read16 (cpu, &selector_ref);
            
            if (cpu_load_segment (cpu, 0, selector) != 0) {
                return -1;
            }
            
            if (cpu->has_operand_override) {
                set_reg32 (cpu, reg, offset);
            } else {
                cpu->regs[reg] = (uint16_t) offset;
            }
            
            break;
        
        }
        
        case 0xC5: {                    /* LDS r16, m16:16 / m16:32 */
        
            uint8_t modrm = fetch8 (cpu);
            int reg = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            OperandRef selector_ref = rm;
            
            uint32_t offset;
            uint16_t selector;
            
            if (!rm.is_mem) {
                return -1;
            }
            
            offset = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
            
            selector_ref.addr = cpu->has_addr_override ? rm.addr + (cpu->has_operand_override ? 4 : 2) : (uint16_t) (rm.addr + (cpu->has_operand_override ? 4 : 2));
            selector = operand_read16 (cpu, &selector_ref);
            
            if (cpu_load_segment (cpu, 3, selector) != 0) {
                return -1;
            }
            
            if (cpu->has_operand_override) {
                set_reg32 (cpu, reg, offset);
            } else {
                cpu->regs[reg] = (uint16_t) offset;
            }
            
            break;
        
        }
        
        case 0xC6: /* MOV r/m8, imm8 */     case 0xC7: /* MOV r/m16, imm16 */ {
        
            uint8_t modrm = fetch8 (cpu);
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0xC6) {
            
                uint8_t imm8 = fetch8 (cpu);                /* Correctly fetch 8 bits */
                operand_write8 (cpu, &rm, imm8);
            
            } else if (cpu->has_operand_override) {
            
                uint32_t imm32 = fetch32 (cpu);
                operand_write32 (cpu, &rm, imm32);
            
            } else {
            
                uint16_t imm16 = fetch16 (cpu);             /* Correctly fetch 16 bits */
                operand_write16 (cpu, &rm, imm16);
            
            }
            
            break;
        
        }
        
        case 0xC8: {                    /* ENTER imm16, imm8 (80186+) */
        
            uint16_t size = fetch16 (cpu);
            
            unsigned level = fetch8 (cpu) & 0x1F;
            unsigned i;
            
            if (cpu->has_operand_override) {
            
                uint32_t frame, parent = get_reg32 (cpu, REG_BP);
                
                push32 (cpu, parent);
                frame = cpu_stack_pointer (cpu);
                
                if (level) {
                
                    for (i = 1; i < level; i++) {
                    
                        uint32_t value = 0;
                        
                        unsigned j;
                        parent -= 4;
                        
                        for (j = 0; j < 4; j++) {
                            value |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, 2, parent + j)) << (8 * j);
                        }
                        
                        push32 (cpu, value);
                    
                    }
                    
                    push32 (cpu, frame);
                
                }
                
                set_reg32 (cpu, REG_BP, frame);
                cpu_set_stack_pointer (cpu, cpu_stack_pointer (cpu) - size);
            
            } else {
            
                uint16_t frame, parent = cpu->regs[REG_BP];
                
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
            
            }
            
            break;
        
        }
        
        case 0xC9:                      /* LEAVE */
        
            if (cpu->has_operand_override) {
            
                cpu_set_stack_pointer (cpu, get_reg32 (cpu, REG_BP));
                set_reg32 (cpu, REG_BP, pop32 (cpu));
            
            } else {
            
                cpu_set_stack_pointer (cpu, cpu->regs[REG_BP]);
                cpu->regs[REG_BP] = pop16 (cpu);
            
            }
            
            break;
        
        case 0xCA: {                    /* RETF imm16 */
        
            uint16_t add_sp = fetch16 (cpu);
            
            uint32_t new_ip;
            uint16_t new_cs;
            
            if (cpu->has_operand_override) {
            
                new_ip = pop32 (cpu);
                new_cs = (uint16_t) pop32 (cpu);
            
            } else {
            
                new_ip = pop16 (cpu);
                new_cs = pop16 (cpu);
            
            }
            
            cpu_set_stack_pointer (cpu, cpu_stack_pointer (cpu) + add_sp);
            
            if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                return -1;
            }
            
            if (new_ip > cpu->segments[1].limit) {
            
                fprintf (stderr, "protected-mode return offset %08X exceeds segment limit\n", new_ip);
                return -1;
            
            }
            
            cpu->ip = new_ip;
            break;
        
        }
        
        case 0xCB: {
        
            uint32_t new_ip;
            uint16_t new_cs;
            
            if (cpu->has_operand_override) {
            
                new_ip = pop32 (cpu);
                new_cs = (uint16_t) pop32 (cpu);
            
            } else {
            
                new_ip = pop16 (cpu);
                new_cs = pop16 (cpu);
            
            }
            
            if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                return -1;
            }
            
            if (new_ip > cpu->segments[1].limit) {
            
                fprintf (stderr, "protected-mode return offset %08X exceeds segment limit\n", new_ip);
                return -1;
            
            }
            
            cpu->ip = new_ip;
            break;
        
        }
        
        case 0xCD: {                    /* INT imm8 */
        
            uint8_t inum = fetch8 (cpu);
            
            if (cpu->cr0 & 1) {
            
                if (cpu_protected_interrupt (cpu, inum) != 0) {
                    return -1;
                }
                
                break;
            
            }
            
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
        
        case 0xCC: {                    /* INT 3 (Breakpoint) */
        
            if (cpu->cr0 & 1) {
            
                if (cpu_protected_interrupt (cpu, 3) != 0) {
                    return -1;
                }
            
            } else {
                handle_interrupt (cpu, 3);
            }
            
            break;
        
        }
        
        case 0xCE: {                    /* INTO (Interrupt on Overflow) */
        
            if (cpu->flags & FLAG_OF) {
                handle_interrupt (cpu, 4);
            }
            
            break;
        
        }
        
        case 0xCF: {
        
            if (cpu->has_operand_override) {
            
                uint32_t new_eip = pop32 (cpu);
                uint16_t new_cs = (uint16_t) pop32 (cpu);
                
                uint32_t new_flags = pop32 (cpu);
                
                if (new_flags & 0x00020000UL) {
                
                    fprintf (stderr, "virtual 8086 mode is not implemented at %04X:%04X\n", cpu->cs, old_ip);
                    return -1;
                
                }
                
                if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                    return -1;
                }
                
                if (new_eip > cpu->segments[1].limit) {
                
                    fprintf (stderr, "protected-mode EIP %08X exceeds segment limit\n", new_eip);
                    return -1;
                
                }
                
                cpu->ip = new_eip;
                
                cpu->flags = (uint16_t) (new_flags | 2);
                cpu->eflags_high = new_flags & 0x003F0000UL;
            
            } else {
            
                uint16_t new_ip = pop16 (cpu);
                uint16_t new_cs = pop16 (cpu);
                
                uint16_t new_flags = pop16 (cpu);
                
                if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                    return -1;
                }
                
                cpu->ip = new_ip;
                cpu->flags = (uint16_t) (new_flags | 2);
            
            }
            
            break;
        
        }
        
        case 0xD4: {                    /* AAM imm8 */
        
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
            
                case 0:                 /* INC r/m16/32 */
                
                    if (cpu->has_operand_override) {
                    
                        uint16_t cf = cpu->flags & FLAG_CF;
                        
                        operand_write32 (cpu, &rm, add32 (cpu, operand_read32 (cpu, &rm), 1));
                        cpu->flags = (cpu->flags & (uint16_t) ~FLAG_CF) | cf;
                    
                    } else {
                        operand_write16 (cpu, &rm, inc16 (cpu, operand_read16 (cpu, &rm)));
                    }
                    
                    break;
                
                case 1:                 /* DEC r/m16/32 */
                
                    if (cpu->has_operand_override) {
                    
                        uint16_t cf = cpu->flags & FLAG_CF;
                        
                        operand_write32 (cpu, &rm, sub32 (cpu, operand_read32 (cpu, &rm), 1));
                        cpu->flags = (cpu->flags & (uint16_t) ~FLAG_CF) | cf;
                    
                    } else {
                        operand_write16 (cpu, &rm, dec16 (cpu, operand_read16 (cpu, &rm)));
                    }
                    
                    break;
                
                case 2: {               /* CALL near indirect */
                
                    uint32_t target = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                    
                    if (cpu->has_operand_override) {
                        push32 (cpu, cpu->ip);
                    } else {
                        push16 (cpu, (uint16_t) cpu->ip);
                    }
                    
                    cpu->ip = target;
                    break;
                
                }
                
                case 3: {               /* CALL far indirect (m16:16 / m16:32) */
                
                    uint32_t new_ip = 0;
                    uint16_t new_cs = 0;
                    uint16_t old_cs = cpu->cs;
                    uint32_t return_ip;
                    
                    unsigned width = cpu->has_operand_override ? 4 : 2;
                    unsigned i;
                
                    if (!rm.is_mem) {
                        return -1;
                    }
                    
                    for (i = 0; i < width; i++) {
                    
                        uint32_t off = cpu->has_addr_override ? rm.addr + i : (uint16_t) (rm.addr + i);
                        new_ip |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, rm.seg_index, off)) << (8 * i);
                    
                    }
                    
                    for (i = 0; i < 2; i++) {
                    
                        uint32_t off = cpu->has_addr_override ? rm.addr + width + i : (uint16_t) (rm.addr + width + i);
                        new_cs |= (uint16_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, rm.seg_index, off)) << (8 * i);
                    
                    }
                    
                    return_ip = cpu->ip;
                    
                    if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                        return -1;
                    }
                    
                    if (new_ip > cpu->segments[1].limit) {
                    
                        fprintf (stderr, "protected-mode far call target %08X exceeds segment limit\n", new_ip);
                        return -1;
                    
                    }
                    
                    if (cpu->has_operand_override) {
                    
                        push32 (cpu, old_cs);
                        push32 (cpu, return_ip);
                    
                    } else {
                    
                        push16 (cpu, old_cs);
                        push16 (cpu, (uint16_t) return_ip);
                    
                    }
                    
                    cpu->ip = new_ip;
                    break;
                
                }
                
                case 4:                 /* JMP near indirect */
                
                    cpu->ip = cpu->has_operand_override ? operand_read32 (cpu, &rm) : operand_read16 (cpu, &rm);
                    break;
                
                case 5: {               /* JMP far indirect (m16:16 / m16:32) */
                
                    uint32_t new_ip = 0;
                    uint16_t new_cs = 0;
                    
                    unsigned width = cpu->has_operand_override ? 4 : 2;
                    unsigned i;
                
                    if (!rm.is_mem) {
                        return -1;
                    }
                    
                    for (i = 0; i < width; i++) {
                    
                        uint32_t off = cpu->has_addr_override ? rm.addr + i : (uint16_t) (rm.addr + i);
                        new_ip |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, rm.seg_index, off)) << (8 * i);
                    
                    }
                    
                    for (i = 0; i < 2; i++) {
                    
                        uint32_t off = cpu->has_addr_override ? rm.addr + width + i : (uint16_t) (rm.addr + width + i);
                        new_cs |= (uint16_t) mem_read8 (cpu->mem, cpu->mem_size, cpu_segment_phys (cpu, rm.seg_index, off)) << (8 * i);
                    
                    }
                    
                    if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                        return -1;
                    }
                    
                    if (new_ip > cpu->segments[1].limit) {
                    
                        fprintf (stderr, "protected-mode far jump target %08X exceeds segment limit\n", new_ip);
                        return -1;
                    
                    }
                    
                    cpu->ip = new_ip;
                    break;
                
                }
                
                case 6:                 /* PUSH r/m16/32 */
                
                    if (cpu->has_operand_override) {
                        push32 (cpu, operand_read32 (cpu, &rm));
                    } else {
                        push16 (cpu, operand_read16 (cpu, &rm));
                    }
                    
                    break;
                
                default:
                
                    fprintf (stderr, "unhandled 0xFF extension /%d (undefined) at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            break;
        
        }
        
        case 0xD0: {                    /*8-bit shift/Rotate by 1 */
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            uint8_t val = operand_read8 (cpu, &rm);
            uint8_t result = val;
            uint8_t old_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
            
            switch (subop) {
            
                case 0:                 /* ROL (Rotate Left) */
                
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
                
                case 1:                 /* ROR (Rotate Right) */
                
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
                
                case 2:                 /* RCL (Rotate through Carry Left) */
                
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
                
                case 3:                 /* RCR (Rotate through Carry Right) */
                
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
                
                case 4:                 /* SHL / SAL */
                
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
                    
                    update_common_flags8 (cpu, result);     /* shifts DO affect ZF/SF/PF */
                    break;
                
                case 5:                 /* SHR */
                
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
                
                case 7:                 /* SAR */
                
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
                
                    fprintf (stderr, "opcode 0xD0 subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write8 (cpu, &rm, result);
            break;
        
        }
        
        case 0xD1: {                    /* shift/Rotate by 1 */
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (cpu->has_operand_override) {
            
                uint32_t value = operand_read32 (cpu, &rm);
                uint32_t result = value;
                
                uint8_t old_cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                
                switch (subop) {
                
                    case 0:             /* ROL */
                    
                        result = (value << 1) | (value >> 31);
                        
                        if (value & 0x80000000UL) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        if (((result >> 31) ^ (value >> 31)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                        
                        break;
                    
                    case 1:             /* ROR */
                    
                        result = (value >> 1) | (value << 31);
                        
                        if (value & 1) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        if (((result >> 31) ^ (result >> 30)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                        
                        break;
                    
                    case 2:             /* RCL */
                    
                        result = (value << 1) | old_cf;
                        
                        if (value & 0x80000000UL) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        if (((result >> 31) ^ (value >> 31)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                        
                        break;
                    
                    case 3:             /* RCR */
                    
                        result = (value >> 1) | ((uint32_t) old_cf << 31);
                        
                        if (value & 1) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        if (((result >> 31) ^ (result >> 30)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                        
                        break;
                    
                    case 4:             /* SHL / SAL */
                    
                        result = value << 1;
                        
                        if (value & 0x80000000UL) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        if (((result >> 31) ^ (value >> 31)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                        
                        update_common_flags32 (cpu, result);
                        break;
                    
                    case 5:             /* SHR */
                    
                        result = value >> 1;
                        
                        if (value & 1) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        if (value & 0x80000000UL) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                        
                        update_common_flags32 (cpu, result);
                        break;
                    
                    case 7:             /* SAR */
                    
                        result = (uint32_t) ((int32_t) value >> 1);
                        
                        if (value & 1) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                        
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                        update_common_flags32 (cpu, result);
                        break;
                    
                    default:
                    
                        fprintf (stderr, "opcode 0xD1 subop /%d not implemented at %04X\n", subop, old_ip);
                        return -1;
                
                }
                
                operand_write32 (cpu, &rm, result);
                break;
            
            }
            
            {
            
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
                    
                        fprintf (stderr, "opcode 0xD1 subop /%d not implemented at %04X\n", subop, old_ip);
                        return -1;
                
                }
                
                operand_write16 (cpu, &rm, result);
            
            }
            
            break;
        
        }
        
        case 0xD2: {                    /* 8-bit shift/Rotate by CL */
        
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
            
                case 0:                 /* ROL */
                
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
                
                case 1:                 /* ROR */
                
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
                
                case 2:                 /* RCL */
                
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
                
                case 3:                 /* RCR */
                
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
                
                case 4:                 /* SHL / SAL */
                
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
                
                case 5:                 /* SHR */
                
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
                
                case 7:                 /* SAR */
                
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
                
                    fprintf (stderr, "opcode 0xD2 subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
            
            }
            
            operand_write8 (cpu, &rm, result);
            break;
        
        }
        
        case 0xD3: {
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (cpu->has_operand_override) {
            
                uint32_t val = operand_read32 (cpu, &rm), result = val;
                uint8_t count = get_reg8 (cpu, REG_CX) & 0x1F;
                
                if (!count) {
                    break;
                }
                
                if (subop == 0) {                           /* ROL r/m32, CL */
                
                    result = (val << count) | (val >> (32 - count));
                    
                    if (result & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if (((result >> 31) ^ (result & 1)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 1) {                    /* ROR r/m32, CL */
                
                    result = (val >> count) | (val << (32 - count));
                    
                    if (result & 0x80000000UL) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if (((result >> 31) ^ (result >> 30)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 2) {                    /* RCL r/m32, CL */
                
                    unsigned i;
                    
                    for (i = 0; i < count; i++) {
                    
                        uint32_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;
                        uint32_t new_carry = result >> 31;
                        
                        result = (result << 1) | carry;
                        
                        if (new_carry) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {
                    
                        if (((result >> 31) ^ ((cpu->flags & FLAG_CF) ? 1 : 0)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 3) {                    /* RCR r/m32, CL */
                
                    unsigned i;
                    
                    for (i = 0; i < count; i++) {
                    
                        uint32_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;
                        uint32_t new_carry = result & 1;
                        
                        result = (result >> 1) | (carry << 31);
                        
                        if (new_carry) {
                            cpu->flags |= FLAG_CF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                        }
                    
                    }
                    
                    if (count == 1) {
                    
                        if (((result >> 31) ^ (result >> 30)) & 1) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 4) {                    /* SHL / SAL r/m32, CL */
                
                    result = val << count;
                    
                    if ((val >> (32 - count)) & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if ((result ^ val) & 0x80000000UL) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 5) {                    /* SHR r/m32, CL */
                
                    result = val >> count;
                    
                    if ((val >> (count - 1)) & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                    
                        if (val & 0x80000000UL) {
                            cpu->flags |= FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~FLAG_OF;
                        }
                    
                    }
                
                } else if (subop == 7) {                    /* SAR r/m32, CL */
                
                    result = (uint32_t) ((int32_t) val >> count);
                    
                    if ((val >> (count - 1)) & 1) {
                        cpu->flags |= FLAG_CF;
                    } else {
                        cpu->flags &= (uint16_t) ~FLAG_CF;
                    }
                    
                    if (count == 1) {
                        cpu->flags &= (uint16_t) ~FLAG_OF;
                    }
                
                } else {
                
                    fprintf (stderr, "shift subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
                
                }
                
                if (subop == 4 || subop == 5 || subop == 7) {
                    update_common_flags32 (cpu, result);
                }
                
                operand_write32 (cpu, &rm, result);
                break;
            
            }
            
            {
            
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
                
                    fprintf (stderr, "shift subop /%d not implemented at %04X\n", subop, old_ip);
                    return -1;
                
                }
                
                operand_write16 (cpu, &rm, result);
                
                if (subop == 4 || subop == 5 || subop == 7) {
                    update_common_flags16 (cpu, result);
                }
            
            }
            
            break;
        
        }
        
        case 0xD7: {                    /* XLAT */
        
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
        
        case 0xE0: {                    /* LOOPNE / LOOPNZ */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint32_t new_cx, target;
            int zf, take;
            
            new_cx = cpu->has_addr_override ? get_reg32 (cpu, REG_CX) - 1 : (uint16_t) (cpu->regs[REG_CX] - 1);
            
            if (cpu->has_addr_override) {
                set_reg32 (cpu, REG_CX, new_cx);
            } else {
                cpu->regs[REG_CX] = (uint16_t) new_cx;
            }
            
            target = cpu_code_32 (cpu) ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
            
            zf = (cpu->flags & FLAG_ZF) != 0;
            take = (new_cx != 0) && !zf;
            
            if (take) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE1: {                    /* LOOPE / LOOPZ */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint32_t new_cx, target;
            int zf, take;
            
            new_cx = cpu->has_addr_override ? get_reg32 (cpu, REG_CX) - 1 : (uint16_t) (cpu->regs[REG_CX] - 1);
            
            if (cpu->has_addr_override) {
                set_reg32 (cpu, REG_CX, new_cx);
            } else {
                cpu->regs[REG_CX] = (uint16_t) new_cx;
            }
            
            target = cpu_code_32 (cpu) ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
            
            zf = (cpu->flags & FLAG_ZF) != 0;
            take = (new_cx != 0) && zf;
            
            if (take) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE2: {                    /* LOOP */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint32_t new_cx, target;
            int take;
            
            new_cx = cpu->has_addr_override ? get_reg32 (cpu, REG_CX) - 1 : (uint16_t) (cpu->regs[REG_CX] - 1);
            
            if (cpu->has_addr_override) {
                set_reg32 (cpu, REG_CX, new_cx);
            } else {
                cpu->regs[REG_CX] = (uint16_t) new_cx;
            }
            
            target = cpu_code_32 (cpu) ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
            
            if ((take = (new_cx != 0))) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE3: {                    /* JCXZ */
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            uint32_t target = cpu_code_32 (cpu) ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
            int take = cpu->has_addr_override ? get_reg32 (cpu, REG_CX) == 0 : cpu->regs[REG_CX] == 0;
            
            if (take) {
                cpu->ip = target;
            }
            
            break;
        
        }
        
        case 0xE4: {                    /* IN AL, imm8 */
        
            uint8_t port = fetch8 (cpu);
            set_reg8 (cpu, 0, io_read8 (cpu, port));
            
            break;
        
        }
        
        case 0xE5: {                    /* IN AX, imm8 */
        
            uint8_t port = fetch8 (cpu);
            cpu->regs[REG_AX] = io_read16 (cpu, port);
            
            break;
        
        }
        
        case 0xE6: {                    /* OUT imm8, AL */
        
            uint8_t port = fetch8 (cpu);
            io_write8 (cpu, port, get_reg8 (cpu, 0));
            
            break;
        
        }
        
        case 0xE7: {                    /* OUT imm8, AX */
        
            uint8_t port = fetch8 (cpu);
            io_write16 (cpu, port, cpu->regs[REG_AX]);
            
            break;
        
        }
        
        case 0xE8: {
        
            int32_t rel = cpu->has_operand_override ? (int32_t) fetch32 (cpu) : (int16_t) fetch16 (cpu);
            
            uint32_t ret = cpu->ip;
            uint32_t target = cpu->has_operand_override ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
            
            if (cpu->has_operand_override) {
                push32 (cpu, ret);
            } else {
                push16 (cpu, (uint16_t) ret);
            }
            
            cpu->ip = target;
            break;
        
        }
        
        case 0xE9: {
        
            int32_t rel = cpu->has_operand_override ? (int32_t) fetch32 (cpu) : (int16_t) fetch16 (cpu);
            cpu->ip = cpu->has_operand_override ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
            
            break;
        
        }
        
        case 0xEA: {                    /* JMP far ptr16:16 / ptr16:32 */
        
            uint32_t new_ip = cpu->has_operand_override ? fetch32 (cpu) : fetch16 (cpu);
            uint16_t new_cs = fetch16 (cpu);
            
            if (cpu_load_segment (cpu, 1, new_cs) != 0) {
                return -1;
            }
            
            cpu->ip = new_ip;
            break;
        
        }
        
        case 0xEB: {
        
            int8_t rel = (int8_t) fetch8 (cpu);
            
            cpu->ip = cpu_code_32 (cpu) ? cpu->ip + rel : (uint16_t) (cpu->ip + rel);
            break;
        
        }
        
        case 0xEC: {                    /* IN AL, DX */
        
            uint16_t port = cpu->regs[REG_DX];
            set_reg8 (cpu, 0, io_read8 (cpu, port));
            
            break;
        
        }
        
        case 0xED: {                    /* IN AX, DX */
        
            uint16_t port = cpu->regs[REG_DX];
            cpu->regs[REG_AX] = io_read16 (cpu, port);
            
            break;
        
        }
        
        case 0xEE: {                    /* OUT DX, AL */
        
            io_write8 (cpu, cpu->regs[REG_DX], get_reg8 (cpu, 0));
            break;
        
        }
        
        case 0xEF: {                    /* OUT DX, AX */
        
            io_write16 (cpu, cpu->regs[REG_DX], cpu->regs[REG_AX]);
            break;
        
        }
        
        case 0xF0:                      /* LOCK prefix */
        
            goto fetch_instruction;
        
        case 0xF2:                      /* REPNZ */
        
            cpu->rep_type = 2;
            goto fetch_instruction;
        
        case 0xF3:                      /* REP / REPE */
        
            cpu->rep_type = 1;
            goto fetch_instruction;
        
        case 0xF4:
        
            cpu->halted = 1;
            break;
        
        case 0xF6: /* Unary rm8 */      case 0xF7: {
        
            uint8_t modrm = fetch8 (cpu);
            int subop = (modrm >> 3) & 7;
            
            OperandRef rm = decode_rm_operand (cpu, modrm);
            
            if (op == 0xF7 && cpu->has_operand_override) {
            
                uint32_t value = operand_read32 (cpu, &rm);
                
                switch (subop) {
                
                    case 0:     case 1:                     /* TEST r/m32, imm32 */
                    
                        and32 (cpu, value, fetch32 (cpu));
                        break;
                    
                    case 2:             /* NOT r/m32 */
                    
                        operand_write32 (cpu, &rm, ~value);
                        break;
                    
                    case 3:             /* NEG r/m32 */
                    
                        operand_write32 (cpu, &rm, sub32 (cpu, 0, value));
                        break;
                    
                    case 4: {           /* MUL r/m32 (EDX:EAX = EAX * r/m32) */
                    
                        uint64_t product = (uint64_t) get_reg32 (cpu, REG_AX) * value;
                        
                        set_reg32 (cpu, REG_AX, (uint32_t) product);
                        set_reg32 (cpu, REG_DX, (uint32_t) (product >> 32));
                        
                        if (product >> 32) {
                            cpu->flags |= FLAG_CF | FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                        }
                        
                        break;
                    
                    }
                    
                    case 5: {           /* IMUL r/m32 (EDX:EAX = EAX * r/m32) */
                    
                        int64_t product = (int64_t) (int32_t) get_reg32 (cpu, REG_AX) * (int32_t) value;
                        
                        set_reg32 (cpu, REG_AX, (uint32_t) product);
                        set_reg32 (cpu, REG_DX, (uint32_t) ((uint64_t) product >> 32));
                        
                        if (product > INT32_MAX || product < INT32_MIN) {
                            cpu->flags |= FLAG_CF | FLAG_OF;
                        } else {
                            cpu->flags &= (uint16_t) ~(FLAG_CF | FLAG_OF);
                        }
                        
                        break;
                    
                    }
                    
                    case 6: {           /* DIV r/m32 (EDX:EAX / r/m32) */
                    
                        uint64_t dividend = ((uint64_t) get_reg32 (cpu, REG_DX) << 32) | get_reg32 (cpu, REG_AX);
                        uint64_t quotient = value ? dividend / value : 0;
                        
                        if (!value || quotient > UINT32_MAX) {
                        
                            if (cpu_divide_error (cpu, (uint32_t) old_ip) != 0) {
                                return -1;
                            }
                            
                            break;
                        
                        }
                        
                        set_reg32 (cpu, REG_AX, (uint32_t) quotient);
                        set_reg32 (cpu, REG_DX, (uint32_t) (dividend % value));
                        
                        break;
                    
                    }
                    
                    case 7: {           /* IDIV r/m32 (signed EDX:EAX / r/m32) */
                    
                        int64_t dividend = (int64_t) (((uint64_t) get_reg32 (cpu, REG_DX) << 32) | get_reg32 (cpu, REG_AX));
                        int32_t divisor = (int32_t) value;
                        int64_t quotient = divisor && !(dividend == INT64_MIN && divisor == -1) ? dividend / divisor : 0;
                        
                        if (!divisor || (dividend == INT64_MIN && divisor == -1) || quotient > INT32_MAX || quotient < INT32_MIN) {
                        
                            if (cpu_divide_error (cpu, (uint32_t) old_ip) != 0) {
                                return -1;
                            }
                            
                            break;
                        
                        }
                        
                        set_reg32 (cpu, REG_AX, (uint32_t) quotient);
                        set_reg32 (cpu, REG_DX, (uint32_t) (dividend % divisor));
                        
                        break;
                    
                    }
                
                }
                
                break;
            
            }
            
            if (op == 0xF6) {           /* 8-bit Operations */
            
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
                    
                    case 7: {           /* IDIV rm8 (Signed AX / rm8) */
                    
                        int16_t dividend = (int16_t) cpu->regs[REG_AX];
                        int8_t divisor = (int8_t) operand_read8 (cpu, &rm);
                        int32_t quotient, remainder;
                        
                        if (!divisor) {
                        
                            if (cpu_divide_error (cpu, (uint32_t) old_ip) != 0) {
                                return -1;
                            }
                            
                            break;
                        
                        }
                        
                        quotient = dividend / divisor;
                        remainder = dividend % divisor;
                        
                        if (quotient > 127 || quotient < -128) {
                        
                            if (cpu_divide_error (cpu, (uint32_t) old_ip) != 0) {
                                return -1;
                            }
                            
                            break;
                        
                        }
                        
                        set_reg8 (cpu, 0, (uint8_t) quotient);                  /* AL */
                        set_reg8 (cpu, 4, (uint8_t) remainder);                 /* AH */
                        
                        break;
                    
                    }
                    
                    default:
                    
                        fprintf (stderr, "F6 Subop /%d not implemented\n", subop);
                        return -1;
                
                }
            
            } else {                    /* 0xF7: 16-bit Operations */
            
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
        
        case 0xF8:                      /* CLC (Clear Carry Flag) */
        
            cpu->flags &= (uint16_t) ~FLAG_CF;
            break;
        
        case 0xF9:                      /* STC (Set Carry Flag) */
        
            cpu->flags |= FLAG_CF;
            break;
        
        case 0xF5:                      /* CMC (Complement Carry Flag) */
        
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
        
        case 0xFC:                      /* CLD (Clear Direction Flag) */
        
            cpu->flags &= (uint16_t) ~FLAG_DF;
            break;
        
        case 0xFD:                      /* STD (Set Direction Flag) */
        
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
            
                fprintf (stderr, "unhandled 0xFE extension /%d at %04X\n", subop, old_ip);
                return -1;
            
            }
            
            break;
        
        }
        
        default:
        
            fprintf (stderr, "unhandled opcode 0x%02X (%s) at %04X\n", op, opcode_name (op), old_ip);
            return -1;
    
    }
    
    if (trap_pending && !trap_inhibit && (cpu->flags & FLAG_TF) && !cpu->halted) {
    
        if (cpu->cr0 & 1) {
        
            if (cpu_protected_interrupt (cpu, 1) != 0) {
                return -1;
            }
        
        } else {
            handle_interrupt (cpu, 1);
        }
    
    }
    
    return 1;

}
