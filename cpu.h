/******************************************************************************
 * @file            cpu.h
 *****************************************************************************/
#ifndef     _CPU_H
#define     _CPU_H

#include    <stdint.h>
#include    <stddef.h>

enum {

    REG_AX = 0,
    REG_CX = 1,
    REG_DX = 2,
    REG_BX = 3,
    REG_SP = 4,
    REG_BP = 5,
    REG_SI = 6,
    REG_DI = 7

};

typedef struct __attribute__ ((packed)) PITCounter {

    uint8_t access, mode, bcd, control;
    uint8_t write_high, read_high, latch_active, status_latch_active;
    uint8_t null_count, status;
    
    uint16_t reload, write_low, latch;
    uint64_t started_ns;

} PITCounter;

typedef struct __attribute__ ((packed)) SegmentCache {

    uint32_t base, limit;
    uint16_t selector;
    uint8_t access, flags, valid, protected;

} SegmentCache;

typedef struct __attribute__ ((packed)) PIC8259 {

    uint8_t mask, offset, irr, isr;
    uint8_t init_step, icw4_required, single, read_isr;

} PIC8259;

typedef struct __attribute__ ((packed)) CPU8086 {

    uint16_t regs[8];                   /* General-purpose registers: AX, BX, CX, DX, SP, BP, SI, DI */
    uint16_t regs_high[8];              /* Upper halves of the 80386 general-purpose registers */
    
    uint32_t eflags_high;               /* EFLAGS bits 16-31 */
    uint32_t cr0, cr2, cr3;
    
    uint32_t gdtr_base, idtr_base;
    uint16_t gdtr_limit, idtr_limit;
    
    SegmentCache segments[6];           /* ES, CS, SS, DS, FS, GS */
    
    uint32_t ip;                        /* Instruction pointer */
    uint16_t flags;                     /* Flags register */
    
    uint16_t cs, ds, es, fs, gs, ss;    /* Segment registers */
    
    uint8_t *mem;                       /* Memory */
    size_t mem_size;                    /* Memory size */
    
    int halted;                         /* Flag to indicate if the CPU is halted */
    
    uint16_t seg_override;
    unsigned seg_override_index;
    
    uint16_t psp_seg;
    
    int has_seg_override;
    int has_addr_override;
    int has_operand_override;
    int rep_type;
    
    uint8_t kbd_typematic_pending, kbd_typematic_rate, kbd_ack_pending;
    uint8_t port61, port92, a20_enabled, vga_status_flip;
    
    PITCounter pit[3];
    PIC8259 pic[2];
    
    uint8_t crtc_index, crtc_registers[18];
    uint16_t crtc_base;

} CPU8086;

uint32_t get_phys (uint16_t seg, uint16_t off);

void cpu_crtc_set_cursor (CPU8086 *cpu, unsigned row, unsigned col, unsigned columns);
int cpu_step (CPU8086 *cpu);

void cpu_reset (CPU8086 *cpu, uint8_t *mem, size_t mem_size);
void cpu_dump (const CPU8086 *cpu);

uint16_t cpu_get_reg (const CPU8086 *cpu, int reg);
void cpu_set_reg (CPU8086 *cpu, int reg, uint16_t value);

void push16 (CPU8086 *cpu, uint16_t value);
uint16_t pop16 (CPU8086 *cpu);

uint8_t fetch8 (CPU8086 *cpu);
uint16_t fetch16 (CPU8086 *cpu);

uint8_t get_reg8 (const CPU8086 *cpu, int reg);
void set_reg8 (CPU8086 *cpu, int reg, uint8_t value);

#endif      /* _CPU_H */
