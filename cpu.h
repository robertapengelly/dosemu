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

typedef struct __attribute__ ((packed)) CPU8086 {

    uint16_t regs[8];                   /* General-purpose registers: AX, BX, CX, DX, SP, BP, SI, DI */
    uint16_t ip;                        /* Instruction pointer */
    uint16_t flags;                     /* Flags register */
    uint16_t cs, ds, es, fs, gs, ss;    /* Segment registers */
    
    uint8_t *mem;                       /* Memory */
    size_t mem_size;                    /* Memory size */
    
    int halted;                         /* Flag to indicate if the CPU is halted */
    
    uint16_t seg_override;
    uint16_t psp_seg;
    
    int has_seg_override;
    int has_addr_override;
    int has_operand_override;
    int rep_type;
    
    uint8_t port61, vga_status_flip;

} CPU8086;

uint32_t get_phys (uint16_t seg, uint16_t off);
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
