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

typedef struct __attribute__ ((packed)) DMA8237 {

    uint16_t address[4], count[4];
    uint16_t base_address[4], base_count[4];
    
    uint8_t page[4], mode[4];
    uint8_t command, request, mask, flipflop;

} DMA8237;

typedef struct __attribute__ ((packed)) CPU8086 {

    uint16_t regs[8];                   /* General-purpose registers: AX, BX, CX, DX, SP, BP, SI, DI */
    uint16_t regs_high[8];              /* Upper halves of the 80386 general-purpose registers */
    
    uint32_t eflags_high;               /* EFLAGS bits 16-31 */
    uint32_t cr0, cr2, cr3;
    
    uint32_t gdtr_base, idtr_base;
    uint16_t gdtr_limit, idtr_limit;
    
    uint16_t ldtr_selector, tr_selector;
    uint32_t ldtr_base, ldtr_limit, tr_base, tr_limit;
    
    uint8_t ldtr_valid, tr_valid;
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
    uint8_t kbd_output_port, kbd_output_port_pending, kbd_output_port_read_pending;
    
    uint8_t kbd_scancode, kbd_scancode_pending, kbd_release_scancode;
    uint64_t kbd_release_time;
    
    uint8_t cmos_index, cmos_regs[128];
    uint8_t port61, port92, a20_enabled, vga_status_flip;
    
    uint8_t sb_dsp_reset, sb_dsp_speaker;
    uint8_t sb_dsp_reply[4], sb_dsp_reply_head, sb_dsp_reply_count;
    uint8_t sb_dsp_command, sb_dsp_param_count, sb_dsp_param_index, sb_dsp_params[2];
    uint8_t sb_dsp_time_constant, sb_dsp_dma_active, sb_dsp_dma_auto, sb_dsp_dma_paused;
    
    uint8_t sb_dsp_irq_pending;
    uint32_t sb_dsp_block_length;
    uint64_t sb_dsp_next_block_ns;
    
    uint8_t opl_index, opl_timer1, opl_timer2, opl_control, opl_status;
    uint64_t opl_timer1_started_ns, opl_timer2_started_ns;
    
    uint64_t opl_clock_ns;
    uint8_t mpu_uart, mpu_ack_pending;
    
    PITCounter pit[3];
    PIC8259 pic[2];
    DMA8237 dma[2];
    
    uint64_t pit_irq_last;
    uint8_t irq_poll_count;
    
    uint8_t crtc_index, crtc_registers[25];
    uint16_t crtc_base;
    
    uint8_t vga_misc_output, vga_enable, vga_graphics_mode;
    uint8_t vga_seq_index, vga_seq_registers[5];
    uint8_t vga_gc_index, vga_gc_registers[9];
    uint8_t vga_attr_index, vga_attr_flipflop, vga_attr_registers[21];
    uint8_t vga_dac_mask, vga_dac_state, vga_dac_read_index, vga_dac_write_index;
    uint8_t vga_dac_read_component, vga_dac_write_component;
    uint8_t vga_dac_palette[256][3];

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
