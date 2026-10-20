/******************************************************************************
 * @file            main.c
 *****************************************************************************/
#include    <ctype.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>

#if     defined (_WIN32)
# include       <windows.h>

# ifndef    ENABLE_VIRTUAL_TERMINAL_PROCESSING
#  define       ENABLE_VIRTUAL_TERMINAL_PROCESSING          0x0004
# endif

# ifndef    DISABLE_NEWLINE_AUTO_RETURN
#  define       DISABLE_NEWLINE_AUTO_RETURN                 0x0008
# endif
#endif

#include    "cpu.h"
#include    "loader.h"
#include    "interrupts.h"
#include    "heap.h"
#include    "memory.h"
#include    "psp.h"

#define     CONVENTIONAL_MEM_SIZE       (640UL << 10)
#define     EXE_LOAD_SEG                0x0060
#define     MEM_SIZE                    (1UL << 20)

#if     defined (_WIN32)
static HANDLE terminal_output_handle;
static DWORD terminal_output_mode;

static int terminal_output_mode_changed;

static void initialize_terminal_output (void) {

    DWORD mode, new_mode;
    terminal_output_handle = GetStdHandle (STD_OUTPUT_HANDLE);
    
    if (terminal_output_handle == NULL || terminal_output_handle == INVALID_HANDLE_VALUE || !GetConsoleMode (terminal_output_handle, &mode)) {
        return;
    }
    
    new_mode = mode | ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING | DISABLE_NEWLINE_AUTO_RETURN;
    
    if (new_mode != mode) {
    
        if (!SetConsoleMode (terminal_output_handle, new_mode)) {
        
            new_mode &= ~DISABLE_NEWLINE_AUTO_RETURN;
            
            if (!SetConsoleMode (terminal_output_handle, new_mode)) {
                return;
            }
        
        }
        
        terminal_output_mode = mode;
        terminal_output_mode_changed = 1;
    
    }

}
#endif

static void restore_terminal_cursor (void) {

#if     defined (_WIN32)
    DWORD mode;
    
    if (terminal_output_handle == NULL || terminal_output_handle == INVALID_HANDLE_VALUE || !GetConsoleMode (terminal_output_handle, &mode)) {
        return;
    }
    
    if (!(mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {
    
        CONSOLE_CURSOR_INFO info;
        fflush (stdout);
        
        if (GetConsoleCursorInfo (terminal_output_handle, &info)) {
        
            info.bVisible = TRUE;
            SetConsoleCursorInfo (terminal_output_handle, &info);
        
        }
    
    } else
#endif
    {
    
        fputs ("\033[?25h", stdout);
        fflush (stdout);
    
    }
    
#if     defined (_WIN32)
    if (terminal_output_mode_changed) {
        SetConsoleMode (terminal_output_handle, terminal_output_mode);
    }
#endif

}

static void usage (const char *argv0) {
    fprintf (stderr, "Usage: %s [--trace] program.com\n", argv0);
}

void init_psp (const char *path, CPU8086 *cpu, const char *args, uint16_t base_seg) {

    uint32_t env_base, p;
    uint16_t env_seg;
    
    cpu->psp_seg = base_seg - 0x10;
    
    env_seg = cpu->psp_seg - 0x10;
    env_base = (uint32_t) env_seg << 4;
    
    p = env_base;
    
    /* initial command interpreter path */
    for (const char *s = "COMSPEC="; *s; s++) {
        mem_write8 (cpu->mem, cpu->mem_size, p++, (uint8_t) *s);
    }
    
    for (const char *s = path; *s; s++) {
        mem_write8 (cpu->mem, cpu->mem_size, p++, (uint8_t) *s);
    }
    
    mem_write8 (cpu->mem, cpu->mem_size, p++, 0);
    mem_write8 (cpu->mem, cpu->mem_size, p++, 0);
    
    /* Count of extra strings following the environment = 1 */
    mem_write16 (cpu->mem, cpu->mem_size, p, 0x0001);
    p += 2;
    
    for (const char *s = path; *s; s++) {
    
        char ch = *s;
        
        if (ch == '/') {
            ch = '\\';
        }
        
        mem_write8 (cpu->mem, cpu->mem_size, p++, (uint8_t) toupper((int) ch));
    
    }
    
    mem_write8 (cpu->mem, cpu->mem_size, p++, 0);
    init_psp_contents (cpu, args, cpu->psp_seg, 0x0000, env_seg, DOS_MEMORY_END_SEG);
    
    /* Environment MCB */
    {
    
        uint32_t env_mcb_phys = (uint32_t) (env_seg - 1) << 4;
        
        /* paragraphs owned by environment block, excluding the MCB itself */
        uint16_t env_block_size = 0x000F;                   /* 15 paragraphs => 0x0040..0x004E */
        
        mem_write8 (cpu->mem, cpu->mem_size, env_mcb_phys + 0x00, 'M');
        
        mem_write16 (cpu->mem, cpu->mem_size, env_mcb_phys + 0x01, cpu->psp_seg);              /* owner = PSP */
        mem_write16 (cpu->mem, cpu->mem_size, env_mcb_phys + 0x03, env_block_size);
        
        for (int i = 5; i < 16; i++) {
            mem_write8 (cpu->mem, cpu->mem_size, env_mcb_phys + i, 0);
        }
    
    }
    
    /* PSP MCB */
    {
    
        uint32_t mcb_phys = (uint32_t) (cpu->psp_seg - 1) << 4;
        uint16_t block_size = (uint16_t) (DOS_MEMORY_END_SEG - cpu->psp_seg);
        
        mem_write8 (cpu->mem, cpu->mem_size, mcb_phys + 0x00, 'Z');
        
        mem_write16 (cpu->mem, cpu->mem_size, mcb_phys + 0x01, cpu->psp_seg);
        mem_write16 (cpu->mem, cpu->mem_size, mcb_phys + 0x03, block_size);
        
        for (int i = 5; i < 16; i++) {
            mem_write8 (cpu->mem, cpu->mem_size, mcb_phys + i, 0);
        }
    
    }
    
    cpu->ds = cpu->es = cpu->psp_seg;

}

int main (int argc, char **argv) {

    uint8_t *mem;
    CPU8086 cpu;
    
    const char *path = NULL;
    const char *dos_path;
    
    char cmd_tail[128] = {0};
    
    int trace = 0, rc;
    size_t loaded = 0;
    
    int offset = 0, len, i;
    unsigned long steps = 0;
    
    for (i = 1; i < argc; i++) {
    
        if (strcmp (argv[i], "--trace") == 0) {
        
            trace = 1;
            continue;
        
        }
        
        if (path == NULL) {
        
            path = argv[i];
            continue;
        
        }
        
        len = strlen (argv[i]);
        
        if (offset + len + 2 < 126) {
        
            strcat(cmd_tail, " ");
            strcat(cmd_tail, argv[i]);
            
            offset += len + 1;
        
        }
    
    }

    if (path == NULL) {
    
        usage (argv[0]);
        return 1;
    
    }

    mem = (uint8_t *) calloc ((MEM_SIZE), 1);
    
    if (mem == NULL) {
    
        fprintf (stderr, "Out of memory\n");
        return 1;
    
    }
    
    cpu_reset (&cpu, mem, MEM_SIZE);
    dos_path = dos_register_initial_program (path);
    
    if (load_mz_file (path, &cpu, EXE_LOAD_SEG << 4, (uint16_t) ((CONVENTIONAL_MEM_SIZE - ((uint32_t) (EXE_LOAD_SEG << 4) << 4)) / 16))) {
        init_psp (dos_path, &cpu, cmd_tail, EXE_LOAD_SEG);
    } else if (load_com_file (path, &cpu, mem, CONVENTIONAL_MEM_SIZE, EXE_LOAD_SEG, &loaded)) {
        init_psp (dos_path, &cpu, cmd_tail, EXE_LOAD_SEG);
    } else {
    
        fprintf (stderr, "Failed to load %s\n", path);
        
        free (mem);
        return 1;
    
    }
    
    if (atexit (restore_terminal_cursor) != 0) {
    
        fprintf (stderr, "Failed to register terminal cursor cleanup\n");
        
        free (mem);
        return 1;
    
    }
    
#if     defined (_WIN32)
    initialize_terminal_output ();
#endif
    
    /* init_dos_heap(&cpu, 0x2000); */
    init_ivt (&cpu);
    
    while (!cpu.halted) {
    
        if (trace) {
        
            printf ("step=%lu ", steps);
            cpu_dump (&cpu);
        
        }
        
        rc = cpu_step (&cpu);
        bios_video_refresh (&cpu);
        
        if (rc <= 0) {
        
            free (mem);
            return 1;
        
        }
        
        steps++;
    
    }
    
    free (mem);
    return 0;

}
