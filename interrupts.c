/******************************************************************************
 * @file            interrupts.c
 *****************************************************************************/
#include    <ctype.h>
#include    <dirent.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <time.h>

#include    <sys/stat.h>
#include    <errno.h>
#include    <fcntl.h>

#if     defined (_WIN32)
# include       <conio.h>
# include       <direct.h>
# include       <io.h>
# include       <windows.h>

# ifndef    ENABLE_VIRTUAL_TERMINAL_PROCESSING
#  define       ENABLE_VIRTUAL_TERMINAL_PROCESSING          0x0004
# endif
#else
# include       <sys/ioctl.h>
# include       <sys/select.h>
# include       <sys/statvfs.h>
# include       <sys/time.h>
# include       <termios.h>
# include       <unistd.h>
#endif

#include    "cpu.h"
#include    "memory.h"
#include    "heap.h"
#include    "interrupts.h"
#include    "loader.h"
#include    "psp.h"

#if     defined (_WIN32)
static int host_lstat (const char *path, struct stat *st) {

    DWORD attributes = GetFileAttributesA (path);
    
    if (attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_REPARSE_POINT)) {
    
        /* A reparse point is neither a regular file nor a directory for lstat. */
        memset (st, 0, sizeof (*st));
        return 0;
    
    }
    
    return stat (path, st);

}
#else
# define       host_lstat               lstat
#endif

/* Interrupt flag masks */
#define     FLAG_CF                     0x0001
#define     FLAG_ZF                     0x0040
#define     FLAG_SF                     0x0080
#define     FLAG_IF                     0x0200

static uint8_t dos_verify_flag;

#define     MAX_DOS_HANDLES             20

static FILE *file_handles[MAX_DOS_HANDLES] = { NULL };
static unsigned file_handle_references[MAX_DOS_HANDLES];

/* Parent execution contexts retained while a DOS child is running. */
#define     MAX_DOS_EXEC_DEPTH          16

static CPU8086 dos_exec_parents[MAX_DOS_EXEC_DEPTH];
static unsigned dos_exec_depth;
static uint16_t dos_exec_child_psps[MAX_DOS_EXEC_DEPTH];

static uint16_t dos_last_return_code[MAX_DOS_EXEC_DEPTH + 1];
static int dos_return_code_pending[MAX_DOS_EXEC_DEPTH + 1];

static int bios_video_host_redraw = 1;

static int dos_exec_save_parent (CPU8086 *cpu) {

    if (dos_exec_depth >= MAX_DOS_EXEC_DEPTH) {
        return 0;
    }
    
    dos_exec_parents[dos_exec_depth++] = *cpu;
    dos_return_code_pending[dos_exec_depth] = 0;
    
    bios_video_host_redraw = 1;
    return 1;

}

static unsigned dos_return_slot (CPU8086 *cpu) {

    if (dos_exec_depth != 0 && cpu->psp_seg == dos_exec_child_psps[dos_exec_depth - 1]) {
        return dos_exec_depth - 1;
    }
    
    return dos_exec_depth;

}

static int is_valid_mcb (CPU8086 *cpu, uint16_t mcb_seg);

static void dos_exec_release_memory (CPU8086 *cpu) {

    uint16_t segment = 0x004F;
    
    while (is_valid_mcb (cpu, segment)) {
    
        DOS_MCB *mcb = (DOS_MCB *) &cpu->mem[(uint32_t) segment << 4];
        
        if (mcb->owner == cpu->psp_seg) {
            mcb->owner = 0;
        }
        
        while (mcb->type == 'M') {
        
            uint16_t next_segment = (uint16_t) (segment + 1 + mcb->size);
            DOS_MCB *next;
            
            if (!is_valid_mcb (cpu, next_segment)) {
                break;
            }
            
            next = (DOS_MCB *) &cpu->mem[(uint32_t) next_segment << 4];
            
            if (next->owner == cpu->psp_seg) {
                next->owner = 0;
            }
            
            if (mcb->owner != 0 || next->owner != 0) {
                break;
            }
            
            mcb->size = (uint16_t) (mcb->size + 1 + next->size);
            mcb->type = next->type;
        
        }
        
        if (mcb->type == 'Z') {
            break;
        }
        
        segment = (uint16_t) (segment + 1 + mcb->size);
    
    }

}

static void dos_close_process_handles (CPU8086 *cpu);

static void dos_terminate_process (CPU8086 *cpu) {

    /* AH=55h creates an in-memory child without pushing an EXEC frame. */
    if (dos_exec_depth != 0 && cpu->psp_seg != dos_exec_child_psps[dos_exec_depth - 1]) {
    
        uint32_t psp_base = (uint32_t) cpu->psp_seg << 4;
        uint16_t parent_psp = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x16);
        uint16_t term_ip = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x0A);
        uint16_t term_cs = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x0C);
        
        if (parent_psp != 0 && parent_psp != cpu->psp_seg && term_cs != DOS_IVT_STUB_SEG && (term_cs != 0 || term_ip != 0)) {
        
            dos_close_process_handles (cpu);
            dos_exec_release_memory (cpu);
            
            cpu->psp_seg = parent_psp;
            cpu->cs = term_cs;
            cpu->ip = term_ip;
            
            return;
        
        }
    
    }
    
    if (dos_exec_depth == 0) {
    
        if (mem_read16 (cpu->mem, cpu->mem_size, ((uint32_t) cpu->psp_seg << 4) + 0x16) == 0) {
        
            cpu->halted = 1;
            return;
        
        } else {
        
            uint32_t current_psp_base = (uint32_t) cpu->psp_seg << 4;
            uint16_t parent_psp = mem_read16 (cpu->mem, cpu->mem_size, current_psp_base + 0x16);
            
            uint16_t term_ip = mem_read16 (cpu->mem, cpu->mem_size, current_psp_base + 0x0A);
            uint16_t term_cs = mem_read16 (cpu->mem, cpu->mem_size, current_psp_base + 0x0C);
            
            if (parent_psp != 0 && parent_psp != cpu->psp_seg && term_cs != DOS_IVT_STUB_SEG && (term_cs != 0 || term_ip != 0)) {
            
                dos_close_process_handles (cpu);
                dos_exec_release_memory (cpu);
                
                cpu->psp_seg = parent_psp;
                cpu->cs = term_cs;
                cpu->ip = term_ip;
                
                return;
            
            }
            
            {
            
                /* DOS 1.x children return through the INT 22h address in their PSP. */
                uint16_t sp = cpu->regs[REG_SP];
                uint16_t caller_psp = mem_read16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, (uint16_t) (sp + 2)));
                
                uint32_t psp_base = (uint32_t) caller_psp << 4;
                
                uint16_t return_ip = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x0A);
                uint16_t return_cs = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x0C);
                
                if (caller_psp != cpu->psp_seg && return_cs != DOS_IVT_STUB_SEG && (return_cs != 0 || return_ip != 0)) {
                
                    mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, sp), return_ip);
                    mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, (uint16_t) (sp + 2)), return_cs);
                    
                    return;
                
                }
                
                /* DOS 2.x can create children through AH=55h without EXEC. */
                if (caller_psp == cpu->psp_seg) {
                
                    uint16_t parent_psp = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x16);
                    
                    uint16_t saved_sp = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x2E);
                    uint16_t saved_ss = mem_read16 (cpu->mem, cpu->mem_size, psp_base + 0x30);
                    
                    if (parent_psp != 0 && parent_psp != cpu->psp_seg && saved_ss != 0 && return_cs != DOS_IVT_STUB_SEG && (return_cs != 0 || return_ip != 0)) {
                    
                        dos_close_process_handles (cpu);
                        dos_exec_release_memory (cpu);
                        
                        cpu->psp_seg = parent_psp;
                        cpu->ss = saved_ss;
                        
                        cpu->regs[REG_SP] = saved_sp;
                        
                        cpu->cs = return_cs;
                        cpu->ip = return_ip;
                        
                        return;
                    
                    }
                
                }
            
            }
        
        }
        
        cpu->halted = 1;
        return;
    
    }
    
    dos_close_process_handles (cpu);
    dos_exec_release_memory (cpu);
    
    *cpu = dos_exec_parents[--dos_exec_depth];

}

/* The DOS handle indexes the PSP's JFT; each JFT byte selects an SFT slot. */
static int dos_get_sft (CPU8086 *cpu, uint16_t handle) {

    uint32_t base = (uint32_t) cpu->psp_seg << 4;
    uint8_t entry;
    
    if (handle >= MAX_DOS_HANDLES) {
        return -1;
    }
    
    entry = mem_read8 (cpu->mem, cpu->mem_size, base + 0x18 + handle);
    
    if (entry >= MAX_DOS_HANDLES || file_handles[entry] == NULL) {
        return -1;
    }
    
    return entry;

}

static FILE *dos_get_handle_file (CPU8086 *cpu, uint16_t handle) {

    int sft = dos_get_sft (cpu, handle);
    return sft < 0 ? NULL : file_handles[sft];

}

static void dos_set_jft (CPU8086 *cpu, uint16_t handle, uint8_t entry) {
    mem_write8 (cpu->mem, cpu->mem_size, ((uint32_t) cpu->psp_seg << 4) + 0x18 + handle, entry);
}

/* The first five DOS handle slots are reserved for the standard devices. */
static int get_free_handle (CPU8086 *cpu) {

    uint32_t base = (uint32_t) cpu->psp_seg << 4;
    int i;
    
    for (i = 0; i < MAX_DOS_HANDLES; i++) {
    
        if (mem_read8 (cpu->mem, cpu->mem_size, base + 0x18 + i) == 0xFF) {
            return i;
        }
    
    }
    
    return -1;

}

static int get_free_sft (void) {

    int i;
    
    for (i = 5; i < MAX_DOS_HANDLES; i++) {
        
        if (file_handles[i] == NULL) {
            return i;
        }
    
    }
    
    return -1;

}

static int dos_close_handle (CPU8086 *cpu, uint16_t handle) {

    int sft = dos_get_sft (cpu, handle);
    
    if (sft < 0) {
        return 0;
    }
    
    dos_set_jft (cpu, handle, 0xFF);
    
    if (file_handle_references[sft] > 0) {
        file_handle_references[sft]--;
    }
    
    if (sft >= 5 && file_handle_references[sft] == 0) {
    
        fclose (file_handles[sft]);
        file_handles[sft] = NULL;
    
    }
    
    return 1;

}

static void dos_close_process_handles (CPU8086 *cpu) {

    uint16_t handle;
    
    for (handle = 0; handle < MAX_DOS_HANDLES; handle++) {
        dos_close_handle (cpu, handle);
    }

}

static void dos_exec_cleanup_child (CPU8086 *cpu, uint16_t child_psp) {

    uint16_t previous_psp = cpu->psp_seg;
    cpu->psp_seg = child_psp;
    
    dos_close_process_handles (cpu);
    dos_exec_release_memory (cpu);
    
    cpu->psp_seg = previous_psp;

}

static void dos_inherit_process_handles (CPU8086 *cpu, uint16_t parent_psp, uint16_t child_psp) {

    uint32_t parent_base = (uint32_t) parent_psp << 4;
    uint32_t child_base = (uint32_t) child_psp << 4;
    
    unsigned handle;
    
    for (handle = 0; handle < MAX_DOS_HANDLES; handle++) {
    
        uint8_t sft = mem_read8 (cpu->mem, cpu->mem_size, parent_base + 0x18 + handle);
        
        if (sft >= MAX_DOS_HANDLES || file_handles[sft] == NULL) {
            sft = 0xFF;
        } else {
            file_handle_references[sft]++;
        }
        
        mem_write8 (cpu->mem, cpu->mem_size, child_base + 0x18 + handle, sft);
    
    }

}

static int is_valid_mcb (CPU8086 *cpu, uint16_t mcb_seg) {

    uint32_t phys;
    DOS_MCB *mcb;
    
    uint32_t end_phys;
    uint32_t next_seg;
    
    /* Arena starts at MCB 004F, data starts at 0050, ends before DOS_MEMORY_END_SEG */
    if (mcb_seg < 0x004F || mcb_seg >= DOS_MEMORY_END_SEG) {
        return 0;
    }
    
    phys = (uint32_t) mcb_seg << 4;
    
    if (phys + sizeof (DOS_MCB) > cpu->mem_size) {
        return 0;
    }
    
    mcb = (DOS_MCB *)&cpu->mem[phys];
    
    if (mcb->type != 'M' && mcb->type != 'Z') {
        return 0;
    }
    
    /**
     * size is in paragraphs of DATA, not including the MCB paragraph itself.
     * The next MCB is at mcb_seg + 1 + size.
     */
    next_seg = (uint32_t) mcb_seg + 1u + (uint32_t) mcb->size;
    
    /**
     * End of this block's data area in bytes.
     * 
     * This must stay within RAM and below DOS_MEMORY_END_SEG.
     */
    end_phys = ((uint32_t) (mcb_seg + 1u) << 4) + ((uint32_t) mcb->size << 4);
    
    if (end_phys > cpu->mem_size) {
        return 0;
    }
    
    if (next_seg > DOS_MEMORY_END_SEG) {
        return 0;
    }
    
    /* If this is not the last block, the next MCB header itself must fit. */
    if (mcb->type == 'M') {
    
        uint32_t next_phys = next_seg << 4;
        
        if (next_seg >= DOS_MEMORY_END_SEG) {
            return 0;
        }
        
        if (next_phys + sizeof (DOS_MCB) > cpu->mem_size) {
            return 0;
        }
    
    } else {
    
        /**
         * Last block should end exactly at DOS_MEMORY_END_SEG if your arena
         * is [0050, DOS_MEMORY_END_SEG). Relax this to <= if you want to
         * tolerate setup differences.
         */
        if (next_seg != DOS_MEMORY_END_SEG) {
            return 0;
        }
    
    }
    
    return 1;

}


static uint16_t dos_dta_seg = 0;
static uint16_t dos_dta_off = 0;

static void dos_get_dta (CPU8086 *cpu, uint16_t *seg, uint16_t *off) {

    if (dos_dta_seg == 0 && dos_dta_off == 0) {
    
        *seg = cpu->psp_seg;
        *off = 0x0080;
    
    } else {
    
        *seg = dos_dta_seg;
        *off = dos_dta_off;
    
    }

}

static const uint16_t cp437[128] = {

    0x00C7, 0x00FC, 0x00E9, 0x00E2, 0x00E4, 0x00E0, 0x00E5, 0x00E7,
    0x00EA, 0x00EB, 0x00E8, 0x00EF, 0x00EE, 0x00EC, 0x00C4, 0x00C5,
    0x00C9, 0x00E6, 0x00C6, 0x00F4, 0x00F6, 0x00F2, 0x00FB, 0x00F9,
    0x00FF, 0x00D6, 0x00DC, 0x00A2, 0x00A3, 0x00A5, 0x20A7, 0x0192,
    0x00E1, 0x00ED, 0x00F3, 0x00FA, 0x00F1, 0x00D1, 0x00AA, 0x00BA,
    0x00BF, 0x2310, 0x00AC, 0x00BD, 0x00BC, 0x00A1, 0x00AB, 0x00BB,
    0x2591, 0x2592, 0x2593, 0x2502, 0x2524, 0x2561, 0x2562, 0x2556,
    0x2555, 0x2563, 0x2551, 0x2557, 0x255D, 0x255C, 0x255B, 0x2510,
    0x2514, 0x2534, 0x252C, 0x251C, 0x2500, 0x253C, 0x255E, 0x255F,
    0x255A, 0x2554, 0x2569, 0x2566, 0x2560, 0x2550, 0x256C, 0x2567,
    0x2568, 0x2564, 0x2565, 0x2559, 0x2558, 0x2552, 0x2553, 0x256B,
    0x256A, 0x2518, 0x250C, 0x2588, 0x2584, 0x258C, 0x2590, 0x2580,
    0x03B1, 0x00DF, 0x0393, 0x03C0, 0x03A3, 0x03C3, 0x00B5, 0x03C4,
    0x03A6, 0x0398, 0x03A9, 0x03B4, 0x221E, 0x03C6, 0x03B5, 0x2229,
    0x2261, 0x00B1, 0x2265, 0x2264, 0x2320, 0x2321, 0x00F7, 0x2248,
    0x00B0, 0x2219, 0x00B7, 0x221A, 0x207F, 0x00B2, 0x25A0, 0x00A0

};

static uint16_t bios_video_codepoint (uint8_t ch) {

    static const uint16_t cp437_low[32] = {
    
        0x0020, 0x263A, 0x263B, 0x2665, 0x2666, 0x2663, 0x2660, 0x2022,
        0x25D8, 0x25CB, 0x25D9, 0x2642, 0x2640, 0x266A, 0x266B, 0x263C,
        0x25BA, 0x25C4, 0x2195, 0x203C, 0x00B6, 0x00A7, 0x25AC, 0x21A8,
        0x2191, 0x2193, 0x2192, 0x2190, 0x221F, 0x2194, 0x25B2, 0x25BC
    
    };
    
    if (ch < 0x20) {
        return cp437_low[ch];
    }
    
    if (ch == 0x7F) {
        return 0x2302;
    }
    
    return ch < 0x80 ? ch : cp437[ch - 0x80];

}

#if     defined (_WIN32)
typedef struct {

    int state;
    int private_mode;
    
    unsigned parameters[8];
    unsigned parameter_index;
    
    WORD default_attributes;
    int default_attributes_valid;
    
    COORD saved_position;
    int saved_position_valid;

} DOS_LEGACY_ANSI;

static DOS_LEGACY_ANSI dos_legacy_stdout;
static DOS_LEGACY_ANSI dos_legacy_stderr;

static HANDLE dos_windows_legacy_handle (FILE *out) {

    HANDLE handle;
    DWORD mode;
    
    if (out == stdout) {
        handle = GetStdHandle (STD_OUTPUT_HANDLE);
    } else if (out == stderr) {
        handle = GetStdHandle (STD_ERROR_HANDLE);
    } else {
        return NULL;
    }
    
    if (handle == NULL || handle == INVALID_HANDLE_VALUE || !GetConsoleMode (handle, &mode) || (mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {
        return NULL;
    }
    
    return handle;

}

static unsigned dos_legacy_parameter (const DOS_LEGACY_ANSI *ansi, unsigned index, unsigned default_value) {

    unsigned value = index <= ansi->parameter_index ? ansi->parameters[index] : 0;
    return value != 0 ? value : default_value;

}

static void dos_windows_clear_screen (HANDLE handle, WORD attribute) {

    CONSOLE_SCREEN_BUFFER_INFO info;
    
    COORD origin = { 0, 0 };
    COORD cursor, size;
    
    SMALL_RECT window;
    SHORT height;
    
    DWORD length, written;
    
    if (!GetConsoleScreenBufferInfo (handle, &info)) {
        return;
    }
    
    length = (DWORD) info.dwSize.X * (DWORD) info.dwSize.Y;
    
    FillConsoleOutputCharacterW (handle, L' ', length, origin, &written);
    FillConsoleOutputAttribute (handle, attribute, length, origin, &written);
    
    height = (SHORT) (info.srWindow.Bottom - info.srWindow.Top + 1);
    
    if (info.dwSize.Y <= height) {
        return;
    }
    
    /* Temporarily shrink the buffer to discard its scrollback. */
    window = info.srWindow;
    window.Top = 0;
    window.Bottom = (SHORT) (height - 1);
    
    cursor = info.dwCursorPosition;
    cursor.Y = (SHORT) (cursor.Y - info.srWindow.Top);
    
    if (cursor.Y < 0) {
        cursor.Y = 0;
    } else if (cursor.Y >= height) {
        cursor.Y = (SHORT) (height - 1);
    }
    
    if (!SetConsoleWindowInfo (handle, TRUE, &window)) {
        return;
    }
    
    if (!SetConsoleCursorPosition (handle, cursor)) {
    
        SetConsoleWindowInfo (handle, TRUE, &info.srWindow);
        return;
    
    }
    
    size = info.dwSize;
    size.Y = height;
    
    if (!SetConsoleScreenBufferSize (handle, size)) {
    
        SetConsoleWindowInfo (handle, TRUE, &info.srWindow);
        SetConsoleCursorPosition (handle, info.dwCursorPosition);
        return;
    
    }
    
    SetConsoleScreenBufferSize (handle, info.dwSize);

}

static void dos_legacy_clear_line (HANDLE handle, const CONSOLE_SCREEN_BUFFER_INFO *info, SHORT row, SHORT left, SHORT right) {

    COORD position;
    
    DWORD written;
    DWORD length;
    
    if (left > right) {
        return;
    }
    
    position.X = left;
    position.Y = row;
    
    length = (DWORD) (right - left + 1);
    
    FillConsoleOutputCharacterW (handle, L' ', length, position, &written);
    FillConsoleOutputAttribute (handle, info->wAttributes, length, position, &written);

}

static void dos_legacy_ansi_execute (HANDLE handle, DOS_LEGACY_ANSI *ansi, uint8_t command) {

    static const WORD colors[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };
    
    CONSOLE_SCREEN_BUFFER_INFO info;
    COORD position;
    
    unsigned amount, mode, i;
    int x, y;
    
    if (!GetConsoleScreenBufferInfo (handle, &info)) {
        return;
    }
    
    if (!ansi->default_attributes_valid) {
    
        ansi->default_attributes = info.wAttributes;
        ansi->default_attributes_valid = 1;
    
    }
    
    position = info.dwCursorPosition;
    
    x = position.X;
    y = position.Y;
    
    amount = dos_legacy_parameter (ansi, 0, 1);
    mode = ansi->parameters[0];
    
    if (ansi->private_mode) {
    
        if ((command == 'h' || command == 'l') && mode == 25) {
        
            CONSOLE_CURSOR_INFO cursor;
            
            if (GetConsoleCursorInfo (handle, &cursor)) {
            
                cursor.bVisible = command == 'h' ? TRUE : FALSE;
                SetConsoleCursorInfo (handle, &cursor);
            
            }
        
        }
        
        return;
    
    }
    
    switch (command) {
    
        case 'A':
        
            y -= (int) amount;
            break;
        
        case 'B':
        
            y += (int) amount;
            break;
        
        case 'C':
        
            x += (int) amount;
            break;
        
        case 'D':
        
            x -= (int) amount;
            break;
        
        case 'E':
        
            y += (int) amount;
            x = info.srWindow.Left;
            
            break;
        
        case 'F':
        
            y -= (int) amount;
            x = info.srWindow.Left;
            
            break;
        
        case 'G':
        
            x = info.srWindow.Left + (int) amount - 1;
            break;
        
        case 'd':
        
            y = info.srWindow.Top + (int) amount - 1;
            break;
        
        case 'H':
        case 'f':
        
            y = info.srWindow.Top + (int) amount - 1;
            x = info.srWindow.Left + (int) dos_legacy_parameter (ansi, 1, 1) - 1;
            
            break;
        
        case 's':
        
            ansi->saved_position = position;
            ansi->saved_position_valid = 1;
            
            return;
        
        case 'u':
        
            if (!ansi->saved_position_valid) {
                return;
            }
            
            x = ansi->saved_position.X;
            y = ansi->saved_position.Y;
            
            break;
        
        case 'J':
        
            if (mode == 2) {
            
                dos_windows_clear_screen (handle, info.wAttributes);
                
                position.X = 0;
                position.Y = 0;
                
                SetConsoleCursorPosition (handle, position);
                return;
            
            }
            
            if (mode <= 1) {
            
                SHORT row;
                
                for (row = info.srWindow.Top; row <= info.srWindow.Bottom; row++) {
                
                    SHORT left = info.srWindow.Left;
                    SHORT right = info.srWindow.Right;
                    
                    if (mode == 0 && row < position.Y) {
                        continue;
                    }
                    
                    if (mode == 1 && row > position.Y) {
                        continue;
                    }
                    
                    if (mode == 0 && row == position.Y) {
                        left = position.X;
                    }
                    
                    if (mode == 1 && row == position.Y) {
                        right = position.X;
                    }
                    
                    dos_legacy_clear_line (handle, &info, row, left, right);
                
                }
            
            }
            
            return;
        
        case 'K':
        
            if (mode == 0) {
                dos_legacy_clear_line (handle, &info, position.Y, position.X, info.srWindow.Right);
            } else if (mode == 1) {
                dos_legacy_clear_line (handle, &info, position.Y, info.srWindow.Left, position.X);
            } else if (mode == 2) {
                dos_legacy_clear_line (handle, &info, position.Y, info.srWindow.Left, info.srWindow.Right);
            }
            
            return;
        
        case 'm': {
        
            WORD attributes = info.wAttributes;
            
            for (i = 0; i <= ansi->parameter_index; i++) {
            
                unsigned code = ansi->parameters[i];
                
                if (code == 0) {
                    attributes = ansi->default_attributes;
                } else if (code == 1) {
                    attributes |= FOREGROUND_INTENSITY;
                } else if (code == 22) {
                    attributes &= ~FOREGROUND_INTENSITY;
                } else if (code >= 30 && code <= 37) {
                    attributes = (WORD) ((attributes & ~0x0007) | colors[code - 30]);
                } else if (code >= 40 && code <= 47) {
                    attributes = (WORD) ((attributes & ~0x0070) | (colors[code - 40] << 4));
                } else if (code >= 90 && code <= 97) {
                    attributes = (WORD) ((attributes & ~0x000F) | colors[code - 90] | FOREGROUND_INTENSITY);
                } else if (code >= 100 && code <= 107) {
                    attributes = (WORD) ((attributes & ~0x00F0) | (colors[code - 100] << 4) | BACKGROUND_INTENSITY);
                } else if (code == 39) {
                    attributes = (WORD) ((attributes & ~0x000F) | (ansi->default_attributes & 0x000F));
                } else if (code == 49) {
                    attributes = (WORD) ((attributes & ~0x00F0) | (ansi->default_attributes & 0x00F0));
                }
            
            }
            
            SetConsoleTextAttribute (handle, attributes);
            return;
        
        }
        
        default:
        
            return;
    
    }
    
    if (x < info.srWindow.Left) {
        x = info.srWindow.Left;
    }
    
    if (x > info.srWindow.Right) {
        x = info.srWindow.Right;
    }
    
    if (y < info.srWindow.Top) {
        y = info.srWindow.Top;
    }
    
    if (y > info.srWindow.Bottom) {
        y = info.srWindow.Bottom;
    }
    
    position.X = (SHORT) x;
    position.Y = (SHORT) y;
    
    SetConsoleCursorPosition (handle, position);

}

static int dos_windows_legacy_ansi (FILE *out, uint8_t ch) {

    HANDLE handle = dos_windows_legacy_handle (out);
    DOS_LEGACY_ANSI *ansi;
    
    if (handle == NULL) {
        return 0;
    }
    
    ansi = out == stdout ? &dos_legacy_stdout : &dos_legacy_stderr;
    
    if (ansi->state == 0) {
    
        if (ch != 0x1B) {
            return 0;
        }
        
        ansi->state = 1;
        return 1;
    
    }
    
    if (ansi->state == 1) {
    
        ansi->state = 0;
        
        if (ch == '[') {
        
            memset (ansi->parameters, 0, sizeof (ansi->parameters));
            
            ansi->parameter_index = 0;
            ansi->private_mode = 0;
            ansi->state = 2;
        
        } else if (ch == ']') {
            ansi->state = 3;
        } else if (ch == '7' || ch == '8') {
        
            fflush (out);
            
            if (ch == '7') {
            
                CONSOLE_SCREEN_BUFFER_INFO info;
                
                if (GetConsoleScreenBufferInfo (handle, &info)) {
                
                    ansi->saved_position = info.dwCursorPosition;
                    ansi->saved_position_valid = 1;
                
                }
            
            } else if (ansi->saved_position_valid) {
                SetConsoleCursorPosition (handle, ansi->saved_position);
            }
        
        } else if (ch == 0x1B) {
            ansi->state = 1;
        }
        
        return 1;
    
    }
    
    if (ansi->state == 3 || ansi->state == 4) {
    
        if (ch == '\a' || (ansi->state == 4 && ch == '\\')) {
            ansi->state = 0;
        } else if (ch == 0x1B) {
            ansi->state = 4;
        } else {
            ansi->state = 3;
        }
        
        return 1;
    
    }
    
    if (ch >= '0' && ch <= '9') {
    
        unsigned *parameter = &ansi->parameters[ansi->parameter_index];
        
        if (*parameter < 1000) {
            *parameter = *parameter * 10 + (ch - '0');
        }
        
        return 1;
    
    }
    
    if (ch == ';') {
    
        if (ansi->parameter_index < 7) {
            ansi->parameter_index++;
        }
        
        return 1;
    
    }
    
    if (ch == '?' && ansi->parameter_index == 0 && ansi->parameters[0] == 0) {
    
        ansi->private_mode = 1;
        return 1;
    
    }
    
    if (ch >= 0x40 && ch <= 0x7E) {
    
        ansi->state = 0;
        
        fflush (out);
        dos_legacy_ansi_execute (handle, ansi, ch);
        
        return 1;
    
    }
    
    ansi->state = ch == 0x1B ? 1 : 0;
    return 1;

}
#endif

static void dos_write_unicode_char (FILE *out, uint16_t codepoint) {

#if     defined (_WIN32)
    HANDLE handle = INVALID_HANDLE_VALUE;
    DWORD written;
    WCHAR wide = (WCHAR) codepoint;
    
    if (out == stdout) {
        handle = GetStdHandle (STD_OUTPUT_HANDLE);
    } else if (out == stderr) {
        handle = GetStdHandle (STD_ERROR_HANDLE);
    }
    
    if (handle != INVALID_HANDLE_VALUE && handle != NULL && GetFileType (handle) == FILE_TYPE_CHAR) {
    
        DWORD mode;
        
        if (GetConsoleMode (handle, &mode)) {
        
            fflush (out);
            
            if (WriteConsoleW (handle, &wide, 1, &written, NULL)) {
                return;
            }
        
        }
    
    }
#endif
    
    if (codepoint < 0x80) {
        fputc (codepoint, out);
    } else if (codepoint < 0x0800) {
    
        fputc (0xC0 | (codepoint >> 6), out);
        fputc (0x80 | (codepoint & 0x3F), out);
    
    } else {
    
        fputc (0xE0 | (codepoint >> 12), out);
        fputc (0x80 | ((codepoint >> 6) & 0x3F), out);
        fputc (0x80 | (codepoint & 0x3F), out);
    
    }

}

#if     defined (_WIN32)
static int bios_windows_legacy_console (void) {

    HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
    DWORD mode;
    
    return handle != NULL && handle != INVALID_HANDLE_VALUE && GetConsoleMode (handle, &mode) && !(mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING);

}

static int bios_windows_vt_console (void) {

    HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
    DWORD mode;
    
    return handle != NULL && handle != INVALID_HANDLE_VALUE && GetConsoleMode (handle, &mode) && (mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING);

}
#endif

static void dos_write_console_char (FILE *out, uint8_t ch) {

    static int ansi_clear_state = 0;
    uint16_t codepoint;
    
#if     defined (_WIN32)
    if (bios_windows_legacy_console () && dos_windows_legacy_ansi (out, ch)) {
        return;
    }
#endif
    
    if (ch < 0x80) {
    
        fputc (ch, out);
        
        if (out == stdout || out == stderr) {
        
            if (ansi_clear_state == 0 && ch == 0x1B) {
                ansi_clear_state = 1;
            } else if (ansi_clear_state == 1 && ch == '[') {
                ansi_clear_state = 2;
            } else if (ansi_clear_state == 2 && ch == '2') {
                ansi_clear_state = 3;
            } else if (ansi_clear_state == 3 && ch == 'J') {
            
                /* DOS ANSI.SYS homes the cursor after ESC[2J. */
                fputs ("\033[H", out);
                fputs ("\033[3J", out);
                
                ansi_clear_state = 0;
            
            } else {
                ansi_clear_state = (ch == 0x1B) ? 1 : 0;
            }
        
        }
        
        return;
    
    }
    
    if (out == stdout || out == stderr) {
        ansi_clear_state = 0;
    }
    
    codepoint = cp437[ch - 0x80];
    dos_write_unicode_char (out, codepoint);

}

static void dos_write_standard_output (CPU8086 *cpu, uint8_t ch) {

    FILE *out = dos_get_handle_file (cpu, 1);
    
    if (out == stdout || out == stderr) {
        dos_write_console_char (out, ch);
    } else if (out != NULL) {
        fputc (ch, out);
    }

}

static int host_extended_key = -1;
static uint16_t dos_last_error = 0;

#if     defined (_WIN32)
#ifndef     ENABLE_VIRTUAL_TERMINAL_INPUT
# define        ENABLE_VIRTUAL_TERMINAL_INPUT               0x0200
#endif

static int host_windows_key_pending = -1;
static int host_windows_key_repeat = -1;

static unsigned host_windows_key_repeats;
static uint8_t host_windows_shift_flags;

static HANDLE host_windows_console_input = INVALID_HANDLE_VALUE;
static DWORD host_windows_console_input_mode;

static int host_windows_console_input_mode_changed;

void host_windows_keyboard_initialize (void) {

    DWORD mode, new_mode;
    HANDLE handle;
    
    if (host_windows_console_input != INVALID_HANDLE_VALUE) {
        return;
    }
    
    handle = CreateFileA ("CONIN$", GENERIC_READ | GENERIC_WRITE,
                          FILE_SHARE_READ | FILE_SHARE_WRITE,
                          NULL, OPEN_EXISTING, 0, NULL);
    
    if (handle == INVALID_HANDLE_VALUE) {
        return;
    }
    
    if (!GetConsoleMode (handle, &mode)) {
    
        CloseHandle (handle);
        return;
    
    }
    
    new_mode = (mode | ENABLE_EXTENDED_FLAGS) & ~(ENABLE_QUICK_EDIT_MODE | ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT | ENABLE_PROCESSED_INPUT | ENABLE_VIRTUAL_TERMINAL_INPUT);
    
    if (new_mode != mode) {
    
        if (!SetConsoleMode (handle, new_mode)) {
        
            CloseHandle (handle);
            return;
        
        }
        
        host_windows_console_input_mode_changed = 1;
    
    }
    
    host_windows_console_input_mode = mode;
    host_windows_console_input = handle;

}

void host_windows_keyboard_shutdown (void) {

    if (host_windows_console_input == INVALID_HANDLE_VALUE) {
        return;
    }
    
    if (host_windows_console_input_mode_changed) {
    
        SetConsoleMode (host_windows_console_input, host_windows_console_input_mode);
        host_windows_console_input_mode_changed = 0;
    
    }
    
    CloseHandle (host_windows_console_input);
    host_windows_console_input = INVALID_HANDLE_VALUE;

}

static int host_windows_translate_key (const KEY_EVENT_RECORD *key) {

    WORD vk = key->wVirtualKeyCode;
    
    unsigned scan = key->wVirtualScanCode;
    unsigned ch = (unsigned char) key->uChar.AsciiChar;
    
    DWORD state = key->dwControlKeyState;
    
    int alt = (state & (LEFT_ALT_PRESSED | RIGHT_ALT_PRESSED)) != 0;
    int ctrl = (state & (LEFT_CTRL_PRESSED | RIGHT_CTRL_PRESSED)) != 0;
    int shift = (state & SHIFT_PRESSED) != 0;
    
    /* AltGr produces Right Alt + Left Ctrl on many keyboard layouts. */
    if ((state & (RIGHT_ALT_PRESSED | LEFT_CTRL_PRESSED)) == (RIGHT_ALT_PRESSED | LEFT_CTRL_PRESSED) && ch != 0) {
        alt = 0;
    }
    
    if (!key->bKeyDown) {
        return -1;
    }
    
    switch (vk) {
    
        case VK_SHIFT:
        case VK_CONTROL:
        case VK_MENU:
        case VK_CAPITAL:
        case VK_NUMLOCK:
        case VK_SCROLL:
        case VK_APPS:
        case VK_LWIN:
        case VK_RWIN:
        
            return -1;
    
    }
    
    if (vk >= VK_F1 && vk <= VK_F12) {
    
        scan = (vk <= VK_F10) ? 0x3B + (vk - VK_F1) : 0x85 + (vk - VK_F11);
        ch = 0;
        
        if (vk <= VK_F10) {
        
            if (alt) scan += 0x2D;
            else if (ctrl) scan += 0x23;
            else if (shift) scan += 0x19;
        
        } else {
        
            if (alt) scan += 6;
            else if (ctrl) scan += 4;
            else if (shift) scan += 2;
        
        }
        
        return (int) (scan << 8);
    
    }
    
    if (alt && vk >= 'A' && vk <= 'Z') {
        return (int) (scan << 8);
    }
    
    if (alt && vk >= '0' && vk <= '9') {
    
        scan = (vk == '0') ? 0x81 : 0x78 + (vk - '1');
        return (int) (scan << 8);
    
    }
    
    if (alt && vk == VK_OEM_MINUS) return 0x8200;
    if (alt && vk == VK_OEM_PLUS) return 0x8300;
    
    if (alt && ch == 0) {
    
        switch (vk) {
        
            case VK_HOME: scan = 0x97; break;
            case VK_UP: scan = 0x98; break;
            case VK_PRIOR: scan = 0x99; break;
            case VK_LEFT: scan = 0x9B; break;
            case VK_RIGHT: scan = 0x9D; break;
            case VK_END: scan = 0x9F; break;
            case VK_DOWN: scan = 0xA0; break;
            case VK_NEXT: scan = 0xA1; break;
            case VK_INSERT: scan = 0xA2; break;
            case VK_DELETE: scan = 0xA3; break;
        
        }
        
    } else if (ctrl && ch == 0) {
    
        switch (vk) {
        
            case VK_HOME: scan = 0x77; break;
            case VK_UP: scan = 0x8D; break;
            case VK_PRIOR: scan = 0x84; break;
            case VK_LEFT: scan = 0x73; break;
            case VK_RIGHT: scan = 0x74; break;
            case VK_END: scan = 0x75; break;
            case VK_DOWN: scan = 0x91; break;
            case VK_NEXT: scan = 0x76; break;
            case VK_INSERT: scan = 0x92; break;
            case VK_DELETE: scan = 0x93; break;
        
        }
    
    }
    
    if (ch != 0) {
        return (int) ((scan << 8) | ch);
    }
    
    switch (vk) {
    
        case VK_HOME:
        case VK_UP:
        case VK_PRIOR:
        case VK_LEFT:
        case VK_RIGHT:
        case VK_END:
        case VK_DOWN:
        case VK_NEXT:
        case VK_INSERT:
        case VK_DELETE:
        
            return (int) (scan << 8);
    
    }
    
    return -1;

}

static int host_windows_console_key (FILE *source, int wait) {

    HANDLE handle;
    
    DWORD read;
    INPUT_RECORD event;
    
    int key = -1;
    
    if (host_windows_key_pending >= 0) {
    
        key = host_windows_key_pending;
        host_windows_key_pending = -1;
        
        return key;
    
    }
    
    if (host_windows_key_repeats != 0) {
    
        host_windows_key_repeats--;
        return host_windows_key_repeat;
    
    }
    
    if (source == stdout || source == stderr) {
        handle = host_windows_console_input;
    } else {
        handle = (HANDLE) _get_osfhandle (_fileno (source));
    }
    
    if (handle == NULL || handle == INVALID_HANDLE_VALUE) {
        return -1;
    }
    
    for (;;) {
    
        if (!wait) {
        
            DWORD available;
            
            if (!GetNumberOfConsoleInputEvents (handle, &available) || available == 0) {
                break;
            }
        
        }
        
        if (!ReadConsoleInputA (handle, &event, 1, &read) || read != 1) {
            break;
        }
        
        if (event.EventType != KEY_EVENT) {
            continue;
        }
        
        host_windows_shift_flags = 0;
        
        if (event.Event.KeyEvent.dwControlKeyState & SHIFT_PRESSED) {
            host_windows_shift_flags |= 2;
        }
        
        if (event.Event.KeyEvent.dwControlKeyState & (LEFT_CTRL_PRESSED | RIGHT_CTRL_PRESSED)) {
            host_windows_shift_flags |= 4;
        }
        
        if (event.Event.KeyEvent.dwControlKeyState & (LEFT_ALT_PRESSED | RIGHT_ALT_PRESSED)) {
            host_windows_shift_flags |= 8;
        }
        
        if (event.Event.KeyEvent.dwControlKeyState & SCROLLLOCK_ON) host_windows_shift_flags |= 0x10;
        if (event.Event.KeyEvent.dwControlKeyState & NUMLOCK_ON) host_windows_shift_flags |= 0x20;
        if (event.Event.KeyEvent.dwControlKeyState & CAPSLOCK_ON) host_windows_shift_flags |= 0x40;
        
        key = host_windows_translate_key (&event.Event.KeyEvent);
        
        if (key >= 0) {
        
            if (event.Event.KeyEvent.wRepeatCount > 1) {
            
                host_windows_key_repeat = key;
                host_windows_key_repeats = event.Event.KeyEvent.wRepeatCount - 1;
            
            }
            
            break;
        
        }
    
    }
    
    return key;

}

static int host_windows_key_available (FILE *source) {

    if (host_windows_key_pending >= 0 || host_windows_key_repeats != 0) {
        return 1;
    }
    
    host_windows_key_pending = host_windows_console_key (source, 0);
    return host_windows_key_pending >= 0;

}

static int host_read_key_from (FILE *source) {

    HANDLE handle;
    DWORD mode;
    
    int key;
    
    if (host_extended_key >= 0) {
    
        key = host_extended_key;
        host_extended_key = -1;
        
        return key;
    
    }
    
    handle = (source == stdout || source == stderr) ? GetStdHandle (STD_INPUT_HANDLE) : (HANDLE) _get_osfhandle (_fileno (source));
    
    if (source == stdout || source == stderr || (handle != NULL && handle != INVALID_HANDLE_VALUE && GetConsoleMode (handle, &mode))) {
    
        key = host_windows_console_key (source, 1);
        
        if (key < 0) {
            return EOF;
        }
        
        if ((key & 0xFF) == 0) {
        
            host_extended_key = (key >> 8) & 0xFF;
            return 0;
        
        }
        
        return key & 0xFF;
    
    }
    
    return fgetc (source);

}

static int host_poll_key_from (FILE *source) {

    HANDLE handle;
    DWORD mode, available;
    
    int key;
    
    if (host_extended_key >= 0) {
    
        key = host_extended_key;
        host_extended_key = -1;
        
        return key;
    
    }
    
    handle = (source == stdout || source == stderr) ? GetStdHandle (STD_INPUT_HANDLE) : (HANDLE) _get_osfhandle (_fileno (source));
    
    if (source == stdout || source == stderr || (handle != NULL && handle != INVALID_HANDLE_VALUE && GetConsoleMode (handle, &mode))) {
    
        key = host_windows_console_key (source, 0);
        
        if (key < 0) {
            return EOF;
        }
        
        if ((key & 0xFF) == 0) {
        
            host_extended_key = (key >> 8) & 0xFF;
            return 0;
        
        }
        
        return key & 0xFF;
    
    }
    
    if (handle == NULL || handle == INVALID_HANDLE_VALUE) {
        return EOF;
    }
    
    if (GetFileType (handle) == FILE_TYPE_PIPE) {
    
        available = 0;
        
        if (PeekNamedPipe (handle, NULL, 0, NULL, &available, NULL) && available != 0) {
            return fgetc (source);
        }
    
    } else if (GetFileType (handle) == FILE_TYPE_DISK) {
        return fgetc (source);
    }
    
    clearerr (source);
    return EOF;

}
#else
static int host_alt_scan (unsigned char key) {

    static const uint8_t letter_scans[26] = {
    
        0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23, 0x17,
        0x24, 0x25, 0x26, 0x32, 0x31, 0x18, 0x19, 0x10, 0x13,
        0x1F, 0x14, 0x16, 0x2F, 0x11, 0x2D, 0x15, 0x2C
    
    };
    
    if (key >= 'A' && key <= 'Z') {
        key = (unsigned char) (key - 'A' + 'a');
    }
    
    if (key < 'a' || key > 'z') {
        return 0;
    }
    
    return letter_scans[key - 'a'];

}

static int host_function_scan (int scan, unsigned modifier) {

    if (modifier == 3) {
    
        if (scan >= 0x3B && scan <= 0x44) {
            return scan + 0x2D;
        }
        
        if (scan == 0x85 || scan == 0x86) {
            return scan + 6;
        }
    
    } else if (modifier == 5) {
    
        if (scan >= 0x3B && scan <= 0x44) {
            return scan + 0x23;
        }
        
        if (scan == 0x85 || scan == 0x86) {
            return scan + 4;
        }
        
        switch (scan) {
        
            case 0x47: return 0x77;     /* Ctrl+Home */
            case 0x48: return 0x8D;     /* Ctrl+Up */
            case 0x49: return 0x84;     /* Ctrl+Page Up */
            case 0x4B: return 0x73;     /* Ctrl+Left */
            case 0x4D: return 0x74;     /* Ctrl+Right */
            case 0x4F: return 0x75;     /* Ctrl+End */
            case 0x50: return 0x91;     /* Ctrl+Down */
            case 0x51: return 0x76;     /* Ctrl+Page Down */
            case 0x52: return 0x92;     /* Ctrl+Insert */
            case 0x53: return 0x93;     /* Ctrl+Delete */
        
        }
    
    }
    
    return scan;

}

static int host_control_scan (int key) {

    if (key >= 1 && key <= 26) {
    
        if (key == 8) return 0x0E;      /* Backspace and Ctrl+H are indistinguishable. */
        if (key == 9) return 0x0F;      /* Tab and Ctrl+I are indistinguishable. */
        if (key == 13) return 0x1C;     /* Enter and Ctrl+M are indistinguishable. */
        
        return host_alt_scan ((unsigned char) ('a' + key - 1));
    
    }
    
    return 0;

}

static int host_read_key_from (FILE *source) {

    struct termios oldt, newt;
    
    FILE *in = source;
    int fd, ch;
    
    unsigned char byte, next;
    int key, scan = 0;
    
    struct timeval timeout;
    fd_set ready;
    
    if (host_extended_key >= 0) {
    
        ch = host_extended_key;
        host_extended_key = -1;
        
        return ch;
    
    }
    
    /* DOS programs can redirect STDIN, then reopen CON via handle duplication. */
    if (source == stdout || source == stderr) {
    
        in = fopen ("/dev/tty", "rb");
        
        if (in == NULL) {
            return EOF;
        }
        
        setvbuf (in, NULL, _IONBF, 0);
    
    }
    
    fd = fileno (in);
    
    if (tcgetattr (fd, &oldt) != 0) {
        ch = fgetc (in);
    } else {
    
        newt = oldt;
        
        newt.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);
        newt.c_iflag &= ~(IXON | ICRNL);
        
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        
        if (tcsetattr (fd, TCSANOW, &newt) != 0) {
            ch = fgetc (in);
        } else {
        
            ch = read (fd, &byte, 1) == 1 ? byte : EOF;
            
            if (ch == 0x7F) {
                ch = 0x08;
            }
            
            if (ch == 0x1B) {
                
                FD_ZERO (&ready);
                FD_SET (fd, &ready);
                
                timeout.tv_sec = 0;
                timeout.tv_usec = 100000;
                
                if (select (fd + 1, &ready, NULL, NULL, &timeout) > 0 && read (fd, &next, 1) == 1) {
                
                    if (next == '[' || next == 'O') {
                    
                        unsigned param = 0, modifier = 1;
                        int have_modifier = 0;
                        
                        for (int i = 0; i < 8; i++) {
                        
                            FD_ZERO (&ready);
                            FD_SET (fd, &ready);
                            
                            timeout.tv_sec = 0;
                            timeout.tv_usec = 100000;
                            
                            if (select (fd + 1, &ready, NULL, NULL, &timeout) <= 0 || read (fd, &byte, 1) != 1) {
                                break;
                            }
                            
                            key = byte;
                            
                            if (key >= '0' && key <= '9') {
                            
                                if (have_modifier) {
                                    modifier = modifier * 10 + (unsigned) (key - '0');
                                } else {
                                    param = param * 10 + (unsigned) (key - '0');
                                }
                                
                                continue;
                            
                            }
                            
                            if (key == ';') {
                            
                                have_modifier = 1;
                                modifier = 0;
                                
                                continue;
                            
                            }
                            
                            if (key == 'A') scan = 0x48;
                            if (key == 'B') scan = 0x50;
                            if (key == 'C') scan = 0x4D;
                            if (key == 'D') scan = 0x4B;
                            if (key == 'H') scan = 0x47;
                            if (key == 'F') scan = 0x4F;
                            
                            if (next == 'O' || (next == '[' && param == 1)) {
                            
                                if (key == 'P') scan = 0x3B;
                                if (key == 'Q') scan = 0x3C;
                                if (key == 'R') scan = 0x3D;
                                if (key == 'S') scan = 0x3E;
                                
                                if (next == 'O' && !have_modifier && param == 3) {
                                    modifier = 3;
                                }
                            
                            }
                            
                            if (key == '~') {
                            
                                switch (param) {
                                
                                    case 2: scan = 0x52; break;
                                    case 3: scan = 0x53; break;
                                    case 5: scan = 0x49; break;
                                    case 6: scan = 0x51; break;
                                    case 15: scan = 0x3F; break;
                                    case 17: scan = 0x40; break;
                                    case 18: scan = 0x41; break;
                                    case 19: scan = 0x42; break;
                                    case 20: scan = 0x43; break;
                                    case 21: scan = 0x44; break;
                                    case 23: scan = 0x85; break;
                                    case 24: scan = 0x86; break;
                                
                                }
                            
                            }
                            
                            scan = host_function_scan (scan, modifier);
                            break;
                        
                        }
                        
                        if (scan != 0) {
                        
                            host_extended_key = scan;
                            ch = 0;
                        
                        }
                    
                    } else if ((scan = host_alt_scan (next)) != 0) {
                    
                        host_extended_key = scan;
                        ch = 0;
                    
                    }
                
                }
            
            }
            
            tcsetattr (fd, TCSANOW, &oldt);
        
        }
    
    }
    
    if (in != source) {
        fclose (in);
    }
    
    return ch;

}

static int host_key_available_from (FILE *source);

static int host_poll_key_from (FILE *source) {

    if (!host_key_available_from (source)) {
        return EOF;
    }
    
    return host_read_key_from (source);

}
#endif

static int host_key_available_from (FILE *source) {

    if (host_extended_key >= 0) {
        return 1;
    }
    
#if     defined (_WIN32)
    HANDLE handle;
    DWORD mode, available;
    
    int ch;
    
    if (source == stdout || source == stderr) {
        return host_windows_key_available (source);
    }
    
    handle = (HANDLE) _get_osfhandle (_fileno (source));
    
    if (handle == NULL || handle == INVALID_HANDLE_VALUE) {
        return 0;
    }
    
    if (GetConsoleMode (handle, &mode)) {
        return host_windows_key_available (source);
    }
    
    if (GetFileType (handle) == FILE_TYPE_PIPE) {
    
        available = 0;
        return PeekNamedPipe (handle, NULL, 0, NULL, &available, NULL) && available != 0;
    
    }
    
    ch = fgetc (source);
    
    if (ch == EOF) {
    
        clearerr (source);
        return 0;
    
    }
    
    ungetc (ch, source);
    return 1;
#else
    struct termios oldt, newt;
    struct timeval timeout;
    
    struct stat st;
    fd_set ready;
    
    FILE *in = source;
    int fd, old_flags, changed = 0, available = 0;
    
    if (source == stdout || source == stderr) {
    
        in = fopen ("/dev/tty", "rb");
        
        if (in == NULL) {
            return 0;
        }
    
    }
    
    fd = fileno (in);
    
    if (fd >= 0 && fstat (fd, &st) == 0 && S_ISREG (st.st_mode)) {
    
        int ch = fgetc (in);
        
        if (ch != EOF) {
        
            ungetc (ch, in);
            available = 1;
        
        } else {
            clearerr (in);
        }
    
    } else if (fd >= 0 && tcgetattr (fd, &oldt) == 0) {
    
        newt = oldt;
        
        newt.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);
        newt.c_iflag &= ~(IXON | ICRNL);
        
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        
        changed = tcsetattr (fd, TCSANOW, &newt) == 0;
        
        FD_ZERO (&ready);
        FD_SET (fd, &ready);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        
        available = select (fd + 1, &ready, NULL, NULL, &timeout) > 0;
        
        if (changed) {
            tcsetattr (fd, TCSANOW, &oldt);
        }
    
    } else if (fd >= 0) {
    
        old_flags = fcntl (fd, F_GETFL, 0);
        
        if (old_flags >= 0 && fcntl (fd, F_SETFL, old_flags | O_NONBLOCK) == 0) {
        
            int ch = fgetc (in);
            
            if (ch != EOF) {
            
                ungetc (ch, in);
                available = 1;
            
            } else {
                clearerr (in);
            }
            
            fcntl (fd, F_SETFL, old_flags);
        
        }
    
    }
    
    if (in != source) {
        fclose (in);
    }
    
    return available;
#endif

}

static int host_read_key (CPU8086 *cpu) {

    FILE *source;
    
#if     defined (_WIN32)
    bios_video_refresh (cpu);
#endif
    
    source = dos_get_handle_file (cpu, 0);
    return host_read_key_from (source ? source : stdin);

}

/* BIOS keyboard input returns ASCII in AL and the scan code in AH. */
static int bios_key_pending = -1;

static int bios_read_key (CPU8086 *cpu, int wait) {

    int ch;
    
    if (bios_key_pending >= 0) {
    
        ch = bios_key_pending;
        bios_key_pending = -1;
        
        return ch;
    
    }
    
#if     defined (_WIN32)
    if (wait) {
        bios_video_refresh (cpu);
    }
    
    ch = host_windows_console_key (stdout, wait);
    mem_write8 (cpu->mem, cpu->mem_size, get_phys (0x0040, 0x0017), host_windows_shift_flags);
#else
    FILE *source = dos_get_handle_file (cpu, 0);
    struct termios oldt, newt;
    
    FILE *in = source ? source : stdin;
    int fd, next;
    
    unsigned char byte;
    fd_set ready;
    
    struct timeval timeout;
    int changed = 0, extended = 0, scan = 0;
    
    if (in == stdout || in == stderr) {
    
        in = fopen ("/dev/tty", "rb");
        
        if (in == NULL) {
            return -1;
        }
    
    }
    
    fd = fileno (in);
    
    if (tcgetattr (fd, &oldt) == 0) {
    
        newt = oldt;
        
        newt.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);
        newt.c_iflag &= ~(IXON | ICRNL);
        
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        
        changed = tcsetattr (fd, TCSANOW, &newt) == 0;
    
    }
    
    if (!wait) {
    
        FD_ZERO (&ready);
        FD_SET (fd, &ready);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        
        if (select (fd + 1, &ready, NULL, NULL, &timeout) <= 0) {
        
            ch = -1;
            goto bios_read_done;
        
        }
    
    }
    
    if (read (fd, &byte, 1) != 1) {
    
        ch = -1;
        goto bios_read_done;
    
    }
    
    ch = byte;
    
    if (ch == 0x7F) {
        ch = 0x08;
    }
    
    if (ch == 0x1B) {
    
        FD_ZERO (&ready);
        FD_SET (fd, &ready);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        
        if (select (fd + 1, &ready, NULL, NULL, &timeout) > 0 && read (fd, &byte, 1) == 1) {
        
            next = byte;
            
            if (next == '[' || next == 'O') {
            
                unsigned param = 0, modifier = 1;
                int have_modifier = 0;
                
                for (int i = 0; i < 8; i++) {
                 
                    FD_ZERO (&ready);
                    FD_SET (fd, &ready);
                    
                    timeout.tv_sec = 0;
                    timeout.tv_usec = 100000;
                    
                    if (select (fd + 1, &ready, NULL, NULL, &timeout) <= 0 || read (fd, &byte, 1) != 1) {
                        break;
                    }
                    
                    if (byte >= '0' && byte <= '9') {
                    
                        if (have_modifier) {
                            modifier = modifier * 10 + (unsigned) (byte - '0');
                        } else {
                            param = param * 10 + (unsigned) (byte - '0');
                        }
                        
                        continue;
                    
                    }
                    
                    if (byte == ';') {
                    
                        have_modifier = 1;
                        modifier = 0;
                        
                        continue;
                    
                    }
                    
                    switch (byte) {
                    
                        case 'A': scan = 0x48; break;
                        case 'B': scan = 0x50; break;
                        case 'C': scan = 0x4D; break;
                        case 'D': scan = 0x4B; break;
                        case 'H': scan = 0x47; break;
                        case 'F': scan = 0x4F; break;
                    
                    }
                    
                    if (next == 'O' || (next == '[' && param == 1)) {
                    
                        switch (byte) {
                        
                            case 'P': scan = 0x3B; break;
                            case 'Q': scan = 0x3C; break;
                            case 'R': scan = 0x3D; break;
                            case 'S': scan = 0x3E; break;
                        
                        }
                        
                        if (next == 'O' && !have_modifier && param == 3) {
                            modifier = 3;
                        }
                    
                    }
                    
                    if (byte == '~') {
                    
                        switch (param) {
                        
                            case 2: scan = 0x52; break;
                            case 3: scan = 0x53; break;
                            case 5: scan = 0x49; break;
                            case 6: scan = 0x51; break;
                            case 15: scan = 0x3F; break;
                            case 17: scan = 0x40; break;
                            case 18: scan = 0x41; break;
                            case 19: scan = 0x42; break;
                            case 20: scan = 0x43; break;
                            case 21: scan = 0x44; break;
                            case 23: scan = 0x85; break;
                            case 24: scan = 0x86; break;
                        
                        }
                    
                    }
                    
                    scan = host_function_scan (scan, modifier);
                    
                    if (scan != 0) {
                        extended = 1;
                    }
                    
                    break;
                
                }
            
            } else if ((scan = host_alt_scan ((unsigned char) next)) != 0) {
                extended = 1;
            }
        
        }
    
    }
    
    if (extended) {
        ch = scan << 8;
    } else if (changed && ch > 0 && ch <= 26) {
        ch |= host_control_scan (ch) << 8;
    }
    
bios_read_done:
    
    if (changed) {
        tcsetattr (fd, TCSANOW, &oldt);
    }
    
    if (in != source && in != stdin) {
        fclose (in);
    }
#endif
    
    return ch;

}

/* Optional CHDIR path extension; AH=47h keeps its 64-byte output limit. */
static int dos_long_paths = 0;

#define     DOS_DIRECTORY_PATH_MAX      64
static char dos_current_dir[260] = "";

/* Extended BPB information for the emulated C: volume. */
static uint32_t dos_volume_serial = 0x80860886;

static uint8_t dos_volume_label[11] = { 'E', 'M', 'U', '8', '0', '8', '6', ' ', ' ', ' ', ' ' };
static uint8_t dos_volume_fs[8] = { 'F', 'A', 'T', '1', '6', ' ', ' ', ' ' };

static void dos_set_error (CPU8086 *cpu, uint16_t err) {

    dos_last_error = err;
    
    cpu->regs[REG_AX] = err;
    cpu->flags |= FLAG_CF;

}

static uint8_t dos_allocation_strategy;

static uint16_t dos_get_version (void) {

    uint16_t ver = 0x0005;
    const char *env_p;
    
    if ((env_p = getenv ("DOS_VER"))) {
    
        if (!strchr (env_p, '.') && env_p[0] == '0') {
        
            if (tolower ((int) env_p[1]) == 'x') {
                ver = (uint16_t) strtoul (env_p, 0, 16);
            } else if (env_p[1] >= '0' && env_p[1] <= '7') {
                ver = (uint16_t) strtoul (env_p, 0, 8);
            } else {
                ver = (uint16_t) strtoul (env_p, 0, 10);
            }
        
        } else {
        
            unsigned long major, minor;
            char *end_p;
            
            major = strtoul (env_p, &end_p, 10);
            minor = 0;
            
            if (*end_p == '.') {
                minor = strtoul (end_p + 1, 0, 10);
            }
            
            ver = (uint16_t) ((minor << 8) | major);
        
        }
    
    }
    
    return ver;

}

static void dos_allocate_memory (CPU8086 *cpu, uint16_t requested, uint16_t owner, uint16_t *allocated_seg, uint16_t *allocated_size) {

    uint16_t current_seg = 0x004F;
    
    uint16_t selected_seg = 0;
    uint16_t selected_size = 0;
    
    uint16_t max_free = 0;
    uint8_t strategy = dos_allocation_strategy;
    
    if ((dos_get_version () & 0xFF) >= 5) {
    
        /* No UMB arena is installed. High-only allocation cannot succeed. */
        if ((strategy & 0xC0) == 0x40) {
        
            dos_set_error (cpu, 0x0008);
            
            cpu->regs[REG_BX] = 0;
            return;
        
        }
        
        strategy &= 0x03;
    
    } else if (strategy > 2) {
    
        /* DOS 3.x/4.x accepts any BL value; values above 2 mean last fit. */
        strategy = 2;
    
    }
    
    while (1) {
    
        uint32_t phys = (uint32_t) current_seg << 4;
        DOS_MCB *mcb;
        
        if (phys + sizeof (DOS_MCB) > cpu->mem_size || !is_valid_mcb (cpu, current_seg)) {
        
            dos_set_error (cpu, 0x0009);
            return;
        
        }
        
        mcb = (DOS_MCB *) &cpu->mem[phys];
        
        if (mcb->owner == 0) {
        
            if (mcb->size > max_free) {
                max_free = mcb->size;
            }
            
            if (mcb->size >= requested) {
            
                if (selected_seg == 0 || strategy == 2 || (strategy == 1 && mcb->size < selected_size)) {
                
                    selected_seg = current_seg;
                    selected_size = mcb->size;
                
                }
            
            }
        
        }
        
        if (mcb->type == 'Z') {
            break;
        }
        
        {
        
            uint16_t next_seg = (uint16_t) (current_seg + 1 + mcb->size);
            
            if (!is_valid_mcb (cpu, next_seg)) {
            
                dos_set_error (cpu, 0x0009);
                return;
            
            }
            
            current_seg = next_seg;
        
        }
    
    }
    
    if (selected_seg == 0) {
    
        dos_set_error (cpu, 0x0008);
        cpu->regs[REG_BX] = max_free;
    
    }
    
    {
    
        DOS_MCB *mcb = (DOS_MCB *) &cpu->mem[(uint32_t) selected_seg << 4];
        uint16_t allocated_mcb_seg = selected_seg;
        
        if (strategy == 2 && mcb->size > requested) {
        
            uint16_t old_size = mcb->size;
            uint8_t old_type = mcb->type;
            
            uint16_t high_seg = (uint16_t) (selected_seg + old_size - requested);
            DOS_MCB *high = (DOS_MCB *) &cpu->mem[(uint32_t) high_seg << 4];
            
            high->type = old_type;
            high->owner = 0;
            high->size = requested;
            
            mcb->type = 'M';
            mcb->size = (uint16_t) (old_size - requested - 1);
            
            allocated_mcb_seg = high_seg;
        
        } else if (mcb->size > requested + 1u) {
        
            uint16_t old_size = mcb->size;
            
            uint8_t old_type = mcb->type;
            uint16_t next_seg = (uint16_t) (selected_seg + 1 + requested);
            
            DOS_MCB *next_mcb = (DOS_MCB *) &cpu->mem[(uint32_t) next_seg << 4];
            
            next_mcb->type = old_type;
            next_mcb->owner = 0;
            next_mcb->size = (uint16_t) (old_size - requested - 1);
            
            mcb->type = 'M';
            mcb->size = requested;
        
        }
        
        if (!is_valid_mcb (cpu, selected_seg) || !is_valid_mcb (cpu, allocated_mcb_seg)) {
        
            dos_set_error (cpu, 0x0009);
            return;
        
        }
        
        mcb = (DOS_MCB *) &cpu->mem[(uint32_t) allocated_mcb_seg << 4];
        mcb->owner = owner ? owner : (uint16_t) (allocated_mcb_seg + 1);
        
        if (allocated_seg != NULL) {
            *allocated_seg = (uint16_t) (allocated_mcb_seg + 1);
        }
        
        if (allocated_size != NULL) {
            *allocated_size = mcb->size;
        }
        
        cpu->regs[REG_AX] = (uint16_t) (allocated_mcb_seg + 1);
        cpu->flags &= (uint16_t) ~FLAG_CF;
    
    }
    
}

static int dos_environment_paragraphs (CPU8086 *cpu, uint16_t source_env, uint16_t *paragraphs) {

    uint32_t base = (uint32_t) source_env << 4;
    uint32_t end, position;
    
    uint16_t count;
    unsigned i;
    
    if (source_env == 0 || base >= cpu->mem_size) {
        return 0;
    }
    
    if (is_valid_mcb (cpu, (uint16_t) (source_env - 1))) {
    
        *paragraphs = ((DOS_MCB *) &cpu->mem[(uint32_t) (source_env - 1) << 4])->size;
        return *paragraphs != 0 && base + ((uint32_t) *paragraphs << 4) <= cpu->mem_size;
    
    }
    
    /* EXEC also accepts an environment in the caller's own memory block. */
    end = base + 32768;
    
    if (end > cpu->mem_size) {
        end = (uint32_t) cpu->mem_size;
    }
    
    position = base;
    
    while (position + 1 < end) {
    
        if (cpu->mem[position] == 0 && cpu->mem[position + 1] == 0) {
        
            position += 2;
            break;
        
        }
        
        position++;
    
    }
    
    if (position + 2 > end) {
        return 0;
    }
    
    /* DOS 3.x appends a word count followed by ASCIIZ program names. */
    count = mem_read16 (cpu->mem, cpu->mem_size, position);
    position += 2;
    
    if (count > 16) {
        return 0;
    }
    
    for (i = 0; i < count; i++) {
    
        while (position < end && cpu->mem[position] != 0) {
            position++;
        }
        
        if (position == end) {
            return 0;
        }
        
        position++;
    
    }
    
    *paragraphs = (uint16_t) ((position - base + 15) / 16);
    return *paragraphs != 0;

}

static int dos_create_child_psp (CPU8086 *cpu, uint16_t paragraphs, const char *args, uint16_t source_env, uint16_t *child_psp) {

    uint16_t segment;
    uint16_t allocated_size;
    uint16_t environment;
    uint16_t env_paragraphs;
    
    uint32_t env_base;
    uint32_t end_segment;
    
    if (paragraphs < 0x0010 || child_psp == NULL || !dos_environment_paragraphs (cpu, source_env, &env_paragraphs)) {
    
        dos_set_error (cpu, 0x000A);
        return 0;
    
    }
    
    env_base = (uint32_t) source_env << 4;
    
    if (env_paragraphs == 0 || env_base + ((uint32_t) env_paragraphs << 4) > cpu->mem_size) {
    
        dos_set_error (cpu, 0x0008);
        return 0;
    
    }
    
    dos_allocate_memory (cpu, env_paragraphs, cpu->psp_seg, &environment, NULL);
    
    if (cpu->flags & FLAG_CF) {
        return 0;
    }
    
    memcpy (&cpu->mem[(uint32_t) environment << 4], &cpu->mem[env_base], (size_t) env_paragraphs << 4);
    dos_allocate_memory (cpu, paragraphs, 0, &segment, &allocated_size);
    
    if (cpu->flags & FLAG_CF) {
    
        ((DOS_MCB *) &cpu->mem[(uint32_t) (environment - 1) << 4])->owner = 0;
        return 0;
    
    }
    
    end_segment = (uint32_t) segment + allocated_size;
    
    if (end_segment > 0xFFFF) {
    
        ((DOS_MCB *) &cpu->mem[(uint32_t) (segment - 1) << 4])->owner = 0;
        ((DOS_MCB *) &cpu->mem[(uint32_t) (environment - 1) << 4])->owner = 0;
        
        dos_set_error (cpu, 0x0008);
        return 0;
    
    }
    
    ((DOS_MCB *) &cpu->mem[(uint32_t) (environment - 1) << 4])->owner = segment;
    
    init_psp_contents (cpu, args, segment, cpu->psp_seg, environment, (uint16_t) end_segment);
    dos_inherit_process_handles (cpu, cpu->psp_seg, segment);
    
    *child_psp = segment;
    return 1;

}

static void dos_clear_success (CPU8086 *cpu) {
    cpu->flags &= (uint16_t) ~FLAG_CF;
}

static void dos_get_file_datetime (time_t modified, uint16_t *dos_date, uint16_t *dos_time) {

    const struct tm *local = localtime (&modified);
    int year;
    
    if (local == NULL) {
    
        *dos_date = 0x0021;             /* 1 January 1980 */
        *dos_time = 0;
        
        return;
    
    }
    
    year = local->tm_year + 1900;
    
    if (year < 1980) {
    
        *dos_date = 0x0021;
        *dos_time = 0;
        
        return;
    
    }
    
    if (year > 2107) {
    
        *dos_date = 0xFF9F;             /* 31 December 2107 */
        *dos_time = 0xBF7D;             /* 23:59:58 */
        
        return;
    
    }
    
    *dos_date = (uint16_t) (((year - 1980) << 9) | ((local->tm_mon + 1) << 5) | local->tm_mday);
    *dos_time = (uint16_t) ((local->tm_hour << 11) | (local->tm_min << 5) | (local->tm_sec / 2));

}

static void dos_write_dta_find_result (CPU8086 *cpu, const char *name, uint32_t size, time_t modified, uint8_t attr) {

    uint16_t dos_date, dos_time;
    int i;
    
    uint16_t dta_seg, dta_off;
    uint32_t base;
    
    dos_get_dta (cpu, &dta_seg, &dta_off);
    base = get_phys (dta_seg, dta_off);
    
    /* Offset 21: Attribute */
    mem_write8 (cpu->mem, cpu->mem_size, base + 21, attr);
    
    /* Offset 22-25: Time and Date */
    dos_get_file_datetime (modified, &dos_date, &dos_time);
    
    mem_write16 (cpu->mem, cpu->mem_size, base + 22, dos_time);
    mem_write16 (cpu->mem, cpu->mem_size, base + 24, dos_date);
    
    /* Offset 26-29: File Size */
    mem_write32 (cpu->mem, cpu->mem_size, base + 26, size);
    
    /* Offset 30-42: Filename (ASCIIZ) */
    /* CRITICAL: You must null-terminate this string! */
    for (i = 0; i < 13; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, base + 30 + i, 0);
    }
    
    for (i = 0; name[i] && i < 12; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, base + 30 + i, (uint8_t) name[i]);
    }
    
    mem_write8 (cpu->mem, cpu->mem_size, base + 30 + i, 0);                /* Explicit null */

}

static void dos_read_asciiz (CPU8086 *cpu, uint16_t seg, uint16_t off, char *out, size_t out_sz) {

    size_t i;
    
    if (out_sz == 0) {
        return;
    }
    
    for (i = 0; i + 1 < out_sz; i++) {
    
        out[i] = (char) mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, (uint16_t) (off + i)));
        
        if (out[i] == '\0') {
            break;
        }
    
    }
    
    out[out_sz - 1] = '\0';

}

static char *dos_path_token (char *path, char **saveptr) {

    char *start, *end;
    
    if (path != NULL) {
        *saveptr = path;
    }
    
    if (*saveptr == NULL) {
        return NULL;
    }
    
    start = *saveptr + strspn (*saveptr, "/\\");
    
    if (*start == '\0') {
    
        *saveptr = NULL;
        return NULL;
    
    }
    
    end = start + strcspn (start, "/\\");
    
    if (*end != '\0') {
    
        *end = '\0';
        *saveptr = end + 1;
    
    } else {
        *saveptr = NULL;
    }
    
    return start;

}

static void dos_append_bounded (char *dest, size_t dest_size, const char *src);

static void dos_collapse_dotdot (char *path) {

    char tmp[260];
    char out[260];
    
    char *saveptr = NULL;
    char *parts[64];
    char *tok;
    
    int n = 0;
    int i;
    
    snprintf (tmp, sizeof (tmp), "%s", path);
    tok = dos_path_token (tmp, &saveptr);
    
    while (tok) {
    
        if (strcmp (tok, ".") == 0) {
            /* skip */
        } else if (strcmp (tok, "..") == 0) {
        
            if (n > 0) {
                n--;
            }
        
        } else {
        
            if (n < 64) {
                parts[n++] = tok;
            }
        
        }
        
        tok = dos_path_token (NULL, &saveptr);
    
    }
    
    out[0] = '\0';
    
    for (i = 0; i < n; i++) {
    
        if (i > 0) {
            dos_append_bounded (out, sizeof (out), "/");
        }
        
        dos_append_bounded (out, sizeof (out), parts[i]);
    
    }
    
    snprintf (path, 260, "%s", out);

}

static void dos_normalize_path (char *path) {

    size_t i, j;
    
    if (path[0] && path[1] == ':') {
        memmove (path, path + 2, strlen (path + 2) + 1);
    }
    
    while (path[0] == '\\' || path[0] == '/') {
        memmove (path, path + 1, strlen (path));
    }
    
    for (i = 0; path[i]; i++) {
    
        if (path[i] == '\\') {
            path[i] = '/';
        }
    
    }
    
    while (path[0] == '.' && path[1] == '/') {
        memmove (path, path + 2, strlen (path + 2) + 1);
    }
    
    for (i = 0, j = 0; path[i]; i++) {
    
        if (path[i] == '/' && j > 0 && path[j - 1] == '/') {
            continue;
        }
        
        path[j++] = path[i];
    
    }
    
    path[j] = '\0';
    dos_collapse_dotdot (path);

}

static char dos_drive_root[260] = ".";

static char dos_initial_host_path[520];
static char dos_initial_guest_path[260];

const char *dos_register_initial_program (const char *path) {

    const char *base = path;
    const char *p;
    
    for (p = path; *p; p++) {
    
        if (*p == '/' || *p == '\\') {
            base = p + 1;
        }
    
    }
    
    snprintf (dos_initial_host_path, sizeof (dos_initial_host_path), "%s", path);
    snprintf (dos_initial_guest_path, sizeof (dos_initial_guest_path), "C:\\%s", base);
    
    return dos_initial_guest_path;

}

static int dos_is_drive_root_path (const char *path) {

    size_t len = strlen (path);
    size_t root_len = strlen (dos_drive_root);
    
    while (len > 1 && (path[len - 1] == '/' || path[len - 1] == '\\')) {
        len--;
    }
    
    while (root_len > 1 && (dos_drive_root[root_len - 1] == '/' || dos_drive_root[root_len - 1] == '\\')) {
        root_len--;
    }
    
    return len == root_len && strncmp (path, dos_drive_root, len) == 0;

}

static const char *dos_find_next_directory_entry (DIR *dir, const char *dirpath, uint8_t *dot_phase) {

    struct dirent *de;
    
    if (!dos_is_drive_root_path (dirpath) && *dot_phase < 2) {
        return (*dot_phase)++ == 0 ? "." : "..";
    }
    
    while ((de = readdir (dir)) != NULL) {
    
        if (strcmp (de->d_name, ".") != 0 && strcmp (de->d_name, "..") != 0) {
            return de->d_name;
        }
    
    }
    
    return NULL;

}

static void dos_append_bounded (char *dest, size_t dest_size, const char *src) {

    size_t used;
    size_t len;
    
    if (dest_size == 0) {
        return;
    }
    
    used = strlen (dest);
    
    if (used >= dest_size - 1) {
        return;
    }
    
    len = strlen (src);
    
    if (len > dest_size - used - 1) {
        len = dest_size - used - 1;
    }
    
    memcpy (dest + used, src, len);
    dest[used + len] = '\0';

}

static int dos_resolve_existing_host_path (const char *dos_rel, char *host_out, size_t host_out_sz);
static int dos_83_cache_get_host (const char *dirpath, const char *name, char *host_out, size_t host_out_sz);

static void dos_resolve_path (const char *in, char *out, size_t out_sz) {

    char tmp[260], dos_rel[520], host_base[520], host_leaf[260];
    
    char *leaf;
    int absolute;
    
    if (out_sz == 0) {
        return;
    }
    
    if (in && dos_initial_guest_path[0] && strcasecmp (in, dos_initial_guest_path) == 0) {
    
        snprintf (out, out_sz, "%s", dos_initial_host_path);
        return;
    
    }
    
    tmp[0] = '\0';
    dos_append_bounded (tmp, sizeof (tmp), in ? in : "");
    
    dos_normalize_path (tmp);
    
    absolute = in && (in[0] == '\\' || in[0] == '/' || (in[0] && in[1] == ':' && (in[2] == '\\' || in[2] == '/')));
    dos_rel[0] = '\0';
    
    if (!absolute && dos_current_dir[0] != '\0') {
    
        dos_append_bounded (dos_rel, sizeof (dos_rel), dos_current_dir);
        dos_append_bounded (dos_rel, sizeof (dos_rel), "/");
    
    }
    
    dos_append_bounded (dos_rel, sizeof (dos_rel), tmp);
    
    host_base[0] = '\0';
    dos_append_bounded (host_base, sizeof (host_base), dos_drive_root);
    
    leaf = strrchr (dos_rel, '/');
    
    if (leaf) {
    
        *leaf++ = '\0';
        
        if (!dos_resolve_existing_host_path (dos_rel, host_base, sizeof (host_base))) {
        
            host_base[0] = '\0';
            
            dos_append_bounded (host_base, sizeof (host_base), dos_drive_root);
            dos_append_bounded (host_base, sizeof (host_base), "/");
            dos_append_bounded (host_base, sizeof (host_base), dos_rel);
        
        }
    
    } else {
        leaf = dos_rel;
    }
    
    if (leaf[0] && !strchr (leaf, '*') && !strchr (leaf, '?') &&
        dos_83_cache_get_host (host_base, leaf, host_leaf, sizeof (host_leaf))) {
        leaf = host_leaf;
    }
    
    out[0] = '\0';
    
    dos_append_bounded (out, out_sz, host_base);
    dos_append_bounded (out, out_sz, "/");
    dos_append_bounded (out, out_sz, leaf);
    
    /* NOTE: Do NOT use dos_normalize_path on 'out' here because it */
    /* will strip the leading '/' required by the Linux filesystem. */

}

typedef struct {
    
    char dos83[12];                     /* 8 + 3 + NUL */
    char host[260];

} DOS83_MAP;

static void dos_make_83_display_parts (const char *src, char name[9], char ext[4]);

static int dos_char_ok_83 (unsigned char c) {

    c = (unsigned char) toupper (c);
    
    if (c >= 'A' && c <= 'Z') {
        return 1;
    }
    
    if (c >= '0' && c <= '9') {
        return 1;
    }
    
    switch (c) {
        
        case '$': case '%': case '\'': case '-':
        case '_': case '@': case '~': case '`':
        case '!': case '(': case ')': case '{':
        case '}': case '^': case '#': case '&':
        
            return 1;
        
    }
    
    return 0;

}

static void dos_make_83_parts (const char *src, char name[9], char ext[4]) {

    const char *base;
    const char *dot;
    
    int i;
    
    memset (name, ' ', 8);
    name[8] = '\0';
    
    memset (ext, ' ', 3);
    ext[3] = '\0';
    
    base = strrchr (src, '/');
    
    if (!base) {
        base = strrchr (src, '\\');
    }
    
    base = base ? base + 1 : src;
    
    if (strcmp (base, ".") == 0 || strcmp (base, "..") == 0) {
    
        for (i = 0; base[i]; i++) {
            name[i] = '.';
        }
        
        return;
    
    }
    
    /* Host dotfiles: expose ".ssh" as DOS name "SSH", not ".SSH" */
    if (base[0] == '.' && base[1] != '\0') {
        base++;
    }
    
    dot = strchr (base, '.');
    
    if (dot) {
    
        for (i = 0; i < 8 && base[i] && &base[i] < dot; i++) {
            name[i] = (char) toupper ((int) base[i]);
        }
        
        dot++;
        
        for (i = 0; i < 3 && dot[i]; i++) {
            ext[i] = (char) toupper ((int) dot[i]);
        }
    
    } else {
    
        for (i = 0; i < 8 && base[i]; i++) {
            name[i] = (char) toupper ((int) base[i]);
        }
    
    }

}

static void dos_make_83_display_parts (const char *src, char name[9], char ext[4]) {

    const char *base;
    const char *dot;
    
    int i, j;
    int needs_short = 0;
    
    memset (name, ' ', 8);
    name[8] = '\0';
    
    memset (ext, ' ', 3);
    ext[3] = '\0';
    
    base = strrchr (src, '/');
    
    if (!base) {
        base = strrchr (src, '\\');
    }
    
    base = base ? base + 1 : src;
    
    if (base[0] == '.' && base[1] != '\0') {
        base++;
    }
    
    dot = strrchr (base, '.');
    
    for (i = 0; base[i]; i++) {
    
        unsigned char c = (unsigned char) base[i];
        
        if (c == '.') {
            continue;
        }
        
        if (!dos_char_ok_83 (c) || c == ' ' || c == '\t') {
        
            needs_short = 1;
            break;
        
        }
    
    }
    
    if (!needs_short && strlen (base) <= 12 && !strchr (base, '_')) {
    
        dos_make_83_parts (src, name, ext);
        return;
    
    }
    
    j = 0;
    
    for (i = 0; base[i] && base + i != dot && j < 6; i++) {
    
        unsigned char c = (unsigned char) base[i];
        
        if (dos_char_ok_83 (c)) {
            name[j++] = (char) toupper (c);
        }
    
    }
    
    if (j == 0) {
        name[j++] = '_';
    }
    
    while (j < 6) {
        name[j++] = ' ';
    }
    
    name[6] = '~';
    name[7] = '1';
    
    if (dot) {
    
        j = 0;
        dot++;
        
        for (i = 0; dot[i] && j < 3; i++) {
        
            unsigned char c = (unsigned char) dot[i];
            
            if (dos_char_ok_83 (c)) {
                ext[j++] = (char) toupper (c);
            }
        
        }
    
    }

}

#define     DOS_83_CACHE_SLOTS          16

typedef struct {

    char *host;
    int index;
    
    int needs_short;
    char dos_name[13];

} DOS_83_NAME_ENTRY;

typedef struct {

    char key[10];
    int count;
    int used;

} DOS_83_COUNT_ENTRY;

typedef struct {

    char key[11];
    int used;

} DOS_83_USED_ENTRY;

typedef struct {

    char *dirpath;
    
    DOS_83_NAME_ENTRY *names;
    size_t capacity;
    
    struct stat identity;
    unsigned long age;

} DOS_83_CACHE;

static DOS_83_CACHE dos_83_cache[DOS_83_CACHE_SLOTS];
static unsigned long dos_83_cache_age;

static unsigned long dos_83_hash (const void *data, size_t length) {

    const unsigned char *p = data;
    unsigned long h = 2166136261UL;
    
    size_t i;
    
    for (i = 0; i < length; i++) {
        h = (h ^ p[i]) * 16777619UL;
    }
    
    return h;

}

static int dos_83_needs_short (const char *host) {

    const char *dot = strchr (host, '.');
    
    int base_len;
    int ext_len;
    
    if (dot) {
    
        base_len = (int) (dot - host);
        ext_len = (int) strlen (dot + 1);
    
    } else {
    
        base_len = (int) strlen (host);
        ext_len = 0;
    
    }
    
    return strchr (host, '_') != NULL || strchr (host, ' ') != NULL || strchr (host, '+') != NULL || strchr (host, '-') != NULL || (dot != NULL && strchr (dot + 1, '.') != NULL) || base_len > 8 || ext_len > 3;

}

static int dos_83_format_alias (const char *host, int index, char *out) {

    char name[9], ext[4];
    char prefix[7];
    char suffix[16];
    
    size_t prefix_len;
    size_t suffix_len;
    
    int i, n = 0;
    dos_make_83_display_parts (host, name, ext);
    
    if (index == 0) {
    
        for (i = 7; i >= 0 && name[i] == ' '; i--) {
            name[i] = '\0';
        }
        
        for (i = 2; i >= 0 && ext[i] == ' '; i--) {
            ext[i] = '\0';
        }
        
        if (ext[0]) {
            snprintf (out, 13, "%s.%s", name, ext);
        } else {
            snprintf (out, 13, "%s", name);
        }
        
        return 1;
    
    }
    
    for (i = 0; i < 8 && name[i] != ' ' && n < 6; i++) {
        prefix[n++] = name[i];
    }
    
    prefix[n] = '\0';
    
    snprintf (suffix, sizeof (suffix), "~%d", index);
    suffix_len = strlen (suffix);
    
    if (suffix_len >= 8) {
        return 0;
    }
    
    prefix_len = 8 - suffix_len;
    
    if (prefix_len > (size_t) n) {
        prefix_len = (size_t) n;
    }
    
    memcpy (out, prefix, prefix_len);
    out[prefix_len] = '\0';
    
    dos_append_bounded (out, 13, suffix);
    
    if (ext[0] != ' ') {
    
        dos_append_bounded (out, 13, ".");
        dos_append_bounded (out, 13, ext);
    
    }
    
    i = (int) strlen (out) - 1;
    
    while (i >= 0 && out[i] == ' ') {
        out[i--] = '\0';
    }
    
    return 1;

}

static void dos_83_alias_key (const char *name, char key[11]) {

    const char *dot = strchr (name, '.');
    size_t i;
    
    memset (key, ' ', 11);
    
    for (i = 0; i < 8 && name[i] && (!dot || name + i < dot); i++) {
        key[i] = (char) toupper ((int) name[i]);
    }
    
    if (dot) {
    
        dot++;
        
        for (i = 0; i < 3 && dot[i]; i++) {
            key[8 + i] = (char) toupper ((int) dot[i]);
        }
    
    }

}

static int dos_83_reserve_name (DOS_83_USED_ENTRY *used, size_t capacity, const char *name) {

    char key[11];
    size_t slot;
    
    dos_83_alias_key (name, key);
    slot = dos_83_hash (key, sizeof (key)) & (capacity - 1);
    
    while (used[slot].used) {
    
        if (memcmp (used[slot].key, key, sizeof (key)) == 0) {
            return 0;
        }
        
        slot = (slot + 1) & (capacity - 1);
    
    }
    
    memcpy (used[slot].key, key, sizeof (key));
    
    used[slot].used = 1;
    return 1;

}

static void dos_83_cache_release (DOS_83_CACHE *cache) {

    size_t i;
    
    if (cache->names) {
    
        for (i = 0; i < cache->capacity; i++) {
            free (cache->names[i].host);
        }
    
    }
    
    free (cache->names);
    free (cache->dirpath);
    
    memset (cache, 0, sizeof (*cache));

}

static void dos_83_cache_invalidate (void) {

    size_t i;

    for (i = 0; i < DOS_83_CACHE_SLOTS; i++) {
        dos_83_cache_release (&dos_83_cache[i]);
    }

}

static int dos_83_cache_same_directory (const struct stat *a, const struct stat *b) {
    return a->st_dev == b->st_dev && a->st_ino == b->st_ino && a->st_size == b->st_size && a->st_mtime == b->st_mtime && a->st_ctime == b->st_ctime;
}

static int dos_83_cache_build (DOS_83_CACHE *cache, const char *dirpath, const struct stat *identity) {

    DOS_83_COUNT_ENTRY *counts = NULL;
    DOS_83_USED_ENTRY *used = NULL;
    
    struct dirent *de;
    DIR *dir;
    
    struct stat after;
    size_t i;
    
    size_t capacity = 16;
    size_t num_entries = 0;
    
    dir = opendir (dirpath);

    if (!dir) {
        return 0;
    }
    
    while ((de = readdir (dir)) != NULL) {
    
        if (strcmp (de->d_name, ".") != 0 && strcmp (de->d_name, "..") != 0) {
            num_entries++;
        }
    
    }
    
    while (capacity <= num_entries && capacity <= ((size_t) -1) / 2) {
        capacity *= 2;
    }
    
    if (capacity <= num_entries || capacity > ((size_t) -1) / 2) {
    
        closedir (dir);
        return 0;
    
    }
    
    capacity *= 2;
    
    cache->names = calloc (capacity, sizeof (*cache->names));
    
    counts = calloc (capacity, sizeof (*counts));
    used = calloc (capacity, sizeof (*used));
    
    cache->capacity = capacity;
    
    if (!cache->names || !counts || !used) {
        goto failure;
    }
    
    cache->dirpath = malloc (strlen (dirpath) + 1);
    
    if (!cache->dirpath) {
        goto failure;
    }
    
    strcpy (cache->dirpath, dirpath);
    rewinddir (dir);
    
    while ((de = readdir (dir)) != NULL) {
    
        char name[9], ext[4], prefix[7], key[10];
        size_t j, index;
        
        int n = 0;
        
        if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
            continue;
        }
        
        dos_make_83_display_parts (de->d_name, name, ext);
        memset (prefix, 0, sizeof (prefix));
        
        for (i = 0; i < 8 && name[i] != ' ' && n < 6; i++) {
            prefix[n++] = name[i];
        }
        
        memcpy (key, prefix, 7);
        memcpy (key + 7, ext, 3);
        
        j = dos_83_hash (key, sizeof (key)) & (capacity - 1);
        
        while (counts[j].used && memcmp (counts[j].key, key, sizeof (key)) != 0) {
            j = (j + 1) & (capacity - 1);
        }
        
        if (!counts[j].used) {
        
            memcpy (counts[j].key, key, sizeof (key));
            counts[j].used = 1;
        
        }
        
        counts[j].count++;
        index = dos_83_hash (de->d_name, strlen (de->d_name)) & (capacity - 1);
        
        while (cache->names[index].host) {
            index = (index + 1) & (capacity - 1);
        }
        
        cache->names[index].host = malloc (strlen (de->d_name) + 1);
        
        if (!cache->names[index].host) {
            goto failure;
        }
        
        strcpy (cache->names[index].host, de->d_name);
        
        cache->names[index].index = counts[j].count;
        cache->names[index].needs_short = dos_83_needs_short (de->d_name);
    
    }
    
    /* Reserve existing DOS-compatible names before assigning generated aliases. */
    for (i = 0; i < capacity; i++) {
    
        DOS_83_NAME_ENTRY *entry = &cache->names[i];
        
        if (!entry->host || entry->needs_short) {
            continue;
        }
        
        dos_83_format_alias (entry->host, 0, entry->dos_name);
        
        if (!dos_83_reserve_name (used, capacity, entry->dos_name)) {
        
            entry->needs_short = 1;
            entry->dos_name[0] = '\0';
        
        }
    
    }
    
    /* Assign suffixes in directory order, checking the whole 8.3 namespace. */
    rewinddir (dir);
    
    while ((de = readdir (dir)) != NULL) {
    
        DOS_83_NAME_ENTRY *entry;
        size_t slot;
        int index;
        
        if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
            continue;
        }
        
        slot = dos_83_hash (de->d_name, strlen (de->d_name)) & (capacity - 1);
        
        while (cache->names[slot].host && strcmp (cache->names[slot].host, de->d_name) != 0) {
            slot = (slot + 1) & (capacity - 1);
        }
        
        if (!cache->names[slot].host) {
            goto failure;
        }
        
        entry = &cache->names[slot];
        
        if (!entry->needs_short) {
            continue;
        }
        
        index = entry->index;
        
        while (dos_83_format_alias (entry->host, index, entry->dos_name)) {
        
            if (dos_83_reserve_name (used, capacity, entry->dos_name)) {
                break;
            }
            
            if (index == 9999999) {
                goto failure;
            }
            
            index++;
        
        }
        
        if (index > 9999999) {
            goto failure;
        }
        
        entry->index = index;
    
    }
    
    if (stat (dirpath, &after) != 0 || !dos_83_cache_same_directory (identity, &after)) {
        goto failure;
    }
    
    cache->identity = after;
    
    free (counts);
    free (used);
    
    closedir (dir);
    return 1;
    
failure:
    
    free (counts);
    free (used);
    
    closedir (dir);
    dos_83_cache_release (cache);
    
    return 0;

}

static DOS_83_CACHE *dos_83_cache_get (const char *dirpath) {

    DOS_83_CACHE *cache = NULL;
    size_t i;
    
    struct stat identity;
    
    if (stat (dirpath, &identity) != 0 || !S_ISDIR (identity.st_mode)) {
        return 0;
    }
    
    for (i = 0; i < DOS_83_CACHE_SLOTS; i++) {
    
        if (dos_83_cache[i].dirpath && strcmp (dos_83_cache[i].dirpath, dirpath) == 0) {
        
            cache = &dos_83_cache[i];
            
            if (!dos_83_cache_same_directory (&cache->identity, &identity)) {
                dos_83_cache_release (cache);
            }
            
            break;
        
        }
    
    }
    
    if (!cache || !cache->dirpath) {
    
        cache = &dos_83_cache[0];
        
        for (i = 0; i < DOS_83_CACHE_SLOTS; i++) {
        
            if (!dos_83_cache[i].dirpath) {
            
                cache = &dos_83_cache[i];
                break;
            
            }
            
            if (dos_83_cache[i].age < cache->age) {
                cache = &dos_83_cache[i];
            }
        
        }
        
        dos_83_cache_release (cache);
        
        if (!dos_83_cache_build (cache, dirpath, &identity)) {
            return 0;
        }
    
    }
    
    cache->age = ++dos_83_cache_age;
    return cache;

}

static int dos_83_cache_get_name (const char *dirpath, const char *host, char out[13]) {

    DOS_83_CACHE *cache = dos_83_cache_get (dirpath);
    size_t j;
    
    if (!cache) {
        return 0;
    }
    
    j = dos_83_hash (host, strlen (host)) & (cache->capacity - 1);
    
    while (cache->names[j].host) {
    
        if (strcmp (cache->names[j].host, host) == 0) {
        
            memcpy (out, cache->names[j].dos_name, sizeof (cache->names[j].dos_name));
            return 1;
        
        }
        
        j = (j + 1) & (cache->capacity - 1);
    
    }
    
    return 0;

}

static int dos_83_cache_get_host (const char *dirpath, const char *name, char *host_out, size_t host_out_sz) {

    DOS_83_CACHE *cache = dos_83_cache_get (dirpath);
    size_t i;
    
    if (!cache || host_out_sz == 0) {
        return 0;
    }
    
    /* Prefer a real host name when it is also an 8.3 alias. */
    for (i = 0; i < cache->capacity; i++) {
    
        if (cache->names[i].host && strcasecmp (cache->names[i].host, name) == 0) {
        
            if (strlen (cache->names[i].host) >= host_out_sz) {
                return 0;
            }
            
            strcpy (host_out, cache->names[i].host);
            return 1;
        
        }
    
    }
    
    for (i = 0; i < cache->capacity; i++) {
    
        if (cache->names[i].host && strcasecmp (cache->names[i].dos_name, name) == 0) {
        
            if (strlen (cache->names[i].host) >= host_out_sz) {
                return 0;
            }
            
            strcpy (host_out, cache->names[i].host);
            return 1;
        
        }
    
    }
    
    return 0;

}

static void dos_make_83_display_name_unique (const char *dirpath, const char *host, char *out) {

    struct dirent *de;
    DIR *d;
    
    char my_n[9], my_e[4];
    char my_prefix[7];
    
    int needs_short;
    int index = 1;
    int k, len;
    
    if (strcmp (host, ".") == 0 || strcmp (host, "..") == 0) {
    
        strcpy (out, host);
        return;
    
    }
    
    if (dos_83_cache_get_name (dirpath, host, out)) {
        return;
    }
    
    dos_make_83_display_parts (host, my_n, my_e);
    
    {
    
        const char *dot;
        
        int base_len;
        int ext_len;
        
        dot = strchr (host, '.');
        
        if (dot) {
        
            base_len = (int) (dot - host);
            ext_len = (int) strlen (dot + 1);
        
        } else {
        
            base_len = (int) strlen (host);
            ext_len = 0;
        
        }
        
        needs_short = strchr (host, '_') != NULL || strchr (host, ' ') != NULL || strchr (host, '+') != NULL || strchr (host, '-') != NULL || (dot != NULL && strchr (dot + 1, '.') != NULL) || base_len > 8 || ext_len > 3;
    
    }
    
    if (!needs_short) {
    
        dos_83_format_alias (host, 0, out);
        return;
    
    }
    
    memset (my_prefix, 0, sizeof (my_prefix));
    len = 0;
    
    for (k = 0; k < 8 && my_n[k] != ' '; k++) {
    
        if (len < 6) {
            my_prefix[len++] = my_n[k];
        }
    
    }
    
    my_prefix[len] = '\0';
    
    /**
     * Use position among matching entries.
     *
     * This intentionally gives:
     *   java2dex_fields.c    -> JAVA2D~1.C
     *   java2dex_.c         -> JAVA2D~2.C
     *   ...
     *   java2dex_ (2).c     -> JAVA2D~10.C
     *
     * Do NOT break before counting this file.
     */
    d = opendir (dirpath);
    
    if (d) {
    
        while ((de = readdir (d)) != NULL) {
        
            char on[9], oe[4];
            char oprefix[7];
            
            int olen = 0;
            
            if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
                continue;
            }
            
            dos_make_83_display_parts (de->d_name, on, oe);
            memset (oprefix, 0, sizeof (oprefix));
            
            for (k = 0; k < 8 && on[k] != ' '; k++) {
            
                if (olen < 6) {
                    oprefix[olen++] = on[k];
                }
            
            }
            
            oprefix[olen] = '\0';
            
            if (strcasecmp (oprefix, my_prefix) == 0 && memcmp (oe, my_e, 3) == 0) {
            
                if (strcasecmp (oprefix, my_prefix) == 0 && memcmp (oe, my_e, 3) == 0) {
                
                    if (strcmp (de->d_name, host) == 0) {
                    
                        /* this is OUR file → count how many come BEFORE it */
                        index = 1;
                        rewinddir (d);
                        
                        while ((de = readdir (d)) != NULL) {
                        
                            char on2[9], oe2[4];
                            char oprefix2[7];
                            
                            int olen2 = 0;
                            
                            if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
                                continue;
                            }
                            
                            if (strcmp (de->d_name, host) == 0) {
                                break;
                            }
                            
                            dos_make_83_display_parts (de->d_name, on2, oe2);
                            
                            memset (oprefix2, 0, sizeof (oprefix2));
                            
                            for (k = 0; k < 8 && on2[k] != ' '; k++) {
                            
                                if (olen2 < 6) {
                                    oprefix2[olen2++] = on2[k];
                                }
                            
                            }
                            
                            oprefix2[olen2] = '\0';
                            
                            if (strcasecmp (oprefix2, my_prefix) == 0 && memcmp (oe2, my_e, 3) == 0) {
                                index++;
                            }
                        
                        }
                        
                        break;
                    
                    }
                
                }
            
            }
        
        }
        
        closedir (d);
    
    }
    
    {
    
        char suffix[16];
        char full[32];
        
        size_t prefix_len;
        size_t suffix_len;
        size_t len;
        
        snprintf (suffix, sizeof (suffix), "~%d", index);
        suffix_len = strlen (suffix);
        
        /* Leave enough room for the complete decimal suffix in 8.3. */
        prefix_len = suffix_len < 8 ? 8 - suffix_len : 0;
        
        if (prefix_len > strlen (my_prefix)) {
            prefix_len = strlen (my_prefix);
        }
        
        memcpy (full, my_prefix, prefix_len);
        full[prefix_len] = '\0';
        
        dos_append_bounded (full, sizeof (full), suffix);
        
        if (my_e[0] != ' ') {
        
            dos_append_bounded (full, sizeof (full), ".");
            dos_append_bounded (full, sizeof (full), my_e);
        
        }
        
        len = strlen (full);
        
        if (len > 12) {
            len = 12;
        }
        
        memcpy (out, full, len);
        out[len] = '\0';
    
    }
    
    k = (int) strlen (out) - 1;
    
    while (k >= 0 && out[k] == ' ') {
        out[k--] = '\0';
    }

}

static int dos_match_83_field (const char *pat, const char *val, int len) {

    int i;
    
    for (i = 0; i < len; i++) {
    
        char p = (char) toupper ((int) pat[i]);
        char v = (char) toupper ((int) val[i]);
        
        if (p == '*') {
            return 1; /* rest matches */
        }
        
        if (p == '?') {
            continue; /* any char in this field */
        }
        
        if (p != v) {
            return 0;
        }
    
    }
    
    return 1;

}

static int dos_wildcard_match (const char *pattern, const char *name) {

    char pat_name[9], pat_ext[4];
    char val_name[9], val_ext[4];
    
    if (strcmp (pattern, "*.*") == 0 || strcmp (pattern, "*") == 0) {
        return 1;
    }
    
    dos_make_83_parts (pattern, pat_name, pat_ext);
    dos_make_83_parts (name,    val_name, val_ext);
    
    return dos_match_83_field (pat_name, val_name, 8) && dos_match_83_field (pat_ext,  val_ext,  3);

}

static void dos_split_dir_pattern (const char *input, char *dir_out, size_t dir_sz, char *pat_out, size_t pat_sz) {

    const char *slash1 = strrchr (input, '/');
    const char *slash2 = strrchr (input, '\\');
    const char *slash = slash1;
    
    if (slash2 && (!slash1 || slash2 > slash1)) {
        slash = slash2;
    }
    
    if (slash) {
    
        size_t dir_len = (size_t) (slash - input);
        
        if (dir_len >= dir_sz) {
            dir_len = dir_sz - 1;
        }
        
        memcpy (dir_out, input, dir_len);
        dir_out[dir_len] = '\0';
        
        strncpy (pat_out, slash + 1, pat_sz - 1);
        pat_out[pat_sz - 1] = '\0';
        
        if (dir_out[0] == '\0') {
            strcpy (dir_out, ".");
        }
    
    } else {
    
        strcpy (dir_out, ".");
        strncpy (pat_out, input, pat_sz - 1);
        
        pat_out[pat_sz - 1] = '\0';
    
    }
    
    if (pat_out[0] == '\0') {
        strcpy (pat_out, "*");
    }

}

FILE *dos_fopen_case_insensitive (const char *path, const char *mode) {

    char dir[260], name[128];
    
    struct dirent *ent;
    DIR *d;
    
    dos_split_dir_pattern (path, dir, sizeof (dir), name, sizeof (name));
    d = opendir (dir[0] ? dir : ".");
    
    if (!d) {
        return NULL;
    }
    
    while ((ent = readdir (d)) != NULL) {
    
        if (strcasecmp (ent->d_name, name) == 0) {
        
            char full[520];
            
            snprintf (full, sizeof (full), "%s/%s", dir[0] ? dir : ".", ent->d_name);
            closedir (d);
            
            return fopen (full, mode);
        
        }
    
    }
    
    closedir (d);
    return NULL;

}

static void dos_buffered_line_input (CPU8086 *cpu) {

    uint16_t off = cpu->regs[REG_DX];
    uint32_t phys = get_phys (cpu->ds, off);
    
    uint8_t maxlen = mem_read8 (cpu->mem, cpu->mem_size, phys + 0);
    uint8_t count = 0;
    
    int c;
    
    while (1) {
    
        c = host_read_key (cpu);
        
        if (c == EOF) {
            break;
        }
        
        /* Ctrl+C: DOS calls INT 23h, it is not returned as a line */
        if (c == 0x03) {
        
            uint32_t vec = 0x23 * 4;
            uint16_t new_ip = mem_read16 (cpu->mem, cpu->mem_size, vec + 0);
            uint16_t new_cs = mem_read16 (cpu->mem, cpu->mem_size, vec + 2);
            
            cpu->regs[REG_SP] -= 2;
            mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP]), cpu->flags);
            
            cpu->regs[REG_SP] -= 2;
            mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP]), cpu->cs);
            
            cpu->regs[REG_SP] -= 2;
            mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP]), cpu->ip);
            
            cpu->cs = new_cs;
            cpu->ip = new_ip;
            
            return;
        
        }
        
        /* Console cursor keys use an extended-key prefix and scan code. */
        if (c == 0 || c == 0xE0) {
        
            host_read_key (cpu);
            continue;
        
        }
        
        /* Enter finishes input */
        if (c == '\r' || c == '\n') {
        
            putchar ('\r');
            break;
        
        }
        
        /* Backspace */
        if (c == 0x08 || c == 0x7F) {
        
            if (count > 0) {
            
                count--;
                
                /* erase from screen */
                putchar ('\b');
                putchar (' ');
                putchar ('\b');
                
                fflush (stdout);
            
            }
            
            continue;
        
        }
        
        /* don't store Ctrl+X, Ctrl+A, etc. as text */
        if (c < 0x20 && c != 0x1A) {
            continue;
        }
        
        /* printable character */
        if (count < maxlen) {
        
            mem_write8 (cpu->mem, cpu->mem_size, phys + 2 + count, (uint8_t) c);
            count++;
            
            /* echo */
            if (c < 0x20) {
            
                putchar ('^');
                putchar (c + 0x40);
            
            } else {
                dos_write_console_char (stdout, (uint8_t) c);
            }
            
            fflush (stdout);
        
        }
    
    }
    
    mem_write8 (cpu->mem, cpu->mem_size, phys + 1, count);
    mem_write8 (cpu->mem, cpu->mem_size, phys + 2 + count, '\r');
    
    cpu->flags &= ~FLAG_CF;

}

/* DOS file attributes not represented by a Unix filesystem are kept for this session. */
#ifndef     _WIN32
#define     MAX_DOS_FILE_ATTRIBUTES     256

typedef struct DOS_FILE_ATTRIBUTES {

    char path[520];
    uint8_t flags;

} DOS_FILE_ATTRIBUTES;

static DOS_FILE_ATTRIBUTES dos_file_attributes[MAX_DOS_FILE_ATTRIBUTES];

static DOS_FILE_ATTRIBUTES *dos_file_attribute_entry (const char *path, int create) {

    DOS_FILE_ATTRIBUTES *empty = NULL;
    unsigned i;
    
    for (i = 0; i < MAX_DOS_FILE_ATTRIBUTES; i++) {
    
        if (dos_file_attributes[i].path[0] == '\0') {
        
            if (!empty) {
                empty = &dos_file_attributes[i];
            }
        
        } else if (strcmp (dos_file_attributes[i].path, path) == 0) {
            return &dos_file_attributes[i];
        }
    
    }
    
    if (create && empty && strlen (path) < sizeof (empty->path)) {
    
        strcpy (empty->path, path);
        return empty;
    
    }
    
    return NULL;

}

static void dos_file_attribute_forget (const char *path) {

    DOS_FILE_ATTRIBUTES *entry = dos_file_attribute_entry (path, 0);
    
    if (entry) {
        entry->path[0] = '\0';
    }

}

static void dos_file_attribute_rename (const char *oldpath, const char *newpath) {

    DOS_FILE_ATTRIBUTES *entry = dos_file_attribute_entry (oldpath, 0);
    
    if (entry) {
    
        dos_file_attribute_forget (newpath);
        snprintf (entry->path, sizeof (entry->path), "%s", newpath);
    
    }

}
#endif

static uint8_t dos_host_file_attributes (const char *path, const struct stat *st) {

#if     defined (_WIN32)
    DWORD attributes = GetFileAttributesA (path);
    
    if (attributes != INVALID_FILE_ATTRIBUTES) {
        return (uint8_t) (attributes & 0x37);
    }
    
    return S_ISDIR (st->st_mode) ? 0x10 : 0x20;
#else
    DOS_FILE_ATTRIBUTES *entry = dos_file_attribute_entry (path, 0);
    
    const char *name = strrchr (path, '/');
    uint8_t attr = S_ISDIR (st->st_mode) ? 0x10 : 0x20;
    
    name = name ? name + 1 : path;
    
    if (!(st->st_mode & S_IWUSR)) {
        attr |= 0x01;
    }
    
    if (name[0] == '.' && strcmp (name, ".") != 0 && strcmp (name, "..") != 0) {
        attr |= 0x02;
    }
    
    if (entry) {
        attr = (uint8_t) ((attr & ~0x26) | entry->flags);
    }
    
    return attr;
#endif

}

static int dos_set_host_file_attributes (const char *path, const struct stat *st, uint8_t attr) {

#if     defined (_WIN32)
    DWORD existing = GetFileAttributesA (path);
    DWORD updated;
    
    (void) st;
    
    if (existing == INVALID_FILE_ATTRIBUTES) {
        return 0;
    }
    
    updated = (existing & ~0x27UL) | (attr & 0x27);
    
    if (!(updated & (FILE_ATTRIBUTE_READONLY | FILE_ATTRIBUTE_HIDDEN | FILE_ATTRIBUTE_SYSTEM | FILE_ATTRIBUTE_ARCHIVE))) {
        updated |= FILE_ATTRIBUTE_NORMAL;
    } else {
        updated &= ~FILE_ATTRIBUTE_NORMAL;
    }
    
    return SetFileAttributesA (path, updated) != 0;
#else
    DOS_FILE_ATTRIBUTES *entry = dos_file_attribute_entry (path, 0);
    
    mode_t mode = st->st_mode;
    unsigned i;
    
    if (!entry) {
    
        for (i = 0; i < MAX_DOS_FILE_ATTRIBUTES; i++) {
        
            if (!dos_file_attributes[i].path[0]) {
            
                entry = &dos_file_attributes[i];
                break;
            
            }
        
        }
        
        if (!entry) {
            return 0;
        }
    
    }
    
    if (attr & 0x01) {
        mode &= ~(S_IWUSR | S_IWGRP | S_IWOTH);
    } else {
        mode |= S_IWUSR;
    }
    
    if (mode != st->st_mode && chmod (path, mode & 07777) != 0) {
        return 0;
    }
    
    if (!entry->path[0]) {
        snprintf (entry->path, sizeof (entry->path), "%s", path);
    }
    
    entry->flags = attr & 0x26;
    return 1;
#endif

}

/* Interrupt Handler Function Prototypes */
static void int_0 (CPU8086 *cpu);
static void int_1 (CPU8086 *cpu);
static void int_2 (CPU8086 *cpu);
static void int_3 (CPU8086 *cpu);
static void int_4 (CPU8086 *cpu);
static void int_5 (CPU8086 *cpu);
static void int_6 (CPU8086 *cpu);
static void int_7 (CPU8086 *cpu);
static void int_10 (CPU8086 *cpu);
static void int_13 (CPU8086 *cpu);
static void int_15 (CPU8086 *cpu);
static void int_16 (CPU8086 *cpu);
static void int_1A (CPU8086 *cpu);
static void int_20 (CPU8086 *cpu);
static void int_21 (CPU8086 *cpu);

/* Interrupt handler functions */
static void int_0 (CPU8086 *cpu) {

    printf ("Interrupt 0: Divide by Zero Exception\n");
    cpu->halted = 1;

}

static void int_1 (CPU8086 *cpu) {

    /* With no guest debugger installed, the BIOS default handler returns. */
    (void) cpu;

}

static void int_2 (CPU8086 *cpu) {

    printf ("Interrupt 2: Non-Maskable Interrupt (NMI)\n");
    cpu->halted = 1;

}

static void int_3 (CPU8086 *cpu) {

    /* Guest breakpoints are dispatched through vector 3. */
    (void) cpu;

}

static void int_4 (CPU8086 *cpu) {

    printf ("Interrupt 4: Overflow Exception\n");
    
    /* Handle Overflow exception here */
    (void) cpu;

}

static void int_5 (CPU8086 *cpu) {

    printf ("\n*** BOUND RANGE EXCEEDED (INT 5) at %04X:%04X ***\n", cpu->cs, cpu->ip);
    
    /* On the IBM PC, INT 5 was also used for Print Screen. */
    /* If your program is old, it might just be trying to print the screen. */
    cpu->halted = 1;
    (void) cpu;

}

static void int_6 (CPU8086 *cpu) {

    printf ("Interrupt 6: Invalid Opcode Exception\n");
    
    /* Handle Invalid Opcode exception */
    (void) cpu;

}

static void int_7 (CPU8086 *cpu) {

    printf ("Interrupt 7: Device Not Available\n");
    
    /* Handle FPU unavailable */
    (void) cpu;

}

static void int_20 (CPU8086 *cpu) {

    /* Legacy DOS Terminate */
    if (dos_exec_depth != 0) {
    
        dos_last_return_code[dos_return_slot (cpu)] = 0;
        dos_return_code_pending[dos_return_slot (cpu)] = 1;
    
    }
    
    dos_terminate_process (cpu);

}

static void int_23 (CPU8086 *cpu) {

    /* Default DOS Ctrl+C / Ctrl+Break handler. */
    if (dos_exec_depth != 0) {
    
        dos_last_return_code[dos_return_slot (cpu)] = 0x0100;
        dos_return_code_pending[dos_return_slot (cpu)] = 1;
    
    }
    
    dos_terminate_process (cpu);

}

extern uint16_t old_ip;

static void int2f_not_installed (CPU8086 *cpu) {

    /* Most INT 2F install checks use AL=00h and expect AL=00h if absent. */
    set_reg8 (cpu, 0, 0x00); /* AL */
    cpu->flags &= ~FLAG_CF;

}

static void int_2F (CPU8086 *cpu) {

    uint16_t ax = cpu->regs[REG_AX];
    
    uint8_t ah = (uint8_t) (ax >> 8);
    uint8_t al = (uint8_t) (ax & 0xFF);
    
    switch (ah) {
    
        case 0x11:
            
            /**
             * DOS network redirector / SHARE multiplex.
             * For local-only emulator: not installed / not handled.
             */
            cpu->flags |= FLAG_CF;
            cpu->regs[REG_AX] = 0x0001;
            
            return;
        
        case 0x12:
            
            /**
             * DOS internal services. Many programs probe this.
             * Do not claim support unless individual subfunctions are implemented.
             */
            cpu->flags |= FLAG_CF;
            return;
        
        case 0x16:
            
            /**
             * Windows enhanced-mode / Windows presence.
             * AX=1600h install check:
             *   AL=00h means Windows not running.
             */
            if (al == 0x00) {
            
                cpu->regs[REG_AX] = 0x0000;
                cpu->flags &= ~FLAG_CF;
                
                return;
            
            }
            
            /* Other Windows INT 2F services absent. */
            set_reg8 (cpu, 0, 0x00);
            
            cpu->flags &= ~FLAG_CF;
            return;
        
        case 0x43:
            
            /**
             * XMS driver install check.
             * AX=4300h:
             *   AL=80h means XMS driver installed.
             *   AL=00h means not installed.
             *
             * Return not installed unless you implement INT 2F AX=4310h
             * returning an XMS entry point.
             */
            if (al == 0x00) {
            
                set_reg8 (cpu, 0, 0x00);
                
                cpu->flags &= ~FLAG_CF;
                return;
            
            }
            
            if (al == 0x10) {
            
                /* Get XMS driver address: unsupported */
                cpu->es = 0x0000;
                
                cpu->regs[REG_BX] = 0x0000;
                cpu->flags |= FLAG_CF;
                
                return;
            
            }
            
            break;
        
        case 0x48:
        
            /**
             * DOSKEY/PCED multiplex. AX=4810h reads a buffered line
             * using the same DS:DX buffer layout as INT 21h AH=0Ah.
             */
            if (al == 0x10) {
            
                dos_buffered_line_input (cpu);
                
                cpu->regs[REG_AX] = 0x0000;
                cpu->flags &= ~FLAG_CF;
                
                return;
            
            }
            
            if (al == 0x00) {
            
                int2f_not_installed (cpu);
                return;
            
            }
            
            cpu->flags |= FLAG_CF;
            return;
        
        case 0x4A:
        
            /* HMA / DOS=HIGH related checks. Say unavailable. */
            cpu->flags |= FLAG_CF;
            return;
        
        case 0xB7:
        
            /**
             * APPEND multiplex.
             * AX=B700h install check:
             *   AL=00h if not installed.
             */
            if (al == 0x00) {
            
                int2f_not_installed (cpu);
                return;
            
            }
            
            int2f_not_installed (cpu);
            return;
        
        case 0xAD:
        
            /**
             * KEYB / keyboard enhancer style multiplex checks.
             * Not installed.
             */
            if (al == 0x00) {
            
                int2f_not_installed (cpu);
                return;
            
            }
            
            int2f_not_installed (cpu);
            return;
        
        case 0xAE:
        
            /**
             * NLSFUNC multiplex checks.
             * Not installed.
             */
            if (al == 0x00) {
            
                int2f_not_installed (cpu);
                return;
            
            }
            
            int2f_not_installed (cpu);
            return;
        
        default:
        
            /**
             * Generic multiplex install check convention:
             * AX=xx00 often asks "are you installed?"
             * AL=00h answer means "not installed".
             */
            if (al == 0x00) {
            
                int2f_not_installed (cpu);
                return;
            
            }
            
            printf ("Unhandled INT 2Fh AX=%04X at %04X:%04X\n", ax, cpu->cs, cpu->ip);
            
            cpu->flags |= FLAG_CF;
            return;
    
    }
    
    printf ("Unhandled INT 2Fh AX=%04X at %04X:%04X\n", ax, cpu->cs, cpu->ip);
    cpu->flags |= FLAG_CF;

}

static void fcb_make_pattern (CPU8086 *cpu, uint16_t seg, uint16_t off, char *out) {

    uint32_t p = get_phys (seg, off);
    
    int i, n = 0;
    int any = 0;
    
    /* Check Name part */
    for (i = 0; i < 8; i++) {
    
        char c = (char) mem_read8 (cpu->mem, cpu->mem_size, p + 1 + i);
        
        if (c != ' ') {
        
            any = 1;
            out[n++] = c;
        
        }
    
    }
    
    /* If name was all spaces, it's a wildcard */
    if (!any) {
    
        strcpy (out, "*.*");
        return;
    
    }
    
    /* Add Dot and Check Extension part */
    out[n++] = '.';
    
    for (i = 0; i < 3; i++) {
    
        char c = (char) mem_read8 (cpu->mem, cpu->mem_size, p + 9 + i);
        
        if (c != ' ') {
            out[n++] = c;
        }
    
    }
    
    /* Cleanup trailing dot if no extension */
    if (out[n-1] == '.') {
        out[n-1] = '\0';
    } else {
        out[n] = '\0';
    }

}

static void fcb_write_result_to_dta (CPU8086 *cpu, const char *name, uint32_t size, time_t modified, uint8_t attr, uint8_t extended) {

    char base[9], ext[4];
    
    uint16_t dos_date, dos_time;
    uint16_t dta_seg, dta_off;
    
    uint32_t dta_base, entry_ptr;
    int i;
    
    dos_get_dta (cpu, &dta_seg, &dta_off);
    dta_base = get_phys (dta_seg, dta_off);
    
    /* Standard FCB result at DTA+0, Extended at DTA+7 */
    entry_ptr = dta_base + (extended ? 7 : 0);
    
    if (extended) {
    
        mem_write8 (cpu->mem, cpu->mem_size, dta_base + 0x00, 0xFF);
        
        for (i = 1; i <= 5; i++) {
            mem_write8 (cpu->mem, cpu->mem_size, dta_base + i, 0);
        }
        
        mem_write8 (cpu->mem, cpu->mem_size, dta_base + 0x06, attr);
    
    }
    
    dos_make_83_display_parts (name, base, ext);
    
    {
    
        const char *dot;
        int j;
        
        memset (base, ' ', 8);
        memset (ext, ' ', 3);
        
        dot = (strcmp (name, ".") == 0 || strcmp (name, "..") == 0) ? NULL : strchr (name, '.');
        
        for (j = 0; name[j] && name + j != dot && j < 8; j++) {
            base[j] = (char) toupper ((int) name[j]);
        }
        
        if (dot) {
        
            dot++;
            
            for (j = 0; dot[j] && j < 3; j++) {
                ext[j] = (char) toupper ((int) dot[j]);
            }
        
        }
    
    }
    
    mem_write8 (cpu->mem, cpu->mem_size, entry_ptr + 0x00, 3);             /* Drive C: */
    
    for (i = 0; i < 8; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, entry_ptr + 0x01 + i, (uint8_t) base[i]);
    }
    
    for (i = 0; i < 3; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, entry_ptr + 0x09 + i, (uint8_t) ext[i]);
    }
    
    mem_write8 (cpu->mem, cpu->mem_size, entry_ptr + 0x0C, attr);
    
    /* Reserved area (0x0D to 0x16) */
    for (i = 0x0D; i <= 0x16; i++) {
        mem_write8 (cpu->mem, cpu->mem_size, entry_ptr + i, 0);
    }
    
    /* Time (0x17) and Date (0x19) */
    if (attr & 0x08) {
    
        dos_time = 0;
        dos_date = 0;
    
    } else {
        dos_get_file_datetime (modified, &dos_date, &dos_time);
    }
    
    mem_write16 (cpu->mem, cpu->mem_size, entry_ptr + 0x17, dos_time);
    mem_write16 (cpu->mem, cpu->mem_size, entry_ptr + 0x19, dos_date);
    
    /* CRITICAL: Starting Cluster (0x1B). DOS skips files with cluster 0. */
    mem_write16 (cpu->mem, cpu->mem_size, entry_ptr + 0x1B, 0x0002);
    
    /* File Size (0x1D) */
    if (attr & 0x10) {
        size = 0;
    }
    
    mem_write32 (cpu->mem, cpu->mem_size, entry_ptr + 0x1D, size);

}

#define     DOS_FCB_SEARCH_MAX          128

typedef struct {
    
    DIR *dir;
    
    char dirpath[260];
    char pattern[32];
    
    uint8_t attr;
    uint8_t extended;
    uint8_t dot_phase;
    
    uint16_t dta_seg;
    uint16_t dta_off;
    
    char last_name[260];
    
    uint32_t last_size;
    uint8_t last_attr;
    
    int have_last;
    int active;
    
    time_t last_mtime;

} DOS_FCB_SEARCH;

static DOS_FCB_SEARCH dos_fcb_search[DOS_FCB_SEARCH_MAX];
static int fcb_forced_slot = -1;

#define     DOS_FCB_OPEN_MAX            64

typedef struct {

    FILE *file;
    
    uint16_t psp;
    uint16_t seg;
    uint16_t off;

} DOS_FCB_OPEN;

static DOS_FCB_OPEN dos_fcb_open[DOS_FCB_OPEN_MAX];

static int fcb_open_slot (uint16_t psp, uint16_t seg, uint16_t off) {

    int i;
    
    for (i = 0; i < DOS_FCB_OPEN_MAX; i++) {
    
        if (dos_fcb_open[i].file && dos_fcb_open[i].psp == psp && dos_fcb_open[i].seg == seg && dos_fcb_open[i].off == off) {
            return i;
        }
    
    }
    
    return -1;

}

static void fcb_close_slot (int slot) {

    if (slot >= 0) {
    
        fclose (dos_fcb_open[slot].file);
        memset (&dos_fcb_open[slot], 0, sizeof (dos_fcb_open[slot]));
    
    }

}

static int fcb_find_slot_lifo (uint16_t seg, uint16_t off, const char *dirpath) {

    int i;
    
    for (i = DOS_FCB_SEARCH_MAX - 1; i >= 0; i--) {
    
        if (dos_fcb_search[i].active && dos_fcb_search[i].dta_seg == seg && dos_fcb_search[i].dta_off == off && strcmp (dos_fcb_search[i].dirpath, dirpath) == 0) {
            return i;
        }
    
    }
    
    return -1;

}


static int fcb_alloc_free_slot (void) {

    int i;
    
    for (i = 0; i < DOS_FCB_SEARCH_MAX; i++) {
    
        if (!dos_fcb_search[i].active) {
        
            memset (&dos_fcb_search[i], 0, sizeof (dos_fcb_search[i]));
            return i;
        
        }
    
    }
    
    return -1;

}

static int fcb_alloc_slot (uint16_t seg, uint16_t off) {

    (void)seg;
    (void)off;
    
    return fcb_alloc_free_slot ();

}

static int dos_append_real_component (char *host_path, size_t host_sz, const char *dos_comp) {

    struct dirent *de;
    DIR *dir;
    
    if (!(dir = opendir (host_path))) {
        return 0;
    }
    
    while ((de = readdir (dir)) != NULL) {
    
        char unique[13];
        
        if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
            continue;
        }
        
        dos_make_83_display_name_unique (host_path, de->d_name, unique);
        
        if (strcasecmp (de->d_name, dos_comp) != 0 && strcasecmp (unique, dos_comp) != 0) {
            continue;
        }
        
        if (host_path[0] && host_path[strlen (host_path) - 1] != '/') {
            dos_append_bounded (host_path, host_sz, "/");
        }
        
        dos_append_bounded (host_path, host_sz, de->d_name);
        
        closedir (dir);
        return 1;
    
    }
    
    closedir (dir);
    return 0;

}

static void fcb_make_current_host_dir (char *dirpath, size_t dirpath_sz) {

    char tmp[260];
    
    char *saveptr = NULL;
    char *tok;
    
    snprintf (dirpath, dirpath_sz, "%s", dos_drive_root);
    snprintf (tmp, sizeof (tmp), "%s", dos_current_dir);
    
    dos_normalize_path (tmp);
    tok = dos_path_token (tmp, &saveptr);
    
    while (tok) {
    
        if (!dos_append_real_component (dirpath, dirpath_sz, tok)) {
        
            if (dirpath[0] && dirpath[strlen (dirpath) - 1] != '/') {
                dos_append_bounded (dirpath, dirpath_sz, "/");
            }
            
            dos_append_bounded (dirpath, dirpath_sz, tok);
        
        }
        
        tok = dos_path_token (NULL, &saveptr);
    
    }

}

static int dos_component_matches_83 (const char *dos, const char *host_dir, const char *host) {

    char unique[13];
    
    if (strcasecmp (host, dos) == 0) {
        return 1;
    }
    
    dos_make_83_display_name_unique (host_dir, host, unique);
    return strcasecmp (unique, dos) == 0;

}

static int dos_resolve_existing_host_path (const char *dos_rel, char *host_out, size_t host_out_sz) {

    char tmp[260];
    char cur[520];
    
    char *saveptr = NULL;
    char *tok;
    
    if (host_out_sz == 0) {
        return 0;
    }
    
    cur[0] = '\0';
    dos_append_bounded (cur, sizeof (cur), dos_drive_root);
    
    tmp[0] = '\0';
    dos_append_bounded (tmp, sizeof (tmp), dos_rel);
    
    tok = dos_path_token (tmp, &saveptr);
    
    while (tok) {
    
        struct dirent *de;
        DIR *d;
        
        int found = 0;
        
        if (!(d = opendir (cur))) {
            return 0;
        }
        
        while ((de = readdir (d)) != NULL) {
        
            char next[520];
            struct stat st;
            
            if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
                continue;
            }
            
            if (strcasecmp (de->d_name, tok) != 0 && !dos_component_matches_83 (tok, cur, de->d_name)) {
                continue;
            }
            
            next[0] = '\0';
            
            dos_append_bounded (next, sizeof (next), cur);
            dos_append_bounded (next, sizeof (next), "/");
            dos_append_bounded (next, sizeof (next), de->d_name);
            
            if (host_lstat (next, &st) == 0 && S_ISDIR (st.st_mode)) {
            
                cur[0] = '\0';
                dos_append_bounded (cur, sizeof (cur), next);
                
                found = 1;
                break;
            
            }
        
        }
        
        closedir (d);
        
        if (!found) {
            return 0;
        }
        
        tok = dos_path_token (NULL, &saveptr);
    
    }
    
    host_out[0] = '\0';
    dos_append_bounded (host_out, host_out_sz, cur);
    
    return 1;

}

static int dos_get_83_current_dir (char *out, size_t out_sz) {

    char current[520];
    char remaining[260];
    
    char host[260];
    char alias[13];
    
    char *saveptr = NULL;
    char *tok;
    
    size_t len;
    
    out[0] = '\0';
    current[0] = '\0';
    
    dos_append_bounded (current, sizeof (current), dos_drive_root);
    remaining[0] = '\0';
    
    dos_append_bounded (remaining, sizeof (remaining), dos_current_dir);
    tok = dos_path_token (remaining, &saveptr);
    
    while (tok) {
    
        if (!dos_83_cache_get_host (current, tok, host, sizeof (host))) {
            return 0;
        }
        
        dos_make_83_display_name_unique (current, host, alias);
        len = strlen (out);
        
        if (len + (len != 0) + strlen (alias) >= out_sz) {
            return 0;
        }
        
        if (len != 0) {
            dos_append_bounded (out, out_sz, "\\");
        }
        
        dos_append_bounded (out, out_sz, alias);
        
        if (strlen (current) + 1 + strlen (host) >= sizeof (current)) {
            return 0;
        }
        
        if (current[0] && current[strlen (current) - 1] != '/') {
            dos_append_bounded (current, sizeof (current), "/");
        }
        
        dos_append_bounded (current, sizeof (current), host);
        tok = dos_path_token (NULL, &saveptr);
    
    }
    
    return 1;

}

static int dos_truename (const char *name, char out[128]) {

    char full[260], cwd[260], canonical[260];
    char host_dir[520], host[260];
    
    char *saveptr = NULL;
    char *tok;
    
    const char *path = name;
    
    int absolute;
    int host_known = 1;
    
    size_t len;
    
    if (name[0] && name[1] == ':') {
    
        if (toupper ((unsigned char) name[0]) != 'C') {
            return 3;
        }
        
        path += 2;
        
        if (*path == '\0') {
            return 2;
        }
    
    } else if (strchr (name, ':') != NULL) {
        return 3;
    }
    
    if (path[0] == '\\' && path[1] == '\\') {
        return 3;                           /* No network drives are mapped. */
    }
    
    absolute = path[0] == '\\' || path[0] == '/';
    
    if (!absolute && !dos_get_83_current_dir (cwd, sizeof (cwd))) {
        return 3;
    }
    
    len = strlen (path);
    
    if (len >= sizeof (full)) {
        return 3;
    }
    
    full[0] = '\0';
    
    if (!absolute && cwd[0] != '\0') {
    
        if (strlen (cwd) + 1 + len >= sizeof (full)) {
            return 3;
        }
        
        dos_append_bounded (full, sizeof (full), cwd);
        dos_append_bounded (full, sizeof (full), "\\");
    
    }
    
    dos_append_bounded (full, sizeof (full), path);
    dos_normalize_path (full);
    
    canonical[0] = '\0';
    dos_append_bounded (canonical, sizeof (canonical), "C:\\");
    
    host_dir[0] = '\0';
    dos_append_bounded (host_dir, sizeof (host_dir), dos_drive_root);
    
    tok = dos_path_token (full, &saveptr);
    
    while (tok) {
    
        char component[13];
        char name_part[9], ext_part[4];
        
        int i, j;
        size_t namelen, extlen;
        
        const char *dot;
        
        /* TRUENAME accepts names that do not yet exist. */
        if (host_known && dos_83_cache_get_host (host_dir, tok, host, sizeof (host))) {
        
            dos_make_83_display_name_unique (host_dir, host, component);
            
            if (strlen (host_dir) + 1 + strlen (host) >= sizeof (host_dir)) {
                return 3;
            }
            
            dos_append_bounded (host_dir, sizeof (host_dir), "/");
            dos_append_bounded (host_dir, sizeof (host_dir), host);
        
        } else {
        
            host_known = 0;
            
            memset (name_part, 0, sizeof (name_part));
            memset (ext_part, 0, sizeof (ext_part));
            
            dot = strchr (tok, '.');
            
            namelen = dot ? (size_t) (dot - tok) : strlen (tok);
            extlen = dot ? strlen (dot + 1) : 0;
            
            if (namelen == 0) {
                return 2;
            }
            
            for (i = 0, j = 0; i < (int) namelen; i++) {
            
                unsigned char ch = (unsigned char) tok[i];
                
                if (ch == '*') {
                
                    while (j < 8) {
                        name_part[j++] = '?';
                    }
                
                } else if (ch == '?' || dos_char_ok_83 (ch) || ch >= 0x80) {
                
                    if (j < 8) {
                        name_part[j++] = (char) toupper (ch);
                    }
                
                } else {
                    return 3;
                }
            
            }
            
            for (i = 0, j = 0; i < (int) extlen; i++) {
            
                unsigned char ch = (unsigned char) dot[i + 1];
                
                if (ch == '*') {
                
                    while (j < 3) {
                        ext_part[j++] = '?';
                    }
                
                } else if (ch == '?' || dos_char_ok_83 (ch) || ch >= 0x80) {
                
                    if (j < 3) {
                        ext_part[j++] = (char) toupper (ch);
                    }
                
                } else {
                    return 3;
                }
            
            }
            
            component[0] = '\0';
            dos_append_bounded (component, sizeof (component), name_part);
            
            if (ext_part[0]) {
            
                dos_append_bounded (component, sizeof (component), ".");
                dos_append_bounded (component, sizeof (component), ext_part);
            
            }
        
        }
        
        if (strlen (canonical) + strlen (component) + 1 >= 128) {
            return 3;
        }
        
        if (canonical[3] != '\0') {
            dos_append_bounded (canonical, sizeof (canonical), "\\");
        }
        
        dos_append_bounded (canonical, sizeof (canonical), component);
        tok = dos_path_token (NULL, &saveptr);
    
    }
    
    strcpy (out, canonical);
    return 0;

}

#define     MAX_HANDLE_SEARCHES         64

typedef struct {
    
    DIR *dir;
    
    char dirpath[260];
    char pattern[64];
    
    uint8_t attr;
    uint8_t dot_phase;
    
    int active;

} HANDLE_SEARCH;

static HANDLE_SEARCH handle_searches[MAX_HANDLE_SEARCHES];

static int alloc_handle_search_slot () {

    for (int i = 0; i < MAX_HANDLE_SEARCHES; i++) {
    
        if (!handle_searches[i].active) {
        
            memset (&handle_searches[i], 0, sizeof (HANDLE_SEARCH));
            return i;
        
        }
    
    }
    
    return -1;

}

static void dos_install_case_map (CPU8086 *cpu) {

    /* FAR CALL: AL = CP437 character, AL = uppercase character. */
    static const uint8_t code[] = {
    
        0x9C,                   /* pushf */
        0x3C, 0x80,             /* cmp al, 80h */
        0x72, 0x0C,             /* jb popf */
        0x53,                   /* push bx */
        0x1E,                   /* push ds */
        0x0E,                   /* push cs */
        0x1F,                   /* pop ds */
        0x2C, 0x80,             /* sub al, 80h */
        0xBB, 0x13, 0x00,       /* mov bx, 0013h (uppercase table) */
        0xD7,                   /* xlat */
        0x1F,                   /* pop ds */
        0x5B,                   /* pop bx */
        0x9D,                   /* popf */
        0xCB                    /* retf */
    
    };
    static const uint8_t uppercase[128] = {
    
        0x80, 0x9A, 0x90, 0x83, 0x8E, 0x85, 0x8F, 0x80,
        0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
        0x90, 0x92, 0x92, 0x93, 0x99, 0x95, 0x96, 0x97,
        0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
        0xA0, 0xA1, 0xA2, 0xA3, 0xA5, 0xA5, 0xA6, 0xA7,
        0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
        0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
        0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
        0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
        0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
        0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
        0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE4, 0xE6, 0xE7,
        0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xE8, 0xEE, 0xEF,
        0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7,
        0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
    
    };
    
    uint32_t phys = (uint32_t) DOS_CASE_MAP_SEG << 4;
    size_t i;
    
    for (i = 0; i < sizeof (code); i++) {
        mem_write8 (cpu->mem, cpu->mem_size, phys + i, code[i]);
    }
    
    for (i = 0; i < sizeof (uppercase); i++) {
        mem_write8 (cpu->mem, cpu->mem_size, phys + sizeof (code) + i, uppercase[i]);
    }

}

static int dos_host_file_in_use (const char *path, const struct stat *identity) {

    int i;
    
#if     defined (_WIN32)
    BY_HANDLE_FILE_INFORMATION target_info, open_info;
    HANDLE target;
    
    (void) identity;
    
    target = CreateFileA (path, FILE_READ_ATTRIBUTES,
                          FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                          NULL, OPEN_EXISTING, FILE_FLAG_OPEN_REPARSE_POINT, NULL);
    
    if (target == INVALID_HANDLE_VALUE) {
        return 0;
    }
    
    if (!GetFileInformationByHandle (target, &target_info)) {
    
        CloseHandle (target);
        return 0;
    
    }
    
    CloseHandle (target);
    
    if (target_info.nFileIndexHigh == 0 && target_info.nFileIndexLow == 0) {
        return 0;
    }
    
    for (i = 0; i < MAX_DOS_HANDLES; i++) {
    
        intptr_t handle;
        
        if (!file_handles[i]) {
            continue;
        }
        
        handle = _get_osfhandle (_fileno (file_handles[i]));
        
        if (handle == (intptr_t) -1 || handle == (intptr_t) -2) {
            continue;
        }
        
        if (!GetFileInformationByHandle ((HANDLE) handle, &open_info)) {
            continue;
        }
        
        if (target_info.dwVolumeSerialNumber == open_info.dwVolumeSerialNumber &&
            target_info.nFileIndexHigh == open_info.nFileIndexHigh &&
            target_info.nFileIndexLow == open_info.nFileIndexLow) {
            return 1;
        }
    
    }
#else
    (void) path;
    
    for (i = 0; i < MAX_DOS_HANDLES; i++) {
    
        struct stat open_st;
        
        if (!file_handles[i]) {
            continue;
        }
        
        if (fstat (fileno (file_handles[i]), &open_st) != 0) {
            continue;
        }
        
        if (open_st.st_dev == identity->st_dev && open_st.st_ino == identity->st_ino) {
            return 1;
        }
    
    }
#endif
    
    return 0;

}

static int dos_get_disk_allocation (uint8_t drive, uint16_t *sectors, uint16_t *free, uint16_t *total) {

    uint64_t total_bytes;
    uint64_t free_bytes;
    uint64_t cluster_bytes;
    uint64_t total_clusters;
    uint64_t free_clusters;
    
    uint16_t sectors_per_cluster = 8;
    
    if (drive != 0 && drive != 3) {
        return 0;
    }
    
#if     defined (_WIN32)
    {
    
        ULARGE_INTEGER available, total, unused;
        
        if (!GetDiskFreeSpaceExA (dos_drive_root, &available, &total, &unused)) {
            return 0;
        }
        
        total_bytes = total.QuadPart;
        free_bytes = available.QuadPart;
    
    }
#else
    {
    
        struct statvfs st;
        uint64_t block_size;
        
        if (statvfs (dos_drive_root, &st) != 0) {
            return 0;
        }
        
        block_size = st.f_frsize ? st.f_frsize : st.f_bsize;
        
        total_bytes = (uint64_t) st.f_blocks * block_size;
        free_bytes = (uint64_t) st.f_bavail * block_size;
    
    }
#endif
    
    /* Fit the host volume's free-space report into DOS's 16-bit cluster fields. */
    cluster_bytes = (uint64_t) sectors_per_cluster * 512;
    
    while (sectors_per_cluster < 64 && total_bytes / cluster_bytes > 0xFFFF) {
    
        sectors_per_cluster *= 2;
        cluster_bytes *= 2;
    
    }
    
    total_clusters = total_bytes / cluster_bytes;
    free_clusters = free_bytes / cluster_bytes;
    
    if (total_clusters > 0xFFFF) {
        total_clusters = 0xFFFF;
    }
    
    if (free_clusters > total_clusters) {
        free_clusters = total_clusters;
    }
    
    *sectors = sectors_per_cluster;
    
    *free = (uint16_t) free_clusters;
    *total = (uint16_t) total_clusters;
    
    return 1;

}

static void int_21 (CPU8086 *cpu) {

    uint8_t ah = (cpu->regs[REG_AX] >> 8) & 0xFF;
    
    switch (ah) {
    
        case 0x00:                      /* Terminate Program */
        
            if (dos_exec_depth != 0) {
            
                dos_last_return_code[dos_return_slot (cpu)] = 0;
                dos_return_code_pending[dos_return_slot (cpu)] = 1;
            
            }
            
            dos_terminate_process (cpu);
            break;
        
        case 0x01: {                    /* Character Input with Echo */
        
            int c = host_read_key (cpu);
            
            if (c == EOF) {
                c = '\r';
            }
            
            if (c == '\n') {
                c = '\r';
            }
            
            if (c == 0x03) {            /* Ctrl+C: invoke INT 23h */
            
                uint32_t vec = 0x23 * 4;
                uint16_t new_ip = mem_read16 (cpu->mem, cpu->mem_size, vec + 0);
                uint16_t new_cs = mem_read16 (cpu->mem, cpu->mem_size, vec + 2);
                
                cpu->regs[REG_SP] -= 2;
                mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP]), cpu->flags);
                
                cpu->regs[REG_SP] -= 2;
                mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP]), cpu->cs);
                
                cpu->regs[REG_SP] -= 2;
                mem_write16 (cpu->mem, cpu->mem_size, get_phys (cpu->ss, cpu->regs[REG_SP]), cpu->ip);
                
                cpu->cs = new_cs;
                cpu->ip = new_ip;
                
                break;
            
            }
            
            set_reg8 (cpu, REG_AX, (uint8_t) c);
            
            if (c == '\r') {
            
                putchar ('\r');
                putchar ('\n');
            
            } else {
                dos_write_console_char (stdout, (uint8_t) c);
            }
            
            fflush (stdout);
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x02: {                    /* Character Output */
        
            uint8_t dl = get_reg8 (cpu, 2);
            
            FILE *out = dos_get_handle_file (cpu, 1);
            dos_write_standard_output (cpu, dl);
            
            if (out != NULL) {
                fflush (out);
            }
            
            break;
        
        }
        
        case 0x06: {                    /* Direct Console Input/Output */
        
            uint8_t dl = get_reg8 (cpu, REG_DX);
            
            if (dl == 0xFF) {
            
                int ch = host_poll_key_from (file_handles[0] ? file_handles[0] : stdin);
                
                if (ch == EOF) {
                
                    set_reg8 (cpu, REG_AX, 0);
                    cpu->flags |= FLAG_ZF;
                
                } else {
                
                    set_reg8 (cpu, REG_AX, (uint8_t) ch);
                    cpu->flags &= (uint16_t) ~FLAG_ZF;
                
                }
            
            } else {
            
                dos_write_console_char (stdout, dl);
                
                fflush (stdout);
                set_reg8 (cpu, REG_AX, dl);
            
            }
            
            break;
        
        }
        
        case 0x07: {                    /* Direct Character Input without Echo */
        
            int c = host_read_key (cpu);
            
            if (c == EOF) {
                c = '\r';
            }
            
            if (c == '\n') {
                c = '\r';
            }
            
            set_reg8 (cpu, REG_AX, (uint8_t) c);
            break;
        
        }
        
        case 0x09: {
        
            FILE *out = dos_get_handle_file (cpu, 1);
            uint16_t off = cpu->regs[REG_DX];
            
            while (1) {
            
                uint8_t ch = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, off));
                
                if (ch == '$') {
                    break;
                }
                
                if (ch == '\n' && (out == stdout || out == stderr)) {
                    dos_write_standard_output (cpu, '\r');
                }
                
                dos_write_standard_output (cpu, ch);
                off = (uint16_t) (off + 1);
            
            }
            
            if (out != NULL) {
                fflush (out);
            }
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x0A: {                    /* Buffered keyboard input */
        
            dos_buffered_line_input (cpu);
            break;
            
        }
        
        case 0x0B: {                    /* Check Standard Input Status */
        
            FILE *source = dos_get_handle_file (cpu, 0);
            
            int available = host_key_available_from (source ? source : stdin);
            set_reg8 (cpu, REG_AX, available ? 0xFF : 0x00);
            
            break;
        
        }
        
        case 0x0C: {                    /* Flush keyboard buffer and invoke input function AL */
        
            uint8_t subfn = get_reg8 (cpu, REG_AX); /* AL */
            
            switch (subfn) {
                
                case 0x01: {            /* Read char with echo */
                
                    int c = host_read_key (cpu);
                    
                    if (c == EOF) {
                        c = '\r';
                    }
                    
                    /* swallow trailing LF after Enter if present */
                    if (c == '\n') {
                        c = '\r';
                    }
                    
                    set_reg8 (cpu, REG_AX, (uint8_t) c);    /* AL */
                    
                    if (c != '\r') {
                    
                        dos_write_console_char (stdout, (uint8_t) c);
                        fflush (stdout);
                    
                    }
                    
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                }
                
                case 0x07:              /* direct char input, no echo */
                case 0x08: {            /* char input, no echo */
                
                    int c = host_read_key (cpu);
                    
                    if (c == EOF) {
                        c = '\r';
                    }
                    
                    set_reg8 (cpu, REG_AX, (uint8_t) c); /* AL */
                    cpu->flags &= ~FLAG_CF;
                    
                    break;
                
                }
                
                case 0x0A: {            /* buffered line input */
                
                    dos_buffered_line_input (cpu);
                    break;
                
                }
                
                default:
                
                    dos_set_error (cpu, 1);
                    break;
            
            }
            
            break;
        
        }
        
        case 0x0D: {                    /* Disk Reset */
        
            /* Flush pending writes without changing registers or flags. */
            fflush (NULL);
            break;
        
        }
        
        case 0x0E: {                    /* Select Default Drive */
        
            uint8_t drive = cpu->regs[REG_DX] & 0x00FF; /* DL */
            
            /* simple stub: accept any requested drive */
            (void) drive;
            
            /* return highest possible drive count in AL; pretend A:, B:, C: exist */
            cpu->regs[REG_AX] = (cpu->regs[REG_AX] & 0xFF00) | 3;
            cpu->flags &= ~FLAG_CF;
            
            break;
        
        }
        
        case 0x0F: {                    /* FCB Open File */
        
            uint16_t off = cpu->regs[REG_DX];
            uint16_t seg = cpu->ds;
            
            uint32_t base;
            uint16_t date, time;
            
            char name[32], path[260];
            
            struct stat st;
            FILE *fp;
            
            int slot, i;
            
            if (mem_read8 (cpu->mem, cpu->mem_size, get_phys (seg, off)) == 0xFF) {
                off = (uint16_t) (off + 7);
            }
            
            base = get_phys (seg, off);
            fcb_make_pattern (cpu, seg, off, name);
            
            if (name[0] == '\0' || strchr (name, '*') || strchr (name, '?')) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            dos_resolve_path (name, path, sizeof (path));
            fp = dos_fopen_case_insensitive (path, "rb+");
            
            if (!fp) {
                fp = dos_fopen_case_insensitive (path, "rb");
            }
            
            if (!fp) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            slot = fcb_open_slot (cpu->psp_seg, seg, off);
            
            if (slot >= 0) {
                fcb_close_slot (slot);
            } else {
            
                for (i = 0; i < DOS_FCB_OPEN_MAX && dos_fcb_open[i].file; i++) {}
                slot = i < DOS_FCB_OPEN_MAX ? i : -1;
            
            }
            
            if (slot < 0 || fstat (fileno (fp), &st) != 0 || !S_ISREG (st.st_mode)) {
            
                fclose (fp);
                
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            dos_fcb_open[slot].file = fp;
            dos_fcb_open[slot].psp = cpu->psp_seg;
            dos_fcb_open[slot].seg = seg;
            dos_fcb_open[slot].off = off;
            
            dos_get_file_datetime (st.st_mtime, &date, &time);
            
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x0C, 0);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x0E, 128);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x10, (uint16_t) st.st_size);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x12, (uint16_t) ((uint64_t) st.st_size >> 16));
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x14, date);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x16, time);
            
            mem_write8 (cpu->mem, cpu->mem_size, base + 0x20, 0);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x21, 0);
            
            mem_write8 (cpu->mem, cpu->mem_size, base + 0x23, 0);
            set_reg8 (cpu, REG_AX, 0);
            
            break;
        
        }
        
        case 0x10: {                    /* FCB Close File */
        
            uint16_t off = cpu->regs[REG_DX];
            int slot;
            
            if (mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, off)) == 0xFF) {
                off = (uint16_t) (off + 7);
            }
            
            slot = fcb_open_slot (cpu->psp_seg, cpu->ds, off);
            
            set_reg8 (cpu, REG_AX, slot < 0 ? 0xFF : 0);
            fcb_close_slot (slot);
            
            break;
        
        }
        
        case 0x11: {                    /* FCB Find First */
        
            char pattern[32];
            char dirpath[260];
            
            uint8_t attr = 0;
            uint8_t extended = 0;
            
            uint16_t fcb_off = cpu->regs[REG_DX];
            
            uint8_t first;
            int slot;
            
            first = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, fcb_off));
            
            if (first == 0xFF) {
            
                attr = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, fcb_off + 6));
                
                fcb_off += 7;
                extended = 1;
            
            } else {
            
                attr = 0;
                extended = 0;
            
            }
            
            /* Volume Label is a one-shot; handle it and return. */
            if (attr & 0x08) {
            
                fcb_write_result_to_dta (cpu, "EMU8086", 0, 0, 0x08, extended);
                
                set_reg8 (cpu, 0, 0x00);
                cpu->flags &= ~FLAG_CF;
                
                break;
            
            }
            
            fcb_make_current_host_dir (dirpath, sizeof (dirpath));
            slot = fcb_find_slot_lifo (cpu->ds, fcb_off, dirpath);
            
            if (slot >= 0) {
            
                if (dos_fcb_search[slot].dir) {
                    closedir (dos_fcb_search[slot].dir);
                }
            
            } else {
                slot = fcb_alloc_slot (cpu->ds, fcb_off);
            }
            
            if (slot < 0) {
            
                set_reg8 (cpu, 0, 0xFF);
                dos_last_error = 0x0004;
                
                cpu->flags &= ~FLAG_CF;
                break;
            
            }
            
            memset (&dos_fcb_search[slot], 0, sizeof (dos_fcb_search[slot]));
            
            fcb_make_pattern (cpu, cpu->ds, fcb_off, pattern);
            
            if (pattern[0] == '\0' || strcmp (pattern, ".") == 0) {
                strcpy (pattern, "*.*");
            }
            
            dos_fcb_search[slot].dir = opendir (dirpath);
            
            if (!dos_fcb_search[slot].dir) {
            
                set_reg8 (cpu, 0, 0xFF);
                dos_last_error = 0x0003;
                
                cpu->flags &= ~FLAG_CF;
                break;
            
            }
            
            /* 2. Set up the slot metadata */
            snprintf (dos_fcb_search[slot].dirpath, sizeof (dos_fcb_search[slot].dirpath), "%s", dirpath);
            snprintf (dos_fcb_search[slot].pattern, sizeof (dos_fcb_search[slot].pattern), "%s", pattern);
            
            dos_fcb_search[slot].attr = attr;
            dos_fcb_search[slot].extended = extended;
            dos_fcb_search[slot].dta_seg = cpu->ds;
            dos_fcb_search[slot].dta_off = fcb_off;
            dos_fcb_search[slot].active = 1;
            
            /* 3. NOW check for Volume Label. If requested, return it as the first result. */
            /* This leaves the directory open for the next AH=12h call. */
            if (attr & 0x08) {
            
                fcb_write_result_to_dta (cpu, "EMU8086", 0, 0, 0x08, extended);
                
                set_reg8 (cpu, 0, 0x00);
                cpu->flags &= ~FLAG_CF;
                
                break;
            
            }
            
            goto fcb_find_next_common;
        
        }
        
        case 0x12: {                    /* FCB Find Next */
        
            fcb_find_next_common:
            {
            
                DOS_FCB_SEARCH *s;
                
                const char *name;
                int found = 0, slot;
                
                uint16_t fcb_off = cpu->regs[REG_DX];
                uint8_t first = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, fcb_off));
                
                if (first == 0xFF) {
                    fcb_off += 7;
                }
                
                if (fcb_forced_slot >= 0) {
                
                    slot = fcb_forced_slot;
                    fcb_forced_slot = -1;
                
                } else {
                
                    char dirpath[260];
                    
                    fcb_make_current_host_dir (dirpath, sizeof (dirpath));
                    
                    if ((slot = fcb_find_slot_lifo (cpu->ds, fcb_off, dirpath)) < 0) {
                    
                        int i;
                        
                        /* FindNext continues the original search even after CHDIR. */
                        for (i = DOS_FCB_SEARCH_MAX - 1; i >= 0; i--) {
                        
                            if (dos_fcb_search[i].active && dos_fcb_search[i].dta_seg == cpu->ds && dos_fcb_search[i].dta_off == fcb_off) {
                            
                                slot = i;
                                break;
                            
                            }
                        
                        }
                    
                    }
                
                }
                
                if (slot < 0 || slot >= DOS_FCB_SEARCH_MAX) {
                
                    set_reg8 (cpu, 0, 0xFF);
                    dos_last_error = 0x0012;
                    
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                }
                
                s = &dos_fcb_search[slot];
                
                if (!s->active || !s->dir) {
                
                    set_reg8 (cpu, 0, 0xFF);
                    dos_last_error = 0x0012;
                    
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                }
                
                while ((name = dos_find_next_directory_entry (s->dir, s->dirpath, &s->dot_phase)) != NULL) {
                
                    struct stat st;
                    
                    char full[520];
                    uint8_t file_attr;
                    
                    if (name[0] == '.' && strcmp (name, ".") != 0 && strcmp (name, "..") != 0) {
                        continue;
                    }
                    
                    {
                    
                        char dos_name[13];
                        dos_make_83_display_name_unique (s->dirpath, name, dos_name);
                        
                        if (!dos_wildcard_match (s->pattern, dos_name)) {
                            continue;
                        }
                    
                    }
                    
                    snprintf (full, sizeof (full), "%s/%s", s->dirpath, name);
                    
                    if (host_lstat (full, &st) != 0) {
                        continue;
                    }
                    
                    file_attr = dos_host_file_attributes (full, &st);
                    
                    if ((file_attr & 0x02) && !(s->attr & 0x02)) {
                        continue;
                    }
                    
                    if ((file_attr & 0x04) && !(s->attr & 0x04)) {
                        continue;
                    }
                    
                    if (S_ISDIR (st.st_mode)) {
                    
                        /* If it's a directory, return it ONLY if 0x10 is in the search mask */
                        if (!(s->attr & 0x10)) {
                            continue;
                        }
                    
                    } else {
                    
                        /* If it's a file, skip it if the search is ONLY for Volume Labels (0x08) */
                        if (s->attr == 0x08) {
                            continue;
                        }
                    
                    }
                    
                    s->last_size = (uint32_t) st.st_size;
                    
                    {
                    
                        char dos_name[13];
                        
                        dos_make_83_display_name_unique (s->dirpath, name, dos_name);
                        snprintf (s->last_name, sizeof (s->last_name), "%s", dos_name);
                        
                        s->last_size = (uint32_t) st.st_size;
                        s->last_mtime = st.st_mtime;
                        
                        s->last_attr = file_attr;
                        s->have_last = 1;
                        
                        fcb_write_result_to_dta (cpu, dos_name, (uint32_t) st.st_size, st.st_mtime, file_attr, s->extended);
                    
                    }
                    
                    found = 1;
                    break;
                
                }
                
                if (!found) {
                
                    dos_last_error = 0x0012;
                    
                    if (s->dir) {
                        closedir (s->dir);
                    }
                    
                    memset (s, 0, sizeof (*s));
                
                }
                
                set_reg8 (cpu, 0, found ? 0x00 : 0xFF);
                cpu->flags &= ~FLAG_CF;
                
                break;
            
            }
        
        }
        
        case 0x13: {                    /* FCB Delete File */
        
            uint16_t fcb_off = cpu->regs[REG_DX];
            
            uint8_t drive;
            uint8_t attr = 0;
            uint8_t first;
            
            char pattern[32];
            char dirpath[260];
            
            struct dirent *de;
            DIR *dir;
            
            int deleted = 0;
            int i;
            int has_name = 0;
            
            first = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, fcb_off));
            
            if (first == 0xFF) {
            
                attr = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, (uint16_t) (fcb_off + 6)));
                fcb_off += 7;
            
            }
            
            drive = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, fcb_off));
            
            for (i = 1; i <= 8; i++) {
            
                uint8_t ch = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, (uint16_t) (fcb_off + i)));
                
                if (ch != ' ' && ch != 0) {
                    has_name = 1;
                }
            
            }
            
            /* Only C: is currently backed by the host filesystem. */
            if ((drive != 0 && drive != 3) || !has_name) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            fcb_make_pattern (cpu, cpu->ds, fcb_off, pattern);
            fcb_make_current_host_dir (dirpath, sizeof (dirpath));
            
            dir = opendir (dirpath);
            
            if (!dir) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            while ((de = readdir (dir)) != NULL) {
            
                char dos_name[13];
                char full[520];
                
                struct stat st;
                
                if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
                    continue;
                }
                
                if (de->d_name[0] == '.' && !(attr & 0x02)) {
                    continue;
                }
                
                dos_make_83_display_name_unique (dirpath, de->d_name, dos_name);
                
                if (!dos_wildcard_match (pattern, dos_name)) {
                    continue;
                }
               
                snprintf (full, sizeof (full), "%s/%s", dirpath, de->d_name);
                
                if (host_lstat (full, &st) != 0 || !S_ISREG (st.st_mode)) {
                    continue;
                }
                
                if (!(st.st_mode & S_IWUSR) && !(attr & 0x01)) {
                    continue;
                }
                
                if (dos_host_file_in_use (full, &st)) {
                    continue;
                }
                
                if (remove (full) == 0) {
                
#ifndef     _WIN32
                    dos_file_attribute_forget (full);
#endif
                    deleted = 1;
                
                }
            
            }
            
            closedir (dir);
            
            if (deleted) {
                dos_83_cache_invalidate ();
            }
            
            set_reg8 (cpu, REG_AX, deleted ? 0x00 : 0xFF);
            break;
        
        }
        
        case 0x16: {                    /* FCB Create File */
            
            uint16_t fcb_off = cpu->regs[REG_DX];
            
            char name[64];
            char path[260];
            
            uint8_t first;
            FILE *fp;
            
            uint32_t base;
            int slot, i;
            
            first = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, fcb_off));
            
            if (first == 0xFF) {
                fcb_off += 7;
            }
            
            fcb_make_pattern (cpu, cpu->ds, fcb_off, name);
            
            if (name[0] == '\0' || strcmp (name, "*.*") == 0) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                
                dos_last_error = 0x0002;
                cpu->flags &= ~FLAG_CF;
                
                break;
            
            }
            
            dos_resolve_path (name, path, sizeof (path));
            
            {
            
                struct stat existing;
                
                if (stat (path, &existing) == 0 && (dos_host_file_attributes (path, &existing) & 0x01)) {
                    
                    set_reg8 (cpu, REG_AX, 0xFF);
                    
                    dos_last_error = 0x0005;
                    cpu->flags &= ~FLAG_CF;
                    
                    break;
                
                }
            
            }
            
            fp = fopen (path, "wb+");
            
            if (!fp) {
            
                set_reg8 (cpu, REG_AX, 0xFF);   /* no room / create failed */
                
                dos_last_error = 0x0005;
                cpu->flags &= ~FLAG_CF;
                
                break;
            
            }
            
            slot = fcb_open_slot (cpu->psp_seg, cpu->ds, fcb_off);
            
            if (slot >= 0) {
                fcb_close_slot (slot);
            } else {
            
                for (i = 0; i < DOS_FCB_OPEN_MAX && dos_fcb_open[i].file; i++) {}
                slot = i < DOS_FCB_OPEN_MAX ? i : -1;
            
            }
            
            if (slot < 0) {
            
                fclose (fp);
                
                set_reg8 (cpu, REG_AX, 0xFF);
                dos_last_error = 0x0004;
                
                cpu->flags &= ~FLAG_CF;
                break;
            
            }
            
            dos_fcb_open[slot].file = fp;
            dos_fcb_open[slot].psp = cpu->psp_seg;
            dos_fcb_open[slot].seg = cpu->ds;
            dos_fcb_open[slot].off = fcb_off;
            
            base = get_phys (cpu->ds, fcb_off);
            
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x0C, 0);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x0E, 128);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x10, 0);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x12, 0);
            
            mem_write8 (cpu->mem, cpu->mem_size, base + 0x20, 0);
            
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x21, 0);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x23, 0);
            
#ifndef     _WIN32
            dos_file_attribute_forget (path);
#endif
            
            dos_83_cache_invalidate ();
            set_reg8 (cpu, REG_AX, 0x00);
            
            dos_last_error = 0;
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x17: {                    /* FCB Rename File */
        
            uint16_t fcb_off = cpu->regs[REG_DX];
            uint32_t base;
            
            uint8_t drive;
            uint8_t new_drive;
            uint8_t first;
            
            char pattern[32];
            char dirpath[260];
            
            struct dirent *de;
            DIR *dir;
            
            int renamed = 0;
            int failed = 0;
            
            first = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, fcb_off));
            
            if (first == 0xFF) {
                fcb_off += 7;
            }
            
            base = get_phys (cpu->ds, fcb_off);
            drive = mem_read8 (cpu->mem, cpu->mem_size, base);
            new_drive = mem_read8 (cpu->mem, cpu->mem_size, base + 0x10);
            
            if ((drive != 0 && drive != 3) || (new_drive != 0 && new_drive != 3) || (drive != 0 && new_drive != 0 && drive != new_drive)) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            fcb_make_pattern (cpu, cpu->ds, fcb_off, pattern);
            fcb_make_current_host_dir (dirpath, sizeof (dirpath));
            
            dir = opendir (dirpath);
            
            if (!dir) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            while ((de = readdir (dir)) != NULL) {
            
                char alias[13];
                char old_parts[11];
                char new_parts[11];
                char new_name[13];
                char old_path[520];
                char new_path[520];
                
                struct stat st;
                int j, n;
                
                if (strcmp (de->d_name, ".") == 0 || strcmp (de->d_name, "..") == 0) {
                    continue;
                }
                
                dos_make_83_display_name_unique (dirpath, de->d_name, alias);
                
                if (!dos_wildcard_match (pattern, alias)) {
                    continue;
                }
                
                snprintf (old_path, sizeof (old_path), "%s/%s", dirpath, de->d_name);
                
                if (host_lstat (old_path, &st) != 0 || !S_ISREG (st.st_mode)) {
                    continue;
                }
                
                memset (old_parts, ' ', sizeof (old_parts));
                n = 0;
                
                for (j = 0; j < 8 && alias[n] && alias[n] != '.'; j++) {
                    old_parts[j] = alias[n++];
                }
                
                while (alias[n] && alias[n] != '.') {
                    n++;
                }
                
                if (alias[n] == '.') {
                    n++;
                }
                
                for (j = 8; j < 11 && alias[n]; j++) {
                    old_parts[j] = alias[n++];
                }
                
                for (j = 0; j < 11; j++) {
                
                    uint8_t ch = mem_read8 (cpu->mem, cpu->mem_size, base + 0x11 + j);
                    new_parts[j] = ch == '?' ? old_parts[j] : (char) ch;
                    
                    if (new_parts[j] == '*' || new_parts[j] == '/' || new_parts[j] == '\\' || new_parts[j] == ':' || new_parts[j] == 0) {
                    
                        failed = 1;
                        break;
                    
                    }
                
                }
                
                if (failed) {
                    break;
                }
                
                n = 0;
                
                for (j = 0; j < 8 && new_parts[j] != ' '; j++) {
                    new_name[n++] = new_parts[j];
                }
                
                if (!n) {
                
                    failed = 1;
                    break;
                
                }
                
                if (new_parts[8] != ' ') {
                
                    new_name[n++] = '.';
                    
                    for (j = 8; j < 11 && new_parts[j] != ' '; j++) {
                        new_name[n++] = new_parts[j];
                    }
                
                }
                
                new_name[n] = 0;
                snprintf (new_path, sizeof (new_path), "%s/%s", dirpath, new_name);
                
                if (strcasecmp (old_path, new_path) != 0 && host_lstat (new_path, &st) == 0) {
                
                    failed = 1;
                    break;
                
                }
                
                if (rename (old_path, new_path) != 0) {
                
                    failed = 1;
                    break;
                
                }
                
#ifndef     _WIN32
                dos_file_attribute_rename (old_path, new_path);
#endif
                renamed = 1;
            
            }
            
            closedir (dir);
            
            if (renamed) {
                dos_83_cache_invalidate ();
            }
            
            set_reg8 (cpu, REG_AX, renamed && !failed ? 0x00 : 0xFF);
            break;
        
        }
        
        case 0x19: {                    /* Get Current Default Drive */
        
            cpu->regs[REG_AX] = (cpu->regs[REG_AX] & 0xFF00) | 2;               /* AL = 2 => C: */
            cpu->flags &= ~FLAG_CF;
            
            break;
        
        }
        
        case 0x1A: {
        
            dos_dta_seg = cpu->ds;
            dos_dta_off = cpu->regs[REG_DX];
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x1B: /* Get Allocation Information for Default Drive */   case 0x1C: /* Get Allocation Information for Specified Drive */ {
        
            uint8_t drive = ah == 0x1B ? 0 : get_reg8 (cpu, REG_DX);
            uint16_t sectors, free_clusters, total_clusters;
            
            if (!dos_get_disk_allocation (drive, &sectors, &free_clusters, &total_clusters)) {
            
                set_reg8 (cpu, REG_AX, 0xFF);
                break;
            
            }
            
            /* This byte resides in reserved DOS memory, outside the application arena. */
            mem_write8 (cpu->mem, cpu->mem_size, ((uint32_t) DOS_CASE_MAP_SEG << 4) + 0xF0, 0xF8);
            set_reg8 (cpu, REG_AX, (uint8_t) sectors);
            
            cpu->regs[REG_BX] = 0x00F0;
            cpu->regs[REG_CX] = 512;
            cpu->regs[REG_DX] = total_clusters;
            
            cpu->ds = DOS_CASE_MAP_SEG;
            break;
        
        }
        
        case 0x25: {                    /* Set Interrupt Vector */
        
            uint8_t int_num = cpu->regs[REG_AX] & 0x00FF;   /* AL */
            uint16_t new_off = cpu->regs[REG_DX];           /* DX */
            uint16_t new_seg = cpu->ds;                     /* DS */
            
            uint32_t ivt_addr = (uint32_t) int_num * 4;
            
            mem_write16 (cpu->mem, cpu->mem_size, ivt_addr + 0, new_off);
            mem_write16 (cpu->mem, cpu->mem_size, ivt_addr + 2, new_seg);
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x26: {                    /* Create Program Segment Prefix */
        
            uint32_t source = (uint32_t) cpu->ds << 4;
            uint32_t destination = (uint32_t) cpu->regs[REG_DX] << 4;
            
            uint8_t contents[256];
            int i;
            
            for (i = 0; i < 256; i++) {
                contents[i] = mem_read8 (cpu->mem, cpu->mem_size, source + i);
            }
            
            for (i = 0; i < 256; i++) {
                mem_write8 (cpu->mem, cpu->mem_size, destination + i, contents[i]);
            }
            
            /* Save the currently installed termination and error vectors. */
            for (i = 0; i < 12; i++) {
            
                mem_write8 (cpu->mem, cpu->mem_size, destination + 0x0A + i,
                    mem_read8 (cpu->mem, cpu->mem_size, (uint32_t) (0x22 * 4) + i));
            
            }
            
            /* AH=26h does not establish a DOS 2+ parent/child relationship. */
            mem_write16 (cpu->mem, cpu->mem_size, destination + 0x16, 0);
            break;
        
        }
        
        case 0x27: {                    /* FCB Random Block Read */
        
            uint16_t off = cpu->regs[REG_DX];
            uint16_t count = cpu->regs[REG_CX];
            uint16_t record_size;
            uint16_t dta_seg, dta_off;
            
            uint32_t base, record, destination;
            uint32_t completed = 0;
            
            uint64_t position;
            
            int slot;
            uint8_t status = 0;
            
            if (mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, off)) == 0xFF) {
                off = (uint16_t) (off + 7);
            }
            
            base = get_phys (cpu->ds, off);
            slot = fcb_open_slot (cpu->psp_seg, cpu->ds, off);
            
            record_size = mem_read16 (cpu->mem, cpu->mem_size, base + 0x0E);
            record = (uint32_t) mem_read16 (cpu->mem, cpu->mem_size, base + 0x21) | ((uint32_t) mem_read16 (cpu->mem, cpu->mem_size, base + 0x23) << 16);
            
            dos_get_dta (cpu, &dta_seg, &dta_off);
            destination = get_phys (dta_seg, dta_off);
            
            if (slot < 0 || record_size == 0) {
                status = 1;
            } else if ((uint32_t) count * record_size > 0x10000UL - dta_off || (uint64_t) destination + (uint32_t) count * record_size > cpu->mem_size) {
                status = 2;
            } else {
            
                position = (uint64_t) record * record_size;
                
                if (position > 0x7FFFFFFFUL || fseek (dos_fcb_open[slot].file, (long) position, SEEK_SET) != 0) {
                    status = 1;
                } else {
                
                    while (completed < count) {
                    
                        size_t received = fread (cpu->mem + destination + completed * record_size, 1, record_size, dos_fcb_open[slot].file);
                        
                        if (received != record_size) {
                        
                            if (received == 0) {
                                status = 1;
                            } else {
                            
                                memset (cpu->mem + destination + completed * record_size + received, 0, record_size - received);
                                
                                completed++;
                                status = 3;
                            
                            }
                            
                            break;
                        
                        }
                        
                        completed++;
                    
                    }
                
                }
            
            }
            
            record += completed;
            
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x21, (uint16_t) record);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x23, (uint16_t) (record >> 16));
            
            cpu->regs[REG_CX] = (uint16_t) completed;
            set_reg8 (cpu, REG_AX, status);
            
            break;
        
        }
        
        case 0x28: {                    /* FCB Random Block Write */
        
            uint16_t off = cpu->regs[REG_DX];
            uint16_t count = cpu->regs[REG_CX];
            uint16_t record_size;
            uint16_t dta_seg, dta_off;
            
            uint32_t base, record, source;
            uint32_t completed = 0;
            
            uint64_t position;
            
            int slot;
            uint8_t status = 0;
            
            if (mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, off)) == 0xFF) {
                off = (uint16_t) (off + 7);
            }
            
            base = get_phys (cpu->ds, off);
            slot = fcb_open_slot (cpu->psp_seg, cpu->ds, off);
            
            record_size = mem_read16 (cpu->mem, cpu->mem_size, base + 0x0E);
            record = (uint32_t) mem_read16 (cpu->mem, cpu->mem_size, base + 0x21) | ((uint32_t) mem_read16 (cpu->mem, cpu->mem_size, base + 0x23) << 16);
            
            dos_get_dta (cpu, &dta_seg, &dta_off);
            source = get_phys (dta_seg, dta_off);
            
            if (slot < 0 || record_size == 0) {
                status = 1;
            } else if ((uint32_t) count * record_size > 0x10000UL - dta_off || (uint64_t) source + (uint32_t) count * record_size > cpu->mem_size) {
                status = 2;
            } else {
            
                position = (uint64_t) record * record_size;
                
                if (position > 0x7FFFFFFFUL || fflush (dos_fcb_open[slot].file) != 0 || fseek (dos_fcb_open[slot].file, (long) position, SEEK_SET) != 0) {
                    status = 1;
                } else if (count == 0) {
                
#if     defined (_WIN32)
                    if (_chsize (_fileno (dos_fcb_open[slot].file), position) != 0) {
#else
                    if (ftruncate (fileno (dos_fcb_open[slot].file), (off_t) position) != 0) {
#endif
                        status = 1;
                    }
                
                } else {
                
                    while (completed < count) {
                    
                        size_t written = fwrite (cpu->mem + source + completed * record_size, 1, record_size, dos_fcb_open[slot].file);
                        
                        if (written != record_size) {
                        
                            status = 1;
                            break;
                        
                        }
                        
                        completed++;
                    
                    }
                    
                    if (fflush (dos_fcb_open[slot].file) != 0) {
                        status = 1;
                    }
                
                }
            
            }
            
            if (slot >= 0 && status != 2) {
            
                struct stat st;
                
                if (fstat (fileno (dos_fcb_open[slot].file), &st) == 0) {
                
                    mem_write16 (cpu->mem, cpu->mem_size, base + 0x10, (uint16_t) st.st_size);
                    mem_write16 (cpu->mem, cpu->mem_size, base + 0x12, (uint16_t) ((uint64_t) st.st_size >> 16));
                
                }
            
            }
            
            record += completed;
            
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x21, (uint16_t) record);
            mem_write16 (cpu->mem, cpu->mem_size, base + 0x23, (uint16_t) (record >> 16));
            
            cpu->regs[REG_CX] = (uint16_t) completed;
            set_reg8 (cpu, REG_AX, status);
            
            break;
        
        }
        
        case 0x29: {                    /* Parse filename for FCB */
        
            uint16_t src_off = cpu->regs[REG_SI];   /* DS:SI -> source string */
            uint16_t fcb_off = cpu->regs[REG_DI];   /* ES:DI -> destination FCB */
            
            uint8_t options = cpu->regs[REG_AX] & 0xFF;
            uint8_t drive = 0;
            uint8_t result = 0;
            uint8_t ch;
            uint8_t next;
            uint8_t name[8];
            uint8_t ext[3];
            
            int i;
            int has_name = 0;
            int has_ext = 0;
            int starred;
            
            memset (name, ' ', sizeof (name));
            memset (ext, ' ', sizeof (ext));
            
            /* Option bit 0 skips leading filename separators. */
            if (options & 0x01) {
            
                for (;;) {
                
                    ch = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, src_off));
                    
                    if (ch < 0x20 || (ch != ' ' && strchr (":;.,=+/\"[]\\<>|", ch) == NULL)) {
                        break;
                    }
                    
                    src_off++;
                
                }
            
            }
            
            ch = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, src_off));
            next = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, (uint16_t) (src_off + 1)));
            
            if (next == ':') {
            
                ch = (uint8_t) toupper ((unsigned char) ch);
                
                if (ch < 'A' || ch > 'Z') {
                
                    set_reg8 (cpu, REG_AX, 0xFF);
                    break;
                
                }
                
                drive = (uint8_t) (ch - 'A' + 1);
                src_off += 2;
            
            } else if (options & 0x02) {
                drive = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, fcb_off));
            }
            
            for (i = 0, starred = 0; ; ) {
            
                ch = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, src_off));
                
                if (ch == '.' || ch <= 0x20 || ch == 0x7F || strchr (";,:=+/\"[]\\<>|", ch) != NULL) {
                    break;
                }
                
                has_name = 1;
                
                if (ch == '*') {
                
                    starred = 1;
                    result = 1;
                    
                    while (i < 8) {
                        name[i++] = '?';
                    }
                
                } else if (!starred && i < 8) {
                
                    name[i++] = (uint8_t) toupper ((unsigned char) ch);
                    
                    if (ch == '?') {
                        result = 1;
                    }
                
                }
            
                src_off++;
            
            }
            
            if (ch == '.') {
            
                src_off++;
                
                for (i = 0, starred = 0; ; ) {
                
                    ch = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, src_off));
                    
                    if (ch <= 0x20 || ch == 0x7F || strchr (";,:.=+/\"[]\\<>|", ch) != NULL) {
                        break;
                    }
                    
                    has_ext = 1;
                    
                    if (ch == '*') {
                    
                        starred = 1;
                        result = 1;
                        
                        while (i < 3) {
                            ext[i++] = '?';
                        }
                    
                    } else if (!starred && i < 3) {
                    
                        ext[i++] = (uint8_t) toupper ((unsigned char) ch);
                        
                        if (ch == '?') {
                            result = 1;
                        }
                    
                    }
                    
                    src_off++;
                
                }
            
            }
            
            mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, fcb_off), drive);
            
            if (has_name || !(options & 0x04)) {
            
                for (i = 0; i < 8; i++) {
                    mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (fcb_off + i + 1)), name[i]);
                }
            
            }
            
            if (has_ext || !(options & 0x08)) {
            
                for (i = 0; i < 3; i++) {
                    mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (fcb_off + i + 9)), ext[i]);
                }
            
            }
            
            cpu->regs[REG_SI] = src_off;
            set_reg8 (cpu, REG_AX, result);
            
            break;
        
        }
        
        case 0x2A: {                    /* Get System Date */
        
            time_t now = time (NULL);
            const struct tm *local = localtime (&now);
            
            if (local != NULL) {
            
                set_reg8 (cpu, REG_AX, (uint8_t) local->tm_wday);
                
                cpu->regs[REG_CX] = (uint16_t) (local->tm_year + 1900);
                cpu->regs[REG_DX] = (uint16_t) (((uint16_t) (local->tm_mon + 1) << 8) | (uint16_t) local->tm_mday);
            
            }
            
            break;
        
        }
        
        case 0x2C: {                    /* Get System Time */
        
#if     defined (_WIN32)
            SYSTEMTIME now;
            GetLocalTime (&now);
            
            cpu->regs[REG_CX] = (uint16_t) (((uint16_t) now.wHour << 8) | now.wMinute);
            cpu->regs[REG_DX] = (uint16_t) (((uint16_t) now.wSecond << 8) | (now.wMilliseconds / 10));
#else
            struct timeval now;
            const struct tm *local;
            
            gettimeofday (&now, NULL);
            local = localtime (&now.tv_sec);
            
            if (local != NULL) {
            
                cpu->regs[REG_CX] = (uint16_t) (((uint16_t) local->tm_hour << 8) | local->tm_min);
                cpu->regs[REG_DX] = (uint16_t) (((uint16_t) local->tm_sec << 8) | (now.tv_usec / 10000));
            
            }
#endif
            
            break;
        
        }
        
        case 0x2E: {                    /* Set Verify Flag */
        
            dos_verify_flag = (cpu->regs[REG_DX] & 0x00FF) != 0;
            break;
        
        }
        
        case 0x2F: {                    /* Get DTA address */
        
            uint16_t dta_seg;
            uint16_t dta_off;
            
            dos_get_dta (cpu, &dta_seg, &dta_off);
            
            cpu->es = dta_seg;
            cpu->regs[REG_BX] = dta_off;
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x30: {                    /* Get DOS version */
        
            cpu->regs[REG_AX] = dos_get_version ();
            
            /* Optional (some programs check these) */
            cpu->regs[REG_BX] = 0x0000;
            cpu->regs[REG_CX] = 0x0000;
            
            cpu->flags &= ~FLAG_CF;     /* success */
            break;
        
        }
        
        case 0x31: {                    /* Terminate and Stay Resident */
        
            cpu->halted = 1;
            break;
        
        }
        
        case 0x33: {                    /* Ctrl-Break check */
        
            uint8_t al = (uint8_t) (cpu->regs[REG_AX] & 0xFF);
            
            switch (al) {
            
                case 0x00:              /* get state */
                
                    set_reg8 (cpu, 1, 0x00);                /* DL = 0 off, 1 on -- adjust if your set_reg8 indexes differ */
                    
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                case 0x01:              /* set state */
                
                    /* DL contains new state; store it if you have a DOS state struct */
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                case 0x02:              /* get/set extended break checking - often DOS 3.3+ */
                
                    set_reg8 (cpu, 1, 0x00);
                    
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                default:
                    
                    cpu->flags |= FLAG_CF;
                    cpu->regs[REG_AX] = 0x0001; /* invalid function */
                    
                    break;
            
            }
            
            break;
        
        }
        
        case 0x35: {                    /* Get Interrupt Vector */
        
            uint8_t int_num = cpu->regs[REG_AX] & 0x00FF;   /* AL */
            uint32_t ivt_addr = (uint32_t) int_num * 4;
            
            cpu->regs[REG_BX] = mem_read16 (cpu->mem, cpu->mem_size, ivt_addr + 0);
            cpu->es           = mem_read16 (cpu->mem, cpu->mem_size, ivt_addr + 2);
            cpu->flags &= ~FLAG_CF;
            
            break;
        
        }
       
        case 0x36: {                    /* Get Disk Free Space */
        
            uint16_t sectors, free_clusters, total_clusters;
            
            if (!dos_get_disk_allocation (get_reg8 (cpu, REG_DX), &sectors, &free_clusters, &total_clusters)) {
            
                cpu->regs[REG_AX] = 0xFFFF;
                break;
            
            }
            
            cpu->regs[REG_AX] = sectors;
            cpu->regs[REG_BX] = free_clusters;
            
            cpu->regs[REG_CX] = 512;
            cpu->regs[REG_DX] = total_clusters;

            break;
        
        }

        
        case 0x37: {                    /* Get/Set switch character */
        
            uint8_t al = cpu->regs[REG_AX] & 0xFF;
            
            switch (al) {
                
                case 0x00:              /* get switch char */
                    
                    cpu->regs[REG_DX] = (cpu->regs[REG_DX] & 0xFF00) | '/'; /* DL='/' */
                    
                    dos_clear_success (cpu);
                    break;
                    
                case 0x01:              /* set switch char */
                    
                    /* DL has requested char; ignore/accept for now */
                    dos_clear_success (cpu);
                    break;
                    
                default:
                    
                    dos_set_error (cpu, 0x0001);
                    break;
                
            }
            
            break;
            
        }
        
        case 0x38: {                    /* Get/Set country info */
        
            uint8_t al = (uint8_t) (cpu->regs[REG_AX] & 0xFF);
            
            if (al == 0x00 || al == 0xFF) {
            
                /**
                 * Get country info:
                 * 
                 *     DS:DX points to 34-byte country info buffer.
                 *     Return BX = country code.
                 */
                uint16_t seg = cpu->ds;
                uint16_t off = cpu->regs[REG_DX];
                uint32_t p = get_phys (seg, off);
                
                /* Default U.S. country information (DOS 2.11+ layout). */
                mem_write16 (cpu->mem, cpu->mem_size, p + 0x00, 0);        /* date format: USA */
                
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x02, '$');       /* currency symbol */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x03, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x04, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x05, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x06, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x07, ',');       /* thousands separator */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x08, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x09, '.');       /* decimal separator */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x0A, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x0B, '/');       /* date separator */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x0C, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x0D, ':');       /* time separator */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x0E, 0);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x0F, 0);        /* currency format */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x10, 2);        /* currency decimals */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x11, 0);        /* 12-hour clock */
                
                dos_install_case_map (cpu);
                
                mem_write16 (cpu->mem, cpu->mem_size, p + 0x12, 0x0000);  /* upper-case routine offset */
                mem_write16 (cpu->mem, cpu->mem_size, p + 0x14, DOS_CASE_MAP_SEG);
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x16, ',');       /* list separator */
                mem_write8 (cpu->mem, cpu->mem_size, p + 0x17, 0);
                
                for (int i = 0; i < 10; i++) {
                    mem_write8 (cpu->mem, cpu->mem_size, p + 0x18 + i, 0);
                }
                
                cpu->regs[REG_BX] = 1;                      /* country code 1 */
                cpu->flags &= ~FLAG_CF;
            
            } else {
            
                /* Set country code - accept but do nothing */
                cpu->flags &= ~FLAG_CF;
            
            }
            
            break;
        
        }
        
        case 0x39: {                    /* MKDIR / Create directory */
        
            char raw[260];
            char path[520];
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], raw, sizeof (raw));
            dos_resolve_path (raw, path, sizeof (path));
            
            if (path[0] == '\0') {
            
                dos_set_error (cpu, 0x0003);                /* path not found */
                break;
            
            }
            
#if     defined (_WIN32)
            if (_mkdir (path) == 0) {
#else
            if (mkdir (path, 0777) == 0) {
#endif
            
                dos_83_cache_invalidate ();
                cpu->flags &= ~FLAG_CF;
            
            } else {
            
                switch (errno) {
                
                    case EEXIST:
                    
                        dos_set_error (cpu, 0x0050); /* file exists */
                        break;
                    
                    case ENOENT:
                    
                        dos_set_error (cpu, 0x0003); /* path not found */
                        break;
                    
                    case EACCES:
                    case EPERM:
                    
                        dos_set_error (cpu, 0x0005); /* access denied */
                        break;
                    
                    default:
                    
                        dos_set_error (cpu, 0x0003);
                        break;
                
                }
            
            }
            
            break;
        
        }
        
        case 0x3A: {                    /* RMDIR / Remove directory */
        
            char raw[260];
            char dos_path[260];
            char host_path[520];
            
            struct stat st;
            int absolute;
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], raw, sizeof (raw));
            
            if (raw[0] == '\0' || (!dos_long_paths && strlen (raw) > DOS_DIRECTORY_PATH_MAX) || strlen (raw) >= sizeof (raw) - 1) {
            
                dos_set_error (cpu, 0x0003);                /* path not found */
                break;
            
            }
            
            absolute = raw[0] == '\\' || raw[0] == '/' || (raw[0] && raw[1] == ':');
            
            if (!absolute && strlen (dos_current_dir) + strlen (raw) + 2 > sizeof (dos_path)) {
            
                dos_set_error (cpu, 0x0003);
                break;
            
            }
            
            dos_path[0] = '\0';
            
            if (!absolute && dos_current_dir[0] != '\0') {
            
                dos_append_bounded (dos_path, sizeof (dos_path), dos_current_dir);
                dos_append_bounded (dos_path, sizeof (dos_path), "/");
            
            }
            
            dos_append_bounded (dos_path, sizeof (dos_path), raw);
            dos_normalize_path (dos_path);
            
            if (dos_path[0] == '\0') {
            
                dos_set_error (cpu, 0x0005);                /* cannot remove root directory */
                break;
            
            }
            
            if (strcasecmp (dos_path, dos_current_dir) == 0) {
            
                dos_set_error (cpu, 0x0010);                /* current directory */
                break;
            
            }
            
            if (!dos_resolve_existing_host_path (dos_path, host_path, sizeof (host_path))) {
            
                dos_resolve_path (raw, host_path, sizeof (host_path));
                
                if (stat (host_path, &st) == 0) {
                    dos_set_error (cpu, 0x0005);            /* not a directory */
                } else {
                    dos_set_error (cpu, errno == EACCES || errno == EPERM ? 0x0005 : 0x0003);
                }
                
                break;
            
            }
            
            if (stat (host_path, &st) != 0) {
            
                dos_set_error (cpu, errno == EACCES || errno == EPERM ? 0x0005 : 0x0003);
                break;
            
            }
            
            if (!S_ISDIR (st.st_mode)) {
            
                dos_set_error (cpu, 0x0005);                /* not a directory */
                break;
            
            }
            
#if     defined (_WIN32)
            if (_rmdir (host_path) == 0) {
#else
            if (rmdir (host_path) == 0) {
#endif
            
                dos_83_cache_invalidate ();
                dos_clear_success (cpu);
            
            } else {
            
                switch (errno) {
                
                    case ENOENT:
                    case ENOTDIR:
                    
                        dos_set_error (cpu, 0x0003);        /* path not found */
                        break;
                    
                    default:
                    
                        dos_set_error (cpu, 0x0005);        /* access denied */
                        break;
                
                }
            
            }
            
            break;
        
        }
        
        case 0x3B: {                    /* Change Current Directory */
        
            struct stat st;
            
            char raw[260];
            char req[260];
            char new_dos[260];
            char host_rel[260];
            char host_path[520];
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], raw, sizeof (raw));
            
            if ((!dos_long_paths && strlen (raw) > DOS_DIRECTORY_PATH_MAX) || strlen (raw) >= sizeof (raw) - 1) {
            
                dos_set_error (cpu, 0x0003);
                break;
            
            }
            
            req[0] = '\0';
            dos_append_bounded (req, sizeof (req), raw);
            
            if (strcmp (req, "..") == 0 || strcmp (req, ". .") == 0) {
            
                char *slash;
                
                dos_normalize_path (dos_current_dir);
                slash = strrchr (dos_current_dir, '/');
                
                if (slash) {
                    *slash = '\0';
                } else {
                    dos_current_dir[0] = '\0';
                }
                
                dos_clear_success (cpu);
                break;
            
            }
            
            new_dos[0] = '\0';
            
            if (strcmp (raw, "\\") == 0 || strcmp (raw, "/") == 0 || (raw[0] && raw[1] == ':' && (raw[2] == '\0' || strcmp (raw + 2, "\\") == 0 || strcmp (raw + 2, "/") == 0))) {
                new_dos[0] = '\0';
            } else {
            
                dos_normalize_path (req);
                
                if (req[0] == '\0' || strcmp (req, ".") == 0) {
                    dos_append_bounded (new_dos, sizeof (new_dos), dos_current_dir);
                } else if (raw[0] == '\\' || raw[0] == '/' || (raw[0] && raw[1] == ':')) {
                
                    dos_append_bounded (new_dos, sizeof (new_dos), req);
                    dos_normalize_path (new_dos);
                
                } else {
                
                    if (strlen (dos_current_dir) + strlen (req) + (dos_current_dir[0] != '\0') >= sizeof (new_dos)) {
                    
                        dos_set_error (cpu, 0x0003);
                        break;
                    
                    }
                    
                    if (dos_current_dir[0] != '\0') {
                    
                        dos_append_bounded (new_dos, sizeof (new_dos), dos_current_dir);
                        dos_append_bounded (new_dos, sizeof (new_dos), "/");
                        dos_append_bounded (new_dos, sizeof (new_dos), req);
                    
                    } else {
                        dos_append_bounded (new_dos, sizeof (new_dos), req);
                    }
                    
                    dos_normalize_path (new_dos);
                
                }
            
            }
            
            if (!dos_long_paths && strlen (new_dos) >= DOS_DIRECTORY_PATH_MAX) {
            
                dos_set_error (cpu, 0x0003);
                break;
            
            }
            
            host_rel[0] = '\0';
            dos_append_bounded (host_rel, sizeof (host_rel), new_dos);
            
            if (host_rel[0] == '\0') {
            
                host_path[0] = '\0';
                dos_append_bounded (host_path, sizeof (host_path), dos_drive_root);
            
            } else {
            
                if (!dos_resolve_existing_host_path (host_rel, host_path, sizeof (host_path))) {
                
                    dos_set_error (cpu, 0x0003);
                    break;
                
                }
            
            }
            
            if (stat (host_path, &st) == 0 && S_ISDIR (st.st_mode)) {
            
                dos_current_dir[0] = '\0';
                dos_append_bounded (dos_current_dir, sizeof (dos_current_dir), new_dos);
                
                /**
                 * Do NOT prune FCB searches here.
                 *
                 * TREE.COM keeps a parent directory enumeration alive while it CHDIRs
                 * into a child directory, then CHDIRs back and continues the parent
                 * search.  If we close the parent slot here, the later FCB FindFirst
                 * restarts at Videos/Documents forever.
                 */
                dos_clear_success (cpu);
            
            } else {
                dos_set_error (cpu, 0x0003);
            }
            
            break;
        
        }
        
        case 0x3C: {                    /* Create File */
        
            char rawname[260];
            char filename[260];
            
            FILE *fp;
            int h, sft;
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], rawname, sizeof (rawname));
            dos_resolve_path (rawname, filename, sizeof (filename));
            
            h = get_free_handle (cpu);
            sft = get_free_sft ();
            
            if (h < 0 || sft < 0) {
            
                dos_set_error (cpu, 0x04);                  /* too many open files */
                break;
            
            }
            
            {
            
                struct stat existing;
                
                if (stat (filename, &existing) == 0 && (dos_host_file_attributes (filename, &existing) & 0x01)) {
                
                    dos_set_error (cpu, 0x0005);
                    break;
                
                }
            
            }
            
            fp = fopen (filename, "w+b");
            
            if (!fp) {
            
                dos_set_error (cpu, 0x05);                  /* access denied / create failed */
                break;
            
            }
            
            file_handles[sft] = fp;
            file_handle_references[sft] = 1;
            
            dos_set_jft (cpu, (uint16_t) h, (uint8_t) sft);
            cpu->regs[REG_AX] = (uint16_t) h;
            
#ifndef     _WIN32
            dos_file_attribute_forget (filename);
#endif
            
            dos_83_cache_invalidate ();
            dos_clear_success (cpu);
            
            break;
        
        }
        
        case 0x3D: {                    /* Open File */
        
            const char *mode = "rb";
            
            char rawname[260];
            char filename[260];
            
            FILE *fp;
            int h, sft;
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], rawname, sizeof (rawname));
            dos_resolve_path (rawname, filename, sizeof (filename));;
            
            switch (get_reg8 (cpu, 0) & 0x03) {
                
                case 0:
                
                    mode = "rb";
                    break;
                
                case 1:
                
                    mode = "rb+";
                    break; /* DOS write-only is awkward on stdio; rb+ is okay for now */
                
                case 2:
                
                    mode = "rb+";
                    break;
            
            }
            
            h = get_free_handle (cpu);
            sft = get_free_sft ();
            
            if (h < 0 || sft < 0) {
            
                dos_set_error (cpu, 0x04);                  /* too many open files */
                break;
            
            }
            
            if ((get_reg8 (cpu, REG_AX) & 0x03) != 0) {
            
                struct stat existing;
                
                if (stat (filename, &existing) == 0 && (dos_host_file_attributes (filename, &existing) & 0x01)) {
                
                    dos_set_error (cpu, 0x0005);
                    break;
                
                }
            
            }
            
            fp = dos_fopen_case_insensitive (filename, mode);
            
            if (!fp) {
            
                dos_set_error (cpu, 0x02);                  /* file not found */
                break;
            
            }
            
            file_handles[sft] = fp;
            file_handle_references[sft] = 1;
            
            dos_set_jft (cpu, (uint16_t) h, (uint8_t) sft);
            cpu->regs[REG_AX] = (uint16_t) h;
            
            dos_clear_success (cpu);
            break;
        
        }
        
        case 0x3E: {                    /* Close File (AH=3Eh) */
        
            uint16_t h = cpu->regs[REG_BX];
            
            if (dos_close_handle (cpu, h)) {
                dos_clear_success (cpu);
            } else {
                dos_set_error (cpu, 0x06);                  /* Invalid handle */
            }
            
            break;
        
        }
        
        case 0x3F: {                    /* Read from File */
        
            uint16_t h = cpu->regs[REG_BX];
            uint16_t count = cpu->regs[REG_CX];
            uint16_t addr = cpu->regs[REG_DX];
            
            FILE *fp = dos_get_handle_file (cpu, h);
            uint16_t bytes_read = 0;
            
            /* Standard and duplicated handles share the same file position. */
            if (fp) {
            
                uint16_t i, off;
                
                for (i = 0; i < count; i++) {
                
                    int c;
                    
                    if (fp == stdout || fp == stderr) {
                        c = host_read_key_from (fp);
                    } else {
                        c = fgetc (fp);
                    }
                    
                    if (c == EOF) {
                        break;
                    }
                    
                    off = (uint16_t) (addr + i);
                    
                    mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, off), (uint8_t) c);
                    bytes_read++;
                
                }
                
                cpu->regs[REG_AX] = bytes_read;
                cpu->flags &= ~FLAG_CF;
            
            } else {
                dos_set_error (cpu, 0x06);                  /* Invalid handle */
            }
            
            break;
        
        }
        
        case 0x40: {                    /* Write to File or Device */
        
            uint16_t handle = cpu->regs[REG_BX];
            uint16_t count = cpu->regs[REG_CX];
            uint16_t addr = cpu->regs[REG_DX];
            
            FILE *out = dos_get_handle_file (cpu, handle);
            cpu->flags &= (uint16_t) ~FLAG_CF;
            
            if (out == stdout || out == stderr) {
            
                uint16_t i;
                
                for (i = 0; i < count; i++) {
                
                    uint8_t c = mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, (uint16_t) (addr + i)));
                    dos_write_console_char (out, c);
                
                }
                
                fflush (out);
                cpu->regs[REG_AX] = count;
            
            } else if (out != NULL) {
            
                uint16_t written = 0, i;
                
                for (i = 0; i < count; i++) {
                
                    uint32_t phys_addr = get_phys (cpu->ds, (uint16_t) (addr + i));
                    uint8_t byte = mem_read8 (cpu->mem, cpu->mem_size, phys_addr);
                    
                    if (fputc (byte, out) != EOF) {
                        written++;
                    } else {
                    
                        dos_set_error (cpu, 0x001D);
                        break;
                    
                    }
                
                }
                
                /* Always report the number of bytes written if no error occurred during this call */
                if (!(cpu->flags & FLAG_CF)) {
                
                    cpu->regs[REG_AX] = written;
                    fflush (out);
                
                }
            
            } else {
                dos_set_error (cpu, 0x06);
            }
            
            break;
        
        }
        
        case 0x41: {
        
            char rawname[260];
            char filename[260];
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], rawname, sizeof (rawname));
            dos_resolve_path (rawname, filename, sizeof (filename));
            
            {
            
                struct stat existing;
                
                if (stat (filename, &existing) == 0 && (dos_host_file_attributes (filename, &existing) & 0x01)) {
                
                    dos_set_error (cpu, 0x0005);
                    break;
                
                }
            
            }
            
            if (remove (filename) == 0) {
            
#if     !defined (_WIN32)
                dos_file_attribute_forget (filename);
#endif
                
                dos_83_cache_invalidate ();
                dos_clear_success (cpu);
            
            } else {
                dos_set_error (cpu, 0x02);
            }
            
            break;
        
        }
        
        case 0x42: {                    /* LSEEK */
        
            uint16_t h = cpu->regs[REG_BX];
            
            uint8_t method = get_reg8 (cpu, REG_AX);   /* AL */
            int32_t offset = (int32_t) (((uint32_t) cpu->regs[REG_CX] << 16) | (uint32_t) cpu->regs[REG_DX]);
            
            FILE *fp = dos_get_handle_file (cpu, h);
            
            if (fp) {
            
                int whence = (method == 0) ? SEEK_SET : (method == 1) ? SEEK_CUR : SEEK_END;
                
                if (fseek (fp, offset, whence) == 0) {
                
                    long new_pos = ftell (fp);
                    
                    cpu->regs[REG_AX] = (uint16_t) (new_pos & 0xFFFF);
                    cpu->regs[REG_DX] = (uint16_t) ((unsigned long) new_pos >> 16);
                    cpu->flags &= ~FLAG_CF;
                
                } else {
                    dos_set_error (cpu, 0x0001);
                }
            
            } else {
                dos_set_error (cpu, 0x0006);
            }
            
            break;
        
        }
        
        case 0x43: {                    /* Get/Set File Attributes */
        
            uint8_t subfn = get_reg8 (cpu, REG_AX);
            char rawname[260], filename[520];
            
            struct stat st;
            
            if (subfn > 1) {
            
                dos_set_error (cpu, 0x0001);
                break;
            
            }
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], rawname, sizeof (rawname));
            
            if (!rawname[0] || (rawname[1] == ':' && toupper ((unsigned char) rawname[0]) != 'C')) {
            
                dos_set_error (cpu, 0x0003);
                break;
            
            }
            
            dos_resolve_path (rawname, filename, sizeof (filename));
            
            if (stat (filename, &st) != 0) {
            
                char parent[520];
                char *slash;
                
                struct stat parent_st;
                
                snprintf (parent, sizeof (parent), "%s", filename);
                slash = strrchr (parent, '/');
                
                if (slash) {
                    slash[1] = '\0';
                }
                
                if (slash && stat (parent, &parent_st) == 0 && S_ISDIR (parent_st.st_mode)) {
                    dos_set_error (cpu, 0x0002);          /* File not found */
                } else {
                    dos_set_error (cpu, 0x0003);          /* Path not found */
                }
                
                break;
            
            }
            
            if (subfn == 0) {
            
                cpu->regs[REG_CX] = dos_host_file_attributes (filename, &st);
                dos_clear_success (cpu);
            
            } else {
            
                uint16_t attr = cpu->regs[REG_CX];
                
                if (attr & ~0x27U) {
                    dos_set_error (cpu, 0x0005);            /* Can't set directory/volume attributes */
                } else if (!dos_set_host_file_attributes (filename, &st, (uint8_t) attr)) {
                    dos_set_error (cpu, 0x0005);            /* Access denied */
                } else {
                    dos_clear_success (cpu);
                }
            
            }
            
            break;
        
        }
        
        case 0x44: {                    /* IOCTL */
        
            uint16_t handle = cpu->regs[REG_BX];
            
            FILE *fp = dos_get_handle_file (cpu, handle);
            uint8_t al = get_reg8 (cpu, 0);
            
            switch (al) {
                
                case 0x00:              /* Get device information */
                
                    if (fp != NULL) {
                    
                        cpu->regs[REG_DX] = isatty (fileno (fp)) ? 0x0080 : 0x0000;
                        cpu->regs[REG_AX] = cpu->regs[REG_DX];
                        
                        dos_clear_success (cpu);
                    
                    } else {
                        dos_set_error (cpu, 0x0006);
                    }
                    
                    break;
                
                case 0x01:              /* Set device information */
                
                    /* For now, accept and ignore for character devices / std handles. */
                    if (fp != NULL) {
                        cpu->flags &= ~FLAG_CF;
                    } else {
                        dos_set_error (cpu, 0x0006);
                    }
                    
                    break;
                
                default:
                
                    dos_set_error (cpu, 0x0001);
                    break;
            
            }
            
            break;
        
        }
        
        case 0x45: {                    /* Duplicate file handle */
        
            uint16_t oldh = cpu->regs[REG_BX];
            int sft = dos_get_sft (cpu, oldh), newh;
            
            if (sft < 0) {
            
                dos_set_error (cpu, 0x0006);                /* invalid handle */
                break;
            
            }
            
            newh = get_free_handle (cpu);
            
            if (newh < 0) {
            
                dos_set_error (cpu, 0x0004);                /* too many open files */
                break;
            
            }
            
            dos_set_jft (cpu, (uint16_t) newh, (uint8_t) sft);
            file_handle_references[sft]++;
            
            cpu->regs[REG_AX] = (uint16_t) newh;
            dos_clear_success (cpu);
            
            break;
        
        }
        
        case 0x46: {                    /* Force duplicate file handle */
        
            uint16_t oldh = cpu->regs[REG_BX];
            uint16_t newh = cpu->regs[REG_CX];
            
            int sft = dos_get_sft (cpu, oldh);
            
            if (sft < 0 || newh >= MAX_DOS_HANDLES) {
            
                dos_set_error (cpu, 0x0006);
                break;
            
            }
            
            if (oldh != newh) {
            
                if (dos_get_sft (cpu, newh) >= 0) {
                    dos_close_handle (cpu, newh);
                }
                
                dos_set_jft (cpu, newh, (uint8_t) sft);
                file_handle_references[sft]++;
            
            }
            
            dos_clear_success (cpu);
            break;
        
        }
        
        case 0x47: {                    /* Get Current Directory */
        
            uint8_t drive = cpu->regs[REG_AX] & 0x00FF;     /* AL, 0 = default */
            uint16_t off = cpu->regs[REG_SI];
            
            char cwd[260];
            size_t i;
            
            (void) drive;               /* single-drive stub for now */
            
            if (!dos_get_83_current_dir (cwd, sizeof (cwd)) || strlen (cwd) >= DOS_DIRECTORY_PATH_MAX) {
            
                dos_set_error (cpu, 0x000F);
                break;
            
            }
            
            /* DOS returns path without leading backslash */
            for (i = 0; ; i++) {
            
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, (uint16_t) (off + i)), (uint8_t) cwd[i]);
                
                if (cwd[i] == '\0') {
                    break;
                }
            
            }
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x48: {
        
            dos_allocate_memory (cpu, cpu->regs[REG_BX], cpu->psp_seg, NULL, NULL);
            break;
        
        }
        
        case 0x49: {
        
            uint16_t seg_to_free = cpu->es;
            
            uint16_t mcb_seg;
            uint32_t mcb_phys;
            
            DOS_MCB *mcb;
            
            if (seg_to_free == 0) {
            
                dos_set_error (cpu, 0x0009);
                break;
            
            }
            
            mcb_seg = (uint16_t) (seg_to_free - 1);
            mcb_phys = (uint32_t) mcb_seg << 4;
            
            if (!is_valid_mcb (cpu, mcb_seg)) {
            
                dos_set_error (cpu, 0x0009);
                break;
            
            }
            
            mcb = (DOS_MCB *) &cpu->mem[mcb_phys];
            
            if (mcb->owner == 0) {
            
                dos_set_error (cpu, 0x0009);
                break;
            
            }
            
            mcb->owner = 0;
            
            while (mcb->type == 'M') {
            
                uint16_t next_seg = (uint16_t) (mcb_seg + 1 + mcb->size);
                uint32_t next_phys = (uint32_t) next_seg << 4;
                
                DOS_MCB *next_mcb;
                
                if (!is_valid_mcb (cpu, next_seg)) {
                    break;
                }
                
                next_mcb = (DOS_MCB *) &cpu->mem[next_phys];
                
                if (next_mcb->owner != 0) {
                    break;
                }
                
                mcb->size = (uint16_t) (mcb->size + 1 + next_mcb->size);
                mcb->type = next_mcb->type;
            
            }
            
            {
            
                uint16_t current_seg = 0x004F;
                DOS_MCB *prev_mcb = NULL;
                
                while (current_seg < mcb_seg) {
                
                    uint32_t phys = (uint32_t) current_seg << 4;
                    DOS_MCB *cur;
                    
                    if (!is_valid_mcb (cpu, current_seg)) {
                        break;
                    }
                    
                    cur = (DOS_MCB *) &cpu->mem[phys];
                    
                    if ((uint16_t) (current_seg + 1 + cur->size) == mcb_seg) {
                    
                        prev_mcb = cur;
                        break;
                    
                    }
                    
                    if (cur->type == 'Z') {
                        break;
                    }
                    
                    current_seg = (uint16_t) (current_seg + 1 + cur->size);
                
                }
                
                if (prev_mcb != NULL && prev_mcb->owner == 0) {
                
                    prev_mcb->size = (uint16_t) (prev_mcb->size + 1 + mcb->size);
                    prev_mcb->type = mcb->type;
                
                }
            
            }
            
            cpu->flags &= (uint16_t) ~FLAG_CF;
            break;
        
        }
        
        case 0x4A: {                    /* Resize Memory Block */
        
            uint16_t requested = cpu->regs[REG_BX];
            uint16_t seg = cpu->es;                         /* block data segment */
            
            uint16_t mcb_seg = (uint16_t) (seg - 1);
            uint32_t mcb_phys = (uint32_t) mcb_seg << 4;
            
            DOS_MCB *mcb;
            
            if (mcb_phys + sizeof (DOS_MCB) > cpu->mem_size) {
            
                dos_set_error (cpu, 0x0009);
                break;
            
            }
            
            mcb = (DOS_MCB *)&cpu->mem[mcb_phys];
            
            if (!is_valid_mcb (cpu, mcb_seg)) {
            
                dos_set_error (cpu, 0x0009);
                break;
            
            }
            
            if (requested == mcb->size) {
            
                cpu->flags &= (uint16_t) ~FLAG_CF;
                break;
            
            }
            
            /* SHRINK */
            if (requested < mcb->size) {
            
                uint16_t old_size = mcb->size;
                uint8_t old_type = mcb->type;
                
                uint16_t new_free_mcb_seg = (uint16_t) (mcb_seg + 1 + requested);
                uint32_t new_free_phys = (uint32_t) new_free_mcb_seg << 4;
                
                DOS_MCB *tail;
                
                if (old_size <= requested) {
                
                    dos_set_error (cpu, 0x0008);
                    break;
                
                }
                
                if (new_free_phys + sizeof (DOS_MCB) > cpu->mem_size) {
                
                    dos_set_error (cpu, 0x0008);
                    break;
                
                }
                
                tail = (DOS_MCB *) &cpu->mem[new_free_phys];
                
                tail->type  = old_type;
                tail->owner = 0;
                tail->size  = (uint16_t) (old_size - requested - 1);
                
                mcb->type = 'M';
                mcb->size = requested;
                
                if (!is_valid_mcb (cpu, mcb_seg) || !is_valid_mcb (cpu, new_free_mcb_seg)) {
                
                    dos_set_error (cpu, 0x0009);
                    break;
                
                }
                
                cpu->flags &= (uint16_t) ~FLAG_CF;
                break;
            
            }
            
            /* EXPAND */
            {
            
                uint16_t next_mcb_seg = (uint16_t) (mcb_seg + 1 + mcb->size);
                uint32_t next_phys = (uint32_t) next_mcb_seg << 4;
                uint16_t max_size = mcb->size;
                
                if (mcb->type == 'M' && next_phys + sizeof (DOS_MCB) <= cpu->mem_size) {
                
                    DOS_MCB *next = (DOS_MCB *)&cpu->mem[next_phys];
                    
                    if ((next->type == 'M' || next->type == 'Z') && next->owner == 0) {
                    
                        max_size = (uint16_t) (mcb->size + 1 + next->size);
                        
                        if (requested <= max_size) {
                        
                            uint16_t remain = (uint16_t) (max_size - requested);
                            
                            uint16_t split_seg = 0;
                            uint32_t split_phys = 0;
                            
                            DOS_MCB *split = NULL;
                            
                            if (remain > 0) {
                            
                                split_seg = (uint16_t) (mcb_seg + 1 + requested);
                                split_phys = (uint32_t) split_seg << 4;
                                
                                if (split_phys + sizeof (DOS_MCB) > cpu->mem_size) {
                                
                                    dos_set_error (cpu, 0x0009);
                                    break;
                                
                                }
                                
                                split = (DOS_MCB *)&cpu->mem[split_phys];
                                
                                split->type  = next->type;
                                split->owner = 0;
                                split->size  = (uint16_t) (remain - 1);
                                
                                mcb->type = 'M';
                            
                            } else {
                                mcb->type = next->type;
                            }
                            
                            mcb->size = requested;
                            
                            {
                            
                                uint16_t expected_next = (uint16_t) (mcb_seg + 1 + mcb->size);
                                
                                if (remain > 0) {
                                
                                    if (expected_next != split_seg) {
                                        printf ("MCB SPLIT BROKEN: %04X -> %04X (expected %04X)\n", mcb_seg, expected_next, split_seg);
                                    }
                                
                                }
                            
                            }
                            
                            if (!is_valid_mcb (cpu, mcb_seg)) {
                            
                                dos_set_error (cpu, 0x0009);
                                break;
                            
                            }
                            
                            if (remain > 0) {
                            
                                if (!is_valid_mcb (cpu, split_seg)) {
                                
                                    dos_set_error (cpu, 0x0009);
                                    break;
                                
                                }
                            
                            }
                            
                            cpu->flags &= (uint16_t) ~FLAG_CF;
                            break;
                        
                        }
                    
                    }
                
                }
                
                dos_set_error (cpu, 0x0008);
                cpu->regs[REG_BX] = max_size;
                
                break;
            
            }
        
        }
        
        case 0x4B: {                    /* EXEC: Load and Execute / Load */
        
            char name[260], path[520], args[128];
            
            uint16_t paragraphs, max_paragraphs, child_psp, source_env, env_size;
            uint16_t command_off, command_seg;
            uint16_t tail_length;
            
            uint32_t command_base;
            
            CPU8086 parent;
            int is_mz;
            
            size_t loaded;
            unsigned i;
            
            uint8_t exec_mode = get_reg8 (cpu, REG_AX);
            
            if (exec_mode == 3) {
            
                uint32_t parameter = get_phys (cpu->es, cpu->regs[REG_BX]);
                
                uint16_t load_seg = mem_read16 (cpu->mem, cpu->mem_size, parameter);
                uint16_t relocation_factor = mem_read16 (cpu->mem, cpu->mem_size, parameter + 2);
                
                uint32_t load_phys = (uint32_t) load_seg << 4;
                uint32_t limit = cpu->mem_size < 0xA0000 ? (uint32_t) cpu->mem_size : 0xA0000;
                
                uint16_t available_paragraphs;
                
                dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], name, sizeof (name));
                dos_resolve_path (name, path, sizeof (path));
                
                if (load_phys >= limit) {
                
                    dos_set_error (cpu, 0x0008);
                    break;
                
                }
                
                available_paragraphs = (uint16_t) ((limit - load_phys) >> 4);
                
                {
                
                    FILE *fp = fopen (path, "rb");
                    
                    if (fp == NULL) {
                    
                        dos_set_error (cpu, 0x0002);
                        break;
                    
                    }
                    
                    fclose (fp);
                
                }
                
                if (!load_overlay_file (path, cpu, load_seg, relocation_factor, available_paragraphs)) {
                
                    dos_set_error (cpu, 0x000B);
                    break;
                
                }
                
                cpu->flags &= (uint16_t) ~FLAG_CF;
                break;
            
            }
            
            if (exec_mode != 0 && exec_mode != 1) {
            
                dos_set_error (cpu, 0x0001);
                break;
            
            }
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], name, sizeof (name));
            dos_resolve_path (name, path, sizeof (path));
            
            if (!dos_executable_memory (path, &paragraphs, &max_paragraphs, &is_mz)) {
            
                dos_set_error (cpu, 0x0002);
                break;
            
            }
            
            source_env = mem_read16 (cpu->mem, cpu->mem_size, get_phys (cpu->es, cpu->regs[REG_BX]));
            
            if (source_env == 0) {
                source_env = mem_read16 (cpu->mem, cpu->mem_size, ((uint32_t) cpu->psp_seg << 4) + 0x2C);
            }
            
            if (!dos_environment_paragraphs (cpu, source_env, &env_size)) {
            
                dos_set_error (cpu, 0x000A);
                break;
            
            }
            
            if (!is_mz || max_paragraphs > paragraphs) {
            
                uint16_t mcb_seg = 0x004F;
                uint16_t largest = 0;
                
                while (is_valid_mcb (cpu, mcb_seg)) {
                
                    DOS_MCB *mcb = (DOS_MCB *) &cpu->mem[(uint32_t) mcb_seg << 4];
                    
                    if (mcb->owner == 0 && mcb->size > largest) {
                        largest = mcb->size;
                    }
                    
                    if (mcb->type == 'Z') {
                        break;
                    }
                    
                    mcb_seg = (uint16_t) (mcb_seg + 1 + mcb->size);
                
                }
                
                if (largest <= env_size + 1 || largest - env_size - 1 < paragraphs) {
                
                    dos_set_error (cpu, 0x0008);
                    break;
                
                }
                
                paragraphs = (uint16_t) (largest - env_size - 1);
                
                if (is_mz && paragraphs > max_paragraphs) {
                    paragraphs = max_paragraphs;
                }
            
            }
            
            command_off = mem_read16 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_BX] + 2)));
            command_seg = mem_read16 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_BX] + 4)));
            command_base = get_phys (command_seg, command_off);
            
            tail_length = mem_read8 (cpu->mem, cpu->mem_size, command_base);
            
            if (tail_length > 126) {
                tail_length = 126;
            }
            
            for (i = 0; i < tail_length; i++) {
                args[i] = (char) mem_read8 (cpu->mem, cpu->mem_size, command_base + 1 + i);
            }
            
            args[tail_length] = '\0';
            parent = *cpu;
            
            if (!dos_create_child_psp (cpu, paragraphs, args, source_env, &child_psp)) {
                break;
            }
            
            /* Copy the two FCBs supplied in the EXEC parameter block. */
            for (i = 0; i < 2; i++) {
            
                uint16_t fcb_off = mem_read16 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_BX] + 6 + i * 4)));
                uint16_t fcb_seg = mem_read16 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_BX] + 8 + i * 4)));
                
                unsigned j;
                
                if (fcb_off == 0 && fcb_seg == 0) {
                    continue;
                }
                
                for (j = 0; j < 16; j++) {
                    mem_write8 (cpu->mem, cpu->mem_size, get_phys (child_psp, (uint16_t) (0x5C + i * 0x10 + j)), mem_read8 (cpu->mem, cpu->mem_size, get_phys (fcb_seg, (uint16_t) (fcb_off + j))));
                }
            
            }
            
            loaded = 0;
            
            if (is_mz ? !load_mz_file (path, cpu, (uint16_t) (child_psp + 0x10), (uint16_t) (((DOS_MCB *) &cpu->mem[(uint32_t) (child_psp - 1) << 4])->size - 0x10)) : !load_com_file (path, cpu, cpu->mem, cpu->mem_size, (uint16_t) (child_psp + 0x10), &loaded)) {
            
                dos_exec_cleanup_child (cpu, child_psp);
                *cpu = parent;
                
                dos_set_error (cpu, 0x000B);
                break;
            
            }
            
            if (exec_mode == 1) {
            
                uint16_t entry_cs = cpu->cs;
                uint16_t entry_ip = cpu->ip;
                uint16_t entry_ss = cpu->ss;
                uint16_t entry_sp = cpu->regs[REG_SP];
                
                uint32_t parameter = get_phys (parent.es, parent.regs[REG_BX]);
                
                /* EXEC AL=01h returns the child's initial SS:SP and CS:IP. */
                mem_write16 (cpu->mem, cpu->mem_size, parameter + 0x0E, entry_sp);
                mem_write16 (cpu->mem, cpu->mem_size, parameter + 0x10, entry_ss);
                mem_write16 (cpu->mem, cpu->mem_size, parameter + 0x12, entry_ip);
                mem_write16 (cpu->mem, cpu->mem_size, parameter + 0x14, entry_cs);
                
                *cpu = parent;
                
                cpu->regs[REG_AX] = 0;
                cpu->flags &= (uint16_t) ~FLAG_CF;
                
                break;
            
            }
            
            /* EXEC reports invalid default FCB drives in the initial AX. */
            cpu->regs[REG_AX] = 0;
            
            if (mem_read8 (cpu->mem, cpu->mem_size, get_phys (child_psp, 0x5C)) > 26) {
                cpu->regs[REG_AX] |= 0x00FF;
            }
            
            if (mem_read8 (cpu->mem, cpu->mem_size, get_phys (child_psp, 0x6C)) > 26) {
                cpu->regs[REG_AX] |= 0xFF00;
            }
            
            cpu->psp_seg = child_psp;
            cpu->ds = child_psp;
            cpu->es = child_psp;
            
            /* Native INT 21h completes through a guest IRET. */
            push16 (cpu, FLAG_IF);
            push16 (cpu, cpu->cs);
            push16 (cpu, cpu->ip);
            
            if (!dos_exec_save_parent (&parent)) {
            
                dos_exec_cleanup_child (cpu, child_psp);
                
                *cpu = parent;
                dos_set_error (cpu, 0x0008);
                
                break;
            
            }
            
            dos_exec_child_psps[dos_exec_depth - 1] = child_psp;
            
            /* The parent will resume after its INT 21h instruction. */
            dos_exec_parents[dos_exec_depth - 1].regs[REG_AX] = 0;
            dos_exec_parents[dos_exec_depth - 1].flags &= (uint16_t) ~FLAG_CF;
            
            cpu->cs = DOS_IVT_STUB_SEG;
            cpu->ip = (uint16_t) (0x21 * 4 + 2);
            
            cpu->flags = FLAG_IF;
            break;
        
        }
        
        case 0x4C:              /* Terminate */
        
            if (dos_exec_depth != 0) {
            
                dos_last_return_code[dos_return_slot (cpu)] = (uint8_t) cpu->regs[REG_AX];
                dos_return_code_pending[dos_return_slot (cpu)] = 1;
            
            }
            
            dos_terminate_process (cpu);
            break;
         
        case 0x4D:                      /* Get Return Code */
        
            cpu->regs[REG_AX] = dos_return_code_pending[dos_exec_depth] ? dos_last_return_code[dos_exec_depth] : 0;
            dos_return_code_pending[dos_exec_depth] = 0;
            
            cpu->flags &= (uint16_t) ~FLAG_CF;
            break;
        
        case 0x4E: {                    /* Find First Matching File */
        
            uint16_t addr = cpu->regs[REG_DX];
            char pattern[260], resolved[260];
            
            uint8_t search_attr = (uint8_t) (cpu->regs[REG_CX] & 0xFF);
            uint16_t dta_seg, dta_off;
            
            char *slash;
            const char *search_pattern;
            
            size_t len;
            int slot;
            
            uint32_t dta_phys;
            
            dos_read_asciiz (cpu, cpu->ds, addr, pattern, sizeof (pattern));
            dos_resolve_path (pattern, resolved, sizeof (resolved));
            
            slot = alloc_handle_search_slot ();
            
            if (slot < 0) {
            
                dos_set_error (cpu, 0x0004);                /* Too many open files */
                break;
            
            }
            
            slash = strrchr (resolved, '/');
            
            if (slash) {
            
                len = (size_t) (slash - resolved);
                
                if (len >= sizeof (handle_searches[slot].dirpath)) {
                    len = sizeof (handle_searches[slot].dirpath) - 1;
                }
                
                memcpy (handle_searches[slot].dirpath, resolved, len);
                handle_searches[slot].dirpath[len] = '\0';
                
                search_pattern = slash + 1;
            
            } else {
            
                strcpy (handle_searches[slot].dirpath, ".");
                search_pattern = resolved;
            
            }
            
            len = strlen (search_pattern);
            
            if (len >= sizeof (handle_searches[slot].pattern)) {
                len = sizeof (handle_searches[slot].pattern) - 1;
            }
            
            memcpy (handle_searches[slot].pattern, search_pattern, len);
            handle_searches[slot].pattern[len] = '\0';
            
            handle_searches[slot].dir = opendir (handle_searches[slot].dirpath[0] ? handle_searches[slot].dirpath : ".");
            
            if (!handle_searches[slot].dir) {
            
                handle_searches[slot].active = 0;
                
                dos_set_error (cpu, 0x0003);                /* Path not found */
                break;
            
            }
            
            handle_searches[slot].attr = search_attr;
            handle_searches[slot].active = 1;
            
            /* Store Slot in DTA Reserved Area (Offsets 0-20) */
            dos_get_dta (cpu, &dta_seg, &dta_off);
            dta_phys = get_phys (dta_seg, dta_off);
            
            mem_write8 (cpu->mem, cpu->mem_size, dta_phys + 0, 0xEA);      /* Signature */
            mem_write8 (cpu->mem, cpu->mem_size, dta_phys + 1, (uint8_t) slot);
            mem_write8 (cpu->mem, cpu->mem_size, dta_phys + 2, search_attr);
            
            /* Trigger the first search */
            cpu->regs[REG_AX] = 0x4F00;                     /* Fake AX for internal jump */
            goto handle_find_next;
        
        }
        
        case 0x4F: {                    /* Find Next Matching File */
        handle_find_next: ;
        
            uint16_t dta_seg, dta_off;
            
            uint32_t dta_phys;
            int slot;
            
            HANDLE_SEARCH *s;
            
            const char *name;
            int found = 0;
            
            dos_get_dta (cpu, &dta_seg, &dta_off);
            dta_phys = get_phys (dta_seg, dta_off);
            
            /* Verify this DTA belongs to our emulator search */
            if (mem_read8 (cpu->mem, cpu->mem_size, dta_phys + 0) != 0xEA) {
                
                dos_set_error (cpu, 0x0012);                /* No more files */
                break;
            
            }
            
            slot = mem_read8 (cpu->mem, cpu->mem_size, dta_phys + 1);
            
            if (slot >= MAX_HANDLE_SEARCHES || !handle_searches[slot].active) {
            
                dos_set_error (cpu, 0x0012);
                break;
            
            }
            
            s = &handle_searches[slot];
            
            while ((name = dos_find_next_directory_entry (s->dir, s->dirpath, &s->dot_phase)) != NULL) {
            
                struct stat st;
                
                char full[520], dos_name[13];
                uint8_t res_attr;
                
                if (!dos_wildcard_match (s->pattern, name)) {
                    continue;
                }
                
                snprintf (full, sizeof (full), "%s/%s", s->dirpath, name);
                
                if (host_lstat (full, &st) != 0) {
                    continue;
                }
                
                res_attr = dos_host_file_attributes (full, &st);
                
                if ((res_attr & 0x02) && !(s->attr & 0x02)) {
                    continue;
                }
                
                if ((res_attr & 0x04) && !(s->attr & 0x04)) {
                    continue;
                }
                
                if (S_ISDIR (st.st_mode) && !(s->attr & 0x10)) {
                    continue;
                }
                
                dos_make_83_display_name_unique (s->dirpath, name, dos_name);
                dos_write_dta_find_result (cpu, dos_name, S_ISDIR (st.st_mode) ? 0 : (uint32_t) st.st_size, st.st_mtime, res_attr);
                
                found = 1;
                break;
            
            }
            
            if (!found) {
            
                closedir (s->dir);
                
                s->active = 0;
                dos_set_error (cpu, 0x0012);
            
            } else {
            
                cpu->regs[REG_AX] = 0;
                dos_clear_success (cpu);
            
            }
            
            break;
        
        }
        
        case 0x50: {                    /* Set current PSP */
        
            cpu->psp_seg = cpu->regs[REG_BX];
            cpu->flags &= ~FLAG_CF;
            
            break;
        
        }
        
        case 0x51: {                    /* Get current PSP */
        
            cpu->regs[REG_BX] = cpu->psp_seg;
            cpu->flags &= ~FLAG_CF;
            
            break;
        
        }
        
        case 0x54: {                    /* Get Verify Flag */
        
            set_reg8 (cpu, REG_AX, dos_verify_flag);
            break;
        
        }
        
        case 0x55: {                    /* Create Child Program Segment Prefix */
        
            uint16_t parent_psp = cpu->psp_seg;
            uint16_t child_psp = cpu->regs[REG_DX];
            
            uint32_t source = (uint32_t) parent_psp << 4;
            uint32_t destination = (uint32_t) child_psp << 4;
            
            const char *version = getenv ("DOS_VER");
            unsigned long major = 5;
            
            uint8_t contents[256];
            int i;
            
            if (version) {
            
                major = strtoul (version, NULL, 10);
                
                if (version[0] == '0' && (version[1] == 'x' || version[1] == 'X')) {
                    major = strtoul (version, NULL, 16) & 0xFF;
                }
            
            }
            
            for (i = 0; i < 256; i++) {
                contents[i] = mem_read8 (cpu->mem, cpu->mem_size, source + i);
            }
            
            for (i = 0; i < 256; i++) {
                mem_write8 (cpu->mem, cpu->mem_size, destination + i, contents[i]);
            }
            
            for (i = 0; i < 12; i++) {
            
                mem_write8 (cpu->mem, cpu->mem_size, destination + 0x0A + i,
                    mem_read8 (cpu->mem, cpu->mem_size, (uint32_t) (0x22 * 4) + i));
            
            }
            
            mem_write16 (cpu->mem, cpu->mem_size, destination + 0x16, parent_psp);
            
            if (major >= 3) {
                mem_write16 (cpu->mem, cpu->mem_size, destination + 0x02, cpu->regs[REG_SI]);
            }
            
            dos_inherit_process_handles (cpu, parent_psp, child_psp);
            cpu->psp_seg = child_psp;
            
            break;
        
        }
        
        case 0x56: {                    /* Rename File */
            
            char oldraw[260];
            char newraw[260];
            char oldname[260];
            char newname[260];
            
            /**
             * DOS AH=56h:
             *   DS:DX -> old ASCIIZ filename
             *   ES:DI -> new ASCIIZ filename
             */
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_DX], oldraw, sizeof (oldraw));
            dos_read_asciiz (cpu, cpu->es, cpu->regs[REG_DI], newraw, sizeof (newraw));
            
            dos_resolve_path (oldraw, oldname, sizeof (oldname));
            dos_resolve_path (newraw, newname, sizeof (newname));
            
            if (rename (oldname, newname) == 0) {
            
#ifndef     _WIN32
                dos_file_attribute_rename (oldname, newname);
#endif
                
                dos_83_cache_invalidate ();
                dos_clear_success (cpu);
            
            } else {
                dos_set_error (cpu, 0x02);
            }
            
            break;
        
        }
        
        case 0x57: {                    /* Get/Set file date and time */
        
            uint8_t subfn = get_reg8 (cpu, REG_AX); /* AL */
            
            switch (subfn) {
                
                case 0x00:              /* get */
                
                    cpu->regs[REG_CX] = 0; /* time */
                    cpu->regs[REG_DX] = 0; /* date */
                    
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                case 0x01:              /* set */
                
                    cpu->flags &= ~FLAG_CF;
                    break;
                
                default:
                
                    dos_set_error (cpu, 1);
                    break;
            
            }
            
            break;
        
        }
        
        case 0x58: {                    /* Get/Set Memory Allocation Strategy */
        
            uint8_t al = get_reg8 (cpu, REG_AX);
            
            uint8_t major = (uint8_t) dos_get_version ();
            uint16_t strategy = cpu->regs[REG_BX];
            
            switch (al) {
            
                case 0x00:              /* Get allocation strategy */
                
                    cpu->regs[REG_AX] = dos_allocation_strategy;
                    
                    dos_clear_success (cpu);
                    break;
                
                case 0x01:              /* Set allocation strategy */
                
                    if (major < 5) {
                    
                        dos_allocation_strategy = (uint8_t) strategy;
                        dos_clear_success (cpu);
                    
                    } else if (strategy <= 2 || strategy == 0x40 || strategy == 0x41 || strategy == 0x42 || strategy == 0x80 || strategy == 0x81 || strategy == 0x82) {
                    
                        dos_allocation_strategy = (uint8_t) strategy;
                        dos_clear_success (cpu);
                    
                    } else {
                        dos_set_error (cpu, 0x0001);
                    }
                    
                    break;
                
                case 0x02:              /* Get UMB link state (DOS 5+) */
                
                    if (major < 5) {
                        dos_set_error (cpu, 0x0001);
                    } else {
                    
                        set_reg8 (cpu, REG_AX, 0);
                        dos_clear_success (cpu);
                    
                    }
                    
                    break;
                
                case 0x03:              /* Set UMB link state (DOS 5+) */
                
                    if (major < 5 || strategy != 0) {
                    
                        /* No upper-memory arena exists to link. */
                        dos_set_error (cpu, 0x0001);
                    
                    } else {
                        dos_clear_success (cpu);
                    }
                    
                    break;
                
                default:
                
                    dos_set_error (cpu, 0x0001);
                    break;
            
            }
            
            break;
        
        }
        
        case 0x5D: {
        
            uint8_t al = (uint8_t) (cpu->regs[REG_AX] & 0xFF);
            
            switch (al) {
                
                case 0x00:
                
                    cpu->flags |= FLAG_CF;
                    cpu->regs[REG_AX] = 0x0001;
                    
                    break;
                
                case 0x06:
                
                    cpu->flags &= ~FLAG_CF;
                    cpu->ds = 0x0050;
                    
                    cpu->regs[REG_SI] = 0x0100;
                    cpu->regs[REG_CX] = 0x0080;
                    
                    break;
                
                case 0x08:
                case 0x09:
                case 0x0A:
                case 0x0B:
                case 0x0C:
                
                    cpu->flags |= FLAG_CF;
                    cpu->regs[REG_AX] = 0x0001;
                    
                    break;
                
                default:
                
                    printf ("Unhandled INT 21h AH=5Dh AL=%02X AX=%04X at %04X:%04X\n", al, cpu->regs[REG_AX], cpu->cs, cpu->ip);
                    
                    cpu->flags |= FLAG_CF;
                    cpu->regs[REG_AX] = 0x0001;
                    
                    break;
            
            }
            
            break;
        
        }
        
        case 0x59: {                    /* Get Extended Error Info */
        
            cpu->regs[REG_AX] = dos_last_error;             /* extended error code */
            cpu->regs[REG_BX] = 0x0000;                     /* error class */
            cpu->regs[REG_CX] = 0x0000;                     /* suggested action */
            cpu->regs[REG_DX] = 0x0000;                     /* error locus */
            
            dos_clear_success (cpu);
            break;
        
        }
        
        case 0x60: {                    /* TRUENAME / Canonicalize Path */
        
            char source[260], canonical[128];
            
            int error;
            size_t i;
            
            dos_read_asciiz (cpu, cpu->ds, cpu->regs[REG_SI], source, sizeof (source));
            
            if (strlen (source) == sizeof (source) - 1) {
            
                dos_set_error (cpu, 0x0003);
                break;
            
            }
            
            error = dos_truename (source, canonical);
            
            if (error) {
            
                dos_set_error (cpu, (uint16_t) error);
                break;
            
            }
            
            for (i = 0; i <= strlen (canonical); i++) {
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->es, (uint16_t) (cpu->regs[REG_DI] + i)), (uint8_t) canonical[i]);
            }
            
            cpu->regs[REG_AX] &= 0x00FF;                    /* AH=00h on success. */
            
            dos_clear_success (cpu);
            break;
        
        }
        
        case 0x62: {                    /* Get PSP address */
        
            cpu->regs[REG_BX] = cpu->psp_seg;
            cpu->flags &= ~FLAG_CF;
            
            break;
        
        }
        
        case 0x63: {                    /* Get DBCS lead byte table */
        
            /**
             * AH=63h AL=00h:
             * 
             *     Returns DS:SI -> table of lead byte ranges.
             *     Empty table is: 00 00
             */
            
            static uint16_t dbcs_seg = 0x0050;              /* PSP-ish/safe low area if valid in your setup */
            static uint16_t dbcs_off = 0x00F0;
            
            mem_write8 (cpu->mem, cpu->mem_size, get_phys (dbcs_seg, dbcs_off), 0x00);
            mem_write8 (cpu->mem, cpu->mem_size, get_phys (dbcs_seg, (uint16_t) (dbcs_off + 1)), 0x00);
            
            cpu->ds = dbcs_seg;
            cpu->regs[REG_SI] = dbcs_off;
            
            cpu->regs[REG_AX] = 0x6300;
            cpu->flags &= (uint16_t) ~FLAG_CF;
            
            break;
        
        }
        
        case 0x65: {                    /* Get extended country info */
        
            uint8_t al = cpu->regs[REG_AX] & 0xFF;
            
            /* Most programs call AL=01h (country info) */
            if (al == 0x01) {
            
                uint16_t seg = cpu->ds;
                uint16_t off = cpu->regs[REG_DX];
                
                /* Minimal safe structure (just zero it) */
                for (int i = 0; i < 32; i++) {
                    mem_write8 (cpu->mem, cpu->mem_size, get_phys (seg, (uint16_t) (off + i)), 0);
                }
                
                cpu->regs[REG_AX] = 0x0001;                 /* success */
                cpu->flags &= ~FLAG_CF;
            
            } else {
            
                /* Unsupported subfunction — still succeed */
                cpu->flags &= ~FLAG_CF;
                cpu->regs[REG_AX] = 0;
            
            }
            
            break;
        
        }
        
        case 0x69: {                    /* Get/Set Disk Serial Number */
        
            uint8_t al = get_reg8 (cpu, 0);
            uint8_t bh = (uint8_t) (cpu->regs[REG_BX] >> 8);
            uint8_t bl = (uint8_t) cpu->regs[REG_BX];
            uint16_t off = cpu->regs[REG_DX];
            uint32_t phys;
            
            int i;
            
            /* Only the emulated C: drive is available (0 = default). */
            if (bl != 0 && bl != 3) {
            
                dos_set_error (cpu, 0x000F);                /* Invalid drive */
                break;
            
            }
            
            if (bh != 0 || al > 1) {
            
                dos_set_error (cpu, 0x0001);                /* Invalid function */
                break;
            
            }
            
            if (al == 0x00) {
            
                /* DS:DX -> 25-byte extended BPB information block. */
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, off), 0);
                mem_write8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, (uint16_t) (off + 1)), 0);
                
                for (i = 0; i < 4; i++) {
                
                    phys = get_phys (cpu->ds, (uint16_t) (off + 2 + i));
                    mem_write8 (cpu->mem, cpu->mem_size, phys, (uint8_t) (dos_volume_serial >> (i * 8)));
                
                }
                
                for (i = 0; i < 11; i++) {
                
                    phys = get_phys (cpu->ds, (uint16_t) (off + 6 + i));
                    mem_write8 (cpu->mem, cpu->mem_size, phys, dos_volume_label[i]);
                
                }
                
                for (i = 0; i < 8; i++) {
                
                    phys = get_phys (cpu->ds, (uint16_t) (off + 17 + i));
                    mem_write8 (cpu->mem, cpu->mem_size, phys, dos_volume_fs[i]);
                
                }
            
            } else {
            
                if (mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, off)) != 0 || mem_read8 (cpu->mem, cpu->mem_size, get_phys (cpu->ds, (uint16_t) (off + 1))) != 0) {
                
                    dos_set_error (cpu, 0x0001);            /* Unsupported info level */
                    break;
                
                }
                
                dos_volume_serial = 0;
                
                for (i = 0; i < 4; i++) {
                
                    phys = get_phys (cpu->ds, (uint16_t) (off + 2 + i));
                    dos_volume_serial |= (uint32_t) mem_read8 (cpu->mem, cpu->mem_size, phys) << (i * 8);
                
                }
                
                for (i = 0; i < 11; i++) {
                
                    phys = get_phys (cpu->ds, (uint16_t) (off + 6 + i));
                    dos_volume_label[i] = mem_read8 (cpu->mem, cpu->mem_size, phys);
                
                }
                
                for (i = 0; i < 8; i++) {
                
                    phys = get_phys (cpu->ds, (uint16_t) (off + 17 + i));
                    dos_volume_fs[i] = mem_read8 (cpu->mem, cpu->mem_size, phys);
                
                }
            
            }
            
            set_reg8 (cpu, 0, al);
            
            dos_clear_success (cpu);
            break;
        
        }
        
        default:
        
            printf ("Unhandled INT 21h Function: AH=0x%02X\n", ah);
            
            dos_set_error (cpu, 0x0001); /* invalid function */
            break;
    
    }

}

static uint8_t bios_video_page;
static int bios_video_force_redraw;

static uint8_t bios_video_mode = 0x03;

static unsigned bios_video_columns = 80;
static unsigned bios_video_rows = 25;

static uint8_t bios_cursor_start = 6;
static uint8_t bios_cursor_end = 7;

static uint8_t bios_cursor_row[8];
static uint8_t bios_cursor_col[8];

static uint8_t bios_video_chars[25][80];
static uint8_t bios_video_attrs[25][80];

static int bios_video_initialized;

static void bios_video_viewport (unsigned *width, unsigned *height) {

    *width = bios_video_columns;
    *height = bios_video_rows;
    
#if     defined (_WIN32)
    CONSOLE_SCREEN_BUFFER_INFO info;
    HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
    
    if (handle != NULL && handle != INVALID_HANDLE_VALUE && GetConsoleScreenBufferInfo (handle, &info)) {
    
        unsigned columns = (unsigned) (info.srWindow.Right - info.srWindow.Left + 1);
        unsigned rows = (unsigned) (info.srWindow.Bottom - info.srWindow.Top + 1);
        
        if (columns != 0 && columns < *width) {
            *width = columns;
        }
        
        if (rows != 0 && rows < *height) {
            *height = rows;
        }
    
    }
#else
    struct winsize window;
    
    if (ioctl (STDOUT_FILENO, TIOCGWINSZ, &window) == 0) {
    
        if (window.ws_col != 0 && window.ws_col < *width) {
            *width = window.ws_col;
        }
        
        if (window.ws_row != 0 && window.ws_row < *height) {
            *height = window.ws_row;
        }
    
    }
#endif

}

#if     defined (_WIN32)
static void bios_windows_set_cursor (uint8_t row, uint8_t col) {

    HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
    
    CONSOLE_SCREEN_BUFFER_INFO info;
    COORD position;
    
    fflush (stdout);
    
    if (!GetConsoleScreenBufferInfo (handle, &info)) {
        return;
    }
    
    position.X = (SHORT) (info.srWindow.Left + col);
    position.Y = (SHORT) (info.srWindow.Top + row);
    
    if (position.X < info.dwSize.X && position.Y < info.dwSize.Y) {
        SetConsoleCursorPosition (handle, position);
    }

}

static void bios_windows_show_cursor (int visible) {

    HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
    
    CONSOLE_CURSOR_INFO info;
    fflush (stdout);
    
    if (GetConsoleCursorInfo (handle, &info)) {
    
        info.bVisible = visible ? TRUE : FALSE;
        SetConsoleCursorInfo (handle, &info);
    
    }

}

static void bios_windows_render (unsigned top, unsigned left, unsigned bottom, unsigned right) {

    static CHAR_INFO cells[80 * 25];
    
    HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO info;
    
    COORD size, origin = { 0, 0 };
    SMALL_RECT region;
    
    unsigned viewport_width, viewport_height;
    unsigned row, col, width;
    
    fflush (stdout);
    
    if (!GetConsoleScreenBufferInfo (handle, &info)) {
        return;
    }
    
    bios_video_viewport (&viewport_width, &viewport_height);
    
    if (left >= viewport_width || top >= viewport_height) {
        return;
    }
    
    if (right >= viewport_width) {
        right = viewport_width - 1;
    }
    
    if (bottom >= viewport_height) {
        bottom = viewport_height - 1;
    }
    
    width = right - left + 1;
    
    if (width * (bottom - top + 1) > sizeof (cells) / sizeof (cells[0])) {
        return;
    }
    
    for (row = top; row <= bottom; row++) {
    
        for (col = left; col <= right; col++) {
        
            uint8_t ch = bios_video_chars[row][col];
            CHAR_INFO *cell = &cells[(row - top) * width + col - left];
            
            if (ch == 0x18) {
                cell->Char.UnicodeChar = 0x2191;
            } else if (ch == 0x19) {
                cell->Char.UnicodeChar = 0x2193;
            } else {
                cell->Char.UnicodeChar = (WCHAR) bios_video_codepoint (ch);
            }
            
            cell->Attributes = bios_video_attrs[row][col] & 0x7F;
        
        }
    
    }
    
    size.X = (SHORT) width;
    size.Y = (SHORT) (bottom - top + 1);
    
    region.Left = (SHORT) (info.srWindow.Left + left);
    region.Top = (SHORT) (info.srWindow.Top + top);
    region.Right = (SHORT) (info.srWindow.Left + right);
    region.Bottom = (SHORT) (info.srWindow.Top + bottom);
    
    WriteConsoleOutputW (handle, cells, size, origin, &region);

}

typedef struct {

    WCHAR buffer[32768];
    HANDLE handle;
    
    unsigned used;
    int failed;

} BIOS_WINDOWS_VT_OUTPUT;

static void bios_windows_vt_flush (BIOS_WINDOWS_VT_OUTPUT *output) {

    unsigned offset = 0;
    DWORD written;
    
    while (offset < output->used && !output->failed) {
    
        if (!WriteConsoleW (output->handle, output->buffer + offset, output->used - offset, &written, NULL) || written == 0) {
        
            output->failed = 1;
            break;
        
        }
        
        offset += written;
    
    }
    
    output->used = 0;

}

static void bios_windows_vt_put (BIOS_WINDOWS_VT_OUTPUT *output, WCHAR ch) {

    if (output->failed) {
        return;
    }
    
    if (output->used == sizeof (output->buffer) / sizeof (output->buffer[0])) {
        bios_windows_vt_flush (output);
    }
    
    if (!output->failed) {
        output->buffer[output->used++] = ch;
    }

}

static void bios_windows_vt_text (BIOS_WINDOWS_VT_OUTPUT *output, const char *text) {

    while (*text) {
        bios_windows_vt_put (output, (WCHAR) (unsigned char) *text++);
    }

}

static void bios_windows_vt_number (BIOS_WINDOWS_VT_OUTPUT *output, unsigned value) {

    WCHAR digits[10];
    unsigned count = 0;
    
    do {
    
        digits[count++] = (WCHAR) ('0' + value % 10);
        value /= 10;
    
    } while (value != 0);
    
    while (count != 0) {
        bios_windows_vt_put (output, digits[--count]);
    }

}

static void bios_windows_vt_attribute (BIOS_WINDOWS_VT_OUTPUT *output, uint8_t attr) {

    static const uint8_t ansi_colors[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };
    uint8_t foreground = attr & 0x0F;
    uint8_t background = (attr >> 4) & 0x07;
    
    bios_windows_vt_text (output, "\033[");
    
    bios_windows_vt_number (output, (unsigned) (foreground & 8 ? 90 : 30) + ansi_colors[foreground & 7]);
    bios_windows_vt_put (output, ';');
    
    bios_windows_vt_number (output, (unsigned) 40 + ansi_colors[background]);
    bios_windows_vt_text (output, attr & 0x80 ? "m\033[5m" : "m\033[25m");

}

static void bios_windows_vt_render (const unsigned *first, const unsigned *last, unsigned viewport_width, unsigned viewport_height) {

    static BIOS_WINDOWS_VT_OUTPUT output;
    unsigned row, col;
    
    int previous_attr = -1;
    int changed = 0;
    
    output.handle = GetStdHandle (STD_OUTPUT_HANDLE);
    output.used = 0;
    output.failed = 0;
    
    for (row = 0; row < bios_video_rows && row < viewport_height; row++) {
    
        if (first[row] < bios_video_columns && first[row] < viewport_width) {
        
            changed = 1;
            break;
        
        }
    
    }
    
    if (!changed) {
        return;
    }
    
    fflush (stdout);
    bios_windows_vt_text (&output, "\0337\033[?7l");
    
    for (row = 0; row < bios_video_rows && row < viewport_height; row++) {
    
        unsigned end;
        
        if (first[row] >= bios_video_columns || first[row] >= viewport_width) {
            continue;
        }
        
        end = last[row] < viewport_width ? last[row] : viewport_width - 1;
        
        bios_windows_vt_text (&output, "\033[");
        bios_windows_vt_number (&output, row + 1);
        bios_windows_vt_put (&output, ';');
        bios_windows_vt_number (&output, first[row] + 1);
        bios_windows_vt_put (&output, 'H');
        
        for (col = first[row]; col <= end; col++) {
        
            uint8_t attr = bios_video_attrs[row][col];
            
            if (previous_attr != attr) {
            
                bios_windows_vt_attribute (&output, attr);
                previous_attr = attr;
            
            }
            
            bios_windows_vt_put (&output, (WCHAR) bios_video_codepoint (bios_video_chars[row][col]));
        
        }
    
    }
    
    bios_windows_vt_text (&output, "\033[0m\033[?7h\0338");
    bios_windows_vt_flush (&output);

}

static void bios_windows_vt_render_region (unsigned top, unsigned left, unsigned bottom, unsigned right) {

    unsigned first[25], last[25];
    unsigned viewport_width, viewport_height;
    unsigned row;
    
    bios_video_viewport (&viewport_width, &viewport_height);
    
    for (row = 0; row < bios_video_rows; row++) {
    
        first[row] = bios_video_columns;
        last[row] = 0;
        
        if (row >= top && row <= bottom) {
        
            first[row] = left;
            last[row] = right;
        
        }
    
    }
    
    bios_windows_vt_render (first, last, viewport_width, viewport_height);

}
#endif

static unsigned bios_video_page_size (void) {
    return bios_video_columns == 40 ? 0x0800 : 0x1000;
}

static unsigned bios_video_page_count (void) {
    return bios_video_mode == 0x07 ? 1 : (bios_video_columns == 40 ? 8 : 4);
}

static uint32_t bios_video_page_base (unsigned page) {
    return (bios_video_mode == 0x07 ? 0xB0000 : 0xB8000) + page * bios_video_page_size ();
}

static uint32_t bios_video_memory_base (void) {
    return bios_video_page_base (bios_video_page);
}

static void bios_video_set_crtc_cursor (CPU8086 *cpu) {

    unsigned address = bios_video_page * (bios_video_page_size () / 2) + bios_cursor_row[bios_video_page] * bios_video_columns + bios_cursor_col[bios_video_page];
    
    cpu->crtc_registers[0x0E] = (uint8_t) ((address >> 8) & 0x3F);
    cpu->crtc_registers[0x0F] = (uint8_t) address;

}

static void bios_video_copy_to_memory (CPU8086 *cpu, unsigned top, unsigned left, unsigned bottom, unsigned right) {

    uint32_t base = bios_video_memory_base ();
    unsigned row, col;
    
    for (row = top; row <= bottom; row++) {
    
        for (col = left; col <= right; col++) {
        
            uint32_t address = base + (row * bios_video_columns + col) * 2;
            
            cpu->mem[address] = bios_video_chars[row][col];
            cpu->mem[address + 1] = bios_video_attrs[row][col];
        
        }
    
    }

}

static void bios_video_copy_from_memory (CPU8086 *cpu) {

    uint32_t base = bios_video_memory_base ();
    unsigned row, col;
    
    for (row = 0; row < bios_video_rows; row++) {
    
        for (col = 0; col < bios_video_columns; col++) {
        
            uint32_t address = base + (row * bios_video_columns + col) * 2;
            
            bios_video_chars[row][col] = cpu->mem[address];
            bios_video_attrs[row][col] = cpu->mem[address + 1];
        
        }
    
    }

}

static void bios_video_clear_other_pages (CPU8086 *cpu) {

    unsigned page, row, col;
    
    for (page = 1; page < bios_video_page_count (); page++) {
    
        uint32_t base = bios_video_page_base (page);
        
        for (row = 0; row < bios_video_rows; row++) {
        
            for (col = 0; col < bios_video_columns; col++) {
            
                uint32_t address = base + (row * bios_video_columns + col) * 2;
                
                cpu->mem[address] = ' ';
                cpu->mem[address + 1] = 0x07;
            
            }
        
        }
    
    }

}

static void bios_video_initialize (void) {

    unsigned row;
    unsigned col;

    if (bios_video_initialized) {
        return;
    }

    for (row = 0; row < bios_video_rows; row++) {
    
        for (col = 0; col < bios_video_columns; col++) {
        
            bios_video_chars[row][col] = ' ';
            bios_video_attrs[row][col] = 0x07;
        
        }
    
    }

    bios_video_initialized = 1;

}

static void bios_video_update_bda (CPU8086 *cpu) {

    unsigned page;

    mem_write8 (cpu->mem, cpu->mem_size, 0x449, bios_video_mode);
    mem_write16 (cpu->mem, cpu->mem_size, 0x44A, bios_video_columns);
    
    mem_write16 (cpu->mem, cpu->mem_size, 0x44C, bios_video_page_size ());
    mem_write16 (cpu->mem, cpu->mem_size, 0x44E, bios_video_page * bios_video_page_size ());
    
    for (page = 0; page < 8; page++) {
        mem_write16 (cpu->mem, cpu->mem_size, 0x450 + page * 2, ((uint16_t) bios_cursor_row[page] << 8) | bios_cursor_col[page]);
    }
    
    mem_write16 (cpu->mem, cpu->mem_size, 0x460, ((uint16_t) bios_cursor_start << 8) | bios_cursor_end);
    mem_write8 (cpu->mem, cpu->mem_size, 0x462, bios_video_page);
    
    mem_write16 (cpu->mem, cpu->mem_size, 0x463, bios_video_mode == 7 ? 0x3B4 : 0x3D4);
    mem_write8 (cpu->mem, cpu->mem_size, 0x466, 0);
    
    cpu->crtc_base = bios_video_mode == 7 ? 0x3B4 : 0x3D4;
    
    cpu->crtc_registers[0x0C] = (uint8_t) ((bios_video_page * (bios_video_page_size () / 2)) >> 8);
    cpu->crtc_registers[0x0D] = (uint8_t) (bios_video_page * (bios_video_page_size () / 2));
    
    bios_video_set_crtc_cursor (cpu);

}

static void bios_write_video_char (uint8_t ch) {

    if (ch >= 0x20 && ch < 0x7F) {
        fputc (ch, stdout);
    } else {
        dos_write_unicode_char (stdout, bios_video_codepoint (ch));
    }

}

static void bios_video_set_attribute (uint8_t attr) {

    static const uint8_t ansi_colors[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };
    
    uint8_t foreground = attr & 0x0F;
    uint8_t background = (attr >> 4) & 0x07;
    
    fprintf (stdout, "\033[%u;%um", (unsigned) (foreground & 8 ? 90 : 30) + ansi_colors[foreground & 7], (unsigned) 40 + ansi_colors[background]);
    
    if (attr & 0x80) {
        fputs ("\033[5m", stdout);
    } else {
        fputs ("\033[25m", stdout);
    }

}

static void int_10 (CPU8086 *cpu) {

    uint8_t ah = get_reg8 (cpu, 4);
    
#if     defined (_WIN32)
    /* BIOS screen operations must see any pending direct video-memory writes. */
    if (mem_video_dirty) {
        bios_video_refresh (cpu);
    }
#endif
    
    switch (ah) {
    
        case 0x00: {                    /* set video mode */
        
            uint8_t requested = get_reg8 (cpu, REG_AX);
            uint8_t mode = requested & 0x7F;
            
            unsigned row, col;
            
            if (mode != 0x00 && mode != 0x01 && mode != 0x02 && mode != 0x03 && mode != 0x07) {
            
                fprintf (stderr, "Unsupported Video Mode 0x%02X\n", mode);
                break;
            
            }
            
            bios_video_columns = mode <= 1 ? 40 : 80;
            bios_video_mode = mode;
            bios_video_page = 0;
            
            bios_cursor_start = mode == 7 ? 0x0B : 0x06;
            bios_cursor_end = mode == 7 ? 0x0C : 0x07;
            
            memset (bios_cursor_row, 0, sizeof (bios_cursor_row));
            memset (bios_cursor_col, 0, sizeof (bios_cursor_col));
            
            bios_video_initialize ();
            
            if (!(requested & 0x80)) {
            
                for (row = 0; row < bios_video_rows; row++) {
                
                    for (col = 0; col < bios_video_columns; col++) {
                    
                        bios_video_chars[row][col] = ' ';
                        bios_video_attrs[row][col] = 0x07;
                    
                    }
                
                }
                
                bios_video_copy_to_memory (cpu, 0, 0, 24, bios_video_columns - 1);
                bios_video_clear_other_pages (cpu);
            
            } else {
                bios_video_copy_from_memory (cpu);
            
            }
            
            bios_video_update_bda (cpu);
            
#if     defined (_WIN32)
            if (bios_windows_legacy_console ()) {
            
                HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
                
                if (!(requested & 0x80)) {
                
                    fflush (stdout);
                    dos_windows_clear_screen (handle, 0x07);
                
                }
                
                bios_windows_render (0, 0, 24, bios_video_columns - 1);
                bios_windows_set_cursor (0, 0);
                bios_windows_show_cursor (1);
            
            } else if (bios_windows_vt_console ()) {
            
                if (!(requested & 0x80)) {
                
                    fputs ("\033[0m\033[2J", stdout);
                    fflush (stdout);
                
                }
                
                bios_windows_vt_render_region (0, 0, 24, bios_video_columns - 1);
                
                bios_video_host_redraw = 0;
                fputs ("\033[H\033[?25h", stdout);
                
                fflush (stdout);
            
            } else
#endif
            {
            
                if (!(requested & 0x80)) {
                    fputs ("\033[0m\033[2J\033[3J", stdout);
                }
                
                fputs ("\033[H\033[?25h", stdout);
                fflush (stdout);
            
            }
            
            break;
        
        }
        
        case 0x01:                      /* set cursor shape */
        
            bios_cursor_start = (uint8_t) (cpu->regs[REG_CX] >> 8);
            bios_cursor_end = (uint8_t) cpu->regs[REG_CX];
            
            mem_write16 (cpu->mem, cpu->mem_size, 0x460, cpu->regs[REG_CX]);
            
#if     defined (_WIN32)
            if (bios_windows_legacy_console ()) {
                bios_windows_show_cursor (!(bios_cursor_start & 0x20));
            } else
#endif
            {
            
                fputs ((bios_cursor_start & 0x20) ? "\033[?25l" : "\033[?25h", stdout);
                fflush (stdout);
            
            }
            
            break;
        
        case 0x02: {                    /* set cursor position */
        
            uint8_t page = (uint8_t) (cpu->regs[REG_BX] >> 8);
            
            uint8_t row = (uint8_t) (cpu->regs[REG_DX] >> 8);
            uint8_t col = (uint8_t) cpu->regs[REG_DX];
            
            if (page < 8) {
            
                bios_cursor_row[page] = row;
                bios_cursor_col[page] = col;
                
                mem_write16 (cpu->mem, cpu->mem_size, 0x450 + (unsigned) page * 2, cpu->regs[REG_DX]);
                
                if (page == bios_video_page) {
                
                    unsigned viewport_width, viewport_height;
                    
                    bios_video_set_crtc_cursor (cpu);
                    bios_video_viewport (&viewport_width, &viewport_height);
                    
                    if (row < viewport_height && col < viewport_width) {
                
#if     defined (_WIN32)
                        if (bios_windows_legacy_console ()) {
                            bios_windows_set_cursor (row, col);
                        } else
#endif
                        {
                        
                            fprintf (stdout, "\033[%u;%uH", (unsigned) row + 1, (unsigned) col + 1);
                            fflush (stdout);
                        
                        }
                    
                    }
                
                }
            
            }
            
            break;
        
        }
        
        case 0x03: {                    /* get cursor position and shape */
        
            uint8_t page = (uint8_t) (cpu->regs[REG_BX] >> 8);
            
            cpu->regs[REG_CX] = (uint16_t) (((uint16_t) bios_cursor_start << 8) | bios_cursor_end);
            cpu->regs[REG_DX] = page < 8 ? (uint16_t) (((uint16_t) bios_cursor_row[page] << 8) | bios_cursor_col[page]) : 0;
            
            break;
        
        }
        
        case 0x05: {                    /* select active video page */
        
            uint8_t page = get_reg8 (cpu, REG_AX);
            
            if (page >= bios_video_page_count () || page == bios_video_page) {
                break;
            }
            
            bios_video_page = page;
            
            mem_write16 (cpu->mem, cpu->mem_size, 0x44E, page * bios_video_page_size ());
            mem_write8 (cpu->mem, cpu->mem_size, 0x462, page);
            
            cpu->crtc_registers[0x0C] = (uint8_t) ((page * (bios_video_page_size () / 2)) >> 8);
            cpu->crtc_registers[0x0D] = (uint8_t) (page * (bios_video_page_size () / 2));
            
            bios_video_set_crtc_cursor (cpu);
            
            bios_video_copy_from_memory (cpu);
            bios_video_force_redraw = 1;
            
#if     defined (_WIN32)
            if (bios_windows_legacy_console ()) {
                bios_windows_set_cursor (bios_cursor_row[page], bios_cursor_col[page]);
            } else
#endif
            {
            
                fprintf (stdout, "\033[%u;%uH", (unsigned) bios_cursor_row[page] + 1, (unsigned) bios_cursor_col[page] + 1);
                fflush (stdout);
            
            }
            
            break;
        
        }
        
        case 0x06: /* scroll window up */   case 0x07: /* scroll window down */ {
        
            uint8_t lines = get_reg8 (cpu, REG_AX);
            uint8_t attr = (uint8_t) (cpu->regs[REG_BX] >> 8);
            
            uint8_t top = (uint8_t) (cpu->regs[REG_CX] >> 8);
            uint8_t left = (uint8_t) cpu->regs[REG_CX];
            
            uint8_t bottom = (uint8_t) (cpu->regs[REG_DX] >> 8);
            uint8_t right = (uint8_t) cpu->regs[REG_DX];
            
            unsigned viewport_width, viewport_height;
            
            unsigned row;
            unsigned col;
            unsigned height;
            
            if (top >= bios_video_rows || left >= bios_video_columns || bottom >= bios_video_rows || right >= bios_video_columns || top > bottom || left > right) {
                break;
            }
            
            bios_video_initialize ();
            
            height = (unsigned) bottom - top + 1;
            bios_video_viewport (&viewport_width, &viewport_height);
            
            if (lines >= height) {
                lines = 0;
            }

            if (lines != 0) {

                if (ah == 0x06) {
                
                    for (row = top; row + lines <= bottom; row++) {
                    
                        for (col = left; col <= right; col++) {
                        
                            bios_video_chars[row][col] = bios_video_chars[row + lines][col];
                            bios_video_attrs[row][col] = bios_video_attrs[row + lines][col];
                        
                        }
                    
                    }
                
                } else {
                
                    for (row = bottom + 1; row > (unsigned) top + lines; ) {
                    
                        row--;
                        
                        for (col = left; col <= right; col++) {
                        
                            bios_video_chars[row][col] = bios_video_chars[row - lines][col];
                            bios_video_attrs[row][col] = bios_video_attrs[row - lines][col];
                        
                        }
                    
                    }
                
                }
            
            }
            
            for (row = top; row <= bottom; row++) {
            
                if (lines != 0 && (ah == 0x06 ? row <= (unsigned) bottom - lines : row >= (unsigned) top + lines)) {
                    continue;
                }
                
                for (col = left; col <= right; col++) {
                
                    bios_video_chars[row][col] = ' ';
                    bios_video_attrs[row][col] = attr;
                
                }
            
            }
            
            bios_video_copy_to_memory (cpu, top, left, bottom, right);
            
#if     defined (_WIN32)
            if (bios_windows_legacy_console ()) {
            
                if (lines == 0 && top == 0 && left == 0 && bottom == 24 && right == bios_video_columns - 1) {
                
                    fflush (stdout);
                    dos_windows_clear_screen (GetStdHandle (STD_OUTPUT_HANDLE), (WORD) (attr & 0x7F));
                
                }
                
                bios_windows_render (top, left, bottom, right);
            
            } else if (bios_windows_vt_console ()) {
                bios_windows_vt_render_region (top, left, bottom, right);
            } else
#endif
            {
            
                if (lines == 0 && top == 0 && left == 0 && bottom == 24 && right == 79) {
                    fputs ("\033[2J\033[3J", stdout);
                }
                
                fputs ("\0337\033[?7l", stdout);
                
                for (row = top; row <= bottom && row < viewport_height; row++) {
                
                    if (left >= viewport_width) {
                        continue;
                    }
                    
                    fprintf (stdout, "\033[%u;%uH", row + 1, (unsigned) left + 1);
                    
                    for (col = left; col <= right && col < viewport_width; col++) {
                    
                        uint8_t cell_attr = bios_video_attrs[row][col];
                        bios_video_set_attribute (cell_attr);
                        
                        bios_write_video_char (bios_video_chars[row][col]);
                    
                    }
                
                }
                
                fputs ("\033[0m\033[?7h\0338", stdout);
                fflush (stdout);
            
            }
            
            break;
        
        }
        
        case 0x08: {                    /* read character and attribute at cursor */
        
            uint8_t page = (uint8_t) (cpu->regs[REG_BX] >> 8);
            
            unsigned row, col;
            uint32_t address;
            
            if (page >= bios_video_page_count ()) {
                break;
            }
            
            row = bios_cursor_row[page];
            col = bios_cursor_col[page];
            
            if (row >= bios_video_rows || col >= bios_video_columns) {
                break;
            }
            
            address = bios_video_page_base (page) + (row * bios_video_columns + col) * 2;
            
            cpu->regs[REG_AX] = mem_read8 (cpu->mem, cpu->mem_size, address) | ((uint16_t) mem_read8 (cpu->mem, cpu->mem_size, address + 1) << 8);
            break;
        
        }
        
        case 0x09: {                    /* write character and attribute */
        
            uint8_t page = (uint8_t) (cpu->regs[REG_BX] >> 8);
            uint8_t al = get_reg8 (cpu, REG_AX);
            
            uint16_t count = cpu->regs[REG_CX];
            uint16_t i;
            
            unsigned viewport_width, viewport_height;
            
            if (page < bios_video_page_count () && page != bios_video_page) {
            
                uint32_t base = bios_video_page_base (page);
                
                for (i = 0; i < count && (unsigned) bios_cursor_col[page] + i < bios_video_columns; i++) {
                
                    if (bios_cursor_row[page] < bios_video_rows) {
                    
                        uint32_t address = base + (bios_cursor_row[page] * bios_video_columns + bios_cursor_col[page] + i) * 2;
                        
                        cpu->mem[address] = al;
                        cpu->mem[address + 1] = (uint8_t) cpu->regs[REG_BX];
                    
                    }
                
                }
            
            } else if (page == bios_video_page) {
            
                bios_video_viewport (&viewport_width, &viewport_height);
                bios_video_initialize ();
                
                for (i = 0; i < count && (unsigned) bios_cursor_col[page] + i < bios_video_columns; i++) {
                
                    if (bios_cursor_row[page] < bios_video_rows) {
                    
                        bios_video_chars[bios_cursor_row[page]][bios_cursor_col[page] + i] = al;
                        bios_video_attrs[bios_cursor_row[page]][bios_cursor_col[page] + i] = (uint8_t) cpu->regs[REG_BX];
                    
                    }
                
                }
                
                if (bios_cursor_row[page] < bios_video_rows && bios_cursor_col[page] < bios_video_columns) {
                
                    unsigned right = (unsigned) bios_cursor_col[page] + (count ? count - 1 : 0);
                    
                    if (right >= bios_video_columns) {
                        right = bios_video_columns - 1;
                    }
                    
                    if (count != 0) {
                        bios_video_copy_to_memory (cpu, bios_cursor_row[page], bios_cursor_col[page], bios_cursor_row[page], right);
                    }
                
                }
                
#if     defined (_WIN32)
                if (bios_windows_legacy_console ()) {
                
                    if (count != 0 && bios_cursor_row[page] < bios_video_rows && bios_cursor_col[page] < bios_video_columns) {
                    
                        unsigned right = (unsigned) bios_cursor_col[page] + count - 1;
                        
                        if (right >= bios_video_columns) {
                            right = bios_video_columns - 1;
                        }
                        
                        bios_windows_render (bios_cursor_row[page], bios_cursor_col[page], bios_cursor_row[page], right);
                    
                    }
                
                } else if (bios_windows_vt_console ()) {
                
                    if (count != 0 && bios_cursor_row[page] < bios_video_rows && bios_cursor_col[page] < bios_video_columns) {
                    
                        unsigned right = (unsigned) bios_cursor_col[page] + count - 1;
                        
                        if (right >= bios_video_columns) {
                            right = bios_video_columns - 1;
                        }
                        
                        bios_windows_vt_render_region (bios_cursor_row[page], bios_cursor_col[page], bios_cursor_row[page], right);
                    
                    }
                
                } else
#endif
                {
                
                    fputs ("\0337\033[?7l", stdout);
                    bios_video_set_attribute ((uint8_t) cpu->regs[REG_BX]);
                    
                    if (bios_cursor_row[page] < viewport_height && bios_cursor_col[page] < viewport_width) {
                    
                        fprintf (stdout, "\033[%u;%uH", (unsigned) bios_cursor_row[page] + 1, (unsigned) bios_cursor_col[page] + 1);
                        
                        for (i = 0; i < count && (unsigned) bios_cursor_col[page] + i < viewport_width; i++) {
                            bios_write_video_char (al);
                        }
                    
                    }
                    
                    fputs ("\033[0m\033[?7h\0338", stdout);
                    fflush (stdout);
                
                }
            
            }
            
            break;
        
        }
        
        case 0x0B: {                    /* set background/border colour or CGA palette */
        
            uint8_t bh = (uint8_t) (cpu->regs[REG_BX] >> 8);
            
            if (bh == 0 && bios_video_mode <= 0x03) {
            
                uint8_t color = mem_read8 (cpu->mem, cpu->mem_size, 0x466);
                uint8_t border = (uint8_t) (cpu->regs[REG_BX] & 0x1F);
                
                /* Text modes only change the border, not character attributes. */
                mem_write8 (cpu->mem, cpu->mem_size, 0x466, (uint8_t) ((color & 0xE0) | border));
            
            }
            
            /* BH=01h selects a CGA graphics palette; graphics modes are not supported. */
            break;
        
        }
        
        case 0x0E: {
        
            uint8_t al = get_reg8 (cpu, 0);
            dos_write_console_char (stdout, al);
            
            fflush (stdout);
            break;
        
        }
        
        case 0x0F:                      /* get current video mode */
        
            set_reg8 (cpu, 0, bios_video_mode);
            set_reg8 (cpu, 7, bios_video_page);             /* page */
            
            set_reg8 (cpu, 4, (uint8_t) bios_video_columns);
            break;
        
        case 0x11:                      /* EGA/VGA character generator */
        
            /* The emulated CGA/MDA text adapters have no programmable fonts. */
            break;
        
        case 0x12:                      /* EGA/VGA alternate function select */
        
            /* No EGA/VGA: leave BL=10h unchanged for EGA detection. */
            break;
        
        case 0x1A:                      /* VGA display combination code */
        
            /* No VGA: leave AL unchanged instead of returning AL=1Ah. */
            break;
        
        default:
        
            printf ("Unknown Video Function 0x%02X\n", ah);
            break;
    
    }

}

void bios_video_refresh (CPU8086 *cpu) {

    unsigned viewport_width, viewport_height;
    unsigned dirty = mem_video_dirty;
    unsigned row, col, first, last;
    
#if     defined (_WIN32)
    unsigned redraw_top = bios_video_rows, redraw_left = bios_video_columns;
    unsigned redraw_bottom = 0, redraw_right = 0;
    unsigned vt_first[25], vt_last[25];
    
    HANDLE output_handle = GetStdHandle (STD_OUTPUT_HANDLE);
    DWORD output_mode;
    
    int legacy_console = bios_windows_legacy_console ();
    int vt_console = !legacy_console && output_handle != NULL && output_handle != INVALID_HANDLE_VALUE && GetConsoleMode (output_handle, &output_mode) && (output_mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING);
#endif
    
    int full_redraw = bios_video_force_redraw;
    uint32_t base;
    
    mem_video_dirty = 0;
    bios_video_force_redraw = 0;
    
    if (!(dirty & (bios_video_mode == 0x07 ? 1 : 2)) && !full_redraw) {
        return;
    }
    
    bios_video_initialize ();
    bios_video_viewport (&viewport_width, &viewport_height);
    
    base = bios_video_memory_base ();
    
#if     defined (_WIN32)
    if (vt_console) {
    
        for (row = 0; row < 25; row++) {
        
            vt_first[row] = bios_video_columns;
            vt_last[row] = 0;
        
        }
    
    }
#endif
    
    if (bios_video_host_redraw) {
    
        for (row = 0; row < bios_video_rows && !full_redraw; row++) {
        
            for (col = 0; col < bios_video_columns; col++) {
            
                uint32_t address = base + (row * bios_video_columns + col) * 2;
                
                if (cpu->mem[address] != bios_video_chars[row][col] || cpu->mem[address + 1] != bios_video_attrs[row][col]) {
                
                    full_redraw = 1;
                    break;
                
                }
            
            }
        
        }
        
        if (full_redraw) {
        
            bios_video_host_redraw = 0;
            
#if     defined (_WIN32)
            if (!legacy_console) {
            
                fflush (stdout);
                dos_windows_clear_screen (GetStdHandle (STD_OUTPUT_HANDLE), 0x07);
            
            } else if (vt_console) {
                fputs ("\033[2J", stdout);
            } else
#endif
            {
                fputs ("\033[2J\033[3J", stdout);
            }
        
        }
    
    }
    
#ifndef     _WIN32
    fputs ("\0337\033[?7l", stdout);
#else
    if (!legacy_console && !vt_console) {
        fputs ("\0337\033[?7l", stdout);
    }
#endif
    
    for (row = 0; row < bios_video_rows; row++) {
    
        first = bios_video_columns;
        last = 0;
        
        for (col = 0; col < bios_video_columns; col++) {
        
            uint32_t address = base + (row * bios_video_columns + col) * 2;
            
            uint8_t ch = cpu->mem[address];
            uint8_t attr = cpu->mem[address + 1];
            
            if (ch != bios_video_chars[row][col] || attr != bios_video_attrs[row][col]) {
            
                bios_video_chars[row][col] = ch;
                bios_video_attrs[row][col] = attr;
                
                if (first == bios_video_columns) {
                    first = col;
                }
                
                last = col;
            
            }
        
        }
        
        if (full_redraw) {
        
            first = 0;
            last = bios_video_columns - 1;
        
        }
        
        if (first == bios_video_columns || row >= viewport_height || first >= viewport_width) {
            continue;
        }
        
#if     defined (_WIN32)
        if (legacy_console) {
        
            if (row < redraw_top) redraw_top = row;
            if (first < redraw_left) redraw_left = first;
            if (row > redraw_bottom) redraw_bottom = row;
            if (last > redraw_right) redraw_right = last;
        
        } else if (vt_console) {
        
            vt_first[row] = first;
            vt_last[row] = last;
        
        } else
#endif
        {
        
            unsigned end = last < viewport_width ? last : viewport_width - 1;
            fprintf (stdout, "\033[%u;%uH", row + 1, first + 1);
            
            for (col = first; col <= end; col++) {
            
                bios_video_set_attribute (bios_video_attrs[row][col]);
                bios_write_video_char (bios_video_chars[row][col]);
            
            }
        
        }
    
    }
    
#ifndef     _WIN32
    fputs ("\033[0m\033[?7h\0338", stdout);
#else
    if (legacy_console && redraw_top < bios_video_rows) {
        bios_windows_render (redraw_top, redraw_left, redraw_bottom, redraw_right);
    }
    
    if (vt_console) {
        bios_windows_vt_render (vt_first, vt_last, viewport_width, viewport_height);
    } else if (!legacy_console) {
        fputs ("\033[0m\033[?7h\0338", stdout);
    }
#endif
    
    fflush (stdout);

}

static void int_13 (CPU8086 *cpu) {

    uint8_t ah = get_reg8 (cpu, 4);
    
    switch (ah) {
    
        case 0x00:                      /* reset disk system */
        
            set_reg8 (cpu, 4, 0x00);
            cpu->flags &= (uint16_t) ~FLAG_CF;
            break;
        
        case 0x02:                      /* read sectors */
        
            /* Stub success for now */
            set_reg8 (cpu, 4, 0x00);    /* AH=status */
            set_reg8 (cpu, 0, get_reg8 (cpu, REG_AX));      /* preserve AL sector count if you want */
            cpu->flags &= (uint16_t) ~FLAG_CF;
            break;
        
        case 0x08:                      /* get drive parameters */
        
            set_reg8 (cpu, 4, 0x00);                        /* success */
            
            cpu->regs[REG_CX] = 0x0001;                     /* tiny fake geometry */
            cpu->regs[REG_DX] = 0x0001;
            
            cpu->flags &= (uint16_t) ~FLAG_CF;
            break;
        
        default:
        
            printf ("Unknown Disk Function 0x%02X\n", ah);
            
            set_reg8 (cpu, 4, 0x01);  /* status */
            cpu->flags |= FLAG_CF;
            
            break;
    
    }

}

static void int_15 (CPU8086 *cpu) {

    if (cpu->regs[REG_AX] == 0x1022 && cpu->regs[REG_BX] == 0x0000) {
    
        /* TopView/DESQview version check: no multitasker is installed. */
        cpu->regs[REG_BX] = 0x0000;
    
    }
    
    /* Unsupported BIOS function. */
    set_reg8 (cpu, 4, 0x86);
    cpu->flags |= FLAG_CF;

}

static void int_16 (CPU8086 *cpu) {

    uint8_t ah = get_reg8 (cpu, 4);
    int ch;
    
    switch (ah) {
    
        case 0x00:                      /* read key */
        case 0x10:                      /* enhanced read key */
        
            ch = bios_read_key (cpu, 1);
            
            cpu->regs[REG_AX] = (uint16_t) (ch < 0 ? 0 : ch);
            cpu->flags &= ~FLAG_ZF;
            
            break;
        
        case 0x01:                      /* check key */
        case 0x11:                      /* enhanced check key */
        
            if (bios_key_pending < 0) {
                bios_key_pending = bios_read_key (cpu, 0);
            }
            
            if (bios_key_pending < 0) {
                cpu->flags |= FLAG_ZF;
            } else {
            
                cpu->regs[REG_AX] = (uint16_t) bios_key_pending;
                cpu->flags &= ~FLAG_ZF;
            
            }
            
            break;
        
        case 0x02:                      /* read keyboard shift flags */
        
#if     defined (_WIN32)
            host_windows_key_available (stdout);
            mem_write8 (cpu->mem, cpu->mem_size, get_phys (0x0040, 0x0017), host_windows_shift_flags);
#endif
            
            set_reg8 (cpu, 0, mem_read8 (cpu->mem, cpu->mem_size, get_phys (0x0040, 0x0017)));
            break;
        
        default:
        
            printf ("Unknown Keyboard Function 0x%02X\n", ah);
            break;
    
    }

}

static int bios_timer_date = -1;

void bios_timer_refresh (CPU8086 *cpu) {

    uint32_t milliseconds;
    uint32_t seconds;
    uint32_t ticks;
    
    uint64_t elapsed;
    int date;
    
#if     defined (_WIN32)
    SYSTEMTIME now;
    GetLocalTime (&now);
    
    seconds = (uint32_t) (now.wHour * 3600 + now.wMinute * 60 + now.wSecond);
    milliseconds = (uint32_t) now.wMilliseconds;
    
    date = (now.wYear * 12 + now.wMonth) * 32 + now.wDay;
#else
    struct timeval now;
    const struct tm *local;
    
    if (gettimeofday (&now, NULL) != 0) {
        return;
    }
    
    local = localtime (&now.tv_sec);
    
    if (local == NULL) {
        return;
    }
    
    seconds = (uint32_t) (local->tm_hour * 3600 + local->tm_min * 60 + local->tm_sec);
    milliseconds = (uint32_t) (now.tv_usec / 1000);
    
    date = local->tm_year * 366 + local->tm_yday;
#endif
    
    elapsed = (uint64_t) seconds * 1000 + milliseconds;
    ticks = (uint32_t) (elapsed * 1193182 / (65536ULL * 1000));
    
    if (bios_timer_date >= 0 && bios_timer_date != date) {
        mem_write8 (cpu->mem, cpu->mem_size, 0x0470, 1);
    }
    
    bios_timer_date = date;
    mem_write32 (cpu->mem, cpu->mem_size, 0x046C, ticks);

}

static uint8_t bios_bcd (unsigned value) {
    return (uint8_t) (((value / 10) << 4) | (value % 10));
}

static void int_1A (CPU8086 *cpu) {

    uint8_t ah = get_reg8 (cpu, 4);
    time_t now = time (NULL);
    
    const struct tm *local = localtime (&now);
    
    if (local == NULL) {
    
        cpu->flags |= FLAG_CF;
        return;
    
    }
    
    switch (ah) {
    
        case 0x00: {                    /* Read BIOS timer tick count */
        
            uint32_t ticks;
            
            bios_timer_refresh (cpu);
            ticks = (uint32_t) mem_read16 (cpu->mem, cpu->mem_size, 0x046C) | ((uint32_t) mem_read16 (cpu->mem, cpu->mem_size, 0x046E) << 16);
            
            cpu->regs[REG_CX] = (uint16_t) (ticks >> 16);
            cpu->regs[REG_DX] = (uint16_t) ticks;
            
            set_reg8 (cpu, REG_AX, mem_read8 (cpu->mem, cpu->mem_size, 0x0470));
            mem_write8 (cpu->mem, cpu->mem_size, 0x0470, 0);
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        case 0x02:                      /* Read RTC time */
        
            cpu->regs[REG_CX] = (uint16_t) ((bios_bcd ((unsigned) local->tm_hour) << 8) | bios_bcd ((unsigned) local->tm_min));
            cpu->regs[REG_DX] = (uint16_t) ((bios_bcd ((unsigned) local->tm_sec) << 8) | (local->tm_isdst > 0 ? 1 : 0));
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        case 0x04: {                    /* Read RTC date */
        
            unsigned year = (unsigned) (local->tm_year + 1900);
            
            cpu->regs[REG_CX] = (uint16_t) ((bios_bcd (year / 100) << 8) | bios_bcd (year % 100));
            cpu->regs[REG_DX] = (uint16_t) ((bios_bcd ((unsigned) local->tm_mon + 1) << 8) | bios_bcd ((unsigned) local->tm_mday));
            
            cpu->flags &= ~FLAG_CF;
            break;
        
        }
        
        default:
        
            printf ("Unhandled INT 1Ah Function: AH=0x%02X\n", ah);
            
            cpu->flags |= FLAG_CF;
            break;

    }

}

/**
 * The IVT contains real guest addresses. All 256 vectors initially point
 * to resident stubs (INT FFh / IRET / NOP). The stub-local INT FFh is the
 * native-services bridge; other INT FFh instructions use the guest IVT.
 */
void init_ivt (CPU8086 *cpu) {

    const char *env_long_paths = getenv ("DOS_LONG_PATHS");
    const char *env_p = getenv ("DOS_FCB_START");
    
    uint32_t stub_base = (uint32_t) DOS_IVT_STUB_SEG << 4;
    unsigned int i;
    
    for (i = 0; i < 256; i++) {
    
        uint32_t entry = stub_base + (i * 4);
        
        mem_write16 (cpu->mem, cpu->mem_size, i * 4, i * 4);
        mem_write16 (cpu->mem, cpu->mem_size, i * 4 + 2, DOS_IVT_STUB_SEG);
        
        mem_write8 (cpu->mem, cpu->mem_size, entry + 0, 0xCD);
        mem_write8 (cpu->mem, cpu->mem_size, entry + 1, 0xFF);
        mem_write8 (cpu->mem, cpu->mem_size, entry + 2, 0xCF);
        mem_write8 (cpu->mem, cpu->mem_size, entry + 3, 0x90);
    
    }
    
    dos_long_paths = env_long_paths && strcmp (env_long_paths, "1") == 0;
    
    if (env_p) {
    
        dos_drive_root[0] = '\0';
        dos_append_bounded (dos_drive_root, sizeof (dos_drive_root), env_p);
    
    } else {
    
        char cwd[260];
        
        const char *relative;
        size_t i;
        
#if     defined (_WIN32)
        if (_getcwd (cwd, sizeof (cwd)) != NULL && cwd[0] && cwd[1] == ':') {
        
            dos_drive_root[0] = cwd[0];
            dos_drive_root[1] = ':';
            dos_drive_root[2] = '/';
            dos_drive_root[3] = '\0';
            
            relative = cwd + 2;
        
#else
        if (getcwd (cwd, sizeof (cwd)) != NULL && cwd[0] == '/') {
        
# if    defined (__ANDROID__)
            dos_drive_root[0] = '\0';
            dos_append_bounded (dos_drive_root, sizeof (dos_drive_root), cwd);
            
            relative = cwd + strlen (cwd);
# else
            dos_drive_root[0] = '/';
            dos_drive_root[1] = '\0';
            
            relative = cwd;
# endif
        
#endif
            while (*relative == '/' || *relative == '\\') {
                relative++;
            }
            
            dos_current_dir[0] = '\0';
            dos_append_bounded (dos_current_dir, sizeof (dos_current_dir), relative);
            
            for (i = 0; dos_current_dir[i]; i++) {
            
                if (dos_current_dir[i] == '\\') {
                    dos_current_dir[i] = '/';
                }
            
            }
        
        }
    
    }
    
#ifndef     _WIN32
    /* Keep terminal input unbuffered so select() sees pending escape bytes. */
    if (isatty (fileno (stdin))) {
        setvbuf (stdin, NULL, _IONBF, 0);
    }
#endif
    
    file_handles[0] = stdin;
    file_handles[1] = stdout;
    file_handles[2] = stderr;
    
    file_handle_references[0] = 1;
    file_handle_references[1] = 1;
    file_handle_references[2] = 1;
    
    bios_video_initialize ();
    
    bios_video_copy_to_memory (cpu, 0, 0, bios_video_rows - 1, bios_video_columns - 1);
    
    bios_video_clear_other_pages (cpu);
    bios_video_mode = 0x07;
    
    bios_video_copy_to_memory (cpu, 0, 0, bios_video_rows - 1, bios_video_columns - 1);
    bios_video_mode = 0x03;
    
    bios_video_update_bda (cpu);
    bios_timer_refresh (cpu);

}

void handle_interrupt (CPU8086 *cpu, uint8_t interrupt_number) {

    uint32_t vector = (uint32_t) interrupt_number * 4;
    
    uint16_t new_ip = mem_read16 (cpu->mem, cpu->mem_size, vector);
    uint16_t new_cs = mem_read16 (cpu->mem, cpu->mem_size, vector + 2);
    
    push16 (cpu, cpu->flags);
    push16 (cpu, cpu->cs);
    push16 (cpu, cpu->ip);
    
    cpu->flags &= ~(0x0100 | FLAG_IF);
    
    cpu->cs = new_cs;
    cpu->ip = new_ip;

}

void native_interrupt (CPU8086 *cpu, uint8_t interrupt_number) {

    uint16_t sp = cpu->regs[REG_SP];
    uint16_t ss = cpu->ss;
    
    uint32_t flags_addr = get_phys (ss, (uint16_t) (sp + 4));
    uint16_t saved_flags = mem_read16 (cpu->mem, cpu->mem_size, flags_addr);
    
    /**
     * The existing C services expect the caller's flags, and return flags
     * by modifying cpu->flags. Carry their results across the guest IRET.
     */
    cpu->flags = saved_flags;
    
    switch (interrupt_number) {
    
        case 0x00:
        
            int_0 (cpu);
            break;
        
        case 0x01:
        
            int_1 (cpu);
            break;
        
        case 0x02:
        
            int_2 (cpu);
            break;
        
        case 0x03:
        
            int_3 (cpu);
            break;
        
        case 0x04:
        
            int_4 (cpu);
            break;
        
        case 0x05:
        
            int_5 (cpu);
            break;
        
        case 0x06:
        
            int_6 (cpu);
            break;
        
        case 0x07:
        
            int_7 (cpu);
            break;
        
        case 0x10:
        
            int_10 (cpu);
            break;
         
        case 0x11:
        
            /* BIOS equipment list: floppy drive and 80-column colour display. */
            cpu->regs[REG_AX] = 0x0021;
            break;
        
        case 0x13:
        
            int_13 (cpu);
            break;
        
        case 0x15:
        
            int_15 (cpu);
            break;
        
        case 0x16:
        
            int_16 (cpu);
            break;
         
        case 0x1A:
        
            int_1A (cpu);
            break;
        
        case 0x20:
        
            int_20 (cpu);
            break;
        
        case 0x21:
        
            int_21 (cpu);
            break;
         
        case 0x23:
        
            int_23 (cpu);
            break;
        
        case 0x28:
        
            /* DOS idle: without an installed handler, return immediately. */
            break;
        
        case 0x2F:
        
            int_2F (cpu);
            break;
        
        default:
        
            printf ("Unhandled Interrupt: 0x%02X\n", interrupt_number);
            break;
    
    }
    
    mem_write16 (cpu->mem, cpu->mem_size, flags_addr, cpu->flags);
    cpu->flags &= ~(0x0100 | FLAG_IF);

}
