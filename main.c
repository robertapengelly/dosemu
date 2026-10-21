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
#else
# include       <sys/ioctl.h>
# include       <termios.h>
# include       <unistd.h>
#endif

#include    "cpu.h"
#include    "loader.h"
#include    "interrupts.h"
#include    "heap.h"
#include    "memory.h"
#include    "psp.h"
#include    "speaker.h"

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
#else
static struct termios terminal_original_input;
static int terminal_input_mode_changed;

static void initialize_terminal_input (void) {

    struct termios mode;
    
    if (!isatty (STDIN_FILENO) || tcgetattr (STDIN_FILENO, &mode) != 0) {
        return;
    }
    
    terminal_original_input = mode;
    
    mode.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);
    mode.c_iflag &= ~(IXON | ICRNL);
    mode.c_cc[VMIN] = 1;
    mode.c_cc[VTIME] = 0;
    
    if (tcsetattr (STDIN_FILENO, TCSANOW, &mode) == 0) {
        terminal_input_mode_changed = 1;
    }

}

static void restore_terminal_input (void) {

    if (terminal_input_mode_changed) {
        tcsetattr (STDIN_FILENO, TCSANOW, &terminal_original_input);
    }

}
#endif

static int terminal_size_changed;

#if     defined (_WIN32)
static HANDLE terminal_resize_handle;
static CONSOLE_SCREEN_BUFFER_INFO terminal_original_size;
static SMALL_RECT terminal_resized_window;
#else
static unsigned terminal_original_columns, terminal_original_rows;
static unsigned terminal_resized_columns, terminal_resized_rows;
#endif

static void resize_terminal_output (void) {

#if     defined (_WIN32)
    CONSOLE_SCREEN_BUFFER_INFO info;
    
    COORD maximum, size;
    SMALL_RECT window;
    
    HANDLE handle = GetStdHandle (STD_OUTPUT_HANDLE);
    SHORT columns, rows;
    
    int buffer_changed;
    
    if (handle == NULL || handle == INVALID_HANDLE_VALUE || !GetConsoleScreenBufferInfo (handle, &info)) {
        return;
    }
    
    columns = (SHORT) (info.srWindow.Right - info.srWindow.Left + 1);
    rows = (SHORT) (info.srWindow.Bottom - info.srWindow.Top + 1);
    
    if (columns >= 80 && rows >= 25) {
        return;
    
    }
    
    if (columns < 80) {
        columns = 80;
    }
    
    if (rows < 25) {
        rows = 25;
    }
    
    maximum = GetLargestConsoleWindowSize (handle);
    
    if (maximum.X < columns || maximum.Y < rows) {
        return;
    }
    
    size = info.dwSize;
    
    if (size.X < columns) {
        size.X = columns;
    }
    
    if (size.Y < rows) {
        size.Y = rows;
    }
    
    buffer_changed = size.X != info.dwSize.X || size.Y != info.dwSize.Y;
    
    if (buffer_changed && !SetConsoleScreenBufferSize (handle, size)) {
        return;
    }
    
    window = info.srWindow;
    
    if (window.Left > size.X - columns) {
        window.Left = (SHORT) (size.X - columns);
    }
    
    if (window.Top > size.Y - rows) {
        window.Top = (SHORT) (size.Y - rows);
    }
    
    window.Right = (SHORT) (window.Left + columns - 1);
    window.Bottom = (SHORT) (window.Top + rows - 1);
    
    if (!SetConsoleWindowInfo (handle, TRUE, &window)) {
    
        if (buffer_changed) {
            SetConsoleScreenBufferSize (handle, info.dwSize);
        }
        
        return;
    
    }
    
    terminal_resize_handle = handle;
    terminal_original_size = info;
    
    terminal_resized_window = window;
    terminal_size_changed = 1;
#else
    struct winsize window;
    unsigned columns, rows;
    
    if (!isatty (STDOUT_FILENO) || ioctl (STDOUT_FILENO, TIOCGWINSZ, &window) != 0 || !window.ws_col || !window.ws_row) {
        return;
    }
    
    columns = window.ws_col < 80 ? 80 : window.ws_col;
    rows = window.ws_row < 25 ? 25 : window.ws_row;
    
    if (columns != window.ws_col || rows != window.ws_row) {
    
        terminal_original_columns = window.ws_col;
        terminal_original_rows = window.ws_row;
        
        terminal_resized_columns = columns;
        terminal_resized_rows = rows;
        
        terminal_size_changed = 1;
        fprintf (stdout, "\033[8;%u;%ut", rows, columns);
        
        fflush (stdout);
    
    }
#endif

}

static void restore_terminal_output_size (void) {

    if (!terminal_size_changed) {
        return;
    }
    
#if     defined (_WIN32)
    CONSOLE_SCREEN_BUFFER_INFO info;
    COORD position;
    
    if (terminal_resize_handle == NULL || terminal_resize_handle == INVALID_HANDLE_VALUE || !GetConsoleScreenBufferInfo (terminal_resize_handle, &info)) {
        return;
    }
    
    if (info.srWindow.Right - info.srWindow.Left != terminal_resized_window.Right - terminal_resized_window.Left ||
        info.srWindow.Bottom - info.srWindow.Top != terminal_resized_window.Bottom - terminal_resized_window.Top) {
        return;
    }
    
    position.X = (SHORT) (terminal_original_size.srWindow.Left + info.dwCursorPosition.X - info.srWindow.Left);
    position.Y = (SHORT) (terminal_original_size.srWindow.Top + info.dwCursorPosition.Y - info.srWindow.Top);
    
    if (position.X < terminal_original_size.srWindow.Left) {
        position.X = terminal_original_size.srWindow.Left;
    }
    
    if (position.X > terminal_original_size.srWindow.Right) {
        position.X = terminal_original_size.srWindow.Right;
    }
    
    if (position.Y < terminal_original_size.srWindow.Top) {
        position.Y = terminal_original_size.srWindow.Top;
    }
    
    if (position.Y > terminal_original_size.srWindow.Bottom) {
        position.Y = terminal_original_size.srWindow.Bottom;
    }
    
    if (SetConsoleWindowInfo (terminal_resize_handle, TRUE, &terminal_original_size.srWindow)) {
    
        SetConsoleCursorPosition (terminal_resize_handle, position);
        SetConsoleScreenBufferSize (terminal_resize_handle, terminal_original_size.dwSize);
    
    }
#else
    struct winsize window;
    
    if (!isatty (STDOUT_FILENO) || ioctl (STDOUT_FILENO, TIOCGWINSZ, &window) != 0) {
        return;
    }
    
    if (window.ws_col == terminal_resized_columns && window.ws_row == terminal_resized_rows) {
    
        fprintf (stdout, "\033[8;%u;%ut", terminal_original_rows, terminal_original_columns);
        fflush (stdout);
    
    }
#endif

}

static void restore_terminal_cursor (void) {

#if     defined (_WIN32)
    DWORD mode;
    host_windows_keyboard_shutdown ();
    
    if (terminal_output_handle == NULL || terminal_output_handle == INVALID_HANDLE_VALUE || !GetConsoleMode (terminal_output_handle, &mode)) {
    
        restore_terminal_output_size ();
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
    
#ifndef     _WIN32
    restore_terminal_input ();
#endif
    
    restore_terminal_output_size ();

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
    
#if     defined (_WIN32)
    DWORD video_refresh_time = GetTickCount ();
#endif
    
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
    
    if (atexit (speaker_shutdown) != 0) {
    
        fprintf (stderr, "Failed to register PC speaker cleanup\n");
        
        free (mem);
        return 1;
    
    }
    
    speaker_initialize ();
    resize_terminal_output ();
    
#if     defined (_WIN32)
    initialize_terminal_output ();
    host_windows_keyboard_initialize ();
#else
    initialize_terminal_input ();
#endif
    
    /* init_dos_heap(&cpu, 0x2000); */
    init_ivt (&cpu);
    
    while (!cpu.halted) {
    
        if (trace) {
        
            printf ("step=%lu ", steps);
            cpu_dump (&cpu);
        
        }
        
        if ((steps & 0x0FFF) == 0) {
            bios_timer_refresh (&cpu);
        }
        
        rc = cpu_step (&cpu);
        
#if     defined (_WIN32)
        if ((steps & 0xFF) == 0) {
        
            DWORD now = GetTickCount ();
            
            if ((DWORD) (now - video_refresh_time) >= 16) {
            
                bios_video_refresh (&cpu);
                video_refresh_time = now;
            
            }
        
        }
#else
        bios_video_refresh (&cpu);
#endif
        
        if (rc <= 0) {
        
#if     defined (_WIN32)
            bios_video_refresh (&cpu);
#endif
            
            free (mem);
            return 1;
        
        }
        
        steps++;
    
    }
    
#if     defined (_WIN32)
    bios_video_refresh (&cpu);
#endif
    
    free (mem);
    return 0;

}
