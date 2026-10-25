/******************************************************************************
 * @file            video_native_win32.c
 *****************************************************************************/
#if     defined (_WIN32)

#include    <stdint.h>
#include    <stdlib.h>
#include    <string.h>
#include    <windows.h>

#include    "video_native.h"
#include    "memory.h"

static HWND video_window;

static int video_closed;
static int video_attempted;

static DWORD video_last_refresh;
static uint32_t video_pixels[640 * 400];

static uint8_t video_palette[256][3];
static uint8_t video_mask;
static uint8_t video_key_queue[128];

static unsigned video_key_head, video_key_tail;

static void video_key (UINT key, int pressed) {

    unsigned scan = MapVirtualKeyA (key, MAPVK_VK_TO_VSC) & 0x7F;
    unsigned next = (video_key_tail + 1) % sizeof (video_key_queue);
    
    if (!scan || next == video_key_head) {
        return;
    }
    
    video_key_queue[video_key_tail] = (uint8_t) (scan | (pressed ? 0 : 0x80));
    video_key_tail = next;

}

static LRESULT CALLBACK video_window_proc (HWND window, UINT message, WPARAM wparam, LPARAM lparam) {

    switch (message) {
    
        case WM_CLOSE:
        
            video_closed = 1;
            
            DestroyWindow (window);
            return 0;
        
        case WM_DESTROY:
        
            video_window = NULL;
            return 0;
        
        case WM_ERASEBKGND:
        
            return 1;
        
        case WM_KEYDOWN:        case WM_SYSKEYDOWN:
        
            video_key ((UINT) wparam, 1);
            return 0;
        
        case WM_KEYUP:          case WM_SYSKEYUP:
        
            video_key ((UINT) wparam, 0);
            return 0;
        
        case WM_PAINT: {
        
            PAINTSTRUCT paint;
            BITMAPINFO bitmap;
            
            RECT area;
            HDC dc = BeginPaint (window, &paint);
            
            memset (&bitmap, 0, sizeof (bitmap));
            
            bitmap.bmiHeader.biSize = sizeof (bitmap.bmiHeader);
            bitmap.bmiHeader.biWidth = 640;
            bitmap.bmiHeader.biHeight = -400;
            bitmap.bmiHeader.biPlanes = 1;
            bitmap.bmiHeader.biBitCount = 32;
            bitmap.bmiHeader.biCompression = BI_RGB;
            
            GetClientRect (window, &area);
            SetStretchBltMode (dc, COLORONCOLOR);
            
            StretchDIBits (dc, 0, 0, area.right, area.bottom, 0, 0, 640, 400, video_pixels, &bitmap, DIB_RGB_COLORS, SRCCOPY);
            EndPaint (window, &paint);
            
            return 0;
        
        }
    
    }
    
    return DefWindowProcA (window, message, wparam, lparam);

}

static int video_initialize (void) {

    WNDCLASSA cls;
    RECT area = { 0, 0, 640, 400 };
    
    const char *requested = getenv ("DOSEMU_VIDEO");
    DWORD style = WS_OVERLAPPEDWINDOW;
    
    if (requested && !strcmp (requested, "terminal")) {
        return 0;
    }
    
    memset (&cls, 0, sizeof (cls));
    
    cls.lpfnWndProc = video_window_proc;
    cls.hInstance = GetModuleHandleA (NULL);
    cls.hCursor = LoadCursor (NULL, IDC_ARROW);
    cls.lpszClassName = "dosemuVGA";
    
    if (!RegisterClassA (&cls) && GetLastError () != ERROR_CLASS_ALREADY_EXISTS) {
        return 0;
    }
    
    AdjustWindowRect (&area, style, FALSE);
    video_window = CreateWindowExA (0, cls.lpszClassName, "dosemu - VGA", style,
        CW_USEDEFAULT, CW_USEDEFAULT, area.right - area.left, area.bottom - area.top,
            NULL, NULL, cls.hInstance, NULL);
    
    if (!video_window) {
        return 0;
    }
    
    ShowWindow (video_window, SW_SHOW);
    UpdateWindow (video_window);
    
    SetFocus (video_window);
    return 1;

}

int video_native_active (void) {
    return video_window != NULL;
}

int video_native_poll (CPU8086 *cpu) {

    MSG message;
    
    while (video_window && PeekMessageA (&message, NULL, 0, 0, PM_REMOVE)) {
    
        TranslateMessage (&message);
        DispatchMessageA (&message);
    
    }
    
    if (video_closed) {
        return -1;
    }
    
    if (!cpu->kbd_scancode_pending && video_key_head != video_key_tail) {
    
        cpu->kbd_release_scancode = 0;
        cpu->kbd_scancode = video_key_queue[video_key_head];
        cpu->kbd_scancode_pending = 1;
        
        cpu->pic[0].irr |= 2;
        video_key_head = (video_key_head + 1) % sizeof (video_key_queue);
    
    }
    
    return 0;

}

int video_native_present (CPU8086 *cpu) {

    unsigned x, y;
    DWORD now;
    
    int palette_changed;
    
    if (video_closed) {
        return 1;
    }
    
    if (!video_attempted) {
    
        video_attempted = 1;
        video_initialize ();
    
    }
    
    if (!video_window) {
        return 0;
    }
    
    video_native_poll (cpu);
    palette_changed = memcmp (video_palette, cpu->vga_dac_palette, sizeof (video_palette)) || video_mask != cpu->vga_dac_mask;
    
    if (!(mem_video_dirty & 4) && !palette_changed) {
        return 1;
    }
    
    now = GetTickCount ();
    
    if (video_last_refresh && (DWORD) (now - video_last_refresh) < 50) {
        return 1;
    }
    
    video_last_refresh = now;
    
    for (y = 0; y < 200; y++) {
    
        for (x = 0; x < 320; x++) {
        
            uint8_t index = mem_vga_pixel (cpu, x, y) & cpu->vga_dac_mask;
            const uint8_t *rgb = cpu->vga_dac_palette[index];
            
            uint32_t color = ((uint32_t) (rgb[0] * 255 + 31) / 63 << 16) |
                ((uint32_t) (rgb[1] * 255 + 31) / 63 << 8) | (uint32_t) (rgb[2] * 255 + 31) / 63;
            
            video_pixels[y * 2 * 640 + x * 2] = color;
            video_pixels[y * 2 * 640 + x * 2 + 1] = color;
            video_pixels[(y * 2 + 1) * 640 + x * 2] = color;
            video_pixels[(y * 2 + 1) * 640 + x * 2 + 1] = color;
        
        }
    
    }
    
    memcpy (video_palette, cpu->vga_dac_palette, sizeof (video_palette));
    
    video_mask = cpu->vga_dac_mask;
    mem_video_dirty &= ~4u;
    
    InvalidateRect (video_window, NULL, FALSE);
    UpdateWindow (video_window);
    
    return 1;

}

void video_native_shutdown (void) {

    if (video_window) {
    
        DestroyWindow (video_window);
        video_window = NULL;
    
    }
    
    video_closed = 0;
    video_attempted = 0;
    
    video_last_refresh = 0;
    video_key_head = video_key_tail = 0;
    
    memset (video_palette, 0, sizeof (video_palette));
    video_mask = 0;

}

#endif
