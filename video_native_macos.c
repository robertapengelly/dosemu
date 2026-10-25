/******************************************************************************
 * @file            video_native_macos.c
 *****************************************************************************/
#if     defined (__APPLE__)

#import      <Cocoa/Cocoa.h>
#include    <stdint.h>
#include    <stdlib.h>
#include    <string.h>

#include    "video_native.h"
#include    "memory.h"

static NSWindow *video_window;
static NSView *video_view;

static int video_last_text_mode = -1;
static int video_last_blink_enabled = -1;

static uint32_t video_pixels[640 * 400];

static uint8_t video_palette[256][3];
static uint8_t video_mask;
static uint8_t video_queue[128];

static unsigned video_head, video_tail;

static NSTimeInterval video_last_refresh;
static int video_attempted, video_closed;

static uint8_t video_macos_scan (unsigned code) {

    switch (code) {
    
        case 0:
        
            return 0x1E;
        
        case 1:
        
            return 0x1F;
        
        case 2:
        
            return 0x20;
        
        case 3:
        
            return 0x21;
        
        case 4:
        
            return 0x23;
        
        case 5:
        
            return 0x22;
        
        case 6:
        
            return 0x2C;
        
        case 7:
        
            return 0x2D;
        
        case 8:
        
            return 0x2E;
        
        case 9:
        
            return 0x2F;
        
        case 11:
        
            return 0x30;
        
        case 12:
        
            return 0x10;
        
        case 13:
        
            return 0x11;
        
        case 14:
        
            return 0x12;
        
        case 15:
        
            return 0x13;
        
        case 16:
        
            return 0x15;
        
        case 17:
        
            return 0x14;
        
        case 31:
        
            return 0x18;
        
        case 32:
        
            return 0x16;
        
        case 34:
        
            return 0x17;
        
        case 35:
        
            return 0x19;
        
        case 37:
        
            return 0x25;
        
        case 38:
        
            return 0x24;
        
        case 40:
        
            return 0x27;
        
        case 45:
        
            return 0x31;
        
        case 46:
        
            return 0x32;
        
        case 18:
        
            return 0x02;
        
        case 19:
        
            return 0x03;
        
        case 20:
        
            return 0x04;
        
        case 21:
        
            return 0x05;
        
        case 23:
        
            return 0x06;
        
        case 22:
        
            return 0x07;
        
        case 26:
        
            return 0x08;
        
        case 28:
        
            return 0x09;
        
        case 25:
        
            return 0x0A;
        
        case 29:
        
            return 0x0B;
        
        case 49:
        
            return 0x39;
        
        case 36:
        
            return 0x1C;
        
        case 51:
        
            return 0x0E;
        
        case 48:
        
            return 0x0F;
        
        case 53:
        
            return 0x01;
        
        case 123:
        
            return 0x4B;
        
        case 124:
        
            return 0x4D;
        
        case 125:
        
            return 0x50;
        
        case 126:
        
            return 0x48;
        
        case 122:
        
            return 0x3B;
        
        case 120:
        
            return 0x3C;
        
        case 99:
        
            return 0x3D;
        
        case 118:
        
            return 0x3E;
        
        case 96:
        
            return 0x3F;
        
        case 97:
        
            return 0x40;
        
        case 98:
        
            return 0x41;
        
        case 100:
        
            return 0x42;
        
        case 101:
        
            return 0x43;
        
        case 109:
        
            return 0x44;
        
        case 103:
        
            return 0x57;
        
        case 111:
        
            return 0x58;
        
        case 56:
        
            return 0x2A;
        
        case 60:
        
            return 0x36;
        
        case 59:
        case 62:
        
            return 0x1D;
        
        case 58:
        case 61:
        
            return 0x38;
    
    }
    
    return 0;

}

static void video_key (NSEvent *event, int pressed) {

    unsigned scan = video_macos_scan ([event keyCode]);
    unsigned next = (video_tail + 1) % sizeof (video_queue);
    
    if (!scan || next == video_head) {
        return;
    }
    
    video_queue[video_tail] = (uint8_t) (scan | (pressed ? 0 : 0x80));
    video_tail = next;

}

@interface DosemuVideoView : NSView
@end

@implementation DosemuVideoView

- (BOOL) acceptsFirstResponder {
    return YES;
}

- (void) keyDown: (NSEvent *) event {
    video_key (event, 1);
}

- (void) keyUp: (NSEvent *) event {
    video_key (event, 0);
}

- (void) drawRect: (NSRect) rect {

    CGColorSpaceRef colorSpace;
    CGContextRef context;
    
    CGImageRef image;
    CGDataProviderRef provider;
    
    (void) rect;
    
    if (!(context = [[NSGraphicsContext currentContext] CGContext])) {
        return;
    }
    
    colorSpace = CGColorSpaceCreateDeviceRGB ();
    provider = CGDataProviderCreateWithData (NULL, video_pixels, sizeof (video_pixels), NULL);
    
    image = CGImageCreate (640, 400, 8, 32, 640 * 4, colorSpace,
        kCGBitmapByteOrder32Little | kCGImageAlphaNoneSkipFirst,
            provider, NULL, false, kCGRenderingIntentDefault);
    
    if (image) {
    
        CGContextSetInterpolationQuality (context, kCGInterpolationNone);
        CGContextDrawImage (context, CGRectMake (0, 0, self.bounds.size.width, self.bounds.size.height), image);
        CGImageRelease (image);
    
    }
    
    CGDataProviderRelease (provider);
    CGColorSpaceRelease (colorSpace);

}

@end

@interface DosemuVideoDelegate : NSObject <NSWindowDelegate>
@end

@implementation DosemuVideoDelegate

- (void) windowWillClose: (NSNotification *) notification {

    (void) notification;
    video_closed = 1;

}

@end

static DosemuVideoDelegate *video_delegate;

static int video_initialize (void) {

    NSRect area = NSMakeRect (0, 0, 640, 400);
    
    NSUInteger style = NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable;
    const char *requested = getenv ("DOSEMU_VIDEO");
    
    if (requested && !strcmp (requested, "terminal")) {
        return 0;
    }
    
    [NSApplication sharedApplication];
    [NSApp setActivationPolicy: NSApplicationActivationPolicyRegular];
    [NSApp finishLaunching];
    
    video_window = [[NSWindow alloc] initWithContentRect: area styleMask: style
        backing: NSBackingStoreBuffered defer: NO];
    
    if (!video_window) {
        return 0;
    }
    
    video_view = [[DosemuVideoView alloc] initWithFrame: area];
    video_delegate = [[DosemuVideoDelegate alloc] init];
    
    [video_window setContentView: video_view];
    [video_window setDelegate: video_delegate];
    [video_window setTitle: @"dosemu - VGA"];
    [video_window center];
    [video_window makeKeyAndOrderFront: nil];
    [video_window makeFirstResponder: video_view];
    [NSApp activateIgnoringOtherApps: YES];
    
    return 1;

}

int video_native_active (void) {
    return video_window != nil;
}

int video_native_poll (CPU8086 *cpu) {

    NSEvent *event;
    
    if (video_window) {
    
        while ((event = [NSApp nextEventMatchingMask: NSEventMaskAny untilDate: [NSDate distantPast] inMode: NSDefaultRunLoopMode dequeue: YES]) != nil) {
            [NSApp sendEvent: event];
        }
        
        [NSApp updateWindows];
    
    }
    
    if (video_closed) {
        return -1;
    }
    
    if (!cpu->kbd_scancode_pending && video_head != video_tail) {
    
        cpu->kbd_release_scancode = 0;
        cpu->kbd_scancode = video_queue[video_head];
        cpu->kbd_scancode_pending = 1;
        cpu->pic[0].irr |= 2;
        
        video_head = (video_head + 1) % sizeof (video_queue);
    
    }
    
    return 0;

}

int video_native_present (CPU8086 *cpu, int text_mode, int blink_enabled) {

    unsigned x, y;
    NSTimeInterval now;
    
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
    
    if (!(mem_video_dirty & (text_mode ? 2u : 4u)) && (text_mode || !palette_changed) && video_last_text_mode == text_mode && (!text_mode || video_last_blink_enabled == blink_enabled)) {
        return 1;
    }
    
    now = [NSDate timeIntervalSinceReferenceDate];
    
    if (video_last_refresh && now - video_last_refresh < 0.05) {
        return 1;
    }
    
    video_last_refresh = now;
    
    if (text_mode) {
        video_native_text_render (cpu, video_pixels, blink_enabled);
    } else {
    
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
    
    }
    
    memcpy (video_palette, cpu->vga_dac_palette, sizeof (video_palette));
    
    video_mask = cpu->vga_dac_mask;
    mem_video_dirty &= ~(text_mode ? 2u : 4u);
    
    video_last_text_mode = text_mode;
    video_last_blink_enabled = blink_enabled;
    
    [video_view setNeedsDisplay: YES];
    [video_view displayIfNeeded];
    
    return 1;

}

void video_native_shutdown (void) {

    if (video_window) {
    
        [video_window setDelegate: nil];
        [video_window close];
        [video_window release];
        
        video_window = nil;
    
    }
    
    [video_view release];
    video_view = nil;
    
    [video_delegate release];
    video_delegate = nil;
    
    video_closed = 0;
    video_attempted = 0;
    video_last_refresh = 0;
    video_head = video_tail = 0;
    
    memset (video_palette, 0, sizeof (video_palette));
    video_mask = 0;
    
    video_last_text_mode = -1;
    video_last_blink_enabled = -1;

}

#endif
