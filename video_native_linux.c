/******************************************************************************
 * @file            video_native_linux.c
 *****************************************************************************/
#if     defined (__linux__) && !defined (__ANDROID__)

#include    <dlfcn.h>
#include    <errno.h>
#include    <fcntl.h>
#include    <poll.h>
#include    <stdint.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <sys/mman.h>
#include    <sys/stat.h>
#include    <sys/time.h>
#include    <unistd.h>

#if     defined (HAVE_X11)
# include       <X11/Xlib.h>
# include       <X11/Xutil.h>
# include       <X11/keysym.h>
#endif

#include    <linux/input-event-codes.h>

#include    "video_native.h"
#include    "memory.h"

#define     VIDEO_WIDTH                 640
#define     VIDEO_HEIGHT                400
#define     VIDEO_MAX_WIDTH             1920
#define     VIDEO_MAX_HEIGHT            1200

/* The Wayland core client ABI is loaded at runtime; no scanner or SDK is needed. */
struct wl_proxy;
struct wl_display;

struct wl_interface {

    const char *name;
    
    int version;
    int method_count;
    
    const struct wl_message *methods;
    int event_count;
    
    const struct wl_message *events;

};

struct wl_message {

    const char *name;
    const char *signature;
    
    const struct wl_interface **types;

};

static const struct wl_message xdg_wm_base_requests[] = {

    { "destroy", "", NULL },
    { "create_positioner", "n", NULL },
    { "get_xdg_surface", "no", NULL },
    { "pong", "u", NULL }

};

static const struct wl_message xdg_wm_base_events[] = {
    { "ping", "u", NULL }
};

static const struct wl_message xdg_surface_requests[] = {

    { "destroy", "", NULL },
    { "get_toplevel", "n", NULL },
    { "get_popup", "noo", NULL },
    { "set_window_geometry", "iiii", NULL },
    { "ack_configure", "u", NULL }

};

static const struct wl_message xdg_surface_events[] = {
    { "configure", "u", NULL }
};

static const struct wl_message xdg_toplevel_requests[] = {

    { "destroy", "", NULL },
    { "set_parent", "?o", NULL },
    { "set_title", "s", NULL },
    { "set_app_id", "s", NULL },
    { "show_window_menu", "ouii", NULL },
    { "move", "ou", NULL },
    { "resize", "ouu", NULL },
    { "set_max_size", "ii", NULL },
    { "set_min_size", "ii", NULL },
    { "set_maximized", "", NULL },
    { "unset_maximized", "", NULL },
    { "set_fullscreen", "?o", NULL },
    { "unset_fullscreen", "", NULL },
    { "set_minimized", "", NULL }

};

static const struct wl_message xdg_toplevel_events[] = {

    { "configure", "iia", NULL },
    { "close", "", NULL },
    { "configure_bounds", "2ii", NULL },
    { "wm_capabilities", "5a", NULL }

};

static const struct wl_interface xdg_wm_base_interface = {
    "xdg_wm_base", 1, 4, xdg_wm_base_requests, 1, xdg_wm_base_events
};

static const struct wl_interface xdg_surface_interface = {
    "xdg_surface", 1, 5, xdg_surface_requests, 1, xdg_surface_events
};

static const struct wl_interface xdg_toplevel_interface = {
    "xdg_toplevel", 1, 14, xdg_toplevel_requests, 2, xdg_toplevel_events
};

static struct {

    void *library;
    
    struct wl_display *display;
    struct wl_proxy *registry, *compositor, *shm, *seat, *keyboard;
    struct wl_proxy *surface, *wm_base, *xdg_surface, *toplevel;
    struct wl_proxy *pool, *buffers[2];
    
    unsigned char *pixels;
    size_t buffer_size;
    
    int busy[2];
    int width, height, configured, closed;
    int fd;
    
    const struct wl_interface *registry_interface, *compositor_interface;
    const struct wl_interface *surface_interface, *shm_interface, *pool_interface;
    const struct wl_interface *buffer_interface, *seat_interface, *keyboard_interface;
    
    struct wl_display *(*connect) (const char *);
    void (*disconnect) (struct wl_display *);
    
    int (*roundtrip) (struct wl_display *);
    int (*dispatch) (struct wl_display *);
    int (*dispatch_pending) (struct wl_display *);
    int (*flush) (struct wl_display *);
    int (*get_fd) (struct wl_display *);
    int (*add_listener) (struct wl_proxy *, void (**)(void), void *);
    
    void (*destroy) (struct wl_proxy *);
    
    uint32_t (*version) (struct wl_proxy *);
    struct wl_proxy *(*marshal) (struct wl_proxy *, uint32_t, const struct wl_interface *, uint32_t, uint32_t, ...);

} wayland;

static struct {

#if     defined (HAVE_X11)
    Display *display;
    Window window;
    Atom close_atom;
    GC gc;
    XImage *image;
#endif
    int width, height, redraw;

} x11;

static int backend;
static int attempted;
static int closed;

static uint64_t last_refresh;

static uint8_t previous_palette[256][3];
static uint8_t previous_mask;
static uint8_t key_queue[128];

static unsigned key_head, key_tail;

static uint64_t video_milliseconds (void) {

    struct timeval now;
    gettimeofday (&now, NULL);
    
    return (uint64_t) now.tv_sec * 1000 + now.tv_usec / 1000;

}

static void video_key (uint8_t scan, int pressed) {

    unsigned next = (key_tail + 1) % sizeof (key_queue);
    
    if (!scan || next == key_head) {
        return;
    }
    
    key_queue[key_tail] = pressed ? scan : (uint8_t) (scan | 0x80);
    key_tail = next;

}

static uint8_t video_evdev_scan (uint32_t code) {

    if (code <= KEY_F10) {
        return (uint8_t) code;
    }
    
    switch (code) {
    
        case KEY_UP:
        
            return 0x48;
        
        case KEY_DOWN:
        
            return 0x50;
        
        case KEY_LEFT:
        
            return 0x4B;
        
        case KEY_RIGHT:
        
            return 0x4D;
        
        case KEY_HOME:
        
            return 0x47;
        
        case KEY_END:
        
            return 0x4F;
        
        case KEY_PAGEUP:
        
            return 0x49;
        
        case KEY_PAGEDOWN:
        
            return 0x51;
        
        case KEY_INSERT:
        
            return 0x52;
        
        case KEY_DELETE:
        
            return 0x53;
        
        case KEY_F11:
        
            return 0x57;
        
        case KEY_F12:
        
            return 0x58;
        
        case KEY_RIGHTCTRL:
        
            return 0x1D;
        
        case KEY_RIGHTALT:
        
            return 0x38;
        
        case KEY_KPENTER:
        
            return 0x1C;
    
    }
    
    return 0;

}

#if     defined (HAVE_X11)
static uint8_t video_x11_scan (KeySym key) {

    if (key >= XK_a && key <= XK_z) {
    
        static const uint8_t letters[] = {
        
            0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23, 0x17,
            0x24, 0x25, 0x26, 0x32, 0x31, 0x18, 0x19, 0x10, 0x13,
            0x1F, 0x14, 0x16, 0x2F, 0x11, 0x2D, 0x15, 0x2C
        
        };
        
        return letters[key - XK_a];
    
    }
    
    if (key >= XK_A && key <= XK_Z) {
        return video_x11_scan (key - XK_A + XK_a);
    }
    
    if (key >= XK_1 && key <= XK_9) {
        return (uint8_t) (key - XK_1 + 2);
    }
    
    switch (key) {
    
        case XK_0:
        
            return 0x0B;
        
        case XK_Escape:
        
            return 0x01;
        
        case XK_BackSpace:
        
            return 0x0E;
        
        case XK_Tab:
        
            return 0x0F;
        
        case XK_Return:     case XK_KP_Enter:
        
            return 0x1C;
        
        case XK_space:
        
            return 0x39;
        
        case XK_Left:
        
            return 0x4B;
        
        case XK_Right:
        
            return 0x4D;
        
        case XK_Up:
        
            return 0x48;
        
        case XK_Down:
        
            return 0x50;
        
        case XK_Shift_L:
        
            return 0x2A;
        
        case XK_Shift_R:
        
            return 0x36;
        
        case XK_Control_L:  case XK_Control_R:
        
            return 0x1D;
        
        case XK_Alt_L:      case XK_Alt_R:
        
            return 0x38;
        
        case XK_F1:
        
            return 0x3B;
        
        case XK_F2:
        
            return 0x3C;
        
        case XK_F3:
        
            return 0x3D;
        
        case XK_F4:
        
            return 0x3E;
        
        case XK_F5:
        
            return 0x3F;
        
        case XK_F6:
        
            return 0x40;
        
        case XK_F7:
        
            return 0x41;
        
        case XK_F8:
        
            return 0x42;
        
        case XK_F9:
        
            return 0x43;
        
        case XK_F10:
        
            return 0x44;
        
        case XK_F11:
        
            return 0x57;
        
        case XK_F12:
        
            return 0x58;
        
        case XK_minus:
        
            return 0x0C;
        
        case XK_equal:
        
            return 0x0D;
        
        case XK_bracketleft:
        
            return 0x1A;
        
        case XK_bracketright:
        
            return 0x1B;
        
        case XK_semicolon:
        
            return 0x27;
        
        case XK_apostrophe:
        
            return 0x28;
        
        case XK_grave:
        
            return 0x29;
        
        case XK_backslash:
        
            return 0x2B;
        
        case XK_comma:
        
            return 0x33;
        
        case XK_period:
        
            return 0x34;
        
        case XK_slash:
        
            return 0x35;
        
        case XK_Delete:
        
            return 0x53;
        
        case XK_Home:
        
            return 0x47;
        
        case XK_End:
        
            return 0x4F;
        
        case XK_Page_Up:
        
            return 0x49;
        
        case XK_Page_Down:
        
            return 0x51;
        
        case XK_Insert:
        
            return 0x52;
    
    }
    
    return 0;

}

static int video_x11_initialize (void) {

    int screen;
    
    XSetWindowAttributes attrs;
    XVisualInfo visual_info;
    
    if (!getenv ("DISPLAY") || !(x11.display = XOpenDisplay (NULL))) {
        return 0;
    }
    
    screen = DefaultScreen (x11.display);
    
    if (!XMatchVisualInfo (x11.display, screen, DefaultDepth (x11.display, screen), TrueColor, &visual_info)) {
    
        XCloseDisplay (x11.display);
        
        x11.display = NULL;
        return 0;
    
    }
    
    attrs.background_pixel = BlackPixel (x11.display, screen);
    attrs.event_mask = ExposureMask | StructureNotifyMask | KeyPressMask | KeyReleaseMask;
    
    x11.window = XCreateWindow (x11.display, RootWindow (x11.display, screen), 0, 0,
        VIDEO_WIDTH, VIDEO_HEIGHT, 0, visual_info.depth, InputOutput, visual_info.visual,
            CWBackPixel | CWEventMask, &attrs);
    
    if (!x11.window) {
    
        XCloseDisplay (x11.display);
        
        x11.display = NULL;
        return 0;
    
    }
    
    x11.close_atom = XInternAtom (x11.display, "WM_DELETE_WINDOW", False);
    
    XSetWMProtocols (x11.display, x11.window, &x11.close_atom, 1);
    XStoreName (x11.display, x11.window, "dosemu - VGA");
    
    x11.gc = XCreateGC (x11.display, x11.window, 0, NULL);
    
    XMapWindow (x11.display, x11.window);
    XFlush (x11.display);
    
    x11.width = VIDEO_WIDTH;
    x11.height = VIDEO_HEIGHT;
    x11.redraw = 1;
    
    return 1;

}

static void video_x11_events (void) {

    XEvent event;
    
    while (XPending (x11.display)) {
    
        XNextEvent (x11.display, &event);
        
        switch (event.type) {
        
            case KeyPress:      case KeyRelease:
            
                video_key (video_x11_scan (XLookupKeysym (&event.xkey, 0)), event.type == KeyPress);
                break;
            
            case ConfigureNotify:
            
                if (event.xconfigure.width != x11.width || event.xconfigure.height != x11.height) {
                
                    x11.width = event.xconfigure.width;
                    x11.height = event.xconfigure.height;
                    x11.redraw = 1;
                
                }
                
                break;
            
            case Expose:
            
                x11.redraw = 1;
                break;
            
            case ClientMessage:
            
                if ((Atom) event.xclient.data.l[0] == x11.close_atom) {
                    closed = 1;
                }
                
                break;
            
            case DestroyNotify:
            
                closed = 1;
                break;
        
        }
    
    }

}

static void video_x11_draw (const uint32_t *pixels, int width, int height) {

    XImage *image;
    size_t size;
    
    int x, y;
    int depth;
    
    Visual *visual;
    
    width = width > VIDEO_MAX_WIDTH ? VIDEO_MAX_WIDTH : width;
    height = height > VIDEO_MAX_HEIGHT ? VIDEO_MAX_HEIGHT : height;
    
    if (width < 1 || height < 1) {
        return;
    }
    
    if (x11.image && (x11.image->width != width || x11.image->height != height)) {
    
        XDestroyImage (x11.image);
        x11.image = NULL;
    
    }
    
    if (!x11.image) {
    
        depth = DefaultDepth (x11.display, DefaultScreen (x11.display));
        visual = DefaultVisual (x11.display, DefaultScreen (x11.display));
        
        size = (size_t) width * height * 4;
        x11.image = XCreateImage (x11.display, visual, (unsigned) depth, ZPixmap, 0, (char *) calloc (1, size), (unsigned) width, (unsigned) height, 32, 0);
        
        if (!x11.image || !x11.image->data) {
        
            if (x11.image) {
            
                XDestroyImage (x11.image);
                x11.image = NULL;
            
            }
            
            closed = 1;
            return;
        
        }
    
    }
    
    image = x11.image;
    
    for (y = 0; y < height; y++) {
    
        for (x = 0; x < width; x++) {
        
            uint32_t rgb = pixels[(y * VIDEO_HEIGHT / height) * VIDEO_WIDTH + (x * VIDEO_WIDTH / width)];
            
            if (image->bits_per_pixel == 32 && image->byte_order == LSBFirst &&
                image->red_mask == 0x00FF0000 && image->green_mask == 0x0000FF00 &&
                    image->blue_mask == 0x000000FF) {
                ((uint32_t *) (image->data + y * image->bytes_per_line))[x] = rgb;
            } else {
                XPutPixel (image, x, y, rgb);
            }
        
        }
    
    }
    
    XPutImage (x11.display, x11.window, x11.gc, image, 0, 0, 0, 0, (unsigned) width, (unsigned) height);
    XFlush (x11.display);
    
    x11.redraw = 0;

}
#else
static int video_x11_initialize (void) {
    return 0;
}
static void video_x11_events (void) {}

static void video_x11_draw (const uint32_t *pixels, int width, int height) {

    (void) pixels;
    (void) width;
    (void) height;

}
#endif

static void video_wayland_global (void *data, struct wl_proxy *registry, uint32_t name, const char *interface, uint32_t version) {

    const struct wl_interface *type = NULL;
    struct wl_proxy **target = NULL;
    
    (void) data;
    
    if (!strcmp (interface, "wl_compositor")) {
    
        target = &wayland.compositor;
        type = wayland.compositor_interface;
    
    } else if (!strcmp (interface, "wl_shm")) {
    
        target = &wayland.shm;
        type = wayland.shm_interface;
    
    } else if (!strcmp (interface, "xdg_wm_base")) {
    
        target = &wayland.wm_base;
        type = &xdg_wm_base_interface;
    
    } else if (!strcmp (interface, "wl_seat")) {
    
        target = &wayland.seat;
        type = wayland.seat_interface;
    
    }
    
    if (!target || *target || !type) {
        return;
    }
    
    if (version > (uint32_t) type->version) {
        version = (uint32_t) type->version;
    }
    
    /* wl_registry.bind(name, interface, version, new_id) */
    *target = wayland.marshal (registry, 0, type, wayland.version (registry), 0,
        name, interface, version, NULL);

}

static void video_wayland_global_remove (void *data, struct wl_proxy *registry, uint32_t name) {

    (void) data;
    (void) registry;
    (void) name;

}

static void video_wayland_ping (void *data, struct wl_proxy *proxy, uint32_t serial) {

    (void) data;
    wayland.marshal (proxy, 3, NULL, wayland.version (proxy), 0, serial);

}

static void video_wayland_configure (void *data, struct wl_proxy *proxy, uint32_t serial) {

    (void) data;
    
    wayland.marshal (proxy, 4, NULL, wayland.version (proxy), 0, serial);
    wayland.configured = 1;

}

static void video_wayland_toplevel_configure (void *data, struct wl_proxy *proxy, int32_t width, int32_t height, void *states) {

    (void) data;
    (void) proxy;
    (void) width;
    (void) height;
    (void) states;

}

static void video_wayland_toplevel_close (void *data, struct wl_proxy *proxy) {

    (void) data;
    (void) proxy;
    
    closed = 1;

}

static void video_wayland_buffer_release (void *data, struct wl_proxy *proxy) {

    int *busy = data;
    
    (void) proxy;
    *busy = 0;

}

static void video_wayland_keyboard_keymap (void *data, struct wl_proxy *proxy, uint32_t format, int fd, uint32_t size) {

    (void) data;
    (void) proxy;
    (void) format;
    (void) size;
    
    close (fd);

}

static void video_wayland_keyboard_enter (void *data, struct wl_proxy *proxy, uint32_t serial, struct wl_proxy *surface, void *keys) {

    (void) data;
    (void) proxy;
    (void) serial;
    (void) surface;
    (void) keys;

}

static void video_wayland_keyboard_leave (void *data, struct wl_proxy *proxy, uint32_t serial, struct wl_proxy *surface) {

    (void) data;
    (void) proxy;
    (void) serial;
    (void) surface;

}

static void video_wayland_keyboard_key (void *data, struct wl_proxy *proxy, uint32_t serial, uint32_t time, uint32_t key, uint32_t state) {

    (void) data;
    (void) proxy;
    (void) serial;
    (void) time;
    
    video_key (video_evdev_scan (key), state != 0);

}

static void video_wayland_keyboard_modifiers (void *data, struct wl_proxy *proxy, uint32_t serial, uint32_t depressed, uint32_t latched, uint32_t locked, uint32_t group) {

    (void) data;
    (void) proxy;
    (void) serial;
    (void) depressed;
    (void) latched;
    (void) locked;
    (void) group;

}

static void video_wayland_keyboard_repeat (void *data, struct wl_proxy *proxy, int32_t rate, int32_t delay) {

    (void) data;
    (void) proxy;
    (void) rate;
    (void) delay;

}

static void video_wayland_seat_capabilities (void *data, struct wl_proxy *proxy, uint32_t capabilities) {

    static void (*keyboard_listener[])(void) = {
    
        (void (*)(void)) video_wayland_keyboard_keymap,
        (void (*)(void)) video_wayland_keyboard_enter,
        (void (*)(void)) video_wayland_keyboard_leave,
        (void (*)(void)) video_wayland_keyboard_key,
        (void (*)(void)) video_wayland_keyboard_modifiers,
        (void (*)(void)) video_wayland_keyboard_repeat
    
    };
    
    (void) data;
    
    if ((capabilities & 2) && !wayland.keyboard) {
    
        wayland.keyboard = wayland.marshal (proxy, 1, wayland.keyboard_interface, wayland.version (proxy), 0, NULL);
        
        if (wayland.keyboard) {
            wayland.add_listener (wayland.keyboard, keyboard_listener, NULL);
        }
    
    }
    
    if (!(capabilities & 2) && wayland.keyboard) {
    
        if (wayland.version (wayland.keyboard) >= 3) {
            wayland.marshal (wayland.keyboard, 0, NULL, wayland.version (wayland.keyboard), 1);
        } else {
            wayland.destroy (wayland.keyboard);
        }
        
        wayland.keyboard = NULL;
    
    }

}

static void video_wayland_seat_name (void *data, struct wl_proxy *proxy, const char *name) {

    (void) data;
    (void) proxy;
    (void) name;

}

static int video_wayland_initialize (void) {

    static void (*registry_listener[])(void) = {
    
        (void (*)(void)) video_wayland_global,
        (void (*)(void)) video_wayland_global_remove
    
    };
    
    static void (*wm_listener[])(void) = {
        (void (*)(void)) video_wayland_ping
    };
    
    static void (*surface_listener[])(void) = {
        (void (*)(void)) video_wayland_configure
    };
    
    static void (*toplevel_listener[])(void) = {
    
        (void (*)(void)) video_wayland_toplevel_configure,
        (void (*)(void)) video_wayland_toplevel_close
    
    };
    
    static void (*seat_listener[])(void) = {
    
        (void (*)(void)) video_wayland_seat_capabilities,
        (void (*)(void)) video_wayland_seat_name
    
    };
    
    struct wl_proxy *proxy;
    int i;
    
    if (!getenv ("WAYLAND_DISPLAY")) {
        return 0;
    }
    
    wayland.fd = -1;
    
    if (!(wayland.library = dlopen ("libwayland-client.so.0", RTLD_NOW | RTLD_LOCAL))) {
        return 0;
    }

#define     WAYLAND_LOAD(member, symbol) do { \
    *(void **) (&wayland.member) = dlsym (wayland.library, symbol); \
    if (!wayland.member) return 0; \
} while (0)
    WAYLAND_LOAD (connect, "wl_display_connect");
    WAYLAND_LOAD (disconnect, "wl_display_disconnect");
    WAYLAND_LOAD (roundtrip, "wl_display_roundtrip");
    WAYLAND_LOAD (dispatch, "wl_display_dispatch");
    WAYLAND_LOAD (dispatch_pending, "wl_display_dispatch_pending");
    WAYLAND_LOAD (flush, "wl_display_flush");
    WAYLAND_LOAD (get_fd, "wl_display_get_fd");
    WAYLAND_LOAD (add_listener, "wl_proxy_add_listener");
    WAYLAND_LOAD (destroy, "wl_proxy_destroy");
    WAYLAND_LOAD (version, "wl_proxy_get_version");
    WAYLAND_LOAD (marshal, "wl_proxy_marshal_flags");
    WAYLAND_LOAD (registry_interface, "wl_registry_interface");
    WAYLAND_LOAD (compositor_interface, "wl_compositor_interface");
    WAYLAND_LOAD (surface_interface, "wl_surface_interface");
    WAYLAND_LOAD (shm_interface, "wl_shm_interface");
    WAYLAND_LOAD (pool_interface, "wl_shm_pool_interface");
    WAYLAND_LOAD (buffer_interface, "wl_buffer_interface");
    WAYLAND_LOAD (seat_interface, "wl_seat_interface");
    WAYLAND_LOAD (keyboard_interface, "wl_keyboard_interface");
#undef      WAYLAND_LOAD

    wayland.display = wayland.connect (NULL);
    
    if (!wayland.display) {
        return 0;
    }
    
    wayland.registry = wayland.marshal ((struct wl_proxy *) wayland.display, 1,
        wayland.registry_interface, wayland.version ((struct wl_proxy *) wayland.display), 0, NULL);
    
    if (!wayland.registry) {
        return 0;
    }
    
    wayland.add_listener (wayland.registry, registry_listener, NULL);
    
    if (wayland.roundtrip (wayland.display) < 0 || !wayland.compositor || !wayland.shm || !wayland.wm_base) {
        return 0;
    }
    
    wayland.add_listener (wayland.wm_base, wm_listener, NULL);
    
    if (wayland.seat) {
        wayland.add_listener (wayland.seat, seat_listener, NULL);
    }
    
    wayland.surface = wayland.marshal (wayland.compositor, 0,
        wayland.surface_interface, wayland.version (wayland.compositor), 0, NULL);
    
    wayland.xdg_surface = wayland.marshal (wayland.wm_base, 2,
        &xdg_surface_interface, wayland.version (wayland.wm_base), 0, NULL, wayland.surface);
    
    if (!wayland.surface || !wayland.xdg_surface) {
        return 0;
    }
    
    wayland.toplevel = wayland.marshal (wayland.xdg_surface, 1,
        &xdg_toplevel_interface, wayland.version (wayland.xdg_surface), 0, NULL);
    
    if (!wayland.toplevel) {
        return 0;
    }
    
    wayland.add_listener (wayland.xdg_surface, surface_listener, NULL);
    wayland.add_listener (wayland.toplevel, toplevel_listener, NULL);
    
    wayland.marshal (wayland.toplevel, 2, NULL, 1, 0, "dosemu - VGA");
    wayland.marshal (wayland.toplevel, 3, NULL, 1, 0, "dosemu");
    wayland.marshal (wayland.toplevel, 7, NULL, 1, 0, VIDEO_WIDTH, VIDEO_HEIGHT);
    wayland.marshal (wayland.toplevel, 8, NULL, 1, 0, VIDEO_WIDTH, VIDEO_HEIGHT);
    wayland.marshal (wayland.surface, 6, NULL, wayland.version (wayland.surface), 0);
    
    if (wayland.roundtrip (wayland.display) < 0 || !wayland.configured) {
        return 0;
    }
    
    /* Two buffers ensure the compositor never sees a buffer being rewritten. */
    wayland.width = VIDEO_WIDTH;
    wayland.height = VIDEO_HEIGHT;
    wayland.buffer_size = (size_t) wayland.width * wayland.height * 4;
    
    {
    
        char path[] = "/dev/shm/dosemu-XXXXXX";
        wayland.fd = mkstemp (path);
        
        if (wayland.fd < 0) {
        
            char fallback[] = "/tmp/dosemu-XXXXXX";
            wayland.fd = mkstemp (fallback);
            
            if (wayland.fd < 0) {
                return 0;
            }
            
            unlink (fallback);
        
        } else {
            unlink (path);
        }
    
    }
    
    if (ftruncate (wayland.fd, (off_t) (wayland.buffer_size * 2)) < 0) {
        return 0;
    }
    
    wayland.pixels = mmap (NULL, wayland.buffer_size * 2, PROT_READ | PROT_WRITE, MAP_SHARED, wayland.fd, 0);
    
    if (wayland.pixels == MAP_FAILED) {
    
        wayland.pixels = NULL;
        return 0;
    
    }
    
    wayland.pool = wayland.marshal (wayland.shm, 0, wayland.pool_interface,
        wayland.version (wayland.shm), 0, NULL, wayland.fd, (int32_t) (wayland.buffer_size * 2));
    
    if (!wayland.pool) {
        return 0;
    }
    
    for (i = 0; i < 2; i++) {
    
        static void (*buffer_listener[])(void) = {
            (void (*)(void)) video_wayland_buffer_release
        };
        
        proxy = wayland.marshal (wayland.pool, 0, wayland.buffer_interface,
            wayland.version (wayland.pool), 0, NULL, (int32_t) (i * wayland.buffer_size),
                wayland.width, wayland.height, wayland.width * 4, 1);
        
        if (!proxy) {
            return 0;
        }
        
        wayland.buffers[i] = proxy;
        wayland.add_listener (proxy, buffer_listener, &wayland.busy[i]);
    
    }
    
    wayland.roundtrip (wayland.display);
    return 1;

}

static void video_wayland_events (void) {

    struct pollfd fd;
    
    if (!wayland.display) {
        return;
    }
    
    if (wayland.dispatch_pending (wayland.display) < 0) {
    
        closed = 1;
        return;
    
    }
    
    wayland.flush (wayland.display);
    
    fd.fd = wayland.get_fd (wayland.display);
    fd.events = POLLIN;
    fd.revents = 0;
    
    if (poll (&fd, 1, 0) > 0 && (fd.revents & POLLIN)) {
    
        if (wayland.dispatch (wayland.display) < 0) {
            closed = 1;
        }
    
    }

}

static int video_wayland_draw (const uint32_t *pixels) {

    int slot = wayland.busy[0] ? (wayland.busy[1] ? -1 : 1) : 0;
    
    if (slot >= 0) {
    
        memcpy (wayland.pixels + slot * wayland.buffer_size, pixels, wayland.buffer_size);
        wayland.busy[slot] = 1;
        
        wayland.marshal (wayland.surface, 1, NULL, wayland.version (wayland.surface), 0, wayland.buffers[slot], 0, 0);
        wayland.marshal (wayland.surface, 2, NULL, wayland.version (wayland.surface), 0, 0, 0, VIDEO_WIDTH, VIDEO_HEIGHT);
        wayland.marshal (wayland.surface, 6, NULL, wayland.version (wayland.surface), 0);
        wayland.flush (wayland.display);
        
        return 1;
    
    }
    
    return 0;

}

static void video_wayland_shutdown (void) {

    int i;
    
    if (wayland.display) {
    
        for (i = 0; i < 2; i++) {
        
            if (wayland.buffers[i]) {
                wayland.marshal (wayland.buffers[i], 0, NULL, 1, 1);
            }
        
        }
        
        if (wayland.pool) {
            wayland.marshal (wayland.pool, 1, NULL, 1, 1);
        }
        
        if (wayland.keyboard) {
        
            if (wayland.version (wayland.keyboard) >= 3) {
                wayland.marshal (wayland.keyboard, 0, NULL, wayland.version (wayland.keyboard), 1);
            } else {
                wayland.destroy (wayland.keyboard);
            }
        
        }
        
        if (wayland.seat) {
        
            if (wayland.version (wayland.seat) >= 5) {
                wayland.marshal (wayland.seat, 3, NULL, wayland.version (wayland.seat), 1);
            } else {
                wayland.destroy (wayland.seat);
            }
        
        }
        
        if (wayland.toplevel) {
            wayland.marshal (wayland.toplevel, 0, NULL, 1, 1);
        }
        
        if (wayland.xdg_surface) {
            wayland.marshal (wayland.xdg_surface, 0, NULL, 1, 1);
        }
        
        if (wayland.surface) {
            wayland.marshal (wayland.surface, 0, NULL, wayland.version (wayland.surface), 1);
        }
        
        if (wayland.wm_base) {
            wayland.marshal (wayland.wm_base, 0, NULL, 1, 1);
        }
        
        wayland.flush (wayland.display);
        wayland.disconnect (wayland.display);
    
    }
    
    if (wayland.pixels) {
        munmap (wayland.pixels, wayland.buffer_size * 2);
    }
    
    if (wayland.fd >= 0) {
        close (wayland.fd);
    }
    
    if (wayland.library) {
        dlclose (wayland.library);
    }
    
    memset (&wayland, 0, sizeof (wayland));
    wayland.fd = -1;

}

static uint32_t video_framebuffer[VIDEO_WIDTH * VIDEO_HEIGHT];

static int video_initialize (void) {

    const char *requested = getenv ("DOSEMU_VIDEO");
    
    if (requested && !strcmp (requested, "terminal")) {
        return 0;
    }
    
    if (!requested || !strcmp (requested, "wayland")) {
    
        if (video_wayland_initialize ()) {
            return 2;
        }
        
        if (wayland.library) {
            video_wayland_shutdown ();
        }
    
    }
    
    if ((!requested || !strcmp (requested, "x11")) && video_x11_initialize ()) {
        return 1;
    }
    
    return 0;

}

int video_native_present (CPU8086 *cpu) {

    int palette_changed;
    
    uint64_t now;
    unsigned x, y;
    
    if (closed) {
        return 1;
    }
    
    if (!attempted) {
    
        attempted = 1;
        backend = video_initialize ();
    
    }
    
    if (!backend) {
        return 0;
    }
    
    if (backend == 1) {
        video_x11_events ();
    } else {
        video_wayland_events ();
    }
    
    if (closed) {
        return 1;
    }
    
    palette_changed = memcmp (previous_palette, cpu->vga_dac_palette, sizeof (previous_palette)) || previous_mask != cpu->vga_dac_mask;
    
    if (!(mem_video_dirty & 4) && !palette_changed && (backend != 1 || !x11.redraw)) {
        return 1;
    }
    
    now = video_milliseconds ();
    
    if (last_refresh && now - last_refresh < 50) {
        return 1;
    }
    
    last_refresh = now;
    
    for (y = 0; y < 200; y++) {
    
        for (x = 0; x < 320; x++) {
        
            uint8_t index = mem_vga_pixel (cpu, x, y) & cpu->vga_dac_mask;
            const uint8_t *rgb = cpu->vga_dac_palette[index];
            
            uint32_t color = ((uint32_t) (rgb[0] * 255 + 31) / 63 << 16) | ((uint32_t) (rgb[1] * 255 + 31) / 63 << 8) | (uint32_t) (rgb[2] * 255 + 31) / 63;
            
            video_framebuffer[y * 2 * VIDEO_WIDTH + x * 2] = color;
            video_framebuffer[y * 2 * VIDEO_WIDTH + x * 2 + 1] = color;
            video_framebuffer[(y * 2 + 1) * VIDEO_WIDTH + x * 2] = color;
            video_framebuffer[(y * 2 + 1) * VIDEO_WIDTH + x * 2 + 1] = color;
        
        }
    
    }
    
    if (backend == 1) {
        video_x11_draw (video_framebuffer, x11.width, x11.height);
    } else if (!video_wayland_draw (video_framebuffer)) {
        return 1;
    }
    
    memcpy (previous_palette, cpu->vga_dac_palette, sizeof (previous_palette));
    
    previous_mask = cpu->vga_dac_mask;
    mem_video_dirty &= ~4u;
    
    return 1;

}

int video_native_active (void) {
    return backend != 0;
}

int video_native_poll (CPU8086 *cpu) {

    if (backend == 1) {
        video_x11_events ();
    }
    
    if (backend == 2) {
        video_wayland_events ();
    }
    
    if (closed) {
        return -1;
    }
    
    if (!cpu->kbd_scancode_pending && key_head != key_tail) {
    
        cpu->kbd_release_scancode = 0;
        cpu->kbd_scancode = key_queue[key_head];
        cpu->kbd_scancode_pending = 1;
        cpu->pic[0].irr |= 2;
        
        key_head = (key_head + 1) % sizeof (key_queue);
    
    }
    
    return 0;

}

void video_native_shutdown (void) {

#if     defined (HAVE_X11)
    if (backend == 1 && x11.display) {
    
        if (x11.image) {
            XDestroyImage (x11.image);
        }
        
        if (x11.gc) {
            XFreeGC (x11.display, x11.gc);
        }
        
        if (x11.window) {
            XDestroyWindow (x11.display, x11.window);
        }
        
        XCloseDisplay (x11.display);
    
    }
#endif
    if (backend == 2) {
        video_wayland_shutdown ();
    }
    
    memset (&x11, 0, sizeof (x11));
    
    backend = 0;
    attempted = 0;
    
    closed = 0;
    last_refresh = 0;
    
    key_head = key_tail = 0;
    memset (previous_palette, 0, sizeof (previous_palette));
    
    previous_mask = 0;

}

#endif
