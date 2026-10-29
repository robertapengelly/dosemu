/******************************************************************************
 * @file            speaker.c
 *****************************************************************************/
#include    <stdint.h>
#include    <stdio.h>

#if     defined (_WIN32)
# include       <windows.h>
# include       <mmsystem.h>
#elif   defined (__APPLE__)
# include       <AudioToolbox/AudioToolbox.h>
# include       <pthread.h>
#elif   defined (__linux__)
# include       <dlfcn.h>
# include       <pthread.h>
#endif

#include    "speaker.h"

#define     SPEAKER_SAMPLES     512
#define     SPEAKER_BUFFERS     4

#define     PIT_FREQUENCY       1193182.0

#define     SPEAKER_RATE        48000
#define     SPEAKER_VOLUME      4096

#define     SB_QUEUE_SAMPLES    65536

#if     defined (_WIN32) || defined (__APPLE__) || defined (__linux__)
typedef struct Speaker {

    double elapsed;
    
    uint32_t reload;
    uint8_t port61, mode;

} Speaker;

static Speaker speaker = { 0, 65536, 0, 0 };

static int16_t sb_samples[SB_QUEUE_SAMPLES];
static unsigned sb_head, sb_tail, sb_count, sb_phase;

static void speaker_render (int16_t *samples, int count) {
    
    double step = PIT_FREQUENCY / SPEAKER_RATE;
    uint32_t reload = speaker.reload;
    
    double elapsed = speaker.elapsed;
    int enabled = (speaker.port61 & 3) == 3;
    
    for (int i = 0; i < count; i++) {
    
        uint32_t phase = (uint32_t) elapsed % reload;
        int output = 1;
        
        switch (speaker.mode) {
        
            case 0:     case 1:
            
                output = elapsed >= reload;
                break;
            
            case 2:
            
                output = phase != reload - 1;
                break;
            
            case 3:
            
                output = phase < (reload + 1) / 2;
                break;
            
            case 4:     case 5:
            
                output = elapsed < reload || elapsed >= reload + 1;
                break;
        
        }
        
        {
        
            int mixed = enabled ? (output ? SPEAKER_VOLUME : -SPEAKER_VOLUME) : 0;
            
            if (sb_count) {
            
                mixed += sb_samples[sb_tail];
                
                sb_tail = (sb_tail + 1) % SB_QUEUE_SAMPLES;
                sb_count--;
            
            }
            
            if (mixed > 32767) {
                mixed = 32767;
            }
            
            if (mixed < -32768) {
                mixed = -32768;
            }
            
            samples[i] = (int16_t) mixed;
        
        }
        
        if (speaker.port61 & 1) {
        
            elapsed += step;
            
            if (speaker.mode == 2 || speaker.mode == 3) {
            
                if (elapsed >= reload) {
                    elapsed -= (uint64_t) (elapsed / reload) * reload;
                }
            
            } else if (elapsed > reload + 2) {
                elapsed = reload + 2;
            }
        
        }
    
    }
    
    speaker.elapsed = elapsed;

}

#endif

#if     defined (_WIN32)
static HWAVEOUT speaker_device;
static HANDLE speaker_event, speaker_thread;

static CRITICAL_SECTION speaker_lock;
static volatile LONG speaker_stopping;

static WAVEHDR speaker_headers[SPEAKER_BUFFERS];
static int16_t speaker_samples[SPEAKER_BUFFERS][SPEAKER_SAMPLES];

static DWORD WINAPI speaker_audio_thread (LPVOID unused) {

    (void) unused;
    
    while (!InterlockedCompareExchange (&speaker_stopping, 0, 0)) {
    
        DWORD result = WaitForSingleObject (speaker_event, INFINITE);
        
        if (result != WAIT_OBJECT_0 || InterlockedCompareExchange (&speaker_stopping, 0, 0)) {
            break;
        }
        
        for (int i = 0; i < SPEAKER_BUFFERS; i++) {
        
            if (speaker_headers[i].dwFlags & WHDR_DONE) {
            
                EnterCriticalSection (&speaker_lock);
                speaker_render (speaker_samples[i], SPEAKER_SAMPLES);
                
                LeaveCriticalSection (&speaker_lock);
                
                if (waveOutWrite (speaker_device, &speaker_headers[i], sizeof (WAVEHDR)) != MMSYSERR_NOERROR) {
                    return 1;
                }
            
            }
        
        }
    
    }
    
    return 0;

}

void speaker_initialize (void) {

    WAVEFORMATEX format = { 0 };
    MMRESULT result;
    
    InitializeCriticalSection (&speaker_lock);
    speaker_event = CreateEvent (NULL, FALSE, FALSE, NULL);
    
    if (!speaker_event) {
    
        DeleteCriticalSection (&speaker_lock);
        return;
    
    }
    
    format.wFormatTag = WAVE_FORMAT_PCM;
    format.nChannels = 1;
    format.nSamplesPerSec = SPEAKER_RATE;
    format.wBitsPerSample = 16;
    format.nBlockAlign = sizeof (int16_t);
    format.nAvgBytesPerSec = SPEAKER_RATE * sizeof (int16_t);
    
    result = waveOutOpen (&speaker_device, WAVE_MAPPER, &format, (DWORD_PTR) speaker_event, 0, CALLBACK_EVENT);
    
    if (result != MMSYSERR_NOERROR) {
    
        fprintf (stderr, "PC speaker audio unavailable (WinMM error %u)\n", (unsigned) result);
        
        CloseHandle (speaker_event);
        speaker_event = NULL;
        
        DeleteCriticalSection (&speaker_lock);
        return;
    
    }
    
    for (int i = 0; i < SPEAKER_BUFFERS; i++) {
    
        speaker_headers[i].lpData = (LPSTR) speaker_samples[i];
        speaker_headers[i].dwBufferLength = sizeof (speaker_samples[i]);
        
        if (waveOutPrepareHeader (speaker_device, &speaker_headers[i], sizeof (WAVEHDR)) != MMSYSERR_NOERROR) {
            goto fail;
        }
        
        speaker_render (speaker_samples[i], SPEAKER_SAMPLES);
        
        if (waveOutWrite (speaker_device, &speaker_headers[i], sizeof (WAVEHDR)) != MMSYSERR_NOERROR) {
            goto fail;
        }
    
    }
    
    InterlockedExchange (&speaker_stopping, 0);
    speaker_thread = CreateThread (NULL, 0, speaker_audio_thread, NULL, 0, NULL);
    
    if (speaker_thread) {
        return;
    }
    
fail:
    
    waveOutReset (speaker_device);
    
    for (int i = 0; i < SPEAKER_BUFFERS; i++) {
    
        if (speaker_headers[i].dwFlags & WHDR_PREPARED) {
            waveOutUnprepareHeader (speaker_device, &speaker_headers[i], sizeof (WAVEHDR));
        }
    
    }
    
    waveOutClose (speaker_device);
    
    speaker_device = NULL;
    CloseHandle (speaker_event);
    
    speaker_event = NULL;
    DeleteCriticalSection (&speaker_lock);

}

void speaker_shutdown (void) {

    if (!speaker_device) {
        return;
    }
    
    InterlockedExchange (&speaker_stopping, 1);
    SetEvent (speaker_event);
    
    WaitForSingleObject (speaker_thread, INFINITE);
    CloseHandle (speaker_thread);
    
    speaker_thread = NULL;
    waveOutReset (speaker_device);
    
    for (int i = 0; i < SPEAKER_BUFFERS; i++) {
        waveOutUnprepareHeader (speaker_device, &speaker_headers[i], sizeof (WAVEHDR));
    }
    
    waveOutClose (speaker_device);
    speaker_device = NULL;
    
    CloseHandle (speaker_event);
    speaker_event = NULL;
    
    DeleteCriticalSection (&speaker_lock);

}
#elif   defined (__APPLE__)
static AudioQueueRef speaker_queue;
static AudioQueueBufferRef speaker_buffers[SPEAKER_BUFFERS];

static pthread_mutex_t speaker_lock = PTHREAD_MUTEX_INITIALIZER;
static int speaker_stopping;

static void speaker_audio_callback (void *unused, AudioQueueRef queue, AudioQueueBufferRef buffer) {

    (void) unused;
    pthread_mutex_lock (&speaker_lock);
    
    if (!speaker_stopping) {
    
        speaker_render ((int16_t *) buffer->mAudioData, SPEAKER_SAMPLES);
        
        buffer->mAudioDataByteSize = SPEAKER_SAMPLES * sizeof (int16_t);
        AudioQueueEnqueueBuffer (queue, buffer, 0, NULL);
    
    }
    
    pthread_mutex_unlock (&speaker_lock);

}

void speaker_initialize (void) {

    AudioStreamBasicDescription format = { 0 };
    OSStatus result;
    
    format.mSampleRate = SPEAKER_RATE;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kLinearPCMFormatFlagIsSignedInteger | kLinearPCMFormatFlagIsPacked;
    format.mBytesPerPacket = sizeof (int16_t);
    format.mFramesPerPacket = 1;
    format.mBytesPerFrame = sizeof (int16_t);
    format.mChannelsPerFrame = 1;
    format.mBitsPerChannel = 16;
    
    result = AudioQueueNewOutput (&format, speaker_audio_callback, NULL, NULL, NULL, 0, &speaker_queue);
    
    if (result != noErr) {
    
        fprintf (stderr, "PC speaker audio unavailable (AudioQueue error %d)\n", (int) result);
        return;
    
    }
    
    speaker_stopping = 0;
    
    for (int i = 0; i < SPEAKER_BUFFERS; i++) {
    
        result = AudioQueueAllocateBuffer (speaker_queue, SPEAKER_SAMPLES * sizeof (int16_t), &speaker_buffers[i]);
        
        if (result != noErr) {
            goto fail;
        }
        
        speaker_audio_callback (NULL, speaker_queue, speaker_buffers[i]);
    
    }
    
    if (AudioQueueStart (speaker_queue, NULL) == noErr) {
        return;
    }
    
fail
    :
    speaker_stopping = 1;
    
    AudioQueueDispose (speaker_queue, 1);
    speaker_queue = NULL;

}

void speaker_shutdown (void) {

    if (speaker_queue) {
    
        pthread_mutex_lock (&speaker_lock);
        speaker_stopping = 1;
        
        pthread_mutex_unlock (&speaker_lock);
        
        AudioQueueDispose (speaker_queue, 1);
        speaker_queue = NULL;
    
    }

}
#elif   defined (__linux__)
/* Load the system audio libraries at runtime; no development packages needed. */
typedef struct PulseSpec {

    int format;
    
    uint32_t rate;
    uint8_t channels;

} PulseSpec;

typedef struct PulseBufferAttr {
    uint32_t maxlength, tlength, prebuf, minreq, fragsize;
} PulseBufferAttr;

typedef struct AlsaPcm AlsaPcm;

typedef struct AudioLinux {

    void *library, *device;
    
    void *(*pulse_open) (const char *, const char *, int, const char *, const char *, const PulseSpec *, const void *, const PulseBufferAttr *, int *);
    int (*pulse_write) (void *, const void *, size_t, int *);
    void (*pulse_close) (void *);
    
    int (*alsa_open) (AlsaPcm **, const char *, int, int);
    int (*alsa_set_params) (AlsaPcm *, int, int, unsigned, unsigned, int, unsigned);
    
    long (*alsa_write) (AlsaPcm *, const void *, unsigned long);
    
    int (*alsa_prepare) (AlsaPcm *);
    int (*alsa_close) (AlsaPcm *);
    int (*alsa_error_handler) (void (*)(const char *, int, const char *, int, const char *, ...));
    
    int pulse;

} AudioLinux;

static AudioLinux audio;

static pthread_t speaker_thread;
static pthread_mutex_t speaker_lock = PTHREAD_MUTEX_INITIALIZER;

static int speaker_stopping, speaker_running;

static int speaker_open_pulse (void) {

    PulseSpec format = { 3, SPEAKER_RATE, 1 };  /* PA_SAMPLE_S16LE */
    PulseBufferAttr buffer = { (uint32_t) -1, SPEAKER_SAMPLES * sizeof (int16_t) * SPEAKER_BUFFERS, (uint32_t) -1, (uint32_t) -1, (uint32_t) -1 };
    
    int error;
    audio.library = dlopen ("libpulse-simple.so.0", RTLD_NOW | RTLD_LOCAL);
    
    if (!audio.library) {
        return 0;
    }
    
    *(void **) &audio.pulse_open = dlsym (audio.library, "pa_simple_new");
    *(void **) &audio.pulse_write = dlsym (audio.library, "pa_simple_write");
    *(void **) &audio.pulse_close = dlsym (audio.library, "pa_simple_free");
    
    if (audio.pulse_open && audio.pulse_write && audio.pulse_close) {
    
        audio.device = audio.pulse_open (NULL, "dosemu", 1, NULL, "PC Speaker", &format, NULL, &buffer, &error);
        
        if (audio.device) {
        
            audio.pulse = 1;
            return 1;
        
        }
    
    }
    
    dlclose (audio.library);
    
    audio.library = NULL;
    return 0;

}

static void speaker_alsa_error (const char *file, int line, const char *function, int error, const char *format, ...) {

    (void) file;
    (void) line;
    (void) function;
    (void) error;
    (void) format;

}

static int speaker_open_alsa (void) {

    AlsaPcm *pcm;
    int opened = 0;
    
    audio.library = dlopen ("libasound.so.2", RTLD_NOW | RTLD_LOCAL);
    
    if (!audio.library) {
        return 0;
    }
    
    *(void **) &audio.alsa_open = dlsym (audio.library, "snd_pcm_open");
    *(void **) &audio.alsa_set_params = dlsym (audio.library, "snd_pcm_set_params");
    *(void **) &audio.alsa_write = dlsym (audio.library, "snd_pcm_writei");
    *(void **) &audio.alsa_prepare = dlsym (audio.library, "snd_pcm_prepare");
    *(void **) &audio.alsa_close = dlsym (audio.library, "snd_pcm_close");
    *(void **) &audio.alsa_error_handler = dlsym (audio.library, "snd_lib_error_set_handler");
    
    if (audio.alsa_open && audio.alsa_set_params && audio.alsa_write && audio.alsa_prepare && audio.alsa_close) {
    
        if (audio.alsa_error_handler) {
            audio.alsa_error_handler (speaker_alsa_error);
        }
        
        /* SND_PCM_STREAM_PLAYBACK = 0, S16_LE = 2, RW_INTERLEAVED = 3. */
        if (audio.alsa_open (&pcm, "default", 0, 0) == 0) {
        
            if (audio.alsa_set_params (pcm, 2, 3, 1, SPEAKER_RATE, 1, 40000) == 0) {
            
                audio.device = pcm;
                opened = 1;
            
            } else {
                audio.alsa_close (pcm);
            }
        
        }
        
        if (audio.alsa_error_handler) {
            audio.alsa_error_handler (NULL);
        }
        
        if (opened) {
            return 1;
        }
    
    }
    
    dlclose (audio.library);
    
    audio.library = NULL;
    return 0;

}

static int speaker_audio_write (const int16_t *samples, size_t count) {

    if (audio.pulse) {
    
        int error;
        return audio.pulse_write (audio.device, samples, count * sizeof (*samples), &error) == 0;
    
    }
    
    while (count) {
    
        long written = audio.alsa_write ((AlsaPcm *) audio.device, samples, (unsigned long) count);
        
        if (written < 0) {
        
            if (audio.alsa_prepare ((AlsaPcm *) audio.device) < 0) {
                return 0;
            }
            
            continue;
        
        }
        
        if (!written) {
            return 0;
        }
        
        samples += written;
        count -= (size_t) written;
    
    }
    
    return 1;

}

static void *speaker_audio_thread (void *unused) {

    int16_t samples[SPEAKER_SAMPLES];
    (void) unused;
    
    for (;;) {
    
        pthread_mutex_lock (&speaker_lock);
        
        if (speaker_stopping) {
        
            pthread_mutex_unlock (&speaker_lock);
            break;
        
        }
        
        speaker_render (samples, SPEAKER_SAMPLES);
        pthread_mutex_unlock (&speaker_lock);
        
        if (!speaker_audio_write (samples, SPEAKER_SAMPLES)) {
            break;
        }
    
    }
    
    return NULL;

}

void speaker_initialize (void) {

    if (!speaker_open_pulse () && !speaker_open_alsa ()) {
    
        fprintf (stderr, "PC speaker audio unavailable (no working PulseAudio or ALSA device)\n");
        return;
    
    }
    
    speaker_stopping = 0;
    
    if (pthread_create (&speaker_thread, NULL, speaker_audio_thread, NULL) == 0) {
    
        speaker_running = 1;
        return;
    
    }
    
    if (audio.pulse) {
        audio.pulse_close (audio.device);
    } else {
        audio.alsa_close ((AlsaPcm *) audio.device);
    }
    
    dlclose (audio.library);
    
    audio.library = NULL;
    audio.device = NULL;

}

void speaker_shutdown (void) {

    if (!speaker_running) {
        return;
    }
    
    pthread_mutex_lock (&speaker_lock);
    speaker_stopping = 1;
    
    pthread_mutex_unlock (&speaker_lock);
    
    pthread_join (speaker_thread, NULL);
    speaker_running = 0;
    
    if (audio.pulse) {
        audio.pulse_close (audio.device);
    } else {
        audio.alsa_close ((AlsaPcm *) audio.device);
    }
    
    dlclose (audio.library);
    
    audio.library = NULL;
    audio.device = NULL;

}
#else
void speaker_initialize (void) {}
void speaker_shutdown (void) {}
#endif

void speaker_update (uint8_t port61, uint8_t mode, uint16_t reload, int restart) {

#if     defined (_WIN32)
    if (!speaker_device) {
        return;
    }
    
    EnterCriticalSection (&speaker_lock);
#elif   defined (__APPLE__)
    if (!speaker_queue) {
        return;
    }
    
    pthread_mutex_lock (&speaker_lock);
#elif   defined (__linux__)
    if (!speaker_running) {
        return;
    }
    
    pthread_mutex_lock (&speaker_lock);
#endif

#if     defined (_WIN32) || defined (__APPLE__) || defined (__linux__)
    speaker.port61 = port61;
    speaker.mode = mode;
    speaker.reload = reload ? reload : 65536;
    
    if (restart) {
        speaker.elapsed = 0;
    }
#endif

#if     defined (_WIN32)
    LeaveCriticalSection (&speaker_lock);
#elif   defined (__APPLE__) || defined (__linux__)
    pthread_mutex_unlock (&speaker_lock);
#else
    (void) port61;
    (void) mode;
    (void) reload;
    (void) restart;
#endif

}

void speaker_sb_write (const uint8_t *samples, size_t count, unsigned rate) {

#if     defined (_WIN32)
    if (!speaker_device || !rate) {
        return;
    }
    
    EnterCriticalSection (&speaker_lock);
#elif   defined (__APPLE__)
    if (!speaker_queue || !rate) {
        return;
    }
    
    pthread_mutex_lock (&speaker_lock);
#elif   defined (__linux__)
    if (!speaker_running || !rate) {
        return;
    }
    
    pthread_mutex_lock (&speaker_lock);
#else
    (void) samples;
    (void) count;
    (void) rate;
    
    return;
#endif

#if     defined (_WIN32) || defined (__APPLE__) || defined (__linux__)
    for (size_t i = 0; i < count; i++) {
    
        int16_t sample = (int16_t) (((int) samples[i] - 128) * 128);
        sb_phase += SPEAKER_RATE;
        
        while (sb_phase >= rate) {
        
            sb_phase -= rate;
            
            if (sb_count < SB_QUEUE_SAMPLES) {
            
                sb_samples[sb_head] = sample;
                
                sb_head = (sb_head + 1) % SB_QUEUE_SAMPLES;
                sb_count++;
            
            }
        
        }
    
    }
#endif

#if     defined (_WIN32)
    LeaveCriticalSection (&speaker_lock);
#elif   defined (__APPLE__) || defined (__linux__)
    pthread_mutex_unlock (&speaker_lock);
#endif

}

void speaker_sb_reset (void) {

#if     defined (_WIN32)
    if (!speaker_device) {
        return;
    }
    
    EnterCriticalSection (&speaker_lock);
#elif   defined (__APPLE__)
    if (!speaker_queue) {
        return;
    }
    
    pthread_mutex_lock (&speaker_lock);
#elif   defined (__linux__)
    if (!speaker_running) {
        return;
    }
    
    pthread_mutex_lock (&speaker_lock);
#endif

#if     defined (_WIN32) || defined (__APPLE__) || defined (__linux__)
    sb_head = sb_tail = sb_count = sb_phase = 0;
#endif

#if     defined (_WIN32)
    LeaveCriticalSection (&speaker_lock);
#elif   defined (__APPLE__) || defined (__linux__)
    pthread_mutex_unlock (&speaker_lock);
#endif

}
