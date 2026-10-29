/******************************************************************************
 * @file            opl2.c
 * @brief           Nine-channel OPL2 FM synthesis for native audio output.
 *
 * This is an approximate YM3812 synthesizer, not a cycle-accurate emulation.
 * The status timers are implemented separately by the I/O controller.
 *****************************************************************************/
#include    <math.h>
#include    <stdint.h>
#include    <string.h>

#include    "opl2.h"
#define     OPL_RATE                    48000.0

#define     OPL_PI                      3.14159265358979323846
#define     OPL_TAU                     (2.0 * OPL_PI)

#define     OPL_OFF                     0
#define     OPL_ATTACK                  1
#define     OPL_DECAY                   2
#define     OPL_SUSTAIN                 3
#define     OPL_RELEASE                 4

typedef struct OPL2Operator {

    double phase, envelope, previous[2];
    uint8_t stage, key;

} OPL2Operator;

static const uint8_t opl_offsets[9][2] = {

    { 0, 3 }, { 1, 4 }, { 2, 5 },
    { 8, 11 }, { 9, 12 }, { 10, 13 },
    { 16, 19 }, { 17, 20 }, { 18, 21 }

};

static const double opl_multiplier[16] = {

    0.5, 1, 2, 3, 4, 5, 6, 7,
    8, 9, 10, 10, 12, 12, 15, 15

};

static uint8_t opl_registers[256];
static OPL2Operator opl_operators[22];
static uint32_t opl_noise = 1;
static double opl_tremolo_phase, opl_vibrato_phase;

static double opl_rate_seconds (unsigned rate, unsigned block, unsigned ksr) {

    if (!rate) {
        return 1000000.0;
    }
    
    rate = rate * 4 + (ksr ? block : block / 4);
    
    if (rate >= 60) {
        return 1.0 / OPL_RATE;
    }
    
    return 3.0 * pow (2.0, -(double) rate / 4.0);

}

static void opl_key (unsigned op, int on) {

    OPL2Operator *state = &opl_operators[op];
    
    if (on && !state->key) {
    
        state->stage = OPL_ATTACK;
        state->envelope = 0;
        state->phase = 0;
        state->previous[0] = state->previous[1] = 0;
    
    } else if (!on && state->key && state->stage != OPL_OFF) {
        state->stage = OPL_RELEASE;
    }
    
    state->key = (uint8_t) !!on;

}

static void opl_update_keys (void) {

    unsigned channel;
    uint8_t rhythm = opl_registers[0xBD];
    
    for (channel = 0; channel < 9; channel++) {
    
        int on = (opl_registers[0xB0 + channel] & 0x20) != 0;
        
        if ((rhythm & 0x20) && channel >= 6) {
        
            if (channel == 6) {
                on = (rhythm & 0x10) != 0;
            } else {
            
                opl_key (opl_offsets[channel][0], (rhythm & (channel == 7 ? 0x01 : 0x04)) != 0);
                opl_key (opl_offsets[channel][1], (rhythm & (channel == 7 ? 0x08 : 0x02)) != 0);
                continue;
            }
        }
        
        opl_key (opl_offsets[channel][0], on);
        opl_key (opl_offsets[channel][1], on);
    
    }

}

void opl2_reset (void) {

    memset (opl_registers, 0, sizeof (opl_registers));
    memset (opl_operators, 0, sizeof (opl_operators));
    
    opl_noise = 1;
    opl_tremolo_phase = opl_vibrato_phase = 0;

}

void opl2_write (uint8_t reg, uint8_t value) {

    opl_registers[reg] = value;
    
    if ((reg >= 0xB0 && reg <= 0xB8) || reg == 0xBD) {
        opl_update_keys ();
    }

}

static double opl_wave (double phase, unsigned waveform) {

    double value;
    
    phase = fmod (phase, OPL_TAU);
    
    if (phase < 0) {
        phase += OPL_TAU;
    }
    
    value = sin (phase);
    
    switch (waveform) {
    
        case 1:
            return value > 0 ? value : 0;
        
        case 2:
            return fabs (value);
        
        case 3:
            return phase < OPL_PI / 2 ? value : 0;
        
        default:
            return value;
    
    }

}

static double opl_operator_sample (unsigned op, unsigned channel, double modulation, double tremolo, double vibrato) {

    OPL2Operator *state = &opl_operators[op];
    
    uint8_t flags = opl_registers[0x20 + op];
    uint8_t level = opl_registers[0x40 + op];
    uint8_t rates = opl_registers[0x60 + op];
    uint8_t sustain = opl_registers[0x80 + op];
    uint8_t pitch = opl_registers[0xB0 + channel];
    
    unsigned block = (pitch >> 2) & 7;
    unsigned fnum = opl_registers[0xA0 + channel] | ((pitch & 3) << 8);
    unsigned ksr = (flags & 0x10) != 0;
    unsigned rate;
    
    double seconds, target, volume, frequency, result;
    
    if (state->stage == OPL_OFF) {
        return 0;
    }
    
    switch (state->stage) {
    
        case OPL_ATTACK:
        
            rate = rates >> 4;
            seconds = opl_rate_seconds (rate, block, ksr);
            state->envelope += 1.0 / (seconds * OPL_RATE);
            
            if (state->envelope >= 1.0) {
                state->envelope = 1.0;
                state->stage = OPL_DECAY;
            }
            
            break;
        
        case OPL_DECAY:
        
            rate = rates & 15;
            target = (sustain >> 4) == 15 ? 0.00001 : pow (10.0, -3.0 * (sustain >> 4) / 20.0);
            seconds = opl_rate_seconds (rate, block, ksr);
            state->envelope -= (1.0 - target) / (seconds * OPL_RATE);
            
            if (state->envelope <= target) {
                state->envelope = target;
                state->stage = OPL_SUSTAIN;
            }
            
            break;
        
        case OPL_SUSTAIN:
        
            if (!(flags & 0x20)) {
                state->stage = OPL_RELEASE;
            }
            
            break;
        
        case OPL_RELEASE:
        
            rate = sustain & 15;
            seconds = opl_rate_seconds (rate, block, ksr);
            state->envelope -= 1.0 / (seconds * OPL_RATE);
            
            if (state->envelope <= 0) {
                state->envelope = 0;
                state->stage = OPL_OFF;
                return 0;
            }
            
            break;
    
    }
    
    frequency = fnum * 49716.0 / (double) (1 << (20 - block));
    frequency *= opl_multiplier[flags & 15];
    
    if (flags & 0x40) {
        frequency *= 1.0 + vibrato * ((opl_registers[0xBD] & 0x40) ? 0.008 : 0.004);
    }
    
    state->phase += OPL_TAU * frequency / OPL_RATE;
    
    if (state->phase >= OPL_TAU) {
        state->phase = fmod (state->phase, OPL_TAU);
    }
    
    volume = pow (10.0, -0.75 * (level & 63) / 20.0);
    
    if (level & 0xC0) {
    
        unsigned scale = level >> 6;
        unsigned octave = (fnum >> 8) + (block << 1);
        unsigned attenuation = octave < 15 ? 15 - octave : 0;
        
        volume *= pow (10.0, -0.75 * attenuation * scale / 20.0);
    
    }
    
    if (flags & 0x80) {
        volume *= 1.0 - tremolo * ((opl_registers[0xBD] & 0x80) ? 0.42 : 0.11);
    }
    
    result = opl_wave (state->phase + modulation, (opl_registers[1] & 0x20) ? (opl_registers[0xE0 + op] & 3) : 0);
    result *= state->envelope * volume;
    
    return result;

}

int opl2_sample (void) {

    double mixed = 0;
    double tremolo, vibrato;
    
    unsigned channel;
    uint8_t rhythm = opl_registers[0xBD];
    
    opl_tremolo_phase += OPL_TAU * 3.7 / OPL_RATE;
    opl_vibrato_phase += OPL_TAU * 6.1 / OPL_RATE;
    
    if (opl_tremolo_phase >= OPL_TAU) {
        opl_tremolo_phase -= OPL_TAU;
    }
    
    if (opl_vibrato_phase >= OPL_TAU) {
        opl_vibrato_phase -= OPL_TAU;
    }
    
    tremolo = 0.5 + 0.5 * sin (opl_tremolo_phase);
    vibrato = sin (opl_vibrato_phase);
    
    opl_noise ^= opl_noise << 13;
    opl_noise ^= opl_noise >> 17;
    opl_noise ^= opl_noise << 5;
    
    for (channel = 0; channel < 9; channel++) {
    
        unsigned mod = opl_offsets[channel][0];
        unsigned car = opl_offsets[channel][1];
        uint8_t connection = opl_registers[0xC0 + channel];
        double feedback = 0, operator_output, carrier_output;
        
        if ((rhythm & 0x20) && channel >= 7) {
        
            double noise = (opl_noise & 1) ? 1.0 : -1.0;
            double a = opl_operator_sample (mod, channel, 0, tremolo, vibrato);
            double b = opl_operator_sample (car, channel, 0, tremolo, vibrato);
            
            mixed += channel == 7 ? a * noise + b * noise : a + b * noise;
            continue;
        
        }
        
        if ((connection >> 1) & 7) {
        
            unsigned shift = 9 - ((connection >> 1) & 7);
            feedback = (opl_operators[mod].previous[0] + opl_operators[mod].previous[1]) * (double) (1 << (8 - shift));
        
        }
        
        operator_output = opl_operator_sample (mod, channel, feedback, tremolo, vibrato);
        
        opl_operators[mod].previous[1] = opl_operators[mod].previous[0];
        opl_operators[mod].previous[0] = operator_output;
        
        carrier_output = opl_operator_sample (car, channel, (connection & 1) ? 0 : operator_output * 4.0, tremolo, vibrato);
        mixed += carrier_output;
        
        if (connection & 1) {
            mixed += operator_output;
        }
    
    }
    
    return (int) (mixed * 5000.0);

}
