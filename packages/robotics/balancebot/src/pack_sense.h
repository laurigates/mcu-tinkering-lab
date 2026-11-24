// Pack-voltage sense on the carrier PCB (D0, GPIO26, ADC0; #747, ADR-038): the
// raw ADC window to volts, the "nothing connected" judgement, and the
// low-battery warning and cutoff with hysteresis. Pure C (no SDK dependencies)
// so the host tests compile it; pack.c reads the ADC on core 0 and hands
// core 1 the cutoff.
//
// The divider is R2 (top, PACK_R_TOP_KOHM) over R3 (bottom, a parameter):
// V_pack = V_pin * (R2 + R3) / R3. The delivered boards (8d18cd8d layout) have
// R3 = 27 kOhm, rev3 has 22 kOhm.
#ifndef BALANCEBOT_PACK_SENSE_H
#define BALANCEBOT_PACK_SENSE_H

#include <stdbool.h>
#include <stdint.h>

#define PACK_R_TOP_KOHM 100.0f
#define PACK_ADC_VREF_V 3.3f
#define PACK_ADC_COUNTS 4096.0f  // 12 bit

// Window the reading is judged on: PACK_SAMPLES raw reads taken back to back.
#define PACK_SAMPLES 8u
// A pin below this is not carrying a pack: the lowest pack worth reading
// (3S at a 2.5 V/cell BMS cutoff, 7.5 V) puts 1.35 V on the pin even through
// the rev3 divider.
#define PACK_ABSENT_PIN_V 0.5f
// A pin whose reads spread wider than this is floating: the 100 nF on the
// carrier holds a connected pin to a few counts.
#define PACK_NOISE_PIN_V 0.05f
// At or above this mean count the ADC is clipped and the voltage is only a
// lower bound (a 4S pack on a 27 kOhm board, or a wrong divider setting).
#define PACK_CLIP_COUNT 4090u

// Thresholds per cell. 3.0 V/cell is 9 V for 3S, above the DRV8825's 8.2 V
// minimum supply and above a BMS cutoff of 2.5 to 2.8 V/cell. The clear level
// sits above the cutoff so the unloaded rebound of a flat pack does not
// release the latch.
#define PACK_WARN_V_PER_CELL 3.5f
#define PACK_WARN_HYST_V_PER_CELL 0.05f
#define PACK_FAULT_V_PER_CELL 3.0f
#define PACK_FAULT_HYST_V_PER_CELL 0.3f
// The pack must stay below the cutoff this long (the motors sag it under
// load) before the monitor latches.
#define PACK_FAULT_MIN_S 1.0f

typedef enum {
    PACK_ABSENT = 0,  // D0 reads nothing: no carrier, or no pack on it
    PACK_OK = 1,
    PACK_WARN = 2,  // below the warning threshold
    PACK_LOW = 3,   // cutoff latched: motors must stay off
} pack_level_t;

typedef struct {
    bool present;  // a pack is on the pin (false: absent; `volts` is 0)
    bool clipped;  // the ADC saturated: `volts` is a lower bound
    float volts;   // pack voltage
} pack_reading_t;

// The divider setting a board has, as the bottom resistor R3 in kOhm: finite
// and in 1 to 1000. The flash record, the CLI and the monitor all use it.
bool pack_r_low_valid(float r_low_kohm);

// Cell count the thresholds scale with: a whole number from 2 to 6.
bool pack_cells_valid(float cells);

// V_pack / V_pin for R3 = r_low_kohm (R2 = PACK_R_TOP_KOHM).
float pack_scale(float r_low_kohm);

// Judge one window of `n` raw 12-bit reads. n == 0 reads as absent.
pack_reading_t pack_reading_from_counts(const uint16_t *raw, unsigned n, float r_low_kohm);

// The reading in whole 10 mV steps for the log row; 0 means absent (a pin
// below PACK_ABSENT_PIN_V is absent by definition, so 0 is never a reading).
uint16_t pack_reading_centivolts(const pack_reading_t *r);

float pack_warn_volts(float cells);
float pack_fault_volts(float cells);

typedef struct {
    pack_level_t level;
    bool fault;     // the cutoff latch; core 1 keeps the motors off while set
    bool warned;    // below the warning threshold, with its hysteresis
    float below_s;  // time spent below the cutoff, towards PACK_FAULT_MIN_S
} pack_monitor_t;

void pack_monitor_init(pack_monitor_t *m);

// Feed one reading `dt_s` after the last. An absent reading clears the
// monitor: nothing is known, so nothing is stopped.
void pack_monitor_update(pack_monitor_t *m, const pack_reading_t *r, float cells, float dt_s);

#endif  // BALANCEBOT_PACK_SENSE_H
