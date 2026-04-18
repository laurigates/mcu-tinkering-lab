/**
 * @file button_mapper.c
 * @brief Xbox-to-Switch input mapping implementation.
 *
 * Bluepad32 button constants (from uni_gamepad.h):
 *   BUTTON_A = (1 << 0)   - Xbox: bottom face button
 *   BUTTON_B = (1 << 1)   - Xbox: right face button
 *   BUTTON_X = (1 << 2)   - Xbox: left face button
 *   BUTTON_Y = (1 << 3)   - Xbox: top face button
 *   BUTTON_SHOULDER_L = (1 << 4)
 *   BUTTON_SHOULDER_R = (1 << 5)
 *   BUTTON_TRIGGER_L  = (1 << 6)
 *   BUTTON_TRIGGER_R  = (1 << 7)
 *   BUTTON_THUMB_L    = (1 << 8)
 *   BUTTON_THUMB_R    = (1 << 9)
 *
 * Bluepad32 dpad constants:
 *   DPAD_UP    = (1 << 0)
 *   DPAD_DOWN  = (1 << 1)
 *   DPAD_RIGHT = (1 << 2)
 *   DPAD_LEFT  = (1 << 3)
 *
 * Bluepad32 misc buttons:
 *   MISC_BUTTON_SYSTEM = (1 << 0)   - Xbox button / Guide
 *   MISC_BUTTON_BACK   = (1 << 1)   - View / Select
 *   MISC_BUTTON_HOME   = (1 << 2)   - (same as system on some)
 *   MISC_BUTTON_SELECT = (1 << 3)   - Share
 */

#include "button_mapper.h"

#include "uni_gamepad.h"

/**
 * @brief Remap an axis from Bluepad32 range to Switch 12-bit range.
 *
 * Bluepad32: -511 to 512 (signed, center at 0)
 * Switch:    0 to 4095 (unsigned, center at 2048)
 */
static uint16_t remap_axis(int16_t bp_val)
{
    /* Clamp to expected range */
    if (bp_val < -512)
        bp_val = -512;
    if (bp_val > 511)
        bp_val = 511;

    /* Map -512..511 → 0..4095 */
    int32_t mapped = ((int32_t)bp_val + 512) * 4095 / 1023;
    if (mapped < 0)
        mapped = 0;
    if (mapped > 4095)
        mapped = 4095;
    return (uint16_t)mapped;
}

/**
 * @brief Remap an axis with inverted Y for Switch (up = higher value).
 */
static uint16_t remap_axis_inverted(int16_t bp_val)
{
    return remap_axis(-bp_val);
}

void button_mapper_convert(const xbox_gamepad_state_t *xbox, switch_pro_input_t *sw)
{
    /* Clear output */
    sw->buttons_right = 0;
    sw->buttons_shared = 0;
    sw->buttons_left = 0;

    /*
     * Face button mapping (position-based, swap labels):
     *   Xbox A (bottom) → Switch B (bottom on Pro Con)
     *   Xbox B (right)  → Switch A (right on Pro Con)
     *   Xbox X (left)   → Switch Y (left on Pro Con)
     *   Xbox Y (top)    → Switch X (top on Pro Con)
     */
    if (xbox->buttons & BUTTON_A)
        sw->buttons_right |= SW_BTN_B;
    if (xbox->buttons & BUTTON_B)
        sw->buttons_right |= SW_BTN_A;
    if (xbox->buttons & BUTTON_X)
        sw->buttons_right |= SW_BTN_Y;
    if (xbox->buttons & BUTTON_Y)
        sw->buttons_right |= SW_BTN_X;

    /* Shoulder buttons: direct mapping */
    if (xbox->buttons & BUTTON_SHOULDER_R)
        sw->buttons_right |= SW_BTN_R;
    if (xbox->buttons & BUTTON_SHOULDER_L)
        sw->buttons_left |= SW_BTN_L;

    /* Triggers: Bluepad32 provides analog (0-1023) + digital in buttons */
    if (xbox->buttons & BUTTON_TRIGGER_R)
        sw->buttons_right |= SW_BTN_ZR;
    if (xbox->buttons & BUTTON_TRIGGER_L)
        sw->buttons_left |= SW_BTN_ZL;

    /* Also check analog trigger threshold for ZL/ZR */
    if (xbox->throttle > 128)
        sw->buttons_right |= SW_BTN_ZR;
    if (xbox->brake > 128)
        sw->buttons_left |= SW_BTN_ZL;

    /* Stick clicks */
    if (xbox->buttons & BUTTON_THUMB_L)
        sw->buttons_shared |= SW_BTN_LSTICK;
    if (xbox->buttons & BUTTON_THUMB_R)
        sw->buttons_shared |= SW_BTN_RSTICK;

    /* Menu buttons */
    if (xbox->misc_buttons & MISC_BUTTON_BACK)
        sw->buttons_shared |= SW_BTN_MINUS;
    if (xbox->misc_buttons & MISC_BUTTON_HOME)
        sw->buttons_shared |= SW_BTN_PLUS;
    if (xbox->misc_buttons & MISC_BUTTON_SYSTEM)
        sw->buttons_shared |= SW_BTN_HOME;
    if (xbox->misc_buttons & MISC_BUTTON_SELECT)
        sw->buttons_shared |= SW_BTN_CAPTURE;

    /* D-pad */
    if (xbox->dpad & DPAD_UP)
        sw->buttons_left |= SW_BTN_DPAD_UP;
    if (xbox->dpad & DPAD_DOWN)
        sw->buttons_left |= SW_BTN_DPAD_DOWN;
    if (xbox->dpad & DPAD_LEFT)
        sw->buttons_left |= SW_BTN_DPAD_LEFT;
    if (xbox->dpad & DPAD_RIGHT)
        sw->buttons_left |= SW_BTN_DPAD_RIGHT;

    /* Analog sticks: remap from Bluepad32 range to Switch 12-bit
     * Note: Y axes are inverted between Xbox and Switch conventions */
    sw->lx = remap_axis(xbox->axis_x);
    sw->ly = remap_axis_inverted(xbox->axis_y);
    sw->rx = remap_axis(xbox->axis_rx);
    sw->ry = remap_axis_inverted(xbox->axis_ry);
}
