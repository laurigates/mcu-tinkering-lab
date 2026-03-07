/**
 * @file button_mapper.h
 * @brief Maps Xbox controller inputs to Nintendo Switch Pro Controller format.
 *
 * Handles the button layout differences between Xbox and Switch:
 *   Xbox A (bottom) → Switch B (bottom, but different label position)
 *   Xbox B (right)  → Switch A (right)
 *   Xbox X (left)   → Switch Y (left)
 *   Xbox Y (top)    → Switch X (top)
 *
 * Also maps analog sticks from Bluepad32's -511..512 range to the
 * Switch's 12-bit 0..4095 range.
 */

#pragma once

#include "bluepad32_host.h"
#include "switch_pro_usb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Convert Xbox gamepad state to Switch Pro Controller input.
 *
 * @param xbox  Input: raw Xbox controller state from Bluepad32.
 * @param sw    Output: mapped Switch Pro Controller input.
 */
void button_mapper_convert(const xbox_gamepad_state_t *xbox, switch_pro_input_t *sw);

#ifdef __cplusplus
}
#endif
