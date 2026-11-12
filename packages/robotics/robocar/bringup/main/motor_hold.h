/**
 * @file motor_hold.h
 * @brief Post-sweep motor hold: the BOOT button steps through steady drive states.
 *
 * The sweep's `motors` check pulses each motor for 220 ms at ~27%, which a
 * multimeter sampling three times a second cannot see and which may not break
 * a gearbox's static friction. Hold mode drives one motor at a time at 100% and
 * KEEPS it there, so every node of the chain — STBY, IN1/IN2, PWM, the driver's
 * outputs — can be read with a DMM at leisure.
 *
 * Driven from the BOOT button rather than the console, for the reason main.c's
 * header gives: both monitors are read-only and reset the board on attach.
 * Each press advances:
 *
 *   off -> left fwd -> left rev -> right fwd -> right rev -> off
 *
 * and prints the voltage every node of the active motor's path should read.
 * A held state returns to off by itself after MOTOR_HOLD_TIMEOUT_MS, so a robot
 * left driving on a bench stops without anyone reaching it.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Configure the BOOT button and announce the mode. Call once, after the sweep. */
void motor_hold_init(void);

/** Poll the button and the timeout. Call every few tens of milliseconds. */
void motor_hold_poll(void);

#ifdef __cplusplus
}
#endif
