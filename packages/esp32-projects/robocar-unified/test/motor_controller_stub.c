/**
 * @file motor_controller_stub.c
 * @brief Stub implementation of motor_controller.h for host-based tests.
 *
 * Records the last call's arguments so tests can assert on motor commands.
 */

#include "motor_controller.h"

#include <stdint.h>
#include <string.h>

/* =========================================================================
 * Last-call recording
 * ========================================================================= */

typedef struct {
    uint8_t left_speed;
    uint8_t right_speed;
    uint8_t left_direction;
    uint8_t right_direction;
} motor_state_t;

static motor_state_t g_last_motor_call = {0};

/**
 * @brief Get the last recorded motor state (for test assertions).
 */
void motor_stub_get_last_state(uint8_t *left_speed, uint8_t *right_speed, uint8_t *left_direction,
                               uint8_t *right_direction)
{
    if (left_speed)
        *left_speed = g_last_motor_call.left_speed;
    if (right_speed)
        *right_speed = g_last_motor_call.right_speed;
    if (left_direction)
        *left_direction = g_last_motor_call.left_direction;
    if (right_direction)
        *right_direction = g_last_motor_call.right_direction;
}

/**
 * @brief Reset recorded state.
 */
void motor_stub_reset(void)
{
    memset(&g_last_motor_call, 0, sizeof(g_last_motor_call));
}

/* =========================================================================
 * Stub API
 * ========================================================================= */

esp_err_t motor_controller_init(void)
{
    motor_stub_reset();
    return ESP_OK;
}

esp_err_t motor_move_forward(uint8_t speed)
{
    g_last_motor_call.left_speed = speed;
    g_last_motor_call.right_speed = speed;
    g_last_motor_call.left_direction = 1;
    g_last_motor_call.right_direction = 1;
    return ESP_OK;
}

esp_err_t motor_move_backward(uint8_t speed)
{
    g_last_motor_call.left_speed = speed;
    g_last_motor_call.right_speed = speed;
    g_last_motor_call.left_direction = 0;
    g_last_motor_call.right_direction = 0;
    return ESP_OK;
}

esp_err_t motor_turn_left(uint8_t speed)
{
    g_last_motor_call.left_speed = speed / 2;
    g_last_motor_call.right_speed = speed;
    g_last_motor_call.left_direction = 1;
    g_last_motor_call.right_direction = 1;
    return ESP_OK;
}

esp_err_t motor_turn_right(uint8_t speed)
{
    g_last_motor_call.left_speed = speed;
    g_last_motor_call.right_speed = speed / 2;
    g_last_motor_call.left_direction = 1;
    g_last_motor_call.right_direction = 1;
    return ESP_OK;
}

esp_err_t motor_rotate_cw(uint8_t speed)
{
    g_last_motor_call.left_speed = speed;
    g_last_motor_call.right_speed = speed;
    g_last_motor_call.left_direction = 1;  /* left forward */
    g_last_motor_call.right_direction = 0; /* right backward */
    return ESP_OK;
}

esp_err_t motor_rotate_ccw(uint8_t speed)
{
    g_last_motor_call.left_speed = speed;
    g_last_motor_call.right_speed = speed;
    g_last_motor_call.left_direction = 0;  /* left backward */
    g_last_motor_call.right_direction = 1; /* right forward */
    return ESP_OK;
}

esp_err_t motor_stop(void)
{
    g_last_motor_call.left_speed = 0;
    g_last_motor_call.right_speed = 0;
    g_last_motor_call.left_direction = 0;
    g_last_motor_call.right_direction = 0;
    return ESP_OK;
}

esp_err_t motor_set_individual(uint8_t left_speed, uint8_t right_speed, uint8_t left_direction,
                               uint8_t right_direction)
{
    g_last_motor_call.left_speed = left_speed;
    g_last_motor_call.right_speed = right_speed;
    g_last_motor_call.left_direction = left_direction;
    g_last_motor_call.right_direction = right_direction;
    return ESP_OK;
}

esp_err_t motor_get_state(uint8_t *left_speed, uint8_t *right_speed, uint8_t *left_direction,
                          uint8_t *right_direction)
{
    if (!left_speed || !right_speed || !left_direction || !right_direction) {
        return ESP_ERR_INVALID_ARG;
    }
    motor_stub_get_last_state(left_speed, right_speed, left_direction, right_direction);
    return ESP_OK;
}
