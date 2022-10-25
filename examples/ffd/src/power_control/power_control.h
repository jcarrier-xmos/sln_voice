// Copyright 2022 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef POWER_CONTROL_H_
#define POWER_CONTROL_H_

// TODO this shares the same name/type as the feature/low_power_trigger
typedef enum power_state {
    POWER_STATE_LOW,
    POWER_STATE_FULL
} power_state_t;

/**
 * @brief Initialize the power control task.
 *
 * @param priority The priority of the task.
 * @param args The arguments to send to the task.
 */
void power_control_task_create(unsigned priority, void *args);

/**
 * @brief Returns the current (requested) power state.
 */
power_state_t power_control_status(void);

#if ON_TILE(1)

/**
 * @brief Notify that the power control task should enter the low power state.
 */
void power_control_enter_low_power(void);

/**
 * @brief Notify that the power control task should exit the low power state.
 */
void power_control_exit_low_power(void);

#else

/**
 * @brief Notify that the requested power control state on tile[0] has completed
 * This serves control when tile[1] is allowed to commence with applying
 * low power mode.
 */
void power_control_req_complete(void);

#endif

#endif /* POWER_CONTROL_H_ */