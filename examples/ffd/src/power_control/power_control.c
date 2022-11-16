// Copyright (c) 2022 XMOS LIMITED. This Software is subject to the terms of the
// XMOS Public License: Version 1

/* System headers */
#include <platform.h>
#include <xs1.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"

/* Library headers */
//#include "rtos_printf.h"
#include "rtos_macros.h"

/* App headers */
#include "platform/platform_conf.h"
#include "platform/driver_instances.h"
#include "leds.h"
#include "power_control.h"

// NOTE on temporary behavior
// This task waits on events from GPIO or TIMER. On an input event, the
// device will switch to low power mode and a timer started, this timer, will
// expire after a period of time and will trigger another event to exit low
// power mode.
// After testing is complete, this event should be triggered via the VNR event (as with the button).
// ALSO NOTE: it may be important to consider what happens if LP ENTER is requested and before this kick off, it is asked to exit.

// Available pins for Tile 1
// Pin X1D09 - Port 4A Bit 3 - Board TP12

#define LOW_POWER_WAIT_TIME_MS           5000
#define TASK_NOTIF_MASK_LP_ENTER         1  // Used by tile[1]
#define TASK_NOTIF_MASK_LP_EXIT          2  // Used by tile[1]
#define TASK_NOTIF_MASK_LP_IND_COMPLETE  4  // Used by tile[0]

static TaskHandle_t ctx_power_control_task = NULL;
static power_state_t requested_power_state = POWER_STATE_FULL;

// Enable/disable scaling of switch clock frequency when entering low power mode.
#define appconfLOW_POWER_ENABLE_SWITCH_CONTROL 1
#define appconfLOW_POWER_SWITCH_CLOCK_DIVIDER  30
#define appconfLOW_POWER_TILE0_CLOCK_DIVIDER   600

#if ON_TILE(1)

static unsigned tile0_div;
static unsigned switch_div;

#endif

#if ON_TILE(1)

#include "rtos_gpio.h" // TODO remove

RTOS_GPIO_ISR_CALLBACK_ATTR
static void tp12_callback(rtos_gpio_t *ctx, void *app_data,
                            rtos_gpio_port_id_t port_id, uint32_t value)
{
    TaskHandle_t task = app_data;
    BaseType_t xYieldRequired = pdFALSE;

    xTaskNotifyFromISR(task, TASK_NOTIF_MASK_LP_ENTER,
                       eSetValueWithOverwrite, &xYieldRequired);
    portYIELD_FROM_ISR(xYieldRequired);
}

static void init_gpio_port_event(void)
{
    const rtos_gpio_port_id_t gpio_port = rtos_gpio_port(PORT_LOW_PWR_CTRL);

    rtos_gpio_port_enable(gpio_ctx_t1, gpio_port);
    rtos_gpio_port_pull_up(gpio_ctx_t1, gpio_port);
    rtos_gpio_isr_callback_set(gpio_ctx_t1, gpio_port, tp12_callback,
                               xTaskGetCurrentTaskHandle());
    rtos_gpio_interrupt_enable(gpio_ctx_t1, gpio_port);
}

static void tmr_callback(TimerHandle_t pxTimer)
{
    xTaskNotify(ctx_power_control_task, TASK_NOTIF_MASK_LP_EXIT, eSetBits);
}

#endif /* ON_TILE(1) */


static void tile0_driver_control_lock(void)
{
#if ON_TILE(1)
    rtos_osal_mutex_get(&gpio_ctx_t0->lock, RTOS_OSAL_WAIT_FOREVER);
#else
    rtos_osal_mutex_get(&qspi_flash_ctx->mutex, RTOS_OSAL_WAIT_FOREVER);
    rtos_osal_mutex_get(&i2c_master_ctx->lock, RTOS_OSAL_WAIT_FOREVER);
    rtos_osal_mutex_get(&uart_tx_ctx->lock, RTOS_OSAL_WAIT_FOREVER);
#endif
}

static void tile0_driver_control_unlock(void)
{
#if ON_TILE(1)
    rtos_osal_mutex_put(&gpio_ctx_t0->lock);
#else
    rtos_osal_mutex_put(&uart_tx_ctx->lock);
    rtos_osal_mutex_put(&i2c_master_ctx->lock);
    rtos_osal_mutex_put(&qspi_flash_ctx->mutex);
#endif
}

#if ON_TILE(1)

static void tile0_low_power_clocks_enable(void)
{
    // Save tile 0 clock config before apply low power configuration.
    tile0_div = rtos_clock_control_get_processor_clk_div(cc_ctx_t0);
    rtos_clock_control_set_processor_clk_div(cc_ctx_t0, appconfLOW_POWER_TILE0_CLOCK_DIVIDER);

#if (appconfLOW_POWER_ENABLE_SWITCH_CONTROL)
    switch_div = rtos_clock_control_get_switch_clk_div(cc_ctx_t0);
    //rtos_printf("- Switch: %d MHz\n", get_local_switch_clock(), get_local_node_switch_clk_div());
    //debug_printf("- Switch Div0: %d.\n", switch_div);
    //debug_printf("- Switch Div0: %d.\n", get_node_switch_clk_div(TILE_ID(0)));
    //debug_printf("- Switch Div1: %d.\n", get_node_switch_clk_div(get_local_tile_id()));
    rtos_clock_control_set_switch_clk_div(cc_ctx_t0, appconfLOW_POWER_SWITCH_CLOCK_DIVIDER);
    //debug_printf("- Switch Div0: %d.\n", get_node_switch_clk_div(TILE_ID(0)));
    //debug_printf("- Switch Div1: %d.\n", get_node_switch_clk_div(get_local_tile_id()));
#endif
}

static void tile0_low_power_clocks_disable(void)
{
    // Restore the original clock divider state(s).
#if (appconfLOW_POWER_ENABLE_SWITCH_CONTROL)
    set_node_switch_clk_div(TILE_ID(0), switch_div);
#endif
    set_tile_processor_clk_div(TILE_ID(0), tile0_div);
}

#endif /* ON_TILE(1) */

static void power_control_task(void *arg)
{
    const uint32_t bits_to_clear_on_entry = 0x00000000UL;
    const uint32_t bits_to_clear_on_exit = 0xFFFFFFFFUL;
    uint32_t notif_value;

#if ON_TILE(1)
    TimerHandle_t lp_exit_tmr;
    lp_exit_tmr = xTimerCreate("lp_exit_tmr", pdMS_TO_TICKS(LOW_POWER_WAIT_TIME_MS),
                    pdFALSE, NULL, tmr_callback);
    init_gpio_port_event();
#endif

    while (1) {
#if ON_TILE(1)
        xTaskNotifyWait(bits_to_clear_on_entry,
                        bits_to_clear_on_exit,
                        &notif_value,
                        portMAX_DELAY);

        if (notif_value & TASK_NOTIF_MASK_LP_EXIT) {
            // Ignore the event if already exited low power mode.
            if (requested_power_state == POWER_STATE_FULL)
                continue;

            requested_power_state = POWER_STATE_FULL;
            debug_printf("Exiting low power...\n");
            tile0_low_power_clocks_disable();
            tile0_driver_control_unlock();
            debug_printf("Exited low power.\n");
        } else if (notif_value & TASK_NOTIF_MASK_LP_ENTER) {
            // Ignore the event if already enterred low power mode.
            if (requested_power_state == POWER_STATE_LOW)
                continue;

            requested_power_state = POWER_STATE_LOW;
        }

        // TODO: ignore POWER_STATE_LOW in certain cases???

        // Send the requested power state to the other tile.
        rtos_intertile_tx(intertile_ctx,
                          appconfPOWER_CONTROL_PORT,
                          &requested_power_state,
                          sizeof(requested_power_state));

        // Wait for a response form other tile (the value is not used/important).
        int ret = 0;
        rtos_intertile_rx_len(intertile_ctx, appconfPOWER_CONTROL_PORT, RTOS_OSAL_WAIT_FOREVER);
        rtos_intertile_rx_data(intertile_ctx, &ret, sizeof(ret));

        if (requested_power_state == POWER_STATE_LOW) {
            // TODO:
            // Check if audio playback is active. If so wait,
            // and then re-check if low power should still be enterred.
            // This check seems to involve audio_response_play() which accesses tile[1]'s
            // i2s_ctx. The issue with this is the playback is done in chunks. So knowing
            // how much audio playback is going to be sent is not know at this resource level.
            // However, if this resource is locked by the playback until completion, then we are good to go.
            debug_printf("Entering low power...\n");
            tile0_driver_control_lock();
            tile0_low_power_clocks_enable();
            debug_printf("Entered low power.\n");
            xTimerStart(lp_exit_tmr, 0);
        }
#else
        // Wait for other tile to send the requested power state.
        rtos_intertile_rx_len(intertile_ctx, appconfPOWER_CONTROL_PORT, RTOS_OSAL_WAIT_FOREVER);
        rtos_intertile_rx_data(intertile_ctx, &requested_power_state, sizeof(requested_power_state));

        if (requested_power_state == POWER_STATE_FULL) {
            tile0_driver_control_unlock();
            led_indicate_awake();

            /* Wait for a notification, signaling that the LED indication has
             * been applied. */
            xTaskNotifyWait(bits_to_clear_on_entry,
                            bits_to_clear_on_exit,
                            &notif_value,
                            portMAX_DELAY);
        } else {
            led_indicate_asleep();

            /* Wait for a notification, signaling that the LED indication has
             * been applied and that the tile is ready to be set to low power
             * mode by the other tile. */
            xTaskNotifyWait(bits_to_clear_on_entry,
                            bits_to_clear_on_exit,
                            &notif_value,
                            portMAX_DELAY);
            tile0_driver_control_lock();
        }

        rtos_intertile_tx(intertile_ctx,
                          appconfPOWER_CONTROL_PORT,
                          &requested_power_state,
                          sizeof(requested_power_state));
#endif
    }
}

power_state_t power_control_status(void)
{
    return requested_power_state;
}

void power_control_task_create(unsigned priority, void *args)
{
    xTaskCreate((TaskFunction_t)power_control_task,
                RTOS_STRINGIFY(power_control_task),
                RTOS_THREAD_STACK_SIZE(power_control_task), args,
                priority, &ctx_power_control_task);
}

#if ON_TILE(1)

void power_control_enter_low_power(void)
{
    xTaskNotify(ctx_power_control_task, TASK_NOTIF_MASK_LP_ENTER, eSetBits);
}

void power_control_exit_low_power(void)
{
    xTaskNotify(ctx_power_control_task, TASK_NOTIF_MASK_LP_EXIT, eSetBits);
}

#else

void power_control_req_complete(void)
{
    xTaskNotify(ctx_power_control_task, TASK_NOTIF_MASK_LP_IND_COMPLETE, eSetBits);
}

#endif
