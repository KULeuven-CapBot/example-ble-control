/**
 * @file main.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Robot control via BLE GATT service
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "capbot.h"

LOG_MODULE_REGISTER(capbot, LOG_LEVEL_DBG);

// -----------------------------------------------------------------------------
// Structure & methods for inter-thread comm/sync
// -----------------------------------------------------------------------------

K_MUTEX_DEFINE(ble_status_mutex);

typedef enum ble_status_e
{
    BLE_ADVERTISING,
    BLE_CONNECTED,
    BLE_ERROR,
} ble_status_t;

/** @brief Global static to hold BLE status. Never access directly but use `ble_status` and `update_ble_status` instead */
static ble_status_t ble_status_g;

ble_status_t ble_status(void)
{
    ble_status_t clone;
    k_mutex_lock(&ble_status_mutex, K_FOREVER);
    clone = ble_status_g;
    k_mutex_unlock(&ble_status_mutex);
    return clone;
}

void update_ble_status(ble_status_t status)
{
    k_mutex_lock(&ble_status_mutex, K_FOREVER);
    // TODO: FSM?
    ble_status_g = status;
    k_mutex_unlock(&ble_status_mutex);
}

// -----------------------------------------------------------------------------
// BLE task
//
// Initialize robot's BLE GATT service
// -----------------------------------------------------------------------------

void t_ble_init_ep(void *, void *, void *)
{
    LOG_DBG("Initializing BLE...");
    for (;;)
    {
        LOG_WRN("Fake BLE status updates");
        update_ble_status(BLE_ADVERTISING);
        k_sleep(K_MSEC(5000));
        update_ble_status(BLE_CONNECTED);
        k_sleep(K_MSEC(5000));
        update_ble_status(BLE_ERROR);
        k_sleep(K_MSEC(5000));
    }
}

#define T_BLE_INIT_STACKSIZE 1024
#define T_BLE_INIT_PRIORITY 1
#define T_BLE_INIT_OPTIONS 0
#define T_BLE_INIT_DELAY 0
K_THREAD_DEFINE(ble_init, T_BLE_INIT_STACKSIZE, t_ble_init_ep, NULL, NULL, NULL, T_BLE_INIT_PRIORITY, T_BLE_INIT_OPTIONS, T_BLE_INIT_DELAY);

// -----------------------------------------------------------------------------
// Status led task
//
// This task makes the status leds blink in different patterns based on the
// current BLE status
// -----------------------------------------------------------------------------

/**
 * @brief Determine pattern mask for BLE status led
 *
 * @param status Status to get pattern for
 * @return uint16_t The pattern
 */
uint16_t ble_led_pattern(ble_status_t status)
{
    switch (status)
    {
    case BLE_ADVERTISING:
        return 0b1111111100000000;
    case BLE_CONNECTED:
        return 0b1111111111111111;
    case BLE_ERROR:
        return 0b1010101010101010;
    default:
        LOG_WRN("Could not determine BLE status led pattern, using ERROR pattern instead");
        return ble_led_pattern(BLE_ERROR);
    }
}

/** @brief Entry point of status led task */
void t_status_led_ep(void *, void *, void *)
{
    const cb_led_t status_led = CB_D15;
    uint16_t led_pattern = 0;
    uint8_t pattern_index = 0;

    if (cb_io_init())
    {
        LOG_ERR("Could not initialize robot IO");
        return;
    }

    cb_measure_init();
    LOG_DBG("VCap = %d mV", cb_measure_vcap());

    for (;;)
    {
        // Get pattern based on current BLE status
        led_pattern = ble_led_pattern(ble_status());
        // Update led based on patters and index
        int err = ((led_pattern >> pattern_index) & 0b1) ? cb_led_set(status_led) : cb_led_clr(status_led);
        if (err)
            LOG_WRN("Error during BLE status led update");
        // Update index
        pattern_index = (pattern_index + 1) % (sizeof(led_pattern) * 8);
        // Sleep before next update => circa one pattern cycle per second
        k_sleep(K_MSEC(62));
    }
}

#define T_STATUS_LED_STACKSIZE 256 * 4
#define T_STATUS_LED_PRIORITY 10
#define T_STATUS_LED_OPTIONS 0
#define T_STATUS_LED_DELAY 0
K_THREAD_DEFINE(status_led, T_STATUS_LED_STACKSIZE, t_status_led_ep, NULL, NULL, NULL, T_STATUS_LED_PRIORITY, T_STATUS_LED_OPTIONS, T_STATUS_LED_DELAY);
