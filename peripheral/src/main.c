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

K_MUTEX_DEFINE(status_mutex);

enum status_e
{
    BLE_ADVERTISING,
    BLE_CONNECTED,
    BLE_ERROR,
} status_v;

void update_status(enum status_e s)
{
    k_mutex_lock(&status_mutex, K_FOREVER);
    // TODO: FSM?
    status_v = s;
    k_mutex_unlock(&status_mutex);
}

// -----------------------------------------------------------------------------
// Status led task
//
// This task makes the status leds blink in different patterns based on the
// current BLE status
// -----------------------------------------------------------------------------

/**
 * @brief Determine pattern mask for BLE status led
 *
 * @param s Status to get pattern for
 * @return uint16_t The pattern
 */
uint16_t ble_led_pattern(enum status_e s)
{
    switch (s)
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
    if(capbot_init_io()) {
        LOG_ERR("Could not initialize robot IO");
        return;
    }

    // DEBUG: Set initial let status
    capbot_led_set(CAPBOT_D15);
    capbot_led_set(CAPBOT_D16);

    for (;;)
    {
        LOG_DBG("ADV: 0x%x", ble_led_pattern(BLE_ADVERTISING));
        LOG_DBG("CON: 0x%x", ble_led_pattern(BLE_CONNECTED));
        LOG_DBG("ERR: 0x%x", ble_led_pattern(BLE_ERROR));
        k_sleep(K_MSEC(5000));
    }
}

#define T_STATUS_LED_STACKSIZE 256 * 4
#define T_STATUS_LED_PRIORITY 10
#define T_STATUS_LED_OPTIONS 0
#define T_STATUS_LED_DELAY 0
K_THREAD_DEFINE(status_led, T_STATUS_LED_STACKSIZE, t_status_led_ep, NULL, NULL, NULL, T_STATUS_LED_PRIORITY, T_STATUS_LED_OPTIONS, T_STATUS_LED_DELAY);
