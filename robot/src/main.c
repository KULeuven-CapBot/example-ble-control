/**
 * @file main.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Robot control via BLE GATT service
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include "capbot.h"
#include "robot_control_service.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// -----------------------------------------------------------------------------
// Structure & methods for inter-thread communication/sync
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
    ble_status_g = status;
    k_mutex_unlock(&ble_status_mutex);
}

// -----------------------------------------------------------------------------
// BLE related constants
// -----------------------------------------------------------------------------

#define BLE_DEVICE_NAME "CapBot"
#define BLE_DEVICE_NAME_LEN (sizeof(BLE_DEVICE_NAME) - 1)

/** @brief BLE Advertisement data */
static const struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN),
};

/** @brief BLE Scan response data */
static const struct bt_data rsp_data[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_SOME, BT_UUID_RCS_VAL),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN),
};

/** @brief On BLE connected callback */
static void on_connected(struct bt_conn *conn, uint8_t err);

/** @brief On BLE disconnected callback */
static void on_disconnected(struct bt_conn *conn, uint8_t reason);

/** @brief BLE connection callbacks */
struct bt_conn_cb connection_cb = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

// -----------------------------------------------------------------------------
// System initialization
// -----------------------------------------------------------------------------

int sys_init(void)
{
    LOG_DBG("Initializing on-board IO ...");
    if (cb_io_init())
    {
        LOG_ERR("Could not initialize on-board IO");
        return -1;
    }
    LOG_INF("On-board IO initialization done");

    LOG_DBG("Initializing ADC ...");
    if (cb_measure_init())
    {
        LOG_ERR("Could not initialize ADC");
        return -1;
    }
    LOG_INF("ADC initialization done");

    LOG_DBG("Initializing motors ...");
    if (cb_motor_init())
    {
        LOG_ERR("Could not initialize motors");
        return -1;
    }
    LOG_INF("Motor initialization done");

    LOG_DBG("Initializing bluetooth ...");
    if (bt_enable(NULL))
    {
        LOG_ERR("Bluetooth initialization failed");
        update_ble_status(BLE_ERROR);
        return -1;
    }
    bt_conn_cb_register(&connection_cb);
    LOG_INF("Bluetooth initialization done");

    if (bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data), rsp_data, ARRAY_SIZE(rsp_data)))
    {
        LOG_ERR("BLE advertising failed to start");
        update_ble_status(BLE_ERROR);
        return -1;
    }
    update_ble_status(BLE_ADVERTISING);
    LOG_INF("BLE advertising started");

    return 0;
}

SYS_INIT(sys_init, APPLICATION, 5);

// -----------------------------------------------------------------------------
// BLE connection callbacks
// -----------------------------------------------------------------------------

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        LOG_ERR("Failed to connect to %s (%u)", addr, err);
        return;
    }
    LOG_INF("Connected: %s", addr);

    update_ble_status(BLE_CONNECTED);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

    if (bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data), rsp_data, ARRAY_SIZE(rsp_data)))
    {
        LOG_ERR("BLE advertising failed to start");
        update_ble_status(BLE_ERROR);
        return;
    }
    update_ble_status(BLE_ADVERTISING);
    LOG_INF("BLE advertising started");
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
        return 0b1010101010101010;
    case BLE_ERROR:
        return 0b1111111111111111;
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

#define T_STATUS_LED_STACKSIZE 256
#define T_STATUS_LED_PRIORITY 10
#define T_STATUS_LED_OPTIONS 0
#define T_STATUS_LED_DELAY 0
K_THREAD_DEFINE(status_led, T_STATUS_LED_STACKSIZE, t_status_led_ep, NULL, NULL, NULL, T_STATUS_LED_PRIORITY, T_STATUS_LED_OPTIONS, T_STATUS_LED_DELAY);
