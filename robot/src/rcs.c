/**
 * @file rcs.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Implementation of the Robot Control Service
 * @date 2025-01-02
 */

#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/gatt.h>

#include "capbot.h"
#include "rcs.h"

LOG_MODULE_REGISTER(rcs, LOG_LEVEL_DBG);

// -----------------------------------------------------------------------------
// Motor drive characteristic
// -----------------------------------------------------------------------------

#define T_DRIVE_TIMEOUT_PRIORITY 7
K_THREAD_STACK_DEFINE(t_drive_timeout_stack, 512);
static struct k_thread t_drive_timeout;
static struct
{
    int16_t mfl_rpm;
    int16_t mfr_rpm;
    int16_t mbl_rpm;
    int16_t mbr_rpm;
    k_timeout_t duration;
} t_drive_timeout_data;

void t_drive_timeout_ep(void *, void *, void *)
{
    LOG_DBG("Setting motors: {%d, %d, %d, %d}",
            t_drive_timeout_data.mfl_rpm,
            t_drive_timeout_data.mfr_rpm,
            t_drive_timeout_data.mbl_rpm,
            t_drive_timeout_data.mbr_rpm);
    cb_set_rpm(CB_M_FRONT_LEFT, t_drive_timeout_data.mfl_rpm);
    cb_set_rpm(CB_M_FRONT_RIGHT, t_drive_timeout_data.mfr_rpm);
    cb_set_rpm(CB_M_BACK_LEFT, t_drive_timeout_data.mbl_rpm);
    cb_set_rpm(CB_M_BACK_RIGHT, t_drive_timeout_data.mbr_rpm);

    LOG_DBG("Waiting for timeout");
    k_sleep(t_drive_timeout_data.duration);

    LOG_DBG("Timeout reached, stopping motors");
    cb_stop();
}

static ssize_t rcs_drive(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                         uint16_t len, uint16_t offset, uint8_t flags)
{
    static k_tid_t drive_tid = 0;
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    // Convert buffer to packet format
    if (len != sizeof(rcs_drive_t))
    {
        LOG_WRN("Incorrect data length for drive attribute");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    if (offset != 0)
    {
        LOG_WRN("Incorrect data offset for drive attribute");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    rcs_drive_t pkt = *((rcs_drive_t *)buf);

    // Setup thread data for drive with timeout
    if (drive_tid != 0)
    {
        LOG_DBG("Aborting previous drive thread");
        k_thread_abort(drive_tid);
    }
    LOG_DBG("Creating new drive with timeout thread");
    t_drive_timeout_data.mfl_rpm = pkt.fl;
    t_drive_timeout_data.mfr_rpm = pkt.fr;
    t_drive_timeout_data.mbl_rpm = pkt.bl;
    t_drive_timeout_data.mbr_rpm = pkt.br;
    t_drive_timeout_data.duration = K_MSEC(pkt.dur);

    // Create new drive with timeout thread
    drive_tid = k_thread_create(&t_drive_timeout, t_drive_timeout_stack,
                                K_THREAD_STACK_SIZEOF(t_drive_timeout_stack), t_drive_timeout_ep,
                                NULL, NULL, NULL, T_DRIVE_TIMEOUT_PRIORITY, 0, K_NO_WAIT);
    // Return pkt len
    return len;
}

// -----------------------------------------------------------------------------
// Motor speed characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_speed_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    rcs_speed_t pkt = {.fl = cb_get_rpm(CB_M_FRONT_LEFT),
                       .fr = cb_get_rpm(CB_M_FRONT_RIGHT),
                       .bl = cb_get_rpm(CB_M_BACK_LEFT),
                       .br = cb_get_rpm(CB_M_BACK_RIGHT)};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Motor angle characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_angle_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    rcs_angle_t pkt = {.fl = cb_get_angle(CB_M_FRONT_LEFT),
                       .fr = cb_get_angle(CB_M_FRONT_RIGHT),
                       .bl = cb_get_angle(CB_M_BACK_LEFT),
                       .br = cb_get_angle(CB_M_BACK_RIGHT)};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Voltage measurement characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_volt_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                             uint16_t len, uint16_t offset)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    rcs_volt_t pkt = {.volt = cb_measure_vcap()};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Robot Control Service (RCS)
// -----------------------------------------------------------------------------

BT_GATT_SERVICE_DEFINE(
    // Robot Control Service
    rcs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_RCS),
    // Motor drive characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_DRIVE, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, rcs_drive, NULL),
    // Motor speed characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_SPEED, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, rcs_speed_read,
                           NULL, NULL),
    // Motor angle characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_ANGLE, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, rcs_angle_read,
                           NULL, NULL),
    // Voltage measurement characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_VOLT, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, rcs_volt_read,
                           NULL, NULL), );
