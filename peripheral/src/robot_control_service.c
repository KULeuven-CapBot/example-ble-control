/**
 * @file robot_control_service.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @date 2025-01-02
 */

#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/gatt.h>

#include "capbot.h"
#include "robot_control_service.h"

LOG_MODULE_REGISTER(robot_control_service, LOG_LEVEL_DBG);

// -----------------------------------------------------------------------------
// Motor drive characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_drive(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

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
    cb_motor_speed_t speeds = {.front_left = pkt.fl, .front_right = pkt.fr, .back_left = pkt.bl, .back_right = pkt.br};
    uint32_t duration = pkt.dur;

    LOG_DBG("Setting motors: {%d %d %d %d} for %lu milliseconds", speeds.front_left, speeds.front_right, speeds.back_left, speeds.back_right, duration);
    cb_set_motor_speed(&speeds);
    k_sleep(K_MSEC(duration)); // FIXME: Can other characteristics be read during this delay?
    LOG_DBG("Setting motors: {0 0 0 0}");
    cb_stop();

    return len;
}

// -----------------------------------------------------------------------------
// Motor speed characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_speed_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    cb_motor_speed_t speeds;
    cb_get_motor_speed(&speeds);

    rcs_speed_t pkt = {.fl = speeds.front_left, .fr = speeds.front_right, .bl = speeds.back_left, .br = speeds.back_right};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Motor angle characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_angle_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    cb_motor_angle_t angles;
    cb_get_motor_angle(&angles);

    rcs_angle_t pkt = {.fl = angles.front_left, .fr = angles.front_right, .bl = angles.back_left, .br = angles.back_right};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Voltage measurement characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_volt_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    int vcap = cb_measure_vcap();
    LOG_DBG("measured voltage: %u", vcap);

    rcs_volt_t pkt = {.volt = vcap};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Robot Control Service (RCS)
// -----------------------------------------------------------------------------

BT_GATT_SERVICE_DEFINE(
    // Robot Control Service
    rcs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_RCS),
    // Motor drive characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_DRIVE,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, rcs_drive, NULL),
    // Motor speed characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_SPEED,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           rcs_speed_read, NULL, NULL),
    // Motor angle characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_ANGLE,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           rcs_angle_read, NULL, NULL),
    // Voltage measurement characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_VOLT,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           rcs_volt_read, NULL, NULL),
    // TODO: Notify when V drops below setvalue
);
