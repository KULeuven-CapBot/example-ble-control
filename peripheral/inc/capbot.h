/**
 * @file capbot.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Header file for CapBot library
 */

#ifndef CAPBOT_H
#define CAPBOT_H

// -----------------------------------------------------------------------------
// On-board IO
// -----------------------------------------------------------------------------

typedef enum
{
    CB_D15,
    CB_D16,
} cb_led_t;

/** @brief Initialize robot's leds & button */
int cb_io_init(void);

/**
 * @brief Turn led on
 *
 * @param led The led to set
 */
int cb_led_set(cb_led_t led);

/**
 * @brief Turn led off
 *
 * @param led The led to clear
 */
int cb_led_clr(cb_led_t led);

/**
 * @brief Toggle a led
 *
 * @param led The led to toggle
 */
int cb_led_tgl(cb_led_t led);

/**
 * @brief Get a led's status
 *
 * @param led The led to get
 * @retval 0: the led is turned off
 * @retval 1: the led is turned on
 */
int cb_led_get(cb_led_t led);

/**
 * @brief Get button status
 *
 * @retval 0: the button isn't pressed
 * @retval 1: the button is pressed
 */
int cb_btn_get(void);

// -----------------------------------------------------------------------------
// On-board voltage measurements
// -----------------------------------------------------------------------------
typedef enum
{
    CB_VNONE,
    CB_VIN,
    CB_VCAP,
    CB_VMOTOR,
} cb_measure_t;

/**
 * @brief Initialize robot's voltage measurement system
 *
 * @return int
 */
int cb_measure_init(void);

/**
 * @brief Measure supercapacitor voltage
 *
 * @retval >0 : The measured voltage in mV
 * @retval `E_ADC_ERR` : Could not read from ADC
 */
int cb_measure_vcap(void);

// -----------------------------------------------------------------------------
// Robot drive API
// -----------------------------------------------------------------------------

/** @brief Motor speeds (rpm) */
typedef struct
{
    /** @brief Front left motor speed (rpm)*/
    int front_left;
    /** @brief Front right motor speed (rpm)*/
    int front_right;
    /** @brief Back left motor speed (rpm)*/
    int back_left;
    /** @brief Back right motor speed (rpm)*/
    int back_right;
} cb_motor_speed_t;

/**
 * @brief Initialize motors
 * @details v1.0 equivalent: Motors_init(void)
 * @details v2.* equivalent: fb_motor_init(void)
 */
int cb_motor_init(void);


/**
 * @brief Set motor speeds
 * @param speeds struct with the new speeds
 */
void cb_set_motor_speed(cb_motor_speed_t *speeds);

/**
 * @brief Get motor speeds
 * @param speeds struct to populate with the new speeds
 */
void cb_get_motor_speed(cb_motor_speed_t *speeds);

/* Convenience API overview:
 *                          ^
 *                          |
 *                       cb_forw
 *                          |
 *
 *                 +----------------+
 *             +---+                +---+
 *             |   |                |   |
 *             |   |                |   |
 *             |   |                |   |
 *             |   |                |   |
 *             +---+                +---+
 *                 |                |
 *       cb_left   |                |   cb_right
 *  <-----------   |    cb_stop     |   ------------>
 *                 |                |
 *                 |                |
 *             +---+                +---+
 *             |   |                |   |
 *             |   |                |   |
 *             |   |                |   |
 *             |   |                |   |
 *             +---+                +---+
 *                 +----------------+
 *
 *                          |
 *                       cb_back
 *                          |
 *                          v
 */

/** @brief Stop the robot */
void cb_stop(void);

/**
 * @brief Move robot forwards
 * @details v1.0 equivalent: FreeBotStraight_Forward(bool MotorSpeed)
 * @details v2.* equivalent: fb_straight_forw(unsigned int speed)
 *
 * @param speed Desired speed (rpm)
 */
// void cb_forw(unsigned int speed);

/**
 * @brief Move robot backwards
 * @details v1.0 equivalent: FreeBotStraight_Backward(bool MotorSpeed)
 * @details v2.* equivalent: fb_straight_back(unsigned int speed)
 *
 * @param speed Desired speed (rpm)
 */
// void cb_back(unsigned int speed);

/**
 * @brief Move robot right
 * @details v1.0 equivalent: FreeBotSide_Right(bool MotorSpeed)
 * @details v2.* equivalent: fb_side_right(unsigned int speed)
 *
 * @param speed Desired speed (rpm)
 */
// void cb_right(unsigned int speed);

/**
 * @brief Move robot  left
 * @details v1.0 equivalent: FreeBotSide_Left(bool MotorSpeed)
 * @details v2.* equivalent: fb_side_left(unsigned int speed)
 *
 * @param speed Desired speed (rpm)
 */
// void cb_left(unsigned int speed);

/**
 * @brief Rotate clockwise
 * @details v1.0 equivalent: FreeBotRotate_CLOCKWISE(bool MotorSpeed)
 * @details v2.* equivalent: fb_rotate_cw(unsigned int speed)
 *
 * @param speed Desired wheel speed (rpm)
 */
// void cb_rotate_cw(unsigned int speed);

/**
 * @brief Rotate counterclockwise
 * @details v1.0 equivalent: FreeBotRotate_COUNTERCLOCKWISE(bool MotorSpeed)
 * @details v2.* equivalent: fb_rotate_ccw(unsigned int speed)
 *
 * @param speed Desired wheel speed (rpm)
 */
// void cb_rotate_ccw(unsigned int speed);

#endif /* CAPBOT_H */
