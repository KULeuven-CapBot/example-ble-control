/**
 * @file capbot_motor.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief CapBot motor control
 * @date 2024-12-23
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "capbot.h"

#define PWM_PERIOD 20000000           /** ns */
#define PWM_MIN_DUTY_CYCLE 0          /** ns */
#define PWM_MAX_DUTY_CYCLE PWM_PERIOD /** ns */

/** @brief Holds a motor's GPIO handles */
struct motor_gpio
{
    const struct pwm_dt_spec in_a;
    const struct pwm_dt_spec in_b;
    const struct gpio_dt_spec hall_c1;
    const struct gpio_dt_spec hall_c2;
};

/** @brief For keeping track of motor timings based on hall sensors */
struct motor_timing
{
    int64_t step_prev;
    int64_t step_delta;
    uint64_t time_prev;
    uint64_t time_delta;
};

/** @brief Holds everything related to one motor */
struct motor
{
    // Motor's IO handles
    const struct motor_gpio *gpio;
    // To keep track of a motor's steps: counts up/down every hall interrupt
    int64_t step_count;
    // Keep track of each motor's step_delta & time_delta for calculating its speed
    struct motor_timing timing;
};

// -----------------------------------------------------------------------------
// GPIO pins for motor control
// -----------------------------------------------------------------------------

static const struct motor_gpio mfr_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), hall_gpios, 1),
};

static const struct motor_gpio mfl_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), hall_gpios, 1),
};

static const struct motor_gpio mbr_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), hall_gpios, 1),
};

static const struct motor_gpio mbl_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), hall_gpios, 1),
};

// -----------------------------------------------------------------------------
// Static objects for motors
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Motor control: private functions
// -----------------------------------------------------------------------------

/**
 * @brief Let a motor turn with given direction
 *
 * @param motor Pointer to motor's GPIO handles
 * @param direction Direction to turn (`1` = forwards, `0` = backwards)
 */
static inline void set_motor_direction(const struct motor_gpio *motor, uint8_t direction, uint32_t duty_cycle)
{
    pwm_set_dt(&(motor->in_a), PWM_PERIOD, direction ? PWM_MIN_DUTY_CYCLE : duty_cycle);
    pwm_set_dt(&(motor->in_b), PWM_PERIOD, direction ? duty_cycle : PWM_MIN_DUTY_CYCLE);
}

/**
 * @brief Stop a motor
 *
 * @param motor Pointer to motor's GPIO handles
 */
static inline void set_motor_stop(const struct motor_gpio *motor)
{
    pwm_set_dt(&(motor->in_a), PWM_PERIOD, PWM_MIN_DUTY_CYCLE);
    pwm_set_dt(&(motor->in_b), PWM_PERIOD, PWM_MIN_DUTY_CYCLE);
}

/**
 * @brief Convert from RPM to PWM duty-cycle
 */
static inline uint32_t duty_cycle_from_rpm(int rpm)
{
    const uint32_t max_rpm = 80;
    const uint32_t min_rpm = 0;
    uint32_t duty_cycle = min_rpm + rpm * (PWM_MAX_DUTY_CYCLE - PWM_MIN_DUTY_CYCLE) / (max_rpm - min_rpm);
    return duty_cycle;
}

// -----------------------------------------------------------------------------
// Motor control: API (public functions)
// -----------------------------------------------------------------------------

int cb_motor_init(void)
{
    int err = 0;

    err |= !pwm_is_ready_dt(&mfr_io.in_a);
    err |= !pwm_is_ready_dt(&mfr_io.in_b);
    err |= !gpio_is_ready_dt(&mfr_io.hall_c1);
    err |= !gpio_is_ready_dt(&mfr_io.hall_c2);

    err |= !pwm_is_ready_dt(&mfl_io.in_a);
    err |= !pwm_is_ready_dt(&mfl_io.in_b);
    err |= !gpio_is_ready_dt(&mfl_io.hall_c1);
    err |= !gpio_is_ready_dt(&mfl_io.hall_c2);

    err |= !pwm_is_ready_dt(&mbr_io.in_a);
    err |= !pwm_is_ready_dt(&mbr_io.in_b);
    err |= !gpio_is_ready_dt(&mbr_io.hall_c1);
    err |= !gpio_is_ready_dt(&mbr_io.hall_c2);

    err |= !pwm_is_ready_dt(&mbl_io.in_a);
    err |= !pwm_is_ready_dt(&mbl_io.in_b);
    err |= !gpio_is_ready_dt(&mbl_io.hall_c1);
    err |= !gpio_is_ready_dt(&mbl_io.hall_c2);

    if (err)
    {
        return err;
    }

    err |= gpio_pin_configure_dt(&mfr_io.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&mfr_io.hall_c2, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&mfl_io.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&mfl_io.hall_c2, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&mbr_io.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&mbr_io.hall_c2, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&mbl_io.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&mbl_io.hall_c2, GPIO_INPUT);

    if (err)
    {
        return err;
    }

    cb_stop();
    return 0;
}

void cb_set_motor_speed(cb_motor_speed_t *speeds)
{
    uint8_t mfl_dir = speeds->front_left > 0;
    uint32_t mfl_rpm = mfl_dir ? speeds->front_left : -speeds->front_left;

    uint8_t mfr_dir = speeds->front_right > 0;
    uint32_t mfr_rpm = mfr_dir ? speeds->front_right : -speeds->front_right;

    uint8_t mbl_dir = speeds->back_left > 0;
    uint32_t mbl_rpm = mbl_dir ? speeds->back_left : -speeds->back_left;

    uint8_t mbr_dir = speeds->back_right > 0;
    uint32_t mbr_rpm = mbr_dir ? speeds->back_right : -speeds->back_right;

    set_motor_direction(&mfl_io, mfl_dir, duty_cycle_from_rpm(mfl_rpm));
    set_motor_direction(&mfr_io, mfr_dir, duty_cycle_from_rpm(mfr_rpm));
    set_motor_direction(&mbl_io, mbl_dir, duty_cycle_from_rpm(mbl_rpm));
    set_motor_direction(&mbr_io, mbr_dir, duty_cycle_from_rpm(mbr_rpm));
}

void cb_stop(void)
{
    set_motor_stop(&mfl_io);
    set_motor_stop(&mfr_io);
    set_motor_stop(&mbl_io);
    set_motor_stop(&mbr_io);
}

void cb_get_motor_angle(cb_motor_angle_t *speeds)
{
    // FIXME: Get motor angles
}

void cb_get_motor_speed(cb_motor_speed_t *speeds)
{
    // FIXME: Get motor speeds
}
