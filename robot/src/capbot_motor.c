/**
 * @file capbot_motor.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief CapBot motor control
 * @date 2024-12-23
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "capbot.h"

#define PWM_PERIOD 20000000           /** ns */
#define PWM_MIN_DUTY_CYCLE 0          /** ns */
#define PWM_MAX_DUTY_CYCLE PWM_PERIOD /** ns */

#define MOTOR_RPM_MIN 0  /** RPM */
#define MOTOR_RPM_MAX 80 /** RPM */

#define HALL_TICKS_PER_REVOLUTION 1380

/** @brief Time delta over which motor RPM is calculated */
K_MUTEX_DEFINE(motor_timing_mutex);
#define T_RPM_STACKSIZE 256
#define T_RPM_PRIORITY 7
#define T_RPM_OPTIONS 0
#define T_RPM_DELAY 0
#define T_RPM_UPDATE_DELTA K_MSEC(20)

#define ABS(n) ((n) > 0 ? (n) : -(n))
#define CLIP(n, min, max) ((n) > (max) ? (max) : ((n) < (min) ? (min) : (n)))
#define MAP(n, in_min, in_max, out_min, out_max) ((n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

/**
 * @brief Convert from RPM to PWM duty-cycle
 */
static inline uint32_t rpm_to_duty_cycle(int rpm)
{
    uint32_t rpm_absolute, rpm_clipped, rpm_scaled;

    rpm_absolute = ABS(rpm);
    rpm_clipped = CLIP(rpm_absolute, MOTOR_RPM_MIN, MOTOR_RPM_MAX);
    rpm_scaled = MAP(rpm_clipped, MOTOR_RPM_MIN, MOTOR_RPM_MAX, PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);

    return rpm_scaled;
}

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

/** @brief Wraps everything related to one motor together*/
struct motor
{
    // Motor's IO handles
    const struct motor_gpio *gpio;
    // Callback info for motor's hall interrupt
    struct gpio_callback hall_cb_data;
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

static struct motor mfl = {.gpio = &mfl_io, .timing = {0, 0, 0, 0}, .step_count = 0};
static struct motor mfr = {.gpio = &mfr_io, .timing = {0, 0, 0, 0}, .step_count = 0};
static struct motor mbl = {.gpio = &mbl_io, .timing = {0, 0, 0, 0}, .step_count = 0};
static struct motor mbr = {.gpio = &mbr_io, .timing = {0, 0, 0, 0}, .step_count = 0};

// -----------------------------------------------------------------------------
// Motor control: private functions
// -----------------------------------------------------------------------------

/**
 * @brief Initialize a motor's IO
 *
 * @param motor
 * @return int
 */
static int init_motor_io(struct motor *motor, gpio_callback_handler_t hall_isr)
{
    int err = 0;

    err |= !pwm_is_ready_dt(&motor->gpio->in_a);
    err |= !pwm_is_ready_dt(&motor->gpio->in_b);

    if (err)
    {
        return err;
    }

    err |= !gpio_is_ready_dt(&motor->gpio->hall_c1);
    err |= !gpio_is_ready_dt(&motor->gpio->hall_c2);

    if (err)
    {
        return err;
    }

    err |= gpio_pin_configure_dt(&motor->gpio->hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&motor->gpio->hall_c2, GPIO_INPUT);

    if (err)
    {
        return err;
    }

    gpio_init_callback(&motor->hall_cb_data, hall_isr, BIT(motor->gpio->hall_c2.pin));
    err |= gpio_pin_interrupt_configure_dt(&motor->gpio->hall_c2, GPIO_INT_EDGE_BOTH);
    err |= gpio_add_callback_dt(&motor->gpio->hall_c2, &motor->hall_cb_data);

    return err;
}

/**
 * @brief Let a motor turn with given direction
 *
 * @param motor Pointer to motor's wrapper struct
 * @param direction Direction to turn (`1` = forwards, `0` = backwards)
 */
static inline void set_motor_direction(const struct motor *motor, uint8_t direction, uint32_t duty_cycle)
{
    pwm_set_dt(&(motor->gpio->in_a), PWM_PERIOD, direction ? PWM_MIN_DUTY_CYCLE : duty_cycle);
    pwm_set_dt(&(motor->gpio->in_b), PWM_PERIOD, direction ? duty_cycle : PWM_MIN_DUTY_CYCLE);
}

/**
 * @brief Stop a motor
 *
 * @param motor Pointer to motor's wrapper struct
 */
static inline void set_motor_stop(const struct motor *motor)
{
    pwm_set_dt(&(motor->gpio->in_a), PWM_PERIOD, PWM_MIN_DUTY_CYCLE);
    pwm_set_dt(&(motor->gpio->in_b), PWM_PERIOD, PWM_MIN_DUTY_CYCLE);
}

/**
 * @brief Update motor timing
 */
static inline void update_motor_timing(struct motor *motor, uint64_t time_curr)
{
    if (k_mutex_lock(&motor_timing_mutex, K_FOREVER) == 0)
    {
        motor->timing.step_delta = motor->step_count - motor->timing.step_prev;
        motor->timing.step_prev = motor->step_count;
        motor->timing.time_delta = time_curr - motor->timing.time_prev;
        motor->timing.time_prev = time_curr;
        k_mutex_unlock(&motor_timing_mutex);
    }
}

/**
 * @brief Get motor's angle in degrees
 */
static inline int get_motor_angle(struct motor *motor)
{
    // TODO: Verify conversion (below is an educated guess)
    return motor->step_count * 360 / HALL_TICKS_PER_REVOLUTION;
}

/**
 * @brief Get motor's speed in RPM
 */
static inline int get_motor_rpm(struct motor *motor)
{
    if (k_mutex_lock(&motor_timing_mutex, T_RPM_UPDATE_DELTA))
    {
        // Error: could not lock mutex
        return -1;
    }
    int64_t step_d = motor->timing.step_delta;
    uint64_t time_d = motor->timing.time_delta;
    k_mutex_unlock(&motor_timing_mutex);

    // TODO: Verify conversion (below is an educated guess)
    return (step_d * 60000) / (int64_t)k_ticks_to_ms_near64(time_d * HALL_TICKS_PER_REVOLUTION);
}

// -----------------------------------------------------------------------------
// Motor control: interrupts and threads
// -----------------------------------------------------------------------------

/**
 * @brief Generic hall sensor interrupt handler (called by motor specific ones)
 */
static inline void hall_isr(struct motor *motor)
{
    int c1 = gpio_pin_get_dt(&motor->gpio->hall_c1);
    int c2 = gpio_pin_get_dt(&motor->gpio->hall_c2);

    if (c1 == c2)
    {
        motor->step_count++;
    }
    else
    {
        motor->step_count--;
    }
}

void mfr_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mfr); }
void mfl_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mfl); }
void mbr_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mbr); }
void mbl_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mbl); }

/**
 * @brief Periodically update timing info for calculating (average) motor speeds
 */
void t_rpm_entry_point(void *, void *, void *)
{
    for (;;)
    {
        update_motor_timing(&mfr, k_uptime_ticks());
        update_motor_timing(&mfl, k_uptime_ticks());
        update_motor_timing(&mbr, k_uptime_ticks());
        update_motor_timing(&mbl, k_uptime_ticks());

        k_sleep(T_RPM_UPDATE_DELTA);
    }
}
K_THREAD_DEFINE(motor_timing_update, T_RPM_STACKSIZE, t_rpm_entry_point, NULL, NULL, NULL, T_RPM_PRIORITY, T_RPM_OPTIONS, T_RPM_DELAY);

// -----------------------------------------------------------------------------
// Motor control: API (public functions)
// -----------------------------------------------------------------------------

int cb_motor_init(void)
{
    int err = 0;

    err = init_motor_io(&mfl, mfl_hall_isr);
    if (err)
    {
        return err;
    }
    set_motor_stop(&mfl);

    err = init_motor_io(&mfr, mfr_hall_isr);
    if (err)
    {
        return err;
    }
    set_motor_stop(&mfr);

    err = init_motor_io(&mbl, mbl_hall_isr);
    if (err)
    {
        return err;
    }
    set_motor_stop(&mbl);

    err = init_motor_io(&mbr, mbr_hall_isr);
    if (err)
    {
        return err;
    }
    set_motor_stop(&mbr);

    return 0;
}

void cb_set_motor_speed(cb_motor_speed_t *speeds)
{
    uint8_t mfl_dir = speeds->front_left > 0;
    uint8_t mfr_dir = speeds->front_right > 0;
    uint8_t mbl_dir = speeds->back_left > 0;
    uint8_t mbr_dir = speeds->back_right > 0;

    set_motor_direction(&mfl, mfl_dir, rpm_to_duty_cycle(speeds->front_left));
    set_motor_direction(&mfr, mfr_dir, rpm_to_duty_cycle(speeds->front_right));
    set_motor_direction(&mbl, mbl_dir, rpm_to_duty_cycle(speeds->back_left));
    set_motor_direction(&mbr, mbr_dir, rpm_to_duty_cycle(speeds->back_right));
}

void cb_stop(void)
{
    set_motor_stop(&mfl);
    set_motor_stop(&mfr);
    set_motor_stop(&mbl);
    set_motor_stop(&mbr);
}

void cb_get_motor_angle(cb_motor_angle_t *angles)
{
    angles->front_left = get_motor_angle(&mfl);
    angles->front_right = get_motor_angle(&mfr);
    angles->back_left = get_motor_angle(&mbl);
    angles->back_right = get_motor_angle(&mbr);
}

void cb_get_motor_speed(cb_motor_speed_t *speeds)
{
    speeds->front_left = get_motor_rpm(&mfl);
    speeds->front_right = get_motor_rpm(&mfr);
    speeds->back_left = get_motor_rpm(&mbl);
    speeds->back_right = get_motor_rpm(&mbr);
}
