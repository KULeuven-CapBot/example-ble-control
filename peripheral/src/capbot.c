/**
 * @file capbot.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief CapBot library implementation
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

#include "capbot.h"

// -----------------------------------------------------------------------------
// Robot's on-board IO
// -----------------------------------------------------------------------------

static const struct gpio_dt_spec d15 = GPIO_DT_SPEC_GET(DT_ALIAS(d15), gpios);
static const struct gpio_dt_spec d16 = GPIO_DT_SPEC_GET(DT_ALIAS(d16), gpios);
static const struct gpio_dt_spec sw2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

int cb_io_init(void)
{
    int err = 0;

    if (gpio_is_ready_dt(&d15))
    {
        err |= gpio_pin_configure_dt(&d15, GPIO_OUTPUT | GPIO_INPUT);
        err |= gpio_pin_set_dt(&d15, 0);
    }
    else
    {
        err = 1;
    }

    if (gpio_is_ready_dt(&d16))
    {
        err |= gpio_pin_configure_dt(&d16, GPIO_OUTPUT | GPIO_INPUT);
        err |= gpio_pin_set_dt(&d16, 0);
    }
    else
    {
        err = 1;
    }

    if (gpio_is_ready_dt(&sw2))
    {
        err |= gpio_pin_configure_dt(&sw2, GPIO_INPUT);
    }
    else
    {
        err = 1;
    }

    return err;
}

const struct gpio_dt_spec *cb_led_from_id(cb_led_t led)
{
    switch (led)
    {
    case CB_D15:
        return &d15;
    case CB_D16:
        return &d16;
    default:
        return NULL;
    }
}

int cb_led_set(cb_led_t led)
{
    const struct gpio_dt_spec *led_dt = cb_led_from_id(led);
    if (led_dt != NULL)
    {
        return gpio_pin_set_dt(led_dt, 1);
    }

    return 1;
}

int cb_led_clr(cb_led_t led)
{
    const struct gpio_dt_spec *led_dt = cb_led_from_id(led);
    if (led_dt != NULL)
    {
        return gpio_pin_set_dt(led_dt, 0);
    }

    return 1;
}

int cb_led_tgl(cb_led_t led)
{
    const struct gpio_dt_spec *led_dt = cb_led_from_id(led);
    if (led_dt != NULL)
    {
        return gpio_pin_toggle_dt(led_dt);
    }

    return 1;
}

int cb_led_get(cb_led_t led)
{
    const struct gpio_dt_spec *led_dt = cb_led_from_id(led);
    if (led_dt != NULL)
    {
        return gpio_pin_get_dt(led_dt);
    }

    return 1;
}

int cb_btn_get(void)
{
    return gpio_pin_get_dt(&sw2);
}

// -----------------------------------------------------------------------------
// On-board voltage measurements
// -----------------------------------------------------------------------------

static const struct adc_dt_spec v_measure = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
static const struct gpio_dt_spec measure_en_vin = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), measure_en_gpios, 0);
static const struct gpio_dt_spec measure_en_vcap = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), measure_en_gpios, 1);
static const struct gpio_dt_spec measure_en_vmotor = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), measure_en_gpios, 2);

int cb_measure_init(void)
{
    int err = 0;

    if (adc_is_ready_dt(&v_measure))
    {
        err |= adc_channel_setup_dt(&v_measure);
    }
    else
    {
        return 1;
    }

    if (gpio_is_ready_dt(&measure_en_vin))
    {
        err |= gpio_pin_configure_dt(&measure_en_vin, GPIO_OUTPUT);
        err |= gpio_pin_set_dt(&measure_en_vin, 0);
    }
    else
    {
        err = 1;
    }

    if (gpio_is_ready_dt(&measure_en_vcap))
    {
        err |= gpio_pin_configure_dt(&measure_en_vcap, GPIO_OUTPUT);
        err |= gpio_pin_set_dt(&measure_en_vcap, 0);
    }
    else
    {
        err = 1;
    }

    if (gpio_is_ready_dt(&measure_en_vmotor))
    {
        err |= gpio_pin_configure_dt(&measure_en_vmotor, GPIO_OUTPUT);
        err |= gpio_pin_set_dt(&measure_en_vmotor, 0);
    }
    else
    {
        err = 1;
    }

    return 0;
}

int cb_measure_select(cb_measure_t sel)
{
    int err = 0;
    switch (sel)
    {
    case CB_VNONE:
        err |= gpio_pin_set_dt(&measure_en_vin, 0);
        err |= gpio_pin_set_dt(&measure_en_vcap, 0);
        err |= gpio_pin_set_dt(&measure_en_vmotor, 0);
        return err;
    case CB_VIN:
        err |= cb_measure_select(CB_VNONE);
        err |= gpio_pin_set_dt(&measure_en_vin, 1);
        return err;
    case CB_VCAP:
        err |= cb_measure_select(CB_VNONE);
        err |= gpio_pin_set_dt(&measure_en_vcap, 1);
        return err;
    case CB_VMOTOR:
        err |= cb_measure_select(CB_VNONE);
        err |= gpio_pin_set_dt(&measure_en_vmotor, 1);
        return err;
    default:
        err |= cb_measure_select(CB_VNONE);
        return err | 1;
    }
}

int cb_measure_vcap(void)
{
    if (!gpio_pin_get_dt(&measure_en_vcap))
    {
        cb_measure_select(CB_VCAP);
        k_sleep(K_MSEC(10));
    }

    int err = 0;
    uint32_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        /* buffer size in bytes, not number of samples */
        .buffer_size = sizeof(buf),
    };

    err |= adc_sequence_init_dt(&v_measure, &sequence);
    err |= adc_read_dt(&v_measure, &sequence);
    err |= adc_raw_to_millivolts_dt(&v_measure, &buf);

    // TODO: Verify conversion (below is a theoretic value according to voltage divider)
    return buf * 40;
}
