/**
 * @file capbot.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief CapBot library implementation
 */

#include <zephyr/drivers/gpio.h>

#include "capbot.h"

// -----------------------------------------------------------------------------
// FreeBot on board IO
// -----------------------------------------------------------------------------

static const struct gpio_dt_spec d15 = GPIO_DT_SPEC_GET(DT_ALIAS(d15), gpios);
static const struct gpio_dt_spec d16 = GPIO_DT_SPEC_GET(DT_ALIAS(d16), gpios);
static const struct gpio_dt_spec sw2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

int cb_init_io(void)
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
