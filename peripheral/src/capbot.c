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

int capbot_init_io(void)
{
    int err = 0;

    if (gpio_is_ready_dt(&d15))
    {
        err |= gpio_pin_configure_dt(&d15, GPIO_OUTPUT | GPIO_INPUT);
    }
    else
    {
        err = 1;
    }

    if (gpio_is_ready_dt(&d16))
    {
        err |= gpio_pin_configure_dt(&d16, GPIO_OUTPUT | GPIO_INPUT);
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

int capbot_led_set(capbot_let_t led)
{
    switch (led)
    {
    case CAPBOT_D15:
        gpio_pin_set_dt(&d15, 1);
        return 0;
    case CAPBOT_D16:
        gpio_pin_set_dt(&d16, 1);
        return 0;
    default:
        return 1;
    }
}
