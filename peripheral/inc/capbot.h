/**
 * @file capbot.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Header file for CapBot library
 */

#ifndef CAPBOT_H
#define CAPBOT_H

typedef enum capbot_led_e {
    CAPBOT_D15,
    CAPBOT_D16,
} capbot_let_t;

/** @brief Initialize robot's leds & button */
int capbot_init_io(void);


/**
 * @brief Turn led on
 *
 * @param led The led to set
 */
int capbot_led_set(capbot_let_t led);

/**
 * @brief Turn led off
 *
 * @param led The led to clear
 */
// TODO: int capbot_led_clr(capbot_let_t led);

/**
 * @brief Toggle a led
 *
 * @param led The led to toggle
 */
// TODO: int capbot_led_tgl(capbot_let_t led);

/**
 * @brief Get a led's status
 *
 * @param led The led to get
 * @retval 0: the led is turned off
 * @retval 1: the led is turned on
 */
// TODO: int capbot_led_get(capbot_let_t led);

/**
 * @brief Get button status
 *
 * @retval 0: the button isn't pressed
 * @retval 1: the button is pressed
 */
// TODO: int capbot_btn_get(void);

#endif /* CAPBOT_H */
