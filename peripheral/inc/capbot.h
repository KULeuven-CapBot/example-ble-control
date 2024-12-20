/**
 * @file capbot.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Header file for CapBot library
 */

#ifndef CAPBOT_H
#define CAPBOT_H

typedef enum {
    CB_D15,
    CB_D16,
} cb_led_t;

/** @brief Initialize robot's leds & button */
int cb_init_io(void);


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

#endif /* CAPBOT_H */
