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

typedef enum {
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
typedef enum {
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

#endif /* CAPBOT_H */
