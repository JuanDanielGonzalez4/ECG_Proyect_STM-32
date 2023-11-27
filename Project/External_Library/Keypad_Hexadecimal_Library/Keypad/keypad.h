
/**
 * @file keypad.h
 * @brief Header file for keypad functions.
 */


#include <stdint.h>

/**
 * @brief Initializes the keypad.
 */
void keypad_init(void);

/**
 * @brief Handles keypad events and debounces key presses.
 *
 * @param column_to_evaluate The column where the event happened.
 * @return 0xFF for an invalid key, [0x00 - 0x0F] for valid keys.
 */
uint8_t keypad_handler(uint16_t column_to_evaluate);


