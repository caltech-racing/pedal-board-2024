/*
 * 74hc595.c
 *
 *  Created on: Mar 19, 2024
 *      Author: gavin
 */


#include "74hc595.h"
#include "string.h"
#include <stdlib.h>

/* Register to digit mapping for the first shift register */
// TODO
static const uint8_t DIGITS1[10] = {
    0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
};

/* Register to digit mapping for the second shift register */
// TODO
static const uint8_t DIGITS2[10] = {
    0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
};

static const uint8_t DP1 = 0b00001000;
static const uint8_t DP2 = 0b10000000;

static SPI_HandleTypeDef *_hspi;
static GPIO_TypeDef *_latch_port;
static uint16_t _latch_pin;
static const int timeout = 100;

/**
 * @brief 	sets up the pins needed for SPI communication with the shift register.
 *
 * @param	hspi	The pointer to the SPI handle. This should be directly available from the cubeMX setup code
 * @param 	latch_port	A pointer to latch signal port on the chip.
 * @param	latch_pin	The pin number of the latch signal pin
 */
void shift_reg_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *latch_port, uint16_t latch_pin) {
	_hspi = hspi;
	_latch_port = latch_port;
	_latch_pin = latch_pin;
}

/**
 * @brief 	write a number to the LED display, possibly with decimal points
 *
 * @param 	num		a two- or one- digit numebr to be written to the display. If a single digit
 * 					is passed, then it is written to the right digit on the display.
 * @param	dp1		the left decimal point on the LED display
 * @param	dp2 	the right decimal point on the LED display
 * @retval	-1 if an error occured, 0 if success
 */
int shift_reg_display(uint8_t num, uint8_t dp1, uint8_t dp2) {
	char num_str[8];
	if (num >= 100 || num < 0) {
		return -1;
	}
	itoa(num, num_str, 10);
	int len = strlen(num_str);
	uint8_t spi_data[2];
	if (len == 1) {
		// defaults to displaying on the lowest digit
		spi_data[0] = DIGITS2[num_str[0]-'0'] | (DP2 && dp2);
		spi_data[1] = DP1 && dp1;
	} else if (len == 2) {
		spi_data[0] = DIGITS2[num_str[1]-'0'] | (DP2 && dp2);
		spi_data[1] = DIGITS1[num_str[0]-'0'] | (DP1 && dp1);
	} else { return -1; }
	// set latch pin to low
	HAL_GPIO_WritePin(_latch_port, _latch_pin, GPIO_PIN_RESET);
	// write using SPI
	HAL_SPI_Transmit(_hspi, spi_data, len, timeout);
	// once data is ready set latch pin to high
	HAL_GPIO_WritePin(_latch_port, _latch_pin, GPIO_PIN_SET);
	return 0;
}
