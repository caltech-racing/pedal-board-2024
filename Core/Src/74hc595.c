/*
 * 74hc595.c
 *
 *  Created on: Mar 19, 2024
 *      Author: gavin
 */


#include "74hc595.h"
#include "string.h"
#include <stdlib.h>

static GPIO_TypeDef *sr_stcp_port;
static GPIO_TypeDef *sr_shcp_port;
static GPIO_TypeDef *sr_data_port;
static uint16_t sr_stcp_pin;
static uint16_t sr_shcp_pin;
static uint16_t sr_data_pin;
static const int timeout = 100;

/**
 * @brief 	sets up the pins needed for SPI communication with the shift register.
 *
 * @param	hspi	The pointer to the SPI handle. This should be directly available from the cubeMX setup code
 * @param 	latch_port	A pointer to latch signal port on the chip.
 * @param	latch_pin	The pin number of the latch signal pin
 */
void shift_reg_init(GPIO_TypeDef *shcp_port, GPIO_TypeDef *stcp_port, GPIO_TypeDef *data_port,
                    uint16_t shcp_pin, uint16_t stcp_pin, uint16_t data_pin) {
    sr_stcp_port = stcp_port;
    sr_shcp_port = shcp_port;
    sr_data_port = data_port;
    sr_stcp_pin = stcp_pin;
    sr_shcp_pin = shcp_pin;
    sr_data_pin = data_pin;
}

/**
 * @brief 	write a number to the LED display, possibly with decimal points
 *
 * @param 	num		a two- or one- digit number to be written to the display. If a single digit
 * 					is passed, then it is written to the right digit on the display.
 * @param	dp1		the left decimal point on the LED display, should be either DP1 or 0
 * @param	dp2 	the right decimal point on the LED display, should be either DP2 or 0
 * @retval	-1 if an error occurred, 0 if success
 */
int shift_reg_display(uint8_t num, uint8_t dp1, uint8_t dp2) {
	char num_str[8];
	if (num >= 100 || num < 0) {
		return -1;
	}
	itoa(num, num_str, 10);
	unsigned int len = strlen(num_str);
	uint8_t spi_data[2];
	if (len == 1) {
		// defaults to displaying on the lowest digit
		spi_data[0] = DIGITS2[num_str[0]-'0'] | dp1;
		spi_data[1] = dp2;
	} else if (len == 2) {
		spi_data[0] = DIGITS2[num_str[1]-'0'] | dp2;
		spi_data[1] = DIGITS1[num_str[0]-'0'] | dp1;
	} else {
		return -1;
	}
	spi_data[0] = 1;
	spi_data[1] = 1;
	// set stcp (storage clock) pin to low, as the data goes to the storage register
    // on the rising edge of the clock
	HAL_GPIO_WritePin(sr_stcp_port, sr_stcp_pin, GPIO_PIN_RESET);
	// write using SPI
	for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 8; j++) {
            // Lower the shift register clock. The shift happens on the rising edge.
            HAL_GPIO_WritePin(sr_shcp_port, sr_shcp_pin, GPIO_PIN_RESET);
            uint8_t lsb = spi_data[i] & 1;
            spi_data[i] >>= 1;
            HAL_GPIO_WritePin(sr_data_port, sr_data_pin, lsb);
            HAL_Delay(1);
            HAL_GPIO_WritePin(sr_shcp_port, sr_shcp_pin, GPIO_PIN_SET);
            HAL_Delay(1);
        }
	}
	// once data is ready set latch pin to high
    HAL_GPIO_WritePin(sr_stcp_port, sr_stcp_pin, GPIO_PIN_SET);
	return 0;
}
