/*
 * 74hc595.h
 *
 *  Created on: Mar 19, 2024
 *      Author: gavin
 */

#ifndef _INC_74HC595_H
#define _INC_74HC595_H

#include "main.h"
/* Register to digit mapping for the first shift register */
// TODO
static const uint8_t DIGITS1[10] = {
		0b01011111,
		0b01000001,
		0b00111011,
		0b01110011,
		0b01100101,
		0b01110110,
		0b01111110,
		0b01000011,
		0b01111111,
		0b01110111
};

/* Register to digit mapping for the second shift register */
// TODO
static const uint8_t DIGITS2[10] = {
		0b10110111,
		0b00010100,
		0b01110011,
		0b01110110,
		0b11010100,
		0b11100110,
		0b11100111,
		0b00110100,
		0b11110111,
		0b11110110
};

static const uint8_t DP1 = 0b10000000;
static const uint8_t DP2 = 0b00001000;

void shift_reg_init(GPIO_TypeDef *shcp_port, GPIO_TypeDef *stcp_port, GPIO_TypeDef *data_port,
                    uint16_t shcp_pin, uint16_t stcp_pin, uint16_t data_pin);
int shift_reg_display(uint8_t num, uint8_t dp1, uint8_t dp2);

#endif /* _INC_74HC595_H */
