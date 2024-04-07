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

static const uint8_t DP1 = 0b11011111;
static const uint8_t DP2 = 0b11111111;

void shift_reg_init(GPIO_TypeDef *shcp_port, GPIO_TypeDef *stcp_port, GPIO_TypeDef *data_port,
                    uint16_t shcp_pin, uint16_t stcp_pin, uint16_t data_pin);
int shift_reg_display(uint8_t num, uint8_t dp1, uint8_t dp2);

#endif /* _INC_74HC595_H */
