/*
 * 74hc595.h
 *
 *  Created on: Mar 19, 2024
 *      Author: gavin
 */

#ifndef _INC_74HC595_H
#define _INC_74HC595_H

#include "main.h"

void shift_reg_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *latch_port, uint16_t latch_pin);
int led_display_num(uint8_t num, uint8_t dp1, uint8_t dp2);

#endif /* _INC_74HC595_H */
