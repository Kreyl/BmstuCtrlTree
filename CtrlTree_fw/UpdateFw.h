/*
 * UpdateFw.h
 *
 *  Created on: May 5, 2020
 *      Author: layst
 */

#pragma once

#include <inttypes.h>

uint8_t UpdateFw(uint8_t *ptr, uint32_t Len, uint16_t CrcIn);

extern uint32_t ApplicationAddr;
