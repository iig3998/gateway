#ifndef COMMON_H
#define COMMON_H

#pragma once

#include <stdint.h>

uint16_t calc_crc16_msg(uint8_t *data, uint32_t length);

#endif