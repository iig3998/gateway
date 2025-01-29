#ifndef KEYBOARD_H
#define KEYBOARD_H

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "header.h"

#define MAC_SIZE 6

/* Struct alarm sensor */
typedef struct {
	node_id_header_t header;
	bool state;
	bool battery_low_detect;
	uint16_t crc;
} __attribute__((__packed__)) node_id_keyboard_request_t;

/* Struct response */
typedef struct {
	node_id_header_t header;
	bool ack;
	uint16_t crc;
} __attribute__((__packed__)) node_id_keyboard_response_t;

#endif