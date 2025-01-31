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
} __attribute__((__packed__)) node_keyboard_msg_t;

node_keyboard_msg_t build_request_keryboard_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

node_keyboard_msg_t build_response_ack_keyboard_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

node_keyboard_msg_t build_response_nack_keyboard_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

#endif