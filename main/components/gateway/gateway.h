#ifndef GATEWAY_H
#define GATEWAY_H

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "header.h"

/* Struct gateway message */
typedef struct {
    node_id_header_t header;
	bool start_siren;
	uint16_t crc;
} __attribute__((__packed__)) node_gateway_msg_t;

node_gateway_msg_t build_response_ack_gateway_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]); 

node_gateway_msg_t build_response_nack_gateway_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

#endif