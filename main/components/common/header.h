#ifndef HEADER_H
#define HEADER_H

#pragma once

#include <stdint.h>

#define MAC_SIZE 6

enum node_type {
    SENSOR = 0,
    SIREN,
	KEYBOARD,
	GATEWAY
};

enum msg_type {
	REQUEST = 0,
	RESPONSE
};

enum cmd_type {
	ADD = 0,
	DEL,
	UPDATE,
	GET,
	ACTIVE,
	DEACTIVE,
	ACK,
	NACK
};

/* Header struct 
+---------------+--------------+-------------+--------------+--------------+
| node (1 byte) | msg (1 byte) | id (1 byte) | mac (6 byte) | cmd (1 byte) |
+---------------+--------------+-------------+--------------+--------------+
*/

typedef struct {
	enum node_type node;
	enum msg_type msg;
	uint8_t id_node;
	uint8_t id_msg;
	uint8_t mac[MAC_SIZE];
	enum cmd_type cmd;
} __attribute__((__packed__)) node_id_header_t;

node_id_header_t build_header_msg(enum msg_type msg, enum node_type node, uint8_t id_node, uint8_t id_msg, uint8_t mac[], enum cmd_type cmd);

#endif