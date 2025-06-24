#ifndef NODE_ARRAY_H
#define NODE_ARRAY_H

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "common.h"

#define MAC_SIZE    6
#define NAME_LEN    15
#define PAYLOAD_LEN 8

/* Define node type */
typedef enum __attribute__((__packed__)) {
    SENSOR = 0,
    SIREN,
	KEYBOARD,
	GATEWAY
} node_type;

/* Define command type */
typedef enum __attribute__((__packed__)) {
	ADD = 0,
	DEL,
	GET,
	UPDATE,
	SYNC,
	ALARM,
	START_SIREN,
	STOP_SIREN,
	ACTIVE_ALARM,
	DEACTIVE_ALARM
} cmd_type;

/* Header struct
+--------------------+-----------------+------------------+--------------+--------------+
| node_type (1 byte) | id_msg (1 byte) | id_node (1 byte) | mac (6 byte) | cmd (1 byte) |
+--------------------+-----------------+------------------+--------------+--------------+
*/

/* Struct node header message */
typedef struct {
	node_type node;
	uint8_t id_node;
	uint8_t id_msg;
	uint8_t mac[MAC_SIZE];
	cmd_type cmd;
} __attribute__((__packed__)) node_id_header;

/* Struct node message */
typedef struct {
	node_id_header header;
	char name_node[NAME_LEN];
	int8_t payload[PAYLOAD_LEN];
	uint16_t crc;
} __attribute__((__packed__)) node_msg_t;

/* Struct node */
typedef struct {
	uint8_t mac[MAC_SIZE];
	uint8_t id_node;
	bool state;
	bool battery_low_detect;
	bool is_alive;
    uint32_t last_sync_cycle;
	time_t time;
} __attribute__((__packed__)) node_t;

/* Struct payload node */
typedef struct {
    bool state;
    bool battery_low_detect;
} __attribute__((__packed__)) status_node;

void print_node_list();

void add_node_to_list(node_msg_t msg);

void del_node_from_list(uint8_t id);

void update_node_to_list(node_msg_t msg);

bool check_node_inside_list(uint8_t id);

node_t get_node_from_list(uint8_t id);

uint8_t get_num_node_from_list();

node_msg_t build_node_msg(cmd_type cmd, uint8_t id_node, node_type node, uint8_t id_msg, uint8_t mac[], const char *name_node, void *payload);

#endif