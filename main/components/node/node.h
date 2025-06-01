#ifndef NODE_H
#define NODE_H

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
} __attribute__((__packed__)) node_header_t;

/* Struct node message */
typedef struct {
	node_header_t header;
	char name_node[NAME_LEN];
	uint8_t payload[PAYLOAD_LEN];
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

/* Struct node list */
struct node_list_t {
	node_t node;
	struct node_list_t *next;
	struct node_list_t *prev;
} __attribute__((__packed__));

/* Struct payload node */
typedef struct {
    bool state;
    bool battery_low_detect;
} __attribute__((__packed__)) status_node;

void print_node_list(struct node_list_t *p);

struct node_list_t *get_first_node_from_list(struct node_list_t *p);

struct node_list_t *add_node_to_list(struct node_list_t *p, node_msg_t pn);

struct node_list_t *del_node_from_list(struct node_list_t *p, uint8_t id);

struct node_list_t *update_node_to_list(struct node_list_t *p, node_msg_t pn);

struct node_list_t *get_node_from_list(struct node_list_t *p, uint8_t id);

uint8_t get_num_node_from_list();

node_msg_t build_node_msg(cmd_type cmd, uint8_t id_node, node_type node, uint8_t id_msg, uint8_t mac[], const char *name_node, void *payload);

#endif