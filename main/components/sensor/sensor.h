#ifndef SENSOR_H
#define SENSOR_H

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "header.h"

#define MAC_SIZE 6

/* Struct node sensor */
typedef struct {
	node_id_header_t header;
	bool state;
	bool battery_low_detect;
	uint16_t crc;
} __attribute__((__packed__)) node_sensor_msg_t;

/* Struct node sensor */
typedef struct {
	uint8_t id_node;
	uint8_t mac[MAC_SIZE];
	bool state;
	bool battery_low_detect;
} __attribute__((__packed__)) node_sensor_t;

/* Struct node alarm and siren list */
struct node_sensors_list_t {
	node_sensor_t node;
	struct node_sensors_list_t *next;
	struct node_sensors_list_t *prev;
} __attribute__((__packed__));

struct node_sensors_list_t *add_sensors_to_list(struct node_sensors_list_t *p, node_sensor_msg_t pn);

struct node_sensors_list_t *del_sensors_from_list(struct node_sensors_list_t *p, uint8_t id);

struct node_sensors_list_t *update_sensors_to_list(struct node_sensors_list_t *p, node_sensor_msg_t pn);

struct node_sensors_list_t *get_sensors_from_list(struct node_sensors_list_t *p, uint8_t id);

uint8_t get_num_sensors_from_list();

void print_sensors_list(struct node_sensors_list_t *p);

node_sensor_msg_t build_request_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

node_sensor_msg_t build_response_ack_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

node_sensor_msg_t build_response_nack_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

#endif