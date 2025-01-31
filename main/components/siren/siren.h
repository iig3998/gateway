#ifndef SIREN_H
#define SIREN_H

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "header.h"

/* Struct siren request */
typedef struct {
	node_id_header_t header;
	bool state;
	bool battery_low_detect;
	uint16_t crc;
} __attribute__((__packed__)) node_siren_msg_t;

/* Struct node siren */
typedef struct {
	uint8_t id_node;
	uint8_t mac[MAC_SIZE];
	bool state;
	bool battery_low_detect;
} __attribute__((__packed__)) node_siren_t;

/* Struct sirens list */
struct node_sirens_list_t {
	node_siren_msg_t node;
	struct node_sirens_list_t *next;
	struct node_sirens_list_t *prev;
} __attribute__((__packed__));

struct node_sirens_list_t *get_sirens_from_list(struct node_sirens_list_t *p, uint8_t id);

struct node_sirens_list_t *add_sirens_to_list(struct node_sirens_list_t *p, node_siren_msg_t pn);

struct node_sirens_list_t *del_sirens_from_list(struct node_sirens_list_t *p, uint8_t id);

struct node_sirens_list_t *update_sirens_to_list(struct node_sirens_list_t *p, node_siren_msg_t pn);

uint8_t get_num_sirens_from_list();

void print_sirens_list(struct node_sirens_list_t *p);

node_siren_msg_t build_response_siren_msg(uint8_t id, uint8_t mac[]);

#endif