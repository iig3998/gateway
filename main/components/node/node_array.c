#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "node_array.h"

#define TAG_NODE "NODE_ARRAY"

static uint8_t counter_node = 0;
static node_t nl[10] = {};

/* Return node from list */
bool check_node_inside_list(uint8_t id) {

    if(nl[id].id_node)
        return true;

    return false;
}

node_t get_node_from_list(uint8_t id) {

    return nl[id];
}

/* Add node to list */
void add_node_to_list(node_msg_t msg) {

    time_t t;
    status_node sn;

    memcpy(&nl[msg.header.id_node - 1].mac, msg.header.mac, 6);

    nl[msg.header.id_node - 1].is_alive = true;
    nl[msg.header.id_node - 1].id_node = msg.header.id_node;

    memcpy(&sn, msg.payload, sizeof(status_node));
    nl[msg.header.id_node - 1].battery_low_detect = sn.battery_low_detect;
    nl[msg.header.id_node - 1].state = sn.state;
    time(&t);
    nl[msg.header.id_node - 1].time = t;

    counter_node++;

    return;
}

/* Delete node from list */
void del_node_from_list(uint8_t id) {

    memcpy(&nl[id - 1], 0, sizeof(node_t));

    return;
}

/* Update node */
void update_node_to_list(node_msg_t msg) {

    status_node sn;

    nl[msg.header.id_node - 1].is_alive = true;

    memcpy(&sn, msg.payload, sizeof(status_node));
    nl[msg.header.id_node - 1].battery_low_detect = sn.battery_low_detect;
    nl[msg.header.id_node - 1].state = sn.state;
    nl[msg.header.id_node - 1].time = time(NULL);

    return;
}

/* Return number of node inside list */
uint8_t get_num_node_from_list() {

    return counter_node;
}

/* Print node id list */
void print_node_list() {

    ESP_LOGD(TAG_NODE, "|---------------------------------------------------------------------------------|");
    ESP_LOGD(TAG_NODE, "|   ID   |  State reed switch  |  Status battery  |  Is alive  | Time last update |");
    ESP_LOGD(TAG_NODE, "|---------------------------------------------------------------------------------|");

    for(uint8_t i = 0; i < 10; i++) {
        if(nl[i].id_node) {
            ESP_LOGD(TAG_NODE, "| %6u | %16u | %16u | %16d | %16llu |", nl[i].id_node, nl[i].state, nl[i].battery_low_detect, nl[i].is_alive, nl[i].time);
        }
    }

    return;
}

/* Build node message */
node_msg_t build_node_msg(cmd_type cmd, uint8_t id_node, node_type node, uint8_t id_msg, uint8_t mac[], const char *name_node, void *payload) {

    node_msg_t msg;

    memset(&msg, 0, sizeof(node_msg_t));

    msg.header.cmd = cmd;
    msg.header.id_node = id_node;
    msg.header.id_msg = id_msg;
    msg.header.node = node;

    /* Set mac address */
    memcpy(msg.header.mac, mac, MAC_SIZE);

    /* Set name node */
    memset(msg.name_node, '\0', NAME_LEN);
    memcpy(msg.name_node, name_node, strlen(name_node));

    /* Set payload */
    memset(msg.payload, 0, PAYLOAD_LEN);
    if (payload) {
        switch(cmd) {
            case ADD:
                memcpy(msg.payload, payload, sizeof(time_t));
            break;
            case UPDATE:
            case SYNC:
                memcpy(msg.payload, payload, sizeof(status_node));
            break;
        }
    }

    msg.crc = calc_crc16_msg((uint8_t *)&msg, sizeof(msg) - sizeof(msg.crc));

    return msg;
}