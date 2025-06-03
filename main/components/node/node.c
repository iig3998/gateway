#include <malloc.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "node.h"

#define TAG_NODE "NODE"

static uint8_t counter_node = 0;

/* Return tail node list */
static struct node_list_t *get_tail_node_list(struct node_list_t *p) {

    if(!p)
        return p;

    while(p->next) {
        p = p->next;
    }

    return p;
}

/* Return head node list */
static struct node_list_t *get_head_node_list(struct node_list_t *p) {

    if(!p)
        return p;

    while(p->prev) {
        p = p->prev;
    }

    return p;
}

/* Return first node form list */
struct node_list_t *get_first_node_from_list(struct node_list_t *p) {

    return get_head_node_list(p);
}

/* Return node from list */
struct node_list_t *get_node_from_list(struct node_list_t *p, uint8_t id) {

    p = get_head_node_list(p);
    if (!p)
        return NULL;

    do {
        if(p->node.id_node == id) {
            return p;
        }

        if(p->next)
            p = p->next;

    } while(p->next);

    if(p->node.id_node == id) {
        return p;
    }

    return NULL;
}

/* Add node to list */
struct node_list_t *add_node_to_list(struct node_list_t *p, node_msg_t pn) {

    status_node sn;

    p = get_tail_node_list(p);

    struct node_list_t *px = (struct node_list_t *)calloc(1, sizeof(struct node_list_t));
    if(!px) {
        return NULL;
    }

    px->node.id_node = pn.header.id_node;
    px->node.is_alive = true;

    memcpy(&sn, pn.payload, sizeof(status_node));
    px->node.battery_low_detect = sn.battery_low_detect;
    px->node.state = sn.state;

    px->node.time = time(NULL);

    px->next = NULL;

    if(!p){
        px->prev = NULL;
    } else {
        px->prev = p;
        p->next = px;
    }

    p = px;
    counter_node++;

    return p;
}

/* Delete node from list */
struct node_list_t *del_node_from_list(struct node_list_t *p, uint8_t id) {

    struct node_list_t *p1 = NULL;
    struct node_list_t *p2 = NULL;
    struct node_list_t *p0 = NULL;

    p0 = get_node_from_list(p, id);
    if(!p0) {
        return p;
    }

    if(p0->prev)
        p1 = p0->prev;

    if(p0->next)
        p2 = p0->next;

    if(p1 && !p2)
        p1->next = NULL;

    if(!p1 && p2)
        p2->prev = NULL;

    if (p1 && p2) {
        p1->next = p2;
        p2->prev = p1;
    }

    free(p0);

    counter_node--;

    if(p2)
        return get_tail_node_list(p2);
    else if(p1)
        return get_tail_node_list(p1);

    return NULL;
}

/* Update node */
struct node_list_t *update_node_to_list(struct node_list_t *p, node_msg_t pn) {

    status_node sn;

    p = get_node_from_list(p, pn.header.id_node);
    if (!p)
        return p;

    p->node.is_alive = true;

    memcpy(&sn, pn.payload, sizeof(status_node));
    p->node.battery_low_detect = sn.battery_low_detect;
    p->node.state = sn.state;
    p->node.time = time(NULL);

    return p;
}

/* Return number of node inside list */
uint8_t get_num_node_from_list() {

    return counter_node;
}

/* Print node id list */
void print_node_list(struct node_list_t *p) {

    ESP_LOGD(TAG_NODE, "|---------------------------------------------------------------------------------|");
    ESP_LOGD(TAG_NODE, "|   ID   |  State reed switch  |  Status battery  |  Is alive  | Time last update |");
    ESP_LOGD(TAG_NODE, "|---------------------------------------------------------------------------------|");

    p = get_head_node_list(p);
    if(!p) {
        ESP_LOGD(TAG_NODE, "No alarm node available");
        return;
    }

    for(; p->next != NULL; p = p->next) {
        ESP_LOGD(TAG_NODE, "| %6u | %16u | %16u | %16d | %16lu |", p->node.id_node, p->node.state, p->node.battery_low_detect, p->node.is_alive, (unsigned long)p->node.time);
    }

    ESP_LOGD(TAG_NODE, "| %6u | %16u | %16u | %16d | %16lu |", p->node.id_node, p->node.state, p->node.battery_low_detect, p->node.is_alive, (unsigned long)p->node.time);

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
                ESP_LOGI(TAG_NODE, "State: %u", msg.payload[0]);
                ESP_LOGI(TAG_NODE, "Battery low detect: %u", msg.payload[1]);
            break;
        }
    if(payload) {
        memcpy(msg.payload, payload, sizeof(status_node));
        ESP_LOGI(TAG_NODE, "State: %u", msg.payload[0]);
        ESP_LOGI(TAG_NODE, "Battery low detect: %u", msg.payload[1]);

        ESP_LOGI(TAG_NODE, "status_node size: %u", sizeof(status_node));
        ESP_LOGI(TAG_NODE, "a: %u", msg.payload[2]);
        ESP_LOGI(TAG_NODE, "b: %u", msg.payload[3]);
        ESP_LOGI(TAG_NODE, "c: %u", msg.payload[4]);
        ESP_LOGI(TAG_NODE, "d: %u", msg.payload[5]);
        ESP_LOGI(TAG_NODE, "e: %u", msg.payload[6]);
        ESP_LOGI(TAG_NODE, "f: %u", msg.payload[7]);
    }

    msg.crc = calc_crc16_msg((uint8_t *)&msg, sizeof(msg) - sizeof(msg.crc));

    ESP_LOGI(TAG_NODE, "Cmd: %u", msg.header.cmd);
    ESP_LOGI(TAG_NODE, "ID node: %u", msg.header.id_node);
    ESP_LOGI(TAG_NODE, "ID msg: %u", msg.header.id_msg);
    ESP_LOGI(TAG_NODE, "Name: %s", msg.name_node);
    ESP_LOGI(TAG_NODE, "Mac: %02X:%02X:%02X:%02X:%02X:%02X",
        msg.header.mac[0], msg.header.mac[1], msg.header.mac[2],
        msg.header.mac[3], msg.header.mac[4], msg.header.mac[5]);
    ESP_LOGI(TAG_NODE, "CRC16: %u", msg.crc);

    return msg;
}