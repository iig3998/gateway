#include <malloc.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "sensor.h"

#define TAG_SENSOR "SENSOR"

static uint8_t counter_node_sensors = 0;

/* Return tail sensors list */
static struct node_sensors_list_t *get_tail_sensors_list(struct node_sensors_list_t *p) {

    if(!p)
        return p;

    while(p->next) {
        p = p->next;
    }

    return p;
}

/* Return head sensors list */
static struct node_sensors_list_t *get_head_sensors_list(struct node_sensors_list_t *p) {

    if(!p)
        return p;

    while(p->prev) {
        p = p->prev;
    }

    return p;
}

/* Return sensor from list */
struct node_sensors_list_t *get_sensors_from_list(struct node_sensors_list_t *p, uint8_t id) {

    p = get_head_sensors_list(p);
    if (!p)
        return NULL;

    do {
        if(p->node.id_node == id) {
            return p;
        }

        p = p->next;

    } while(p->next != NULL);

    if(p->node.id_node == id) {
        return p;
    }

    return NULL;
}

/* Add sensor to list */
struct node_sensors_list_t *add_sensors_to_list(struct node_sensors_list_t *p, node_sensor_msg_t pn) {

    p = get_tail_sensors_list(p);

    struct node_sensors_list_t *px = (struct node_sensors_list_t *)calloc(1, sizeof(struct node_sensors_list_t));
    if(!px)
        return NULL;

    px->node.id_node = pn.header.id_node;
    memcpy(px->node.mac, pn.header.mac, MAC_SIZE);

    px->node.state = pn.state;
    px->node.battery_low_detect = pn.battery_low_detect;

    px->next = NULL;

    if(!p){
        px->prev = NULL;
    } else {
        px->prev = p;
        p->next = px;
    }

    p = px;
    counter_node_sensors++;

    return p;
}

/* Delete sensor from list */
struct node_sensors_list_t *del_sensors_from_list(struct node_sensors_list_t *p, uint8_t id) {

    struct node_sensors_list_t *p1 = NULL;
    struct node_sensors_list_t *p2 = NULL;
    struct node_sensors_list_t *p0 = NULL;

    p0 = get_sensors_from_list(p, id);
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

    counter_node_sensors--;

    if(p2)
        return get_tail_sensors_list(p2);
    else if(p1)
        return get_tail_sensors_list(p1);

    return NULL;
}

/* Update node sensor */
struct node_sensors_list_t *update_sensors_to_list(struct node_sensors_list_t *p, node_sensor_msg_t pn) {

    p = get_sensors_from_list(p, pn.header.id_node);
    if (!p)
        return p;

    memcpy(p, &pn, sizeof(pn));

    return p;
}

/* Return number of node inside list */
uint8_t get_num_sensors_from_list() {

    return counter_node_sensors;
}

/* Print node id list */
void print_sensors_list(struct node_sensors_list_t *p) {

    ESP_LOGI(TAG_SENSOR, "|-----------------------------------------------------|");
    ESP_LOGI(TAG_SENSOR, "|   ID   |   State reed switch   |   Status battery   |");
    ESP_LOGI(TAG_SENSOR, "|-----------------------------------------------------|");

    p = get_head_sensors_list(p);
    if(!p) {
        ESP_LOGW(TAG_SENSOR, "No alarm sensor available");
        return;
    }

    for(; p->next != NULL; p = p->next){
        ESP_LOGI(TAG_SENSOR, "| %6u | %16u | %16u |", p->node.id_node, p->node.state, p->node.battery_low_detect);
    }

    ESP_LOGI(TAG_SENSOR, "| %6u | %16u | %16u |", p->node.id_node, p->node.state, p->node.battery_low_detect);

    return;
}

/* Build response ack sensor message */
node_sensor_msg_t build_response_ack_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]) {

    node_sensor_msg_t resp;
    memset(&resp, 0, sizeof(resp));

    resp.header = build_header_msg(RESPONSE, SENSOR, id_node, id_msg, mac, ACK);
    resp.crc = calc_crc16((uint8_t *)&resp, sizeof(resp) - sizeof(resp.crc));

    return resp;
}

/* Build response nack sensor message */
node_sensor_msg_t build_response_nack_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]) {

    node_sensor_msg_t resp;
    memset(&resp, 0, sizeof(resp));

    resp.header = build_header_msg(RESPONSE, SENSOR, id_node, id_msg, mac, NACK);
    resp.crc = calc_crc16((uint8_t *)&resp, sizeof(resp) - sizeof(resp.crc));

    return resp;
}