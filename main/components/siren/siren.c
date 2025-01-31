#include <malloc.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "siren.h"

#define TAG_SIREN "SIREN"

static uint8_t counter_node_sirens = 0;

/* Return tail sirens list */
static struct node_sirens_list_t *get_tail_sirens_list(struct node_sirens_list_t *p) {

    if(!p)
        return p;

    while(p->next) {
        p = p->next;
    }

    return p;
}

/* Return head sirens list */
static struct node_sirens_list_t *get_head_sirens_list(struct node_sirens_list_t *p) {

    if(!p)
        return p;

    while(p->prev) {
        p = p->prev;
    }

    return p;
}

/* Return siren from list */
struct node_sirens_list_t *get_sirens_from_list(struct node_sirens_list_t *p, uint8_t id) {

    p = get_head_sirens_list(p);
    if (!p)
        return NULL;

    do {
        if(p->node.header.id_node == id) {
            return p;
        }

        p = p->next;

    } while(p->next != NULL);

    if(p->node.header.id_node == id) {
        return p;
    }

    return NULL;
}

/* Add node to list */
struct node_sirens_list_t *add_sirens_to_list(struct node_sirens_list_t *p, node_siren_msg_t pn) {

    p = get_tail_sirens_list(p);

    struct node_sirens_list_t *px = (struct node_sirens_list_t *)calloc(1, sizeof(struct node_sirens_list_t));
    if(!px)
        return NULL;

    memcpy(&(px->node), &pn, sizeof(node_siren_msg_t));
    px->next = NULL;
    
    if(!p){
        px->prev = NULL;
    } else {
        px->prev = p;
        p->next = px;
    }

    p = px;
    counter_node_sirens++;

    return p;
}

/* Delete siren from list */
struct node_sirens_list_t *del_sirens_from_list(struct node_sirens_list_t *p, uint8_t id) {

    struct node_sirens_list_t *p1 = NULL;
    struct node_sirens_list_t *p2 = NULL;
    struct node_sirens_list_t *p0 = NULL;

    p0 = get_sirens_from_list(p, id);
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

    counter_node_sirens--;

    if(p2)
        return get_tail_sirens_list(p2);
    else if(p1)
        return get_tail_sirens_list(p1);

    return NULL;
}

/* Update siren in to list */
struct node_sirens_list_t *update_sirens_to_list(struct node_sirens_list_t *p, node_siren_msg_t pn) {

    p = get_sirens_from_list(p, pn.header.id_node);
    if (!p)
        return p;

    memcpy(p, &pn, sizeof(pn));

    return p;
}

/* Return number of sirens inside list */
uint8_t get_num_sirens_from_list() {

    return counter_node_sirens;
}

/* Print sirens list */
void print_sirens_list(struct node_sirens_list_t *p) {

    ESP_LOGI(TAG_SIREN, "|-----------------------------------------------|");
    ESP_LOGI(TAG_SIREN, "|   ID   |   State siren   |   Status battery   |");
    ESP_LOGI(TAG_SIREN, "|-----------------------------------------------|");

    p = get_head_sirens_list(p);
    if(!p) {
        ESP_LOGW(TAG_SIREN, "No alarm sensor available");
        return;
    }

    for(; p->next != NULL; p = p->next){
        ESP_LOGI(TAG_SIREN, "| %6u | %16u | %16u |", p->node.header.id_node, p->node.state, p->node.battery_low_detect);
    }

    ESP_LOGI(TAG_SIREN, "| %6u | %16u | %16u |", p->node.header.id_node, p->node.state, p->node.battery_low_detect);

    return;
}

/* Build siren request message */
node_siren_msg_t build_request_siren_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[], enum cmd_type cmd) {

    node_siren_msg_t req;
    memset(&req, 0, sizeof(req));

    req.header = build_header_msg(REQUEST, SIREN, id_node, id_msg, mac, cmd);
    req.crc = calc_crc16((uint8_t *)&req, sizeof(req) - sizeof(req.crc));

    return req;
}

/* Build siren ack response message */
node_siren_msg_t build_response_ack_siren_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]) {

    node_siren_msg_t resp;
    memset(&resp, 0, sizeof(resp));

    resp.header = build_header_msg(RESPONSE, SIREN, id_node, id_msg, mac, ACK);
    resp.crc = calc_crc16((uint8_t *)&resp, sizeof(resp) - sizeof(resp.crc));

    return resp;
}

/* Build siren nack response message */
node_siren_msg_t build_response_nack_siren_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]) {

    node_siren_msg_t resp;
    memset(&resp, 0, sizeof(resp));

    resp.header = build_header_msg(RESPONSE, SIREN, id_node, id_msg, mac, NACK);
    resp.crc = calc_crc16((uint8_t *)&resp, sizeof(resp) - sizeof(resp.crc));

    return resp;
}
