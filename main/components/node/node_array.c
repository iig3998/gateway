#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "node_array.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define NODE_SIZE 20

#define TAG_NODE "NODE_ARRAY"

static uint8_t counter_node = 0;
static node_t nl[NODE_SIZE] = {0};
static SemaphoreHandle_t xNodeListMutex = NULL;

/* Init the node array mutex */
bool init_node_list_mutex(void) {

    xNodeListMutex = xSemaphoreCreateMutex();
    if (!xNodeListMutex) {
        ESP_LOGE(TAG_NODE, "Error, node mutex not allocated");
        return false;
    }

    return true;
}

/* Return node from list */
bool check_node_inside_list(uint8_t id) {

    bool result = false;

    if (!id || id > NODE_SIZE)
        return result;

    if (xSemaphoreTake(xNodeListMutex, portMAX_DELAY)) {
        result = (nl[id - 1].id_node != 0);
        xSemaphoreGive(xNodeListMutex);
    }

    return result;
}

/* Get node from list */
node_t get_node_from_list(uint8_t id) {

    node_t node;
    memset(&node, 0, sizeof(node_t));

    if (xSemaphoreTake(xNodeListMutex, portMAX_DELAY)) {
        memcpy(&node, &nl[id - 1], sizeof(node));
        xSemaphoreGive(xNodeListMutex);
    }

    return node;
}

/* Add node to list */
void add_node_to_list(node_msg_t msg) {

    if (xSemaphoreTake(xNodeListMutex, portMAX_DELAY)) {
        status_node sn;

        uint8_t index = msg.header.id_node - 1;
        memcpy(&nl[index].mac, msg.header.mac, 6);

        nl[index].is_alive = true;
        nl[index].id_node = msg.header.id_node;

        memcpy(&sn, msg.payload, sizeof(status_node));
        nl[index].battery_low_detect = sn.battery_low_detect;
        nl[index].state = sn.state;
        nl[index].time = time(NULL);

        counter_node++;

        xSemaphoreGive(xNodeListMutex);
    }

    return;
}

/* Delete node from list */
void del_node_from_list(uint8_t id) {

    if (xSemaphoreTake(xNodeListMutex, portMAX_DELAY)) {
        memset(&nl[id - 1], 0, sizeof(node_t));
        counter_node--;
        xSemaphoreGive(xNodeListMutex);
    }

    return;
}

/* Update node */
void update_node_to_list(node_msg_t msg) {

    if (xSemaphoreTake(xNodeListMutex, portMAX_DELAY)) {
        status_node sn;
        uint8_t index = msg.header.id_node - 1;

        nl[index].is_alive = true;

        memcpy(&sn, msg.payload, sizeof(status_node));
        nl[index].battery_low_detect = sn.battery_low_detect;
        nl[index].state = sn.state;
        nl[index].time = time(NULL);

        xSemaphoreGive(xNodeListMutex);
    }

    return;
}

/* Return number of node inside list */
uint8_t get_num_node_from_list() {

    uint8_t num = 0;

    if (xSemaphoreTake(xNodeListMutex, portMAX_DELAY)) {
        num = counter_node;
        xSemaphoreGive(xNodeListMutex);
    }

    return num;
}

/* Print node id list */
void print_node_list() {

    if (xSemaphoreTake(xNodeListMutex, portMAX_DELAY)) {
        ESP_LOGI(TAG_NODE, "|---------------------------------------------------------------------------------|");
        ESP_LOGI(TAG_NODE, "|   ID   |  State reed switch  |  Status battery  |  Is alive  | Time last update |");
        ESP_LOGI(TAG_NODE, "|---------------------------------------------------------------------------------|");

        for (uint8_t i = 0; i < 10; i++) {
            if (nl[i-1].id_node) {
                ESP_LOGI(TAG_NODE, "| %6u | %19u | %19u | %13d | %13lld |", nl[i-1].id_node, nl[i-1].state, nl[i-1].battery_low_detect, nl[i-1].is_alive, nl[i-1].time);
            }
        }

        xSemaphoreGive(xNodeListMutex);
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
        memcpy(msg.payload, payload, sizeof(status_node));
        ESP_LOGI(TAG_NODE, "State: %u", msg.payload[0]);
        ESP_LOGI(TAG_NODE, "Battery low detect: %u", msg.payload[1]);
    }

    msg.crc = calc_crc16_msg((uint8_t *)&msg, sizeof(msg) - sizeof(msg.crc));

    return msg;
}
