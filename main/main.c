#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_sntp.h"

#include "common.h"
#include "wifi.h"
#include "node_array.h"
#include "mqtt.h"
#include "ethernet.h"
#include "conf.h"

#define TAG_MAIN               "MAIN"
#define TAG_MAIN_SEND          "MAIN_SEND"
#define TAG_MAIN_RECEIVE       "MAIN_RECEIVE"

/* IDs device */
#define ID_GATEWAY             0x78
#define NODE_QUEUE_SIZE        10
#define ESPNOW_WIFI_CHANNEL    7
#define NUM_RETRASMISSION_TIME 3
#define CYCLE_TIME_S           60
#define SLOT_DURATION_S        10
#define MAX_NUMBER_NODE        10
#define NUMBER_ATTEMPTS        3
#define RETRASMISSION_TIME_MS  50
#define DATA_SENT_SUCCESS      (1 << 0)
#define DATA_SENT_FAILED       (1 << 1)
#define DATA_RECEIVED_SUCCESS  (1 << 2)
#define DATA_RECEIVED_FAILED   (1 << 3)
#define ACTIVE_DEACTIVE_ALARM  (1 << 2)

static uint8_t mac_wifi[MAC_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static struct node_list_t *node_list = NULL;

static QueueHandle_t receive_queue;
static QueueHandle_t send_queue;
static SemaphoreHandle_t pending_msg_mutex;
static EventGroupHandle_t xEventGroupAlarm;

static time_t target_time;

/* Define pending message struct */
struct pending_msg {
    uint8_t id_device;
    uint8_t cmd_type;
    uint8_t id_msg;
    time_t time;
    struct pending_msg *next;
} __attribute__((__packed__));

static struct pending_msg *p_msg_head = NULL;
static struct pending_msg *p_msg_tail = NULL;

/* Add pending message */
static void add_pending_msg(uint8_t id_device, uint8_t cmd_type, uint8_t id_msg) {

    struct pending_msg *p = calloc(1, sizeof(struct pending_msg));
    if (!p)
        return;

    p->id_device = id_device;
    p->cmd_type = cmd_type;
    p->id_msg = id_msg;
    p->time = time(NULL);
    p->next = NULL;

    if (xSemaphoreTake(pending_msg_mutex, portMAX_DELAY)) {
        if (!p_msg_tail) {
            p_msg_head = p_msg_tail = p;
        } else {
            p_msg_tail->next = p;
            p_msg_tail = p;
        }
        xSemaphoreGive(pending_msg_mutex);
    }

    return;
}

/* Remove peding message from list */
static void remove_pending_msg(uint8_t id_device, uint8_t cmd_type, uint8_t id_msg) {

    if (xSemaphoreTake(pending_msg_mutex, portMAX_DELAY)) {
        struct pending_msg *current = p_msg_head;
        struct pending_msg *prev = NULL;

        while (current) {
            if (current->id_device == id_device && current->cmd_type == cmd_type && current->id_msg == id_msg) {

                if (!prev) {
                    p_msg_head = current->next;
                    if (p_msg_tail == current)
                        p_msg_tail = NULL;
                } else {
                    prev->next = current->next;
                    if (p_msg_tail == current)
                        p_msg_tail = prev;
                }

                free(current);
                break;
            }

            prev = current;
            current = current->next;
        }

        xSemaphoreGive(pending_msg_mutex);
    }

    return;
}

/* Send callback function */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    if (!mac_addr) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address not valid");
        return;
    }

    if(status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG_MAIN, "Message sent correctly");
        xEventGroupSetBits(xEventGroupAlarm, DATA_SENT_SUCCESS);
    } else if (status == ESP_NOW_SEND_FAIL){
        ESP_LOGE(TAG_MAIN, "Message not sent correctly");
        xEventGroupClearBits(xEventGroupAlarm, DATA_SENT_FAILED);
    }

    return;
}

/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    if (!recv_info->src_addr || !data || len <= 0) {
        ESP_LOGE(TAG_MAIN, "Receive callback args error. Discard message");
        return;
    }

    ESP_LOGD(TAG_MAIN, "Receive message from sensor or siren");

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xQueueSendFromISR(receive_queue, data, &xHigherPriorityTaskWoken) != pdTRUE) {
        ESP_LOGW(TAG_MAIN, "Warning, queue is full, discard message");
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

    return;
}

/* Send message by espnow protocol */
static bool send_message(uint8_t dst_mac[], node_msg_t msg) {

    esp_err_t err = ESP_FAIL;
    uint8_t num_tentative = NUMBER_ATTEMPTS;
    EventBits_t uxBits;

    xEventGroupClearBits(xEventGroupAlarm, DATA_SENT_SUCCESS | DATA_SENT_FAILED);
    do {
        /* Send packet */
        err = esp_now_send(dst_mac, (uint8_t *)&msg, sizeof(msg));
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error send: %s", esp_err_to_name(err));
            num_tentative = 0;
        }
        /* Wait until the data is sent */
        uxBits = xEventGroupWaitBits(xEventGroupAlarm, DATA_SENT_SUCCESS | DATA_SENT_FAILED, pdTRUE, pdFALSE, portMAX_DELAY);
        num_tentative--;
        vTaskDelay(pdMS_TO_TICKS(RETRASMISSION_TIME_MS));
    }
    while((uxBits & DATA_SENT_FAILED) && num_tentative > 0);

    if(!num_tentative)
        return false;

    return true;
}

/* Print hours */
static void print_hour(time_t t) {

    struct tm *local_time = localtime(&t);

    if (!local_time) {
        ESP_LOGE(TAG_MAIN, "Error, hour not converted");
        return;
    }

    ESP_LOGI(TAG_MAIN, "Hour: %02d:%02d:%02d", local_time->tm_hour, local_time->tm_min, local_time->tm_sec);

    return;
}

/* Print message */
static void print_msg(node_msg_t node_msg) {

    ESP_LOGI(TAG_MAIN, "Cmd: %u", node_msg.header.cmd);
    ESP_LOGI(TAG_MAIN, "Node type: %u", node_msg.header.node);
    ESP_LOGI(TAG_MAIN, "Mac: %02X:%02X:%02X:%02X:%02X:%02X", node_msg.header.mac[0], node_msg.header.mac[1], node_msg.header.mac[2], node_msg.header.mac[3], node_msg.header.mac[4], node_msg.header.mac[5]);
    ESP_LOGI(TAG_MAIN, "ID node: %u", node_msg.header.id_node);
    ESP_LOGI(TAG_MAIN, "ID msg: %u", node_msg.header.id_msg);
    ESP_LOGI(TAG_MAIN, "Name: %s", node_msg.name_node);

    switch(node_msg.header.cmd) {
        case ADD:
            for(uint8_t i = 0; i < 8; i++) {
                ESP_LOGI(TAG_MAIN, "Payload [%u]: %u", i, node_msg.payload[i]);
            }
        break;
        case UPDATE:
        case SYNC:
            ESP_LOGI(TAG_MAIN, "State: %u", node_msg.payload[0]);
            ESP_LOGI(TAG_MAIN, "Battery low detect: %u", node_msg.payload[1]);
        break;
    }

    ESP_LOGI(TAG_MAIN, "CRC16: %d", node_msg.crc);

    return;
}

/* Run receive task */
void receive_task(void *arg) {

    static esp_err_t err = ESP_FAIL;
    static esp_now_peer_info_t peer;
    static node_msg_t node_msg;

    while(1) {
        memset(&node_msg, 0, sizeof(node_msg_t));
        if(xQueueReceive(receive_queue, &node_msg, portMAX_DELAY) == pdTRUE) {

            if(calc_crc16_msg((uint8_t *)&node_msg, sizeof(node_msg) - sizeof(uint16_t)) != node_msg.crc) {
                ESP_LOGW(TAG_MAIN_RECEIVE, "Warning, crc16 not correct discard message");
                continue;
            }

            switch(node_msg.header.cmd) {
                case ADD:
                    ESP_LOGD(TAG_MAIN_RECEIVE, "Receive add message");
                    if(!get_node_from_list(node_list, node_msg.header.id_node)) {
                        ESP_LOGI(TAG_MAIN_RECEIVE, "Add new sensor or siren with device id: %u", node_msg.header.id_node);

                        /* Add new peer to list */
                        memset(&peer, 0, sizeof(esp_now_peer_info_t));
                        peer.channel = ESPNOW_WIFI_CHANNEL;
                        peer.ifidx = WIFI_IF_STA;
                        peer.encrypt = false;
                        memcpy(peer.peer_addr, node_msg.header.mac, MAC_SIZE);

                        err = esp_now_add_peer(&peer);
                        if(err != ESP_OK) {
                            ESP_LOGE(TAG_MAIN_RECEIVE, "Error, peer node not added");
                        } else {
                            ESP_LOGI(TAG_MAIN_RECEIVE, "Peer node added correctly");
                            /* Adds sensor to list */
                            node_list = add_node_to_list(node_list, node_msg);

                            /* Send response to node */
                            time_t t;
                            time(&t);
                            node_msg = build_node_msg(ADD, ID_GATEWAY, GATEWAY, (esp_random() % 256), mac_wifi, "gateway", &t);
                            if(xQueueSend(send_queue, &node_msg, pdMS_TO_TICKS(50)) != pdTRUE) {
                                ESP_LOGE(TAG_MAIN_RECEIVE, "Error, queue full discard message");
                            }
                        }
                    } else {
                        ESP_LOGW(TAG_MAIN_RECEIVE, "Warning, node device id %u already exist. Skip", node_msg.header.id_node);
                    }
                break;
                case DEL:
                    ESP_LOGD(TAG_MAIN_RECEIVE, "Receive delete message");
                break;
                case SYNC:
                    ESP_LOGD(TAG_MAIN_RECEIVE, "Receive sync message response");
                    /* Receive response from node */
                    remove_pending_msg(node_msg.header.id_node, node_msg.header.cmd, node_msg.header.id_msg);
                break;
                case UPDATE:
                    ESP_LOGD(TAG_MAIN_RECEIVE, "Receive update message response");

                    /* Receive response from node */
                    if(update_node_to_list(node_list, node_msg)) {
                        ESP_LOGD(TAG_MAIN_RECEIVE, "Update state node id: %u inside list", node_msg.header.id_node);
                        remove_pending_msg(node_msg.header.id_node, node_msg.header.cmd, node_msg.header.id_msg);
                    } else {
                        ESP_LOGW(TAG_MAIN_RECEIVE, "Warning, node id %u not registered", node_msg.header.id_node);
                    }
                break;
                case ACTIVE_ALARM:
                    ESP_LOGD(TAG_MAIN_RECEIVE, "Active alarm");
                break;
                case DEACTIVE_ALARM:
                    ESP_LOGD(TAG_MAIN_RECEIVE, "Deactive alarm");
                    /* Stop siren if running */
                    node_msg = build_node_msg(STOP_SIREN, ID_GATEWAY, GATEWAY, (esp_random() % 256), mac_wifi, "gateway", NULL);
                    if(xQueueSend(send_queue, &node_msg, pdMS_TO_TICKS(50)) == pdPASS) {
                        ESP_LOGE(TAG_MAIN, "Error, queue full discard message");
                    }
                break;
                case ALARM:
                    status_node sn;
                    memcpy(&sn, node_msg.payload, sizeof(status_node));
                    ESP_LOGI(TAG_MAIN_RECEIVE, "State %u for id sensor: %u", sn.state, node_msg.header.id_node);
                    ESP_LOGI(TAG_MAIN_RECEIVE, "Battery low detect: %u for id sensor: %u", sn.battery_low_detect, node_msg.header.id_node);

                    if(sn.state && (xEventGroupWaitBits(xEventGroupAlarm, ACTIVE_DEACTIVE_ALARM, pdFALSE, pdFALSE, 0) & ACTIVE_DEACTIVE_ALARM)) {
                        ESP_LOGI(TAG_MAIN_RECEIVE, "Send command for start siren");
                        node_msg = build_node_msg(START_SIREN, ID_GATEWAY, GATEWAY, (esp_random() % 256), mac_wifi, "gateway", NULL);
                        if(xQueueSend(send_queue, &node_msg, pdMS_TO_TICKS(50)) == pdPASS) {
                            ESP_LOGE(TAG_MAIN_RECEIVE, "Error, queue full discard message");
                        }
                    }
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return;
}

/* Run send task */
void send_task(void *arg) {

    static uint32_t slot_start = 0;
    static uint32_t slot_end = 0;
    static time_t now = 0;
    static uint64_t current_cycle = 0; // (now - target_time) / CYCLE_TIME_S;
    static uint64_t slot_time = 0;     // (now - target_time) % CYCLE_TIME_S;
    static node_msg_t msg;

    ESP_LOGI(TAG_MAIN_SEND, "Real time: %lld", target_time);

    while(1) {

        time(&now);
        if ((now - target_time) >= 240) { // 4 minutes
            ESP_LOGI(TAG_MAIN_SEND, "Attention, reset cicle");
            target_time = now;
        }

        current_cycle = (now - target_time) / CYCLE_TIME_S;
        slot_time = (now - target_time) % CYCLE_TIME_S;     // Verifico che nodo devo sincronizzare
        
        ESP_LOGI(TAG_MAIN_SEND, "Slot number: %llu", slot_time);
        ESP_LOGI(TAG_MAIN_SEND, "Current cycle: %llu", current_cycle);

        node_list = get_first_node_from_list(node_list);
        while (node_list != NULL) {

            if (node_list->node.id_node == 0) {
                node_list = node_list->next;
                continue;
            }

            slot_start = node_list->node.id_node * SLOT_DURATION_S;
            slot_end = slot_start + SLOT_DURATION_S;

            /* Sync and update */
            if (slot_time >= slot_start && slot_time < slot_end) {
                if (node_list->node.last_sync_cycle != current_cycle) {
                    uint8_t id_msg = (esp_random() % 256);
                    msg = build_node_msg(UPDATE, ID_GATEWAY, GATEWAY, id_msg, mac_wifi, "gateway", NULL);
                    if (!send_message(node_list->node.mac, msg)) {
                        ESP_LOGE(TAG_MAIN_SEND, "Error, sync to node id %u failed", node_list->node.id_node);
                    } else {
                        /* Add pending message to list */
                        add_pending_msg(node_list->node.id_node, UPDATE, id_msg);
                        ESP_LOGD(TAG_MAIN_SEND, "Sent update message to node id %u in cycle %llu", node_list->node.id_node, current_cycle);
                        node_list->node.last_sync_cycle = current_cycle;
                    }
                }
            }
        
            node_list = node_list->next;
        }

        /* Receive message from send queue */
        if(xQueueReceive(send_queue, &msg, portMAX_DELAY) == pdTRUE) {
            /* Check send message to node or siren */
            node_list = get_node_from_list(node_list, msg.header.id_node);
            if(!send_message(node_list->node.mac, msg)) {
                ESP_LOGE(TAG_MAIN_SEND, "Error, message not send to node");
            } else {
                /* Add pending message to list */
                add_pending_msg(msg.header.id_node, msg.header.cmd, msg.header.id_msg);
                ESP_LOGD(TAG_MAIN_SEND, "Add pending message to list");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return;
}

/* Main program */
void app_main(void) {

    esp_err_t err = ESP_FAIL;

    /* Init nvs flash */
    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, flash not init");

        /* Erase flash */
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            ESP_LOGD(TAG_MAIN, "Error, flash not erase");
            return;
        }
        nvs_flash_init();
    }

    /* Initialize stack */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, network interface WiFi not init");
        return;
    }

    esp_event_loop_create_default();

    /* Read MAC WiFi address */
    err = esp_read_mac(mac_wifi, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, MAC address WiFi not read");
        return;
    }

    /* Init ethernet */
    err = init_eth();
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, Ethernet not init");
        return;
    }

    /* Init wifi station */
    err = init_wifi_sta();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, WiFi not init");
        return;
    }

    /* Init espnow protocol */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, espnow not init");
        return;
    }

    /* Create event group alarm */
    xEventGroupAlarm = xEventGroupCreate();
    if (!xEventGroupAlarm) {
        ESP_LOGD(TAG_MAIN, "Error, event group not created");
        return;
    }

    /* Create pending mutex for messages list */
    pending_msg_mutex = xSemaphoreCreateMutex();
    if(!pending_msg_mutex) {
        ESP_LOGD(TAG_MAIN, "Error, mutex not allocted");
        return;
    }

    /* Create send queue */
    send_queue = xQueueCreate(NODE_QUEUE_SIZE, sizeof(node_msg_t));
    if(!send_queue) {
        ESP_LOGD(TAG_MAIN, "Error, send queue not allocated");
        return;
    }

    /* Create receive queue */
    receive_queue = xQueueCreate(NODE_QUEUE_SIZE, sizeof(node_msg_t));
    if(!receive_queue) {
        ESP_LOGD(TAG_MAIN, "Error, receive queue not allocated");
        return;
    }

    /* Register send callback function */
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, send callback function no registered");
        return;
    }

    /* Register receive callback function */
    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, receive callback function no registered");
        return;
    }

    /* Init mqtt service */
    // init_mqtt();

    /* Start receive task */
    if(xTaskCreatePinnedToCore(receive_task, "receive_task", 1024 * 3, NULL, 0, NULL, 0) != pdPASS) {
        ESP_LOGD(TAG_MAIN, "Error, receive task not started");
        return;
    }

    /* Start send task */
    if(xTaskCreatePinnedToCore(send_task, "send_task", 1024 * 3, NULL, 0, NULL, 1) != pdPASS) {
        ESP_LOGD(TAG_MAIN, "Error, send task not started");
        return;
    }

    return;
}
