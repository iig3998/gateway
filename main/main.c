#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

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

#include "common.h"
#include "wifi.h"

#include "sensor.h"
#include "siren.h"
#include "keyboard.h"
#include "gateway.h"
#include "ethernet.h"

#define TAG_MAIN               "MAIN"
#define ID_GATEWAY             0x78
#define SENSOR_QUEUE_SIZE      20
#define SIREN_QUEUE_SIZE       5
#define KEYBOARD_QUEUE_SIZE    5
#define ESPNOW_WIFI_CHANNEL    7
#define NUM_RETRASMISSION_TIME 3

/* Define events */
#define DATA_SENT             (1 << 0)
#define DATA_RECEIVED         (1 << 1)

EventGroupHandle_t xEventGroupAlarm;

static bool active_alarm = false;

static uint8_t mac[6];
static const uint8_t dest_mac[6];

static QueueHandle_t node_sensor_queue;
static QueueHandle_t node_siren_queue;
static QueueHandle_t node_keyboard_queue;

/* Send callback function */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    if (!mac_addr) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address not valid");
        return;
    }

    if(status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG_MAIN, "Data sent correctly");
    } else if (status == ESP_NOW_SEND_FAIL){
        ESP_LOGE(TAG_MAIN, "Data not sent correctly");
    }

    xEventGroupClearBits(xEventGroupAlarm, DATA_SENT);

    return;
}

/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    enum node_type n_type = data[0];

    if (!recv_info->src_addr || !data || len <= 0) {
        ESP_LOGE(TAG_MAIN, "Receive callback args error");
        return;
    }

    switch (n_type) {
        case SENSOR:
            ESP_LOGI(TAG_MAIN, "Sensor node type");
            node_sensor_msg_t msg_sensor;
            memcpy(&msg_sensor, data, sizeof(msg_sensor));
            if(xQueueSend(node_sensor_queue, &msg_sensor, portMAX_DELAY) != pdTRUE) {
                ESP_LOGI(TAG_MAIN, "Data node sensor insert correctly to queue");
            }
        break;

        case SIREN:
            ESP_LOGI(TAG_MAIN, "Siren node type");
            node_siren_response_t node_siren;
            memcpy(&node_siren, data, sizeof(node_siren));
            if(xQueueSend(node_siren_queue, &node_siren, portMAX_DELAY) != pdTRUE) {
                ESP_LOGI(TAG_MAIN, "Data node siren insert correctly to queue");
            }
        break;

        default:
            ESP_LOGW(TAG_MAIN, "Warning, node not identified");
        break;
    }

    return;
}

/* Sent message */
static void send_message(uint8_t *mac, void *msg) {

    esp_err_t err = ESP_FAIL;
    uint8_t num_retrasmission_time = NUM_RETRASMISSION_TIME;

    xEventGroupSetBits(xEventGroupAlarm, DATA_SENT_STATUS);

    do {
        err = esp_now_send(mac, (uint8_t *)&msg, sizeof(msg));
        if(err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, data not send");
        }
        num_retrasmission_time --;
    }
    while(xEventGroupWaitBits(xEventGroupAlarm, DATA_SENT_STATUS, pdTRUE, pdFALSE, portMAX_DELAY) && num_retrasmission_time > 0);

    if(!num_retrasmission_time) {
        ESP_LOGW(TAG_MAIN, "Warning, trasmission error");
    }

    return;
}

/* Run sensor task */
void sensor_task(void *arg) {

    ESP_LOGI(TAG_MAIN, "Start alarm task");

    esp_err_t err = ESP_FAIL;
    uint16_t crc = 0;
    esp_now_peer_num_t num_peer;
    esp_now_peer_info_t peer;
    node_sensor_msg_t msg_sensor;
    node_gateway_msg_t msg_gateway;
    struct node_sensors_list_t *node_sesnors_list = NULL;

    memset(&msg_sensor, 0, sizeof(msg_sensor));

    while(1) {

        if (xQueueReceive(node_sensor_queue, &msg_sensor, portMAX_DELAY) == pdTRUE) {

            ESP_LOGI(TAG_MAIN, "Receive message from sensor id: %u", msg_sensor.header.id_node);

            crc = calc_crc16((uint8_t *)&msg_sensor, sizeof(msg_sensor) - sizeof(crc));
            ESP_LOGI(TAG_MAIN, "CRC16 calculated: %u", crc);
            if (crc != msg_sensor.crc) {
                ESP_LOGI(TAG_MAIN, "Warning, crc16 correct discard message");
                msg_gateway = build_response_nack_gateway_msg(ID_GATEWAY, msg_sensor.header.id_msg, mac);
            } else {
                ESP_LOGI(TAG_MAIN, "Success, crc16 correct");
                msg_gateway = build_response_ack_gateway_msg(ID_GATEWAY, msg_sensor.header.id_msg, mac);
                switch(msg_sensor.header.msg) {

                    case REQUEST:
                        switch (msg_sensor.header.cmd) {

                            case ADD:
                                if(!get_sensors_from_list(node_sesnors_list, msg_sensor.header.id_node)) {
                                    ESP_LOGI(TAG_MAIN, "Add new device");
                                    node_sesnors_list = add_sensors_to_list(node_sesnors_list, msg_sensor);

                                    /* Add new peer to list */
                                    memset(&peer, 0, sizeof(esp_now_peer_info_t));
                                    peer.channel = ESPNOW_WIFI_CHANNEL;
                                    peer.ifidx = WIFI_IF_STA;
                                    peer.encrypt = false;

                                    err = esp_now_add_peer(&peer);
                                    if(err != ESP_OK){
                                        ESP_LOGE(TAG_MAIN, "Error, peer not added");
                                    }

                                    err = esp_now_get_peer_num(&num_peer);
                                    if(err == ESP_OK){
                                        ESP_LOGI(TAG_MAIN, "There are %d nodes registered", num_peer.total_num);
                                    } else {
                                        ESP_LOGI(TAG_MAIN, "Number nodes not available");
                                    }
                                } else {
                                    ESP_LOGW(TAG_MAIN, "Warning, device id: %u already exist. Skip", msg_sensor.header.id_node);
                                }
                            break;

                            case UPDATE:
                                if(get_sensors_from_list(node_sesnors_list, msg_sensor.header.id_node)) {
                                    ESP_LOGI(TAG_MAIN, "Update state sensor inside list");
                                    if(msg_sensor.state && active_alarm) {
                                        ESP_LOGI(TAG_MAIN, "Send command for start siren");
                                    }
                                } else {
                                    ESP_LOGW(TAG_MAIN, "Warning, sensor id %u not registered", msg_sensor.header.id_node);
                                }
                            break;
                            
                            case DEL:
                                ESP_LOGI(TAG_MAIN, "Delete command not used");
                            break;

                            default:
                                ESP_LOGW(TAG_MAIN, "Warning, command not valid");
                            break;
                        }
                    break;
                    case RESPONSE:
                    /* Delete timer */

                    break;
                }
            }

        }

        /* Set delay for 50 ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return;
}

/* Alarm siren task */
void siren_task(void *arg) {

    esp_err_t err = ESP_FAIL;
    uint16_t crc = 0;
    esp_now_peer_info_t peer;
    node_siren_request_t node_siren;

    memset(&node_siren, 0, sizeof(node_siren));

    while(1) {

        if (xQueueReceive(node_siren_queue, &node_siren, portMAX_DELAY) == pdTRUE) {

            ESP_LOGI(TAG_MAIN, "Receive message from siren");

            crc = calc_crc16((uint8_t *)&node_siren, sizeof(node_siren) - sizeof(crc));
            ESP_LOGI(TAG_MAIN, "CRC16 calculate: %u", crc);

            if (crc != node_siren.crc) {
                ESP_LOGI(TAG_MAIN, "Warning, crc16 correct discard message");
            } else {
                ESP_LOGI(TAG_MAIN, "Success, crc16 correct ");
                switch (node_siren.header.cmd) { 

                    case ADD:
                        ESP_LOGI(TAG_MAIN, "Add siren alarm");
                        memset(&peer, 0, sizeof(esp_now_peer_info_t));
                        peer.channel = ESPNOW_WIFI_CHANNEL;
                        peer.ifidx = WIFI_IF_STA;
                        peer.encrypt = false;

                        memcpy(peer.peer_addr, node_siren.header.mac, ESP_NOW_ETH_ALEN);
                        err = esp_now_add_peer(&peer);
                        if(err != ESP_OK){
                            ESP_LOGE(TAG_MAIN, "Error, peer not added");
                        }
                    break;
        
                    case UPDATE:
                        ESP_LOGI(TAG_MAIN, "Update status siren");

                    break;
                    
                    case DEL:
                        ESP_LOGI(TAG_MAIN, "Del command not used");
                        /* TODO */
                    break;

                    default:
                        ESP_LOGW(TAG_MAIN, "Warning, command not valid");
                    break;

                }
            }
        }

        /* Set delay for 50 ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return;
}

// ack_timer = xTimerCreate("AckTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, ack_timeout_callback);

/* Alarm keyboard task */
void keyboard_task(void *arg) {

    while(1) {
    }

    return;
}

//esp_err_t esp_wifi_config_espnow_rate(wifi_interface_t ifx, wifi_phy_rate_t rate)

/* Main program */
void app_main(void) {

    esp_err_t err = ESP_FAIL;
    esp_now_peer_info_t peer;

    /* Init nvs flash */
    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, flash not init");

        /* Erase flash */
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, flash not erase");
            return;
        }
        nvs_flash_init();
    }

    /* Read MAC address */
    err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address not read");
        return;
    }

    ESP_LOGI(TAG_MAIN, "MAC address gateway: %X:%X:%X:%X:%X:%X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* Read MAC Ethernet address */
    err = esp_read_mac(mac_eth, ESP_MAC_ETH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address ethernet not read");
        return;
    }

    ESP_LOGI(TAG_MAIN, "MAC address Ethernet gateway: %X:%X:%X:%X:%X:%X", mac_eth[0], mac_eth[1], mac_eth[2], mac_eth[3], mac_eth[4], mac_eth[5]);

    /* Init eth */
    err = init_eth();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, Ethernet not setting");
        return;
    }

    err = init_wifi_sta();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, Wifi not setting");
        return;
    }

    /* Init espnow protocol */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGI(TAG_MAIN, "Error, espnow not init");
        return;
    }

    xEventGroupAlarm = xEventGroupCreate();
    if (!xEventGroupAlarm) {
        ESP_LOGE(TAG_MAIN, "Error, event group not created");
        return;
    }

    node_sensor_queue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(node_sensor_msg_t));
    if(!node_sensor_queue) {
        ESP_LOGE(TAG_MAIN, "Error, alarm queue not allocated");
        return;
    }

    node_siren_queue = xQueueCreate(SIREN_QUEUE_SIZE, sizeof(node_siren_request_t));
    if(!node_siren_queue) {
        ESP_LOGE(TAG_MAIN, "Error, siren queue not allocated");
        return;
    }

    /*
    node_keyboard_queue = xQueueCreate(KEYBOARD_QUEUE_SIZE, sizeof(node_id_keyboard));
    if(!node_keyboard_queue) {
        ESP_LOGE(TAG_MAIN, "Error, keyboard queue not allocated");
        return;
    }*/

    /* Register send callback function */
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function no registered");
        return;
    }

    /* Receive callback function */
    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, receive callback function no registered");
        return;
    }

    /* Add peer to list */
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    memcpy(peer.peer_addr, dest_mac, ESP_NOW_ETH_ALEN);
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, peer not add");
        return;
    }

    xTaskCreate(sensor_task, "sensor_task", 1024 * 2, NULL, 0, NULL);
    xTaskCreate(siren_task, "siren_task", 1024 * 2, NULL, 1, NULL);
    //xTaskCreate(keyboard_task, "keyboard_task", 1024 * 2, NULL, 2, NULL);

    return;
}
