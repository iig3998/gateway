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

#include "common.h"
#include "wifi.h"
#include "node.h"
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

static uint8_t mac_wifi[MAC_SIZE];
static uint8_t mac_eth[MAC_SIZE];


static QueueHandle_t receive_queue;
static QueueHandle_t send_queue;



static time_t t;

/* Send callback function */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    if (!mac_addr) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address not valid");
        return;
    }

    if(status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG_MAIN, "Message sent correctly");
        xEventGroupSetBits(xEventGroupAlarm, DATA_SENT_STATUS);
    } else if (status == ESP_NOW_SEND_FAIL){
        ESP_LOGE(TAG_MAIN, "Message not sent correctly");
        xEventGroupClearBits(xEventGroupAlarm, DATA_SENT_STATUS);
    }

    return;
}

/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    if (!recv_info->src_addr || !data || len <= 0) {
        ESP_LOGE(TAG_MAIN, "Receive callback args error. Discard messgae");
        return;
    }

    switch (n_type) {
        case SENSOR:
            ESP_LOGI(TAG_MAIN, "Receive message from sensor");
            if(xQueueSend(node_sensor_queue, &data, portMAX_DELAY) != pdTRUE) {
                ESP_LOGI(TAG_MAIN, "Data node sensor insert correctly to queue");
            }
        break;

        case SIREN:
            ESP_LOGI(TAG_MAIN, "Recive message from siren");
            if(xQueueSend(node_siren_queue, &data, portMAX_DELAY) != pdTRUE) {
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

    xEventGroupClearBits(xEventGroupAlarm, DATA_SENT_STATUS);

    do {
        err = esp_now_send(mac, (uint8_t *)&msg, sizeof(*msg));
        if(err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, data not send");
        }
        num_retrasmission_time --;
    }
    while(!(xEventGroupWaitBits(xEventGroupAlarm, DATA_SENT_STATUS, pdTRUE, pdFALSE, pdMS_TO_TICKS(10)) && DATA_SENT_STATUS) && num_retrasmission_time > 0);

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
    struct node_sensors_list_t *node_sensors_list = NULL;

    memset(&msg_sensor, 0, sizeof(msg_sensor));

    ESP_LOGI(TAG_MAIN, "Stack rimanente: %d byte\n", uxTaskGetStackHighWaterMark(NULL));
    while(xHandleTask_sensor) {

        if (xQueueReceive(node_sensor_queue, &msg_sensor, portMAX_DELAY) == pdTRUE) {

            ESP_LOGI(TAG_MAIN, "Receive message from sensor id: %u", msg_sensor.header.id_node);

            crc = calc_crc16((uint8_t *)&msg_sensor, sizeof(msg_sensor) - sizeof(crc));
            ESP_LOGI(TAG_MAIN, "CRC16 calculated: %u", crc);
            if (crc != msg_sensor.crc) {
                ESP_LOGI(TAG_MAIN, "Warning, crc16 correct discard message");
            } else {
                ESP_LOGI(TAG_MAIN, "Success, crc16 correct");
                switch(msg_sensor.header.msg) {

                    case REQUEST:
                        switch (msg_sensor.header.cmd) {

                            case ADD:
                                if(!get_sensors_from_list(node_sensors_list, msg_sensor.header.id_node)) {
                                    ESP_LOGI(TAG_MAIN, "Add new sensor device id: %u", msg_sensor.header.id_node);

                                    /* Add new peer to list */
                                    memset(&peer, 0, sizeof(esp_now_peer_info_t));
                                    peer.channel = ESPNOW_WIFI_CHANNEL;
                                    peer.ifidx = WIFI_IF_STA;
                                    peer.encrypt = false;

                                    err = esp_now_add_peer(&peer);
                                    if(err != ESP_OK) {
                                        ESP_LOGE(TAG_MAIN, "Error, peer sensor not added");
                                    } else {
                                        /* Add sensor to list */
                                        node_sensors_list = add_sensors_to_list(node_sensors_list, msg_sensor);
                                        err = esp_now_get_peer_num(&num_peer);
                                        if(err == ESP_OK){
                                            ESP_LOGI(TAG_MAIN, "There are %d nodes registered", num_peer.total_num);
                                        } else {
                                            ESP_LOGI(TAG_MAIN, "Number nodes not available");
                                        }

                                        /* Save node in memory */
                                    }
                                } else {
                                    ESP_LOGW(TAG_MAIN, "Warning, device id: %u already exist. Skip", msg_sensor.header.id_node);
                                }
                            break;

                            case UPDATE:
                                if(update_sensors_to_list(node_sensors_list, msg_sensor)) {
                                    node_gateway_msg_t msg_gateway;

                                    ESP_LOGI(TAG_MAIN, "Update state sensor inside list");

                                    if(msg_sensor.state && (xEventGroupWaitBits(xEventGroupAlarm, ACTIVE_DEACTIVE_ALARM, pdFALSE, pdFALSE, 0) & ACTIVE_DEACTIVE_ALARM)) {
                                        ESP_LOGI(TAG_MAIN, "Send command for start siren");
                                        msg_gateway = build_request_gateway_msg(ID_GATEWAY, (esp_random() % 256), mac_wifi, true);
                                        send_message(msg_sensor.header.mac, (void *)&msg_gateway);
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
                    /* TODO */

                    break;
                }

                print_sensors_list(node_sensors_list);
            }

        }

        /* Set delay for 50 ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    vTaskDelete(xHandleTask_sensor);

    return;
}

/* Alarm siren task */
void siren_task(void *arg) {

    esp_err_t err = ESP_FAIL;
    uint16_t crc = 0;
    esp_now_peer_info_t peer;
    node_siren_msg_t msg_siren;
    node_gateway_msg_t msg_gateway;

    memset(&msg_siren, 0, sizeof(msg_siren));

    ESP_LOGI(TAG_MAIN, "Stack rimanente: %d byte\n", uxTaskGetStackHighWaterMark(NULL));
    while(xHandleTask_siren) {

        if (xQueueReceive(node_siren_queue, &msg_siren, portMAX_DELAY) == pdTRUE) {

            ESP_LOGI(TAG_MAIN, "Receive message from siren");

            crc = calc_crc16((uint8_t *)&msg_siren, sizeof(msg_siren) - sizeof(crc));
            ESP_LOGI(TAG_MAIN, "CRC16 calculate: %u", crc);

            if (crc != msg_siren.crc) {
                ESP_LOGI(TAG_MAIN, "Warning, crc16 correct discard message");
            } else {
                ESP_LOGI(TAG_MAIN, "Success, crc16 correct");
                switch (msg_siren.header.cmd) { 

                    case ADD:
                        ESP_LOGI(TAG_MAIN, "Add new siren device");

                        memset(&peer, 0, sizeof(esp_now_peer_info_t));
                        peer.channel = ESPNOW_WIFI_CHANNEL;
                        peer.ifidx = WIFI_IF_STA;
                        peer.encrypt = false;

                        memcpy(peer.peer_addr, msg_siren.header.mac, ESP_NOW_ETH_ALEN);
                        err = esp_now_add_peer(&peer);
                        if(err != ESP_OK){
                            ESP_LOGE(TAG_MAIN, "Error, peer siren not added");
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

    vTaskDelete(xHandleTask_siren);

    return;
}

/* Alarm keyboard task */
void keyboard_task(void *arg) {

    esp_err_t err = ESP_FAIL;
    uint16_t crc = 0;
    esp_now_peer_info_t peer;
    node_keyboard_msg_t msg_keyboard;
    node_gateway_msg_t msg_gateway;

    memset(&msg_keyboard, 0, sizeof(msg_keyboard));

    while(xHandleTask_keyboard) {

        if (xQueueReceive(node_keyboard_queue, &msg_keyboard, portMAX_DELAY) == pdTRUE) {

            ESP_LOGI(TAG_MAIN, "Receive message from keyboard");

            crc = calc_crc16((uint8_t *)&msg_keyboard, sizeof(msg_keyboard) - sizeof(crc));
            ESP_LOGI(TAG_MAIN, "CRC16 calculate: %u", crc);

            if (crc != msg_keyboard.crc) {
                ESP_LOGI(TAG_MAIN, "Warning, crc16 correct discard message");
            } else {

                ESP_LOGI(TAG_MAIN, "Success, crc16 correct ");
                switch (msg_keyboard.header.cmd) {
                    case ADD:
                        ESP_LOGI(TAG_MAIN, "Add new keyboard device");

                        memset(&peer, 0, sizeof(esp_now_peer_info_t));
                        peer.channel = ESPNOW_WIFI_CHANNEL;
                        peer.ifidx = WIFI_IF_STA;
                        peer.encrypt = false;

                        memcpy(peer.peer_addr, msg_keyboard.header.mac, 6);
                        err = esp_now_add_peer(&peer);
                        if(err != ESP_OK) {
                            ESP_LOGE(TAG_MAIN, "Error, peer keyboard not added");
                        }
                    break;
                    case ACTIVE:
                        ESP_LOGI(TAG_MAIN, "Active alarm");
                        xEventGroupSetBits(xEventGroupAlarm, ACTIVE_DEACTIVE_ALARM);
                    break;
                    case GET:
                        ESP_LOGI(TAG_MAIN, "Get status sensors and siren");
                    break;
                    case DEACTIVE:
                        ESP_LOGI(TAG_MAIN, "Deactive alarm");
                        xEventGroupClearBits(xEventGroupAlarm, ACTIVE_DEACTIVE_ALARM);
                    break;
                    default:
                        ESP_LOGW(TAG_MAIN, "Warning, command not valid");
                    break;
                }

            }

            /* Set delay for 50 ms */
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    vTaskDelete(xHandleTask_keyboard);

    return;
}

/* Monitoring task */
void monitoring_task(void *arg) {

    while(1) {

        /* Restart sensor task if is dead */
        if (eTaskGetState(xHandleTask_sensor) == eDeleted) {
            ESP_LOGW(TAG_MAIN, "Warning, sensor task is dead. Restart");
            xTaskCreate(keyboard_task, "sensor_task", 1024 * 2, NULL, 2, &xHandleTask_sensor);
        }

        /* Restart siren task if is dead */
        if (eTaskGetState(xHandleTask_siren) == eDeleted) {
            ESP_LOGW(TAG_MAIN, "Warning, siren task is dead. Restart");
            xTaskCreate(keyboard_task, "siren_task", 1024 * 2, NULL, 2, &xHandleTask_siren);
        }

        /* Restart keyboard task if is dead */
        if (eTaskGetState(xHandleTask_keyboard) == eDeleted) {
            ESP_LOGW(TAG_MAIN, "Warning, keyboard task is dead. Restart");
            xTaskCreate(keyboard_task, "keyboard_task", 1024 * 2, NULL, 2, &xHandleTask_keyboard);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(xHandleTask_monitoring);
}

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

    /* Initialize stack TCP/IP */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, network interface WiFi not init");
        return;
    }

    esp_event_loop_create_default();

    /* Read MAC WiFi address */
    err = esp_read_mac(mac_wifi, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address WiFi not read");
        return;
    }

    ESP_LOGI(TAG_MAIN, "MAC address WiFi gateway: %X:%X:%X:%X:%X:%X", mac_wifi[0], mac_wifi[1], mac_wifi[2], mac_wifi[3], mac_wifi[4], mac_wifi[5]);

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

    /* Init wifi station */
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

    /* Create event group alarm */
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

    node_siren_queue = xQueueCreate(SIREN_QUEUE_SIZE, sizeof(node_siren_msg_t));
    if(!node_siren_queue) {
        ESP_LOGE(TAG_MAIN, "Error, siren queue not allocated");
        return;
    }

    node_keyboard_queue = xQueueCreate(KEYBOARD_QUEUE_SIZE, sizeof(node_keyboard_msg_t));
    if(!node_keyboard_queue) {
        ESP_LOGE(TAG_MAIN, "Error, keyboard queue not allocated");
        return;
    }

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
        ESP_LOGE(TAG_MAIN, "Error, peer not added");
        return;
    }

    /* Start sensor task */
    if(xTaskCreate(sensor_task, "sensor_task", 1024 * 2, NULL, 0, &xHandleTask_sensor) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Error, sensor task not started");
        return;
    }

    /* Start siren task */
    if(xTaskCreate(siren_task, "siren_task", 1024 * 2, NULL, 1, &xHandleTask_siren) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Error, siren task not started");
        return;
    }

    /* Start keyboard task */
    if(xTaskCreate(keyboard_task, "keyboard_task", 1024 * 2, NULL, 2, &xHandleTask_keyboard) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Error, keyboard task not started");
        return;
    }

    /* Start monitoring task */
    if(xTaskCreate(monitoring_task, "monitoring_task", 1024 * 2, NULL, 3, &xHandleTask_monitoring) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Error, monitoring task not started");
        return;
    }

    return;
}
