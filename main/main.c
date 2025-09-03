#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_sntp.h"

#include "common.h"
#include "wifi.h"
#include "node_array.h"
#include "ethernet.h"
#include "conf.h"
#include "eeprom.h"

#define TAG_MAIN               "MAIN"
#define TAG_MAIN_SEND          "MAIN_SEND"
#define TAG_MAIN_RECEIVE       "MAIN_RECEIVE"

#define ID_GATEWAY             0x78 /* ID device */
#define NODE_QUEUE_SIZE        10
#define ESPNOW_WIFI_CHANNEL    7

#define NUMBER_ATTEMPTS        3
#define RETRASMISSION_TIME_MS  50
#define MIN_VALID_YEAR         2022

#define DATA_SENT_SUCCESS      (1 << 0)
#define DATA_SENT_FAILED       (1 << 1)
#define DATA_RECEIVED_SUCCESS  (1 << 2)
#define DATA_RECEIVED_FAILED   (1 << 3)
#define ACTIVE_ALARM           (1 << 4)
#define DEACTIVE_ALARM         (1 << 5)

static time_t target_time;
static uint8_t mac_wifi[MAC_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static QueueHandle_t receive_queue;
static EventGroupHandle_t xEventGroupAlarm;

/* Obtian time form the server */
static void obtain_time(void) {

    uint8_t retry = 0;
    const uint8_t max_retries = 10;
    time_t now = 0;
    struct tm timeinfo = {0};

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    while (retry++ < max_retries) {

        time(&now);
        localtime_r(&now, &timeinfo);

        if ((timeinfo.tm_year + 1900) >= MIN_VALID_YEAR) {
            ESP_LOGD("SNTP", "Time synchronized successfully: %s", asctime(&timeinfo));
            break;
        }

        ESP_LOGW("SNTP", "Waiting for time sync... attempt %d", retry);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    if ((timeinfo.tm_year + 1900) < MIN_VALID_YEAR) {
        ESP_LOGE("SNTP", "Time synchronization failed after %d attempts", max_retries);
    }

    return;
}

/* Send callback function */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    if (!mac_addr) {
        return;
    }

    if(status == ESP_NOW_SEND_SUCCESS) {
        /* Message sent correctly */
        xEventGroupSetBits(xEventGroupAlarm, DATA_SENT_SUCCESS);
    } else if (status == ESP_NOW_SEND_FAIL){
        /* Message not sent correctly */
        xEventGroupSetBits(xEventGroupAlarm, DATA_SENT_FAILED);
    }

    return;
}

/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    if (!recv_info->src_addr || !data || len <= 0) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xQueueSendFromISR(receive_queue, data, &xHigherPriorityTaskWoken) != pdTRUE) {
        ESP_LOGW(TAG_MAIN, "Warning, queue is full, discard message");
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

    return;
}

/* Send message */
static bool send_message(uint8_t dst_mac[], node_msg_t msg) {

    esp_err_t err = ESP_FAIL;
    EventBits_t uxBits;

    for (uint8_t i = 0; i < NUMBER_ATTEMPTS; i++) {

        xEventGroupClearBits(xEventGroupAlarm, DATA_SENT_SUCCESS | DATA_SENT_FAILED);

        err = esp_now_send(dst_mac, (uint8_t *)&msg, sizeof(msg));
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Waits callback function result */
        uxBits = xEventGroupWaitBits(
            xEventGroupAlarm,
            DATA_SENT_SUCCESS | DATA_SENT_FAILED,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(RETRASMISSION_TIME_MS)
        );

        /* Success */
        if (uxBits & DATA_SENT_SUCCESS) {
            return true;
        }

        /* Failed */
        if (uxBits & DATA_SENT_FAILED) {
            ESP_LOGW(TAG_MAIN, "Send failed, retrying... (%u/%u)", i + 1, NUMBER_ATTEMPTS);
        } else {
            ESP_LOGW(TAG_MAIN, "Timeout waiting for send result, retrying... (%u/%u)", i + 1, NUMBER_ATTEMPTS);
        }
    }

    return false;
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
            } else {
                switch(node_msg.header.cmd) {
                    case ADD:
                        if(!check_node_inside_list(node_msg.header.id_node)) {

                            /* Add new peer to list */
                            memset(&peer, 0, sizeof(esp_now_peer_info_t));
                            peer.channel = ESPNOW_WIFI_CHANNEL;
                            peer.ifidx = WIFI_IF_STA;
                            peer.encrypt = false;
                            memcpy(peer.peer_addr, node_msg.header.mac, MAC_SIZE);

                            err = esp_now_add_peer(&peer);
                            if(err != ESP_OK) {
                                ESP_LOGE(TAG_MAIN_RECEIVE, "Error, peer node %u not added", node_msg.header.id_node);
                            } else {
                                ESP_LOGI(TAG_MAIN_RECEIVE, "Peer node %u added correctly", node_msg.header.id_node);
                                /* Adds sensor to list */
                                add_node_to_list(node_msg);

                                /* Send response to node */
                                node_msg = build_node_msg(ADD, ID_GATEWAY, GATEWAY, node_msg.header.id_msg, mac_wifi, "gateway", NULL);
                                send_message(peer.peer_addr, node_msg);
                            }
                        } else {
                            ESP_LOGW(TAG_MAIN_RECEIVE, "Warning, node device id %u already exist. Skip", node_msg.header.id_node);
                        }
                    break;
                    case DEL:
                        if(check_node_inside_list(node_msg.header.id_node)) {

                            /* Send response to node */
                            node_msg_t node_del_msg = build_node_msg(DEL, ID_GATEWAY, GATEWAY, node_msg.header.id_msg, mac_wifi, "gateway", NULL);
                            send_message(node_msg.header.mac, node_del_msg);

                            err = esp_now_del_peer(node_msg.header.mac);
                            if(err != ESP_OK) {
                                ESP_LOGE(TAG_MAIN_RECEIVE, "Error, peer node %u not removed", node_msg.header.id_node);
                            } else {
                                /* Adds sensor to list */
                                del_node_from_list(node_msg.header.id_node);
                                ESP_LOGI(TAG_MAIN_RECEIVE, "Peer node %u deleted correctly", node_msg.header.id_node);
                            }
                        } else {
                            ESP_LOGW(TAG_MAIN_RECEIVE, "Warning, node device id %u already deleted. Skip", node_msg.header.id_node);
                        }
                    break;
                    case UPDATE:
                        /* Receive response from node */
                        update_node_to_list(node_msg);
                        ESP_LOGI(TAG_MAIN_RECEIVE, "Update state node id: %u inside list", node_msg.header.id_node);

                        node_msg_t node_update_msg = build_node_msg(UPDATE, ID_GATEWAY, GATEWAY, node_msg.header.id_msg, mac_wifi, "gateway", NULL);
                        send_message(node_msg.header.mac, node_update_msg);
                    break;
                    case POWER_ON_ALARM:
                        ESP_LOGD(TAG_MAIN_RECEIVE, "Power on alarm");
                        xEventGroupSetBits(xEventGroupAlarm, ACTIVE_ALARM);
                        xEventGroupClearBits(xEventGroupAlarm, DEACTIVE_ALARM);
                    break;
                    case POWER_OFF_ALARM:
                        ESP_LOGD(TAG_MAIN_RECEIVE, "Power off alarm");

                        /* Stop siren if running */
                        node_msg = build_node_msg(STOP_SIREN, ID_GATEWAY, GATEWAY, (esp_random() % 256), mac_wifi, "gateway", NULL);
                        send_message(peer.peer_addr, node_msg);
                        xEventGroupSetBits(xEventGroupAlarm, DEACTIVE_ALARM);
                        xEventGroupClearBits(xEventGroupAlarm, ACTIVE_ALARM);

                    break;
                    case ALARM:
                        status_node sn;
                        memcpy(&sn, node_msg.payload, sizeof(status_node));
                        ESP_LOGI(TAG_MAIN_RECEIVE, "State %u for id sensor: %u", sn.state, node_msg.header.id_node);
                        ESP_LOGI(TAG_MAIN_RECEIVE, "Battery low detect: %u for id sensor: %u", sn.battery_low_detect, node_msg.header.id_node);

                        if(sn.state && (xEventGroupWaitBits(xEventGroupAlarm, ACTIVE_ALARM, pdFALSE, pdFALSE, 0) & ACTIVE_ALARM)) {
                            ESP_LOGI(TAG_MAIN_RECEIVE, "Send command for start siren");
                            node_msg = build_node_msg(START_SIREN, ID_GATEWAY, GATEWAY, (esp_random() % 256), mac_wifi, "gateway", NULL);
                            send_message(peer.peer_addr, node_msg);
                        } else {
                            ESP_LOGI(TAG_MAIN_RECEIVE, "Alarm not active, not start siren");
                        }
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return;
}

/* Run send task */
void sync_time() {

    static char strftime_buf[64];
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2100 - 1900)) {
        obtain_time();
        time(&now);
    }

    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();

    time(&target_time);
    localtime_r(&target_time, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG_MAIN, "Current time: %s", strftime_buf);

    return;
}

/* Main program */
void app_main(void) {

    esp_err_t err = ESP_FAIL;

    /* Init nvs flash */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    /* Initialize stack */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, network interface WiFi not init");
        return;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        ESP_LOGD(TAG_MAIN, "Error, event loop not created");
        return;
    }

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

    /* Sync time */
    sync_time();

    /* Init node list mutex */
    if (!init_node_list_mutex())
        return;

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

    /* Start receive task */
    if(xTaskCreatePinnedToCore(receive_task, "receive_task", 1024 * 4, NULL, 1, NULL, 0) != pdPASS) {
        ESP_LOGD(TAG_MAIN, "Error, receive task not started");
        return;
    }

    return;
}
