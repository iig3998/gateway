#include "esp_log.h"
#include "esp_err.h"
#include "mqtt_client.h"

#define TAG_MQTT "MQTT"

static esp_mqtt_client_handle_t mqtt_client;

/**/
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {

    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "Connected to MQTT broker");

            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT broker disconnected");
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_MQTT, "Message published");
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG_MQTT, "Error MQTT");
            break;

        default:
            ESP_LOGW(TAG_MQTT, "Mqtt event not managed");
            break;
    }
}

/* Init mqtt */
void init_mqtt() {

    esp_err_t err = ESP_FAIL;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "da80e3ba49314d569e796196ef66537c.s2.eu.hivemq.cloud",
        //.password = "domotichouse",
        //.password = "D0m0t1cH0use",
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    /* Register mqtt event */
    err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_MQTT, "Error, function events not registered");
        return;
    }

    /* Start mqtt client */
    err = esp_mqtt_client_start(mqtt_client);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_MQTT, "Error, mqtt client not started");
        return;
    }
}
