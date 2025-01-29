#include "esp_log.h"
#include "esp_wifi.h"

#include "wifi.h"

/* Init WiFi station */
esp_err_t init_wifi_sta() {

    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG_WIFI, "Start WiFi station");

    /* Initialize stack TCP/IP */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, network interface WiFi not init");
        return err;
    }

    /* Start handler event function */
    esp_event_loop_create_default();

    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_initiation);
    if (err != ESP_OK) {
        return err;
    }

    /* Set storage mode */
    err = esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, set mode store not setting");
        return err;
    }

    /* Set station mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK){
        ESP_LOGE(TAG_WIFI, "Error, WiFi mode not set");
        return err;
    }

    /* Start WiFi */
    while(esp_wifi_start() != ESP_OK) {
        /* Re-try starts WiFi if fails */
        ESP_LOGW(TAG_WIFI, "Error, re-try starts WiFi");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* Set WiFi protcol */
    err = esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, WiFi protcol not setting");
        return err;
    }

    /* Set channel wifi */
    err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_ABOVE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, WiFi channel not setting");
        return err;
    }

    /* Disable power management */
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* Set max power trasmission */
    err = esp_wifi_set_max_tx_power(MAX_POWER_TRASMISSION);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error to set max power trasmission");
        return err;
    }

    return err;
}
