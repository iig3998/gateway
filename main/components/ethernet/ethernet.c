#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"

#include "driver/gpio.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#define ETH_MDC_GPIO     23
#define ETH_MDIO_GPIO    18
#define ETH_PHY_RST_GPIO 5
#define ETH_PHY_ADDR     1
#define ETH_POWER_PIN    16

#define TAG_ETHERNET "ETHERNET"


/* Ethernet Event Handler */
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {

    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGD(TAG_ETHERNET, "Ethernet connected");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGD(TAG_ETHERNET, "Ethernet Link Down");
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGD(TAG_ETHERNET, "Ethernet Started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGD(TAG_ETHERNET, "Ethernet Stopped");
            break;
        default:
            break;
    }
}

/* Event IP function */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if(!event)
        return;

    ESP_LOGI(TAG_ETHERNET, "IP gateway: %u.%u.%u.%u", IP2STR(&event->ip_info.ip));

    return;
}

/* Init ethernet */
esp_err_t init_eth() {

    esp_err_t err = ESP_FAIL;

    ESP_LOGD(TAG_ETHERNET, "Start Ethernet");

    /* Power on device */
    gpio_set_direction(ETH_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ETH_POWER_PIN, 1);

    gpio_set_direction(ETH_PHY_RST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ETH_PHY_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(ETH_PHY_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();

    esp32_emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;

    /* Create MAC instance */
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    if(!mac) {
        ESP_LOGE(TAG_ETHERNET, "Error, MAC instance not created");
        return err;
    }

    /* Configure PHY ethernet */
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;
    phy_config.autonego_timeout_ms = 1000;

    /* Init LAN87xx */
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
    if(!phy) {
        ESP_LOGE(TAG_ETHERNET, "Error, LAN87xx not init");
        return err;
    }

    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);

    /* Install driver */
    err = esp_eth_driver_install(&config, &eth_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_ETHERNET, "Error, ethernet driver not installed");
        return err;
    }

    /* Attach network interface */
    err = esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ETHERNET, "Error. Ethernet interface not installed");
        return err;
    }

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL);

    /* Start ethernet */
    err = esp_eth_start(eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ETHERNET, "Error, ethernet not started");
        return err;
    }

    return err;
}
