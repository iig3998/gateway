idf_component_register(
                    SRCS "main.c"
                    SRCS "components/wifi/wifi.c"
                    SRCS "components/common/common.c"
                    SRCS "components/node/node_array.c"
                    SRCS "components/mqtt/mqtt.c"
                    SRCS "components/ethernet/ethernet.c"
                    SRCS "components/conf/conf.c"
                    INCLUDE_DIRS "components/wifi"
                    INCLUDE_DIRS "components/common"
                    INCLUDE_DIRS "components/node"
                    INCLUDE_DIRS "components/mqtt"
                    INCLUDE_DIRS "components/ethernet"
                    INCLUDE_DIRS "components/conf"
                    REQUIRES esp_wifi nvs_flash esp_timer mqtt esp_eth esp_driver_gpio spiffs json
                    )
