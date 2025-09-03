idf_component_register(
                    SRCS "main.c"
                    SRCS "components/wifi/wifi.c"
                    SRCS "components/common/common.c"
                    SRCS "components/node/node_array.c"
                    SRCS "components/mqtt/mqtt.c"
                    SRCS "components/ethernet/ethernet.c"
                    SRCS "components/conf/conf.c"
                    SRCS "components/conf/eeprom/eeprom.c"
                    INCLUDE_DIRS "components/wifi"
                    INCLUDE_DIRS "components/common"
                    INCLUDE_DIRS "components/node"
                    INCLUDE_DIRS "components/mqtt"
                    INCLUDE_DIRS "components/ethernet"
                    INCLUDE_DIRS "components/conf"
                    INCLUDE_DIRS "components/conf/eeprom"
                    REQUIRES esp_wifi nvs_flash esp_timer mqtt esp_eth json driver esp_driver_i2c esp_driver_gpio
                    )
