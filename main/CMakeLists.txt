idf_component_register(
                    SRCS "main.c"
                    SRCS "components/wifi/wifi.c"
                    SRCS "components/common/common.c"
                    SRCS "components/mqtt/mqtt.c"
                    SRCS "components/ethernet/ethernet.c"
                    INCLUDE_DIRS "components/wifi"
                    INCLUDE_DIRS "components/common"
                    INCLUDE_DIRS "components/mqtt"
                    INCLUDE_DIRS "components/ethernet"
                    )
