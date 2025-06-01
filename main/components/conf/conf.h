#ifndef CONF_H
#define CONF_H

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include "esp_system.h"

esp_err_t init_conf();

esp_err_t add_node2conf(uint8_t number_devices, uint8_t device_id, char *device_name, uint8_t *mac);

esp_err_t del_node2conf(uint8_t number_devices);

esp_err_t load_conf();

#endif