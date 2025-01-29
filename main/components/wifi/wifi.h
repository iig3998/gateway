#ifndef WIFI_H
#define WIFI_H

#pragma once

#include "esp_err.h"

#define ESPNOW_CHANNEL        7
#define MAX_POWER_TRASMISSION 78

#define TAG_WIFI              "WIFI"

esp_err_t init_wifi_sta();

#endif