#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t init_eeprom();

esp_err_t eeprom_write_byte(uint16_t mem_addr, uint8_t data);

esp_err_t eeprom_read_byte(uint16_t mem_addr, uint8_t *data);

#endif