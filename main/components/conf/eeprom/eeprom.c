#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "eeprom.h"

#define PIN_SDA                GPIO_NUM_4
#define PIN_SCL                GPIO_NUM_14
#define EEPROM_ADDR            0x50
#define ADDRESS_EEPROM_BLOCK_0 0x50
#define ADDRESS_EEPROM_BLOCK_1 0x51
#define I2C_PORT               0
#define I2C_CLOCK_SPEED        100000
#define EEPROM_PAGE_SIZE       16
#define EEPROM_TOTAL_BYTES     512
#define TAG_EEPROM             "EEPROM"

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/* Init eeprom */
esp_err_t init_eeprom() {

    esp_err_t err = ESP_FAIL;
    i2c_master_bus_handle_t bus_handle;

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = PIN_SCL,
        .sda_io_num = PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_EEPROM, "Error, impossibile to add new bus master");
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDRESS_EEPROM_BLOCK_0,
        .scl_speed_hz = I2C_CLOCK_SPEED,
    };

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_EEPROM, "Error, impossibile to configure master");
        return err;
    }

    return err;
}

/* Write byte to eeprom */
esp_err_t eeprom_write_byte(uint16_t mem_addr, uint8_t data) {

    esp_err_t err = ESP_FAIL;

    uint8_t buffer[2] = {mem_addr & 0xFF, data};

    err = i2c_master_transmit(dev_handle, buffer, sizeof(buffer), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_EEPROM, "Error, impossibile to write data %u in eeprom", data);
        return err;
    }

    return err;
}

/* Read byte form eeprom */
esp_err_t eeprom_read_byte(uint16_t mem_addr, uint8_t *data) {

    uint8_t addr_low = 0x00;
    esp_err_t err = ESP_FAIL;

    addr_low = mem_addr & 0xFF;
    err = i2c_master_transmit(dev_handle, &addr_low, 1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_EEPROM, "Error, impossibile to write data %u in eeprom", *data);
        return err;
    }

    err = i2c_master_receive(dev_handle, data, 1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_EEPROM, "Error, impossibile to read data from eeprom");
        return err;
    }

    return err;
}

