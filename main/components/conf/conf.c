#include <string.h>
#include "conf.h"

#define TAG_CONF "CONF"

esp_err_t init_conf() {

    ESP_LOGI("SPIFFS", "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/domotichouse",
        .partition_label = NULL,  // NULL significa che usa la prima partizione SPIFFS trovata
        .max_files = 5,           // Numero massimo di file aperti simultaneamente
        .format_if_mount_failed = true  // Formatta se il montaggio fallisce
    };

    esp_spiffs_format(NULL);
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE("SPIFFS", "Failed to mount or format filesystem: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI("SPIFFS", "SPIFFS mounted successfully");

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI("SPIFFS", "Total: %d bytes, Used: %d bytes", total, used);
    } else {
        ESP_LOGE("SPIFFS", "Errore nel recuperare info SPIFFS");
    }

    return ESP_OK;
}

/* Add node to configuration */
esp_err_t add_node2conf(uint8_t number_devices, uint8_t device_id, char *device_name, uint8_t *mac) {

    esp_err_t err = ESP_FAIL;
    char key[4];
    uint32_t file_size = 0;
    FILE *conf = NULL;

    memset(key, '\0', sizeof(key));
    conf = fopen("/domotichouse/conf.json", "w+");
    if (!conf) {
        ESP_LOGE(TAG_CONF, "Errore nell'aprire il file per la lettura e scrittura");
        return err;
    }
    
    fseek(conf, 0, SEEK_END);
    file_size = ftell(conf);
    fseek(conf, 0, SEEK_SET);

    char *file_content = malloc(file_size + 1);
    if (!file_content) {
        ESP_LOGE(TAG_CONF, "Error, memory not allocated for read file");
        fclose(conf);
        return err;
    }

    memset(file_content, '\0', file_size + 1);
    fread(file_content, 1, file_size, conf);

    cJSON *root = cJSON_Parse(file_content);
    if (!root) {
        ESP_LOGE(TAG_CONF, "Error, json file not parsing");
        free(file_content);
        fclose(conf);
        return err;
    }

    cJSON *id_array = cJSON_CreateArray();
    if(!id_array) {
        ESP_LOGE(TAG_CONF, "Error, array not created");
        return err;
    }
    cJSON_AddItemToArray(id_array, cJSON_CreateNumber(device_id));
    cJSON_AddItemToArray(id_array, cJSON_CreateString(device_name));
    cJSON_AddItemToArray(id_array, cJSON_CreateString("ab:cd:ef:gh"));

    sprintf(key, "%u", number_devices);

    cJSON_AddItemToObject(root, key, id_array);

    fseek(conf, 0, SEEK_SET);
    char *updated_json = cJSON_Print(root);
    fprintf(conf, "%s", updated_json);

    /* Close and clean memory */
    fclose(conf);
    free(file_content);
    free(updated_json);
    cJSON_Delete(root);
    
    ESP_LOGI(TAG_CONF, "JSON file update with successfully");

    return ESP_OK;
}

/* Delete node from configuration */
esp_err_t del_node2conf(uint8_t number_devices) {

    esp_err_t err = ESP_FAIL;

    return err;
}