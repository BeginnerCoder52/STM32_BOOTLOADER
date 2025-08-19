#include <esp_vfs_fat.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include "spiffs.h"
static const char* TAG = "SPIFFS";

void mount_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) ESP_LOGE(TAG, "SPIFFS mount failed");
}