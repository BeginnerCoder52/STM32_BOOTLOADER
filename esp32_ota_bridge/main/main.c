#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

/* BẮT BUỘC PHẢI CÓ DÒNG NÀY ĐỂ NHẬN DIỆN ESP_WIFI_SSID */
#include "ota_config.h" 

void start_webserver(void); 

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI("WIFI", "Station connected");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI("WIFI", "Station disconnected");
    }
}

void wifi_init_softap(void) {
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .ap = {
            /* Các hằng số này được lấy từ ota_config.h */
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
    ESP_LOGI("WIFI", "SoftAP started. SSID:%s password:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
}

void app_main(void) {
    // 1. Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // 2. Init Netif & Event Loop
    esp_netif_init();
    esp_event_loop_create_default();

    // 3. Init UART 
    /* Hàm này được khai báo trong ota_config.h */
    uart_init_custom();

    // 4. Init Wifi SoftAP
    wifi_init_softap();

    // 5. Start Web Server
    start_webserver();
}
