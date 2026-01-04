idf_component_register(SRCS "main.c" "http_server.c" "uart_transfer.c" 
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_server spiffs nvs_flash driver esp_wifi esp_event esp_timer
                    EMBED_FILES "index.html")