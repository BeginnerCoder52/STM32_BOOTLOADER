#include <esp_http_server.h>
#include <sys/param.h>
#include "esp_log.h"
#include "ota_config.h"
#include <stdlib.h>

static const char *TAG = "HTTP_SERVER";

/* Nhúng file html vào code (cần khai báo trong CMakeLists.txt hoặc dùng tool, 
   nhưng ở đây mình dùng chuỗi string raw cho đơn giản và dễ sửa đổi) */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

/* Handler cho trang chủ (GET /) */
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

/* Handler cho việc upload firmware (POST /upload) */
static esp_err_t upload_handler(httpd_req_t *req) {
    char *buf = NULL;
    esp_err_t ret = ESP_FAIL;
    
    // Lấy kích thước file upload
    size_t content_len = req->content_len;
    if (content_len > 50 * 1024) { // Giới hạn 50KB (vì App STM32 ~47KB)
        ESP_LOGE(TAG, "File too large!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Cấp phát bộ nhớ để chứa file bin
    buf = malloc(content_len);
    if (!buf) {
        ESP_LOGE(TAG, "Malloc failed!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Nhận dữ liệu từ Web
    int received = 0;
    while (received < content_len) {
        int ret = httpd_req_recv(req, buf + received, content_len - received);
        if (ret <= 0) {
            free(buf);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        received += ret;
    }

    ESP_LOGI(TAG, "File received: %d bytes. Starting UART transfer...", content_len);

    // GỌI HÀM FLASH SANG STM32
    if (start_ota_update((uint8_t*)buf, content_len) == ESP_OK) {
        httpd_resp_send(req, "Update Success", HTTPD_RESP_USE_STRLEN);
        ret = ESP_OK;
    } else {
        httpd_resp_send_500(req); // Báo lỗi về Web
        ret = ESP_FAIL;
    }

    free(buf);
    return ret;
}

/* Đăng ký URL */
static const httpd_uri_t uri_index = { .uri = "/", .method = HTTP_GET, .handler = index_handler };
static const httpd_uri_t uri_upload = { .uri = "/upload", .method = HTTP_POST, .handler = upload_handler };

/* Khởi động Web Server */
void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; // Tăng stack để xử lý data lớn
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_index);
        httpd_register_uri_handler(server, &uri_upload);
        ESP_LOGI(TAG, "Web Server Started");
    }
}