// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "esp_camera.h"
#include "bitmap.h"
#include "../components/http_server/http_server.h"

static void handle_grayscale_pgm(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp_stream(http_context_t http_ctx, void* ctx);
static void handle_jpg(http_context_t http_ctx, void* ctx);
static void handle_jpg_stream(http_context_t http_ctx, void* ctx);
static esp_err_t event_handler(void *ctx, system_event_t *event);
static void initialise_wifi(void);

static const char* TAG = "camera_demo";

static const char* STREAM_CONTENT_TYPE =
        "multipart/x-mixed-replace; boundary=123456789000000000000987654321";

static const char* STREAM_BOUNDARY = "--123456789000000000000987654321";

static EventGroupHandle_t s_wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    static camera_config_t camera_config = {
        .pin_pwdn  = -1,                        // power down is not used
        .pin_reset = CONFIG_RESET,              // software reset will be performed
        .pin_xclk = CONFIG_XCLK,
        .pin_sscb_sda = CONFIG_SDA,
        .pin_sscb_scl = CONFIG_SCL,

        .pin_d7 = CONFIG_D7,
        .pin_d6 = CONFIG_D6,
        .pin_d5 = CONFIG_D5,
        .pin_d4 = CONFIG_D4,
        .pin_d3 = CONFIG_D3,
        .pin_d2 = CONFIG_D2,
        .pin_d1 = CONFIG_D1,
        .pin_d0 = CONFIG_D0,
        .pin_vsync = CONFIG_VSYNC,
        .pin_href = CONFIG_HREF,
        .pin_pclk = CONFIG_PCLK,

        //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = /*PIXFORMAT_GRAYSCALE,*/ PIXFORMAT_RGB565, 
        .frame_size = /*FRAMESIZE_QVGA,*/ FRAMESIZE_QQVGA,     //QQVGA-QXGA Do not use sizes above QVGA when not JPEG

        .jpeg_quality = 12, //0-63 lower number means higher quality
        .fb_count = 1 //if more than one, i2s runs in continuous mode. Use only with JPEG
    };

    err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    initialise_wifi();

    http_server_t server;
    http_server_options_t http_options = HTTP_SERVER_OPTIONS_DEFAULT();
    ESP_ERROR_CHECK( http_server_start(&http_options, &server) );

    // Convert the pcm grayscale to bmp and bmp_stream.
    if (camera_config.pixel_format == PIXFORMAT_GRAYSCALE) {
        ESP_ERROR_CHECK( http_register_handler(server, "/bmp", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_rgb_bmp, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/bmp for a single image/bmp gray image", IP2STR(&s_ip_addr));
        ESP_ERROR_CHECK( http_register_handler(server, "/bmp_stream", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_rgb_bmp_stream, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/bmp_stream for multipart/x-mixed-replace stream of gray bitmaps", IP2STR(&s_ip_addr));
        ESP_ERROR_CHECK( http_register_handler(server, "/pgm", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_grayscale_pgm, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/pgm for a single image/x-portable-graymap image", IP2STR(&s_ip_addr));
    }
    if (camera_config.pixel_format == PIXFORMAT_RGB565) {
        ESP_ERROR_CHECK( http_register_handler(server, "/bmp", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_rgb_bmp, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
        ESP_ERROR_CHECK( http_register_handler(server, "/bmp_stream", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_rgb_bmp_stream, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/bmp_stream for multipart/x-mixed-replace stream of bitmaps", IP2STR(&s_ip_addr));
    }
    if (camera_config.pixel_format == PIXFORMAT_JPEG) {
        ESP_ERROR_CHECK( http_register_handler(server, "/jpg", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/jpg for single image/jpg image", IP2STR(&s_ip_addr));
        ESP_ERROR_CHECK( http_register_handler(server, "/jpg_stream", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg_stream, NULL) );
        ESP_LOGI(TAG, "Open http://" IPSTR "/jpg_stream for multipart/x-mixed-replace stream of JPEGs", IP2STR(&s_ip_addr));
    }
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");

}

static esp_err_t write_frame(http_context_t http_ctx, camera_fb_t * fb)
{
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    http_buffer_t fb_data = {
            .data = fb->buf,
            .size = fb->len,
            .data_is_persistent = true
    };

    return http_response_write(http_ctx, &fb_data);
}

#define BUFFER_LEN 512

static esp_err_t write_gray_frame(http_context_t http_ctx, camera_fb_t * fb)
{
uint8_t* buf;
int x = 0;
int size;
esp_err_t err = ESP_OK;
    
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    buf = malloc( BUFFER_LEN * 3 );
    
    if (!buf ){
        ESP_LOGE(TAG, "Dinamic memory failed");
        return ESP_FAIL;    
    }

    http_buffer_t fb_data = {
            .data = buf,
            .size = 0,
            .data_is_persistent = false
    };

    while( (x<fb->len) && (err == ESP_OK) ) {
        size = (fb->len >= BUFFER_LEN) ? BUFFER_LEN : fb->len;       

        for(int i=0; i<size; i++){
            buf[i * 3 ] = fb->buf[i + x];
            buf[(i * 3) + 1 ] = fb->buf[i + x];
            buf[(i * 3) + 2 ] = fb->buf[i + x];        
        }
    
        fb_data.size = size * 3;
        
        err = http_response_write(http_ctx, &fb_data);
 
        x += size;
    }

    free( buf );

    return err;
}

static void handle_grayscale_pgm(http_context_t http_ctx, void* ctx)
{
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();

    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return;
    }

    char* pgm_header_str;
    asprintf(&pgm_header_str, "P5 %d %d %d\n", fb->width, fb->height, 255);
    if (pgm_header_str == NULL) {
        return;
    }

    size_t response_size = strlen(pgm_header_str) + fb->len;
    http_response_begin(http_ctx, 200, "image/x-portable-graymap", response_size);
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.pgm");
    http_buffer_t pgm_header = { .data = pgm_header_str };
    http_response_write(http_ctx, &pgm_header);
    free(pgm_header_str);

    write_frame(http_ctx, fb);
    http_response_end(http_ctx);

    esp_camera_fb_return(fb);
}

static void handle_rgb_bmp(http_context_t http_ctx, void* ctx)
{
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();

    sensor_t * sensor = esp_camera_sensor_get();
    
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return;
    }

    bitmap_header_t* header = bmp_create_header(fb->width, fb->height);
    if (header == NULL) {
        return;
    }

    int len = fb->len;

    if(sensor->pixformat == PIXFORMAT_GRAYSCALE){
        len *= 3;
    }    
    
    http_response_begin(http_ctx, 200, "image/bmp", sizeof(*header) + len);
    http_buffer_t bmp_header = {
            .data = header,
            .size = sizeof(*header)
    };
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.bmp");
    http_response_write(http_ctx, &bmp_header);
    free(header);

    if(sensor->pixformat == PIXFORMAT_GRAYSCALE){
        write_gray_frame(http_ctx, fb);
    }else{
        write_frame(http_ctx, fb);
    }

    http_response_end(http_ctx);

    esp_camera_fb_return(fb);
}

static void handle_jpg(http_context_t http_ctx, void* ctx)
{
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();

    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return;
    }

    http_response_begin(http_ctx, 200, "image/jpeg", fb->len);
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.jpg");
    write_frame(http_ctx, fb);
    http_response_end(http_ctx);

    esp_camera_fb_return(fb);
}

static void handle_rgb_bmp_stream(http_context_t http_ctx, void* ctx)
{
    camera_fb_t * fb = esp_camera_fb_get();
    sensor_t * sensor = esp_camera_sensor_get();

    http_response_begin(http_ctx, 200, STREAM_CONTENT_TYPE, HTTP_RESPONSE_SIZE_UNKNOWN);
    bitmap_header_t* header = bmp_create_header(fb->width, fb->height);
    if (header == NULL) {
        return;
    }

    http_buffer_t bmp_header = {
            .data = header,
            .size = sizeof(*header)
    };

    esp_camera_fb_return(fb);

    while (true) {
        esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera Capture Failed");
            break;
        }

        int len = fb->len;

        if(sensor->pixformat == PIXFORMAT_GRAYSCALE){
            len *= 3;
        } 

        esp_err_t err = http_response_begin_multipart(http_ctx, "image/bitmap", len + sizeof(*header));
        if (err != ESP_OK) {
            break;
        }

        err = http_response_write(http_ctx, &bmp_header);
        if (err != ESP_OK) {
            break;
        }

        if(sensor->pixformat == PIXFORMAT_GRAYSCALE){
            err = write_gray_frame(http_ctx, fb);
        }else{
            err = write_frame(http_ctx, fb);
        }
        
        if (err != ESP_OK) {
            break;
        }

        err = http_response_end_multipart(http_ctx, STREAM_BOUNDARY);

        esp_camera_fb_return(fb);

        if (err != ESP_OK) {
            break;
        }
    }

    free(header);
    http_response_end(http_ctx);
}

static void handle_jpg_stream(http_context_t http_ctx, void* ctx)
{
    http_response_begin(http_ctx, 200, STREAM_CONTENT_TYPE, HTTP_RESPONSE_SIZE_UNKNOWN);

    while (true) {
        //acquire a frame
        camera_fb_t * fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera Capture Failed");
            return;
        }

        esp_err_t err = http_response_begin_multipart(http_ctx, "image/jpg", fb->len );

        if (err != ESP_OK) {
            break;
        }
        err = write_frame(http_ctx, fb);
        if (err != ESP_OK) {
            break;
        }
        err = http_response_end_multipart(http_ctx, STREAM_BOUNDARY);
        if (err != ESP_OK) {
            break;
        }
    }
    http_response_end(http_ctx);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}

