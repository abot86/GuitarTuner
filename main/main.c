#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_rom_sys.h"
#include "rom/ets_sys.h"
#include "sdkconfig.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

//LCD
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 17
#define DISP_WIDTH 128
#define DISP_HEIGHT 64
#define RST_PIN 21 
#define I2C_HOST 0
#define I2C_HW_ADDR 0x3C
#define LCD_PIXEL_CLOCK_HZ (500 * 1000)

//WIFI
#define ESP_WIFI_SSID "DukeVisitor"
#define ESP_WIFI_PASS ""

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

//HTTP
#define URL1 "http://ip-api.com/json/?fields=lat,lon"
#define URL2 "http://api.openweathermap.org/data/2.5/weather?lat=%f&lon=%f&appid=%s"
#define API_KEY "8b0bc5843cf2e74911072225e2c7af9f"
#define MAX_HTTP_OUTPUT_BUFFER 1024

//Macros
#define K2F(k) (((k) - 273.15) * 9 / 5 + 32)

//Tags
static const char *TAG = "HTTP";

// Handles
static lv_disp_t *disp_handle;
static lv_obj_t *temp_label;
static lv_obj_t *humd_label;
static lv_obj_t *wind_label;
static EventGroupHandle_t s_wifi_event_group;


// Geo printed variables
float lat_var, lon_var;

// Weather printed variables
float humd_var = 0.0;
float wind_var = 0.0;
float temp_var = 0.0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    int s_retry_num = 0; 
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata, 
                                    void *user_ctx) {
    lv_disp_t *disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

static void set_display() {
    char buffer[32];
    lv_obj_t *scr = lv_disp_get_scr_act(disp_handle);

    if (lvgl_port_lock(0)) {
        lv_obj_clean(scr);

        //Print temperature
        temp_label = lv_label_create(scr);
        snprintf(buffer, sizeof(buffer), "Temp: %.2f F", temp_var);
        lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_10, 0); 
        lv_label_set_text(temp_label, buffer);
        lv_obj_align(temp_label, LV_ALIGN_TOP_MID, 0, 0);
        
        //Print humidity
        humd_label = lv_label_create(scr);
        snprintf(buffer, sizeof(buffer), "Humidity: %.2f %%", humd_var);
        lv_obj_set_style_text_font(humd_label, &lv_font_montserrat_10, 0); 
        lv_label_set_text(humd_label, buffer);
        lv_obj_align(humd_label, LV_ALIGN_TOP_MID, 0, 15);

        //Print wind
        wind_label = lv_label_create(scr);
        snprintf(buffer, sizeof(buffer), "Wind: %.2f m/s", wind_var);
        lv_obj_set_style_text_font(wind_label, &lv_font_montserrat_10, 0); 
        lv_label_set_text(wind_label, buffer);
        lv_obj_align(wind_label, LV_ALIGN_TOP_MID, 0, 30);

        lvgl_port_unlock();
    }
}

static void http_native_request_geo(void)
{
    // Declare local_response_buffer with size (MAX_HTTP_OUTPUT_BUFFER + 1) to prevent out of bound access when
    // it is used by functions like strlen(). The buffer should only be used upto size MAX_HTTP_OUTPUT_BUFFER
    // char output_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0};   // Buffer to store response of http request
    char *output_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER + 1);
    int content_length = 0;
    esp_http_client_config_t config = {
        .url = URL1,
        //.event_handler = _http_event_handler,
        .user_data = output_buffer,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET Request
    //esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_open(client, 0);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } else {
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
                ESP_LOG_BUFFER_HEX(TAG, output_buffer, data_read);

                output_buffer[data_read] = '\0';  // Null-terminate the buffer

                // Parse the JSON response
                cJSON *json = cJSON_Parse(output_buffer);
                if (json == NULL) {
                    ESP_LOGE(TAG, "Failed to parse JSON");
                }
                else {
                    // Take out specific fields from json object (parsing)
                    cJSON *lat = cJSON_GetObjectItem(json, "lat");
                    cJSON *lon = cJSON_GetObjectItem(json, "lon");

                    // Check if the fields are in the expected format
                    if (cJSON_IsNumber(lat) && cJSON_IsNumber(lon)) {
                        ESP_LOGI(TAG, "Lat: %f", lat->valuedouble);
                        lat_var = lat->valuedouble;
                        ESP_LOGI(TAG, "Lon: %f", lon->valuedouble);
                        lon_var = lon->valuedouble;
                    } else {
                        ESP_LOGE(TAG, "Failed to extract valid fields from JSON");
                    }

                    // Free the JSON object
                    cJSON_Delete(json);
                }
            } else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static void http_native_request_weather(void)
{
    // Declare local_response_buffer with size (MAX_HTTP_OUTPUT_BUFFER + 1) to prevent out of bound access when
    // it is used by functions like strlen(). The buffer should only be used upto size MAX_HTTP_OUTPUT_BUFFER
    // char output_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0};   // Buffer to store response of http request
    // char url[256];

    char *output_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER + 1);
    char *url = (char *)malloc(256);
    int content_length = 0;
    snprintf(url, 256, URL2, lat_var, lon_var, API_KEY);

    esp_http_client_config_t config = {
        .url = url,
        .user_data = output_buffer,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET Request
    //esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_open(client, 0);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } else {
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
                ESP_LOG_BUFFER_HEX(TAG, output_buffer, data_read);

                output_buffer[data_read] = '\0';  // Null-terminate the buffer

                // Parse the JSON response
                cJSON *json = cJSON_Parse(output_buffer);
                if (json == NULL) {
                    ESP_LOGE(TAG, "Failed to parse JSON");
                }
                else {
                    // Take out specific fields from json object (parsing)
                    cJSON *temp;
                    cJSON *humd;
                    cJSON *wind_speed;
                    cJSON *main = cJSON_GetObjectItem(json, "main");
                    cJSON *wind = cJSON_GetObjectItem(json, "wind");

                    if (main != NULL) {
                        temp = cJSON_GetObjectItem(main, "temp");
                        humd = cJSON_GetObjectItem(main, "humidity");

                        if (cJSON_IsNumber(temp) && cJSON_IsNumber(humd)) {
                            ESP_LOGI(TAG, "Temp: %f", temp->valuedouble);
                            temp_var = K2F(temp->valuedouble);  //Convert from standard (K) to F
                            ESP_LOGI(TAG, "Humidity: %f", humd->valuedouble);
                            humd_var = humd->valuedouble;
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to extract valid fields from JSON");
                    }

                    if (wind != NULL) {
                        wind_speed = cJSON_GetObjectItem(wind, "speed");
                        
                        if (cJSON_IsNumber(wind_speed)) {
                            ESP_LOGI(TAG, "Wind: %f", wind_speed->valuedouble);
                            wind_var = wind_speed->valuedouble;
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to extract valid fields from JSON");
                    }

                    cJSON_Delete(json);
                }
            } else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(url);
}


void app_main(void)
{
    // Configure i2c
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_PIXEL_CLOCK_HZ, // 100 KHz (Changed)
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    //Allocate I2C bus to the LCD
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    //Install LCD controller driver
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = RST_PIN,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    //Reset and initialize the OLED
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    //Intitialize esp_lvgl_port
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t err = lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle, //doc said lcd_panel_handle
        .buffer_size = DISP_WIDTH*DISP_HEIGHT,
        .double_buffer = true,
        .hres = DISP_WIDTH, 
        .vres = DISP_HEIGHT, 
        .monochrome = true,

        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
    };
    disp_handle = lvgl_port_add_disp(&disp_cfg);

    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp_handle);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Initialize wifi
    wifi_init_sta();
    vTaskDelay(5000/portTICK_PERIOD_MS);

    // Get longitude and latitude
    http_native_request_geo();
    vTaskDelay(1000/portTICK_PERIOD_MS);

    // Get weather by specifying lat and lon
    http_native_request_weather();

    while(1){
        set_display();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
    