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

#include "kiss_fft.h"
#include "kiss_fftr.h"

#define ADC_CHANNEL ADC1_CHANNEL_2  // For microphone

#define SDA_PIN GPIO_NUM_17
#define SCL_PIN GPIO_NUM_18
#define RST_PIN GPIO_NUM_21

#define BUTTON_PIN  GPIO_NUM_26

#define CLOCK_HZ    (400 * 1000)
#define I2C_ADDR    0x3C
#define I2C_HOST    0

#define DISP_WIDTH  128
#define DISP_HEIGHT 64 

#define LCD_CMD_BITS    8
#define LCD_PARAM_BITS  8

#define SAMPLES 1024
#define SAMPLING_FREQUENCY  16000   // Adjust as needed

int16_t samples[SAMPLES];   // ADC Sampling

// Frequency to musical note mapping
typedef struct {
    float freq;
    char *note;
} Note;

Note noteTable[] = {
    {261.63, "C4"}, {277.18, "C#4"}, {293.66, "D4"}, {311.13, "D#4"},
    {329.63, "E4"}, {349.23, "F4"}, {369.99, "F#4"}, {392.00, "G4"},
    {415.30, "G#4"}, {440.00, "A4"}, {466.16, "A#4"}, {493.88, "B4"},
    {523.25, "C5"}
};

static esp_timer_handle_t system_timer;
static lv_disp_t *disp_handle;

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata, 
                                    void *user_ctx) {
    lv_disp_t *disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

char *detected_note = "Unknown";

// Label Object
lv_obj_t* note_label = NULL;

// Label Character Array
char note_num[50];

// Temp Display

// HAS TO BE CHANGED FOR NEW DATA
static void display() {
    lv_obj_t *scr = lv_disp_get_scr_act(disp_handle);
    if (lvgl_port_lock(0)) {
        lv_obj_clean(scr);

        note_label = lv_label_create(scr);
        lv_obj_set_style_text_font(note_label, &lv_font_montserrat_14, 0); 
        lv_obj_align(note_label, LV_ALIGN_TOP_MID, 0, 0);
        snprintf(note_num, sizeof(note_num), "Note: %s", detected_note);
        lv_label_set_text(note_label, note_num);

        lvgl_port_unlock();
    }
}


// ADC reading 
static void read_analog() {
    for (int i = 0; i < SAMPLES; i++) {
        int raw_value = adc1_get_raw(ADC_CHANNEL);
        samples[i] = (raw_value / 4096.0) * 2.0 - 1.0;  // Normalize between -1 and 1
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLING_FREQUENCY));   // Wait for next sample
    }
}

// Perform FFT and find dominant frequency
float get_dominant_frequency() {
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);
    kiss_fft_scalar *in = (kiss_fft_scalar*)malloc(sizeof(kiss_fft_scalar) * SAMPLES);
    kiss_fft_cpx *out = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx) * (SAMPLES/2 + 1));

    // Copy ADC data
    for (int i = 0; i < SAMPLES; i++) {
        in[i] = samples[i];
    }

    // Perform FFT
    kiss_fftr(cfg, in, out);
    free(cfg);

    // Find peak frequency
    int max_idx = 1;
    float max_magnitude = 0;
    for (int i = 1; i < SAMPLES / 2; i++) {
        float magnitude = sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_idx = i;
        }
    }

    // Calculate frequency
    float frequency = (max_idx * SAMPLING_FREQUENCY) / SAMPLES;
    
    free(in);
    free(out);
    
    return frequency;
}


// Get closest musical note
char* get_note_from_freq(float freq) {
    float minDiff = 1000;
    char* bestNote = "n/a";
    for (int i = 0; i < sizeof(noteTable)/sizeof(Note); i++) {
        float diff = fabs(freq - noteTable[i].freq);
        if (diff < minDiff) {
            minDiff = diff;
            bestNote = noteTable[i].note;
        }
    }
    return bestNote;
}


// Timer callback
static void timer_callback(void *arg) {
    read_analog();
    float freq = get_dominant_frequency(); 
    detected_note = get_note_from_freq(freq);
    display();
}

void app_main(void) {
    // Configure GPIO input w/ interrupt on any edge
    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&button_conf);

    // Configure ADC for TMP36
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_12);

    // Create system timer
    esp_timer_create_args_t timer_args = {
        .callback = timer_callback,
        .name = "system_timer"
    };
    esp_timer_create(&timer_args, &system_timer);
    esp_timer_start_periodic(system_timer, 1000000);

    // Configuring I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,    // I2C LCD is a master node
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));
    
    // Allocate the I2C bus to the LCD I/O device
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_ADDR,   // I2C address for OLED displays
        .control_phase_bytes = 1, 
        .dc_bit_offset = 6, 
        .lcd_cmd_bits = LCD_CMD_BITS,  // 8-bit commands
        .lcd_param_bits = LCD_PARAM_BITS,    // 8-bit parameters
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    // LCD controller driver
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,  
        .reset_gpio_num = RST_PIN,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    // Reset and Initialize OLED
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Initialize lvgl
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t err = lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = DISP_WIDTH * DISP_HEIGHT,  
        .double_buffer = true,
        .hres = DISP_WIDTH,
        .vres = DISP_HEIGHT,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        }
    };
    disp_handle = lvgl_port_add_disp(&disp_cfg);

    // Put LCD and lvgl together
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };

}