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


//GPIOs
#define BUTTON_PIN  GPIO_NUM_19

#define SDA_PIN GPIO_NUM_17
#define SCL_PIN GPIO_NUM_18
#define RST_PIN GPIO_NUM_21

#define MOTOR_IN1 GPIO_NUM_5
#define MOTOR_IN2 GPIO_NUM_4

//I2C
#define CLOCK_HZ    (400 * 1000)
#define I2C_ADDR    0x3C
#define I2C_HOST    0

//Display   
#define DISP_WIDTH  128
#define DISP_HEIGHT 64 

#define LCD_CMD_BITS    8
#define LCD_PARAM_BITS  8

//ADC
#define SAMPLES 4096
#define SAMPLING_FREQUENCY  4096   // Adjust as needed
#define ADC_CHANNEL ADC1_CHANNEL_6  // For microphone

//Tuning constants
#define TUNING_TOLERANCE 1
#define DELAY_SCALE 60
#define FREQUENCY_SAMPLES 3

float samples[SAMPLES];   // ADC Sampling

static float last_valid_freq = 0;

// Frequency to musical note mapping
typedef struct {
    float freq;
    char *note;
} Note;

// Note table
Note noteTable[] = {
    {185.05, "E2"}, {124.10, "A2"}, {164.90, "D3"}, {220.50, "G3"},
    {278.90, "B3"}, {64.80, "E4"}
};

static esp_timer_handle_t debounce_timer;
static lv_disp_t *disp_handle;

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata, 
                                    void *user_ctx) {
    lv_disp_t *disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

// Label Objects
static lv_obj_t* status_label = NULL;
static lv_obj_t* note_label = NULL;
static lv_obj_t* freq_label = NULL;

static char freq_text[50];
static char note_text[50];
static char status_text[50];

// Display tuning status
static void display(float freq_diff, char* detected_note, int tuning_in_progress) {
    lv_obj_t *scr = lv_disp_get_scr_act(disp_handle);
    if (lvgl_port_lock(0)) {
        lv_obj_clean(scr);

        status_label = lv_label_create(scr);
        lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
        lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 0);

        if (tuning_in_progress) {
            // Show which string is being tuned
            lv_label_set_text(status_label, "Tuning...");
            
            // Note label
            note_label = lv_label_create(scr);
            lv_obj_set_style_text_font(note_label, &lv_font_montserrat_14, 0);
            lv_obj_align(note_label, LV_ALIGN_CENTER, 0, 0);
            lv_label_set_text(note_label, detected_note);

            // Frequency difference label
            freq_label = lv_label_create(scr);
            lv_obj_set_style_text_font(freq_label, &lv_font_montserrat_14, 0);
            lv_obj_align(freq_label, LV_ALIGN_BOTTOM_MID, 0, 0);
            snprintf(freq_text, sizeof(freq_text), "%.2f", freq_diff);
            lv_label_set_text(freq_label, freq_text);
        }
        else {
            // Show tuning complete screen
            lv_label_set_text(status_label, "Tuning complete");

            // Note label (which string was just tuned)
            note_label = lv_label_create(scr);
            lv_obj_set_style_text_font(note_label, &lv_font_montserrat_14, 0);
            lv_obj_align(note_label, LV_ALIGN_CENTER, 0, 0);
            lv_label_set_text(note_label, detected_note);
        }

        lvgl_port_unlock();
    }
}


#define FILTER_SIZE 10
int filter_buffer[FILTER_SIZE];
int filter_index = 0;

// Apply moving average filter to ADC readings
int apply_moving_average(int new_sample) {
    filter_buffer[filter_index] = new_sample;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    // Calculate average of last N samples
    int sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
    }
    return sum / FILTER_SIZE;
}


// ADC reading of microphone
static void read_analog() {
    for (int i = 0; i < SAMPLES; i++) {
        float raw_value = apply_moving_average(adc1_get_raw(ADC_CHANNEL));
        // ESP_LOGI("ADC", "Raw Val: %f", raw_value);

        /**
        float dc_offset = 1.25 * (4095.0 / 3.3);
        float max_amplitude = (2.0 / 3.3) * 4095.0;
        samples[i] = 1.5 * (raw_value - dc_offset) / (max_amplitude / 2);
        **/
       samples[i] = (float)raw_value;
       ets_delay_us(1000000 / SAMPLING_FREQUENCY);
    }

    /**
    ESP_LOGI("ADC", "ADC Sampling Complete. Here are the values:");
    for (int i = 0; i < SAMPLES; i++) {
        ESP_LOGI("ADC", "Sample[%d]: %f", i, samples[i]);
    }
    **/
    
}

// Perform FFT and find dominant frequency
float get_dominant_frequency() {
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);
    kiss_fft_cpx *out = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx) * (SAMPLES / 2 + 1));

    // Apply FFT
    kiss_fftr(cfg, samples, out);

    int max_idx = 1;
    float max_mag = 0;

    for (int i = 1; i < SAMPLES / 2; i++) {
        float magnitude = sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
        if (magnitude > max_mag) {
            max_mag = magnitude;
            max_idx = i;
        }
    }

    // Quadratic Interpolation for more accuracy
    float f_bin = (float)SAMPLING_FREQUENCY / SAMPLES;
    float left = sqrt(out[max_idx - 1].r * out[max_idx - 1].r + out[max_idx - 1].i * out[max_idx - 1].i);
    float right = sqrt(out[max_idx + 1].r * out[max_idx + 1].r + out[max_idx + 1].i * out[max_idx + 1].i);

    float shift = 0.5 * (left - right) / (left - 2 * max_mag + right);
    float dominant_freq = (max_idx + shift) * f_bin;

    // Filtering notes above and below certain frequencies
    if (dominant_freq > 350.0f || dominant_freq < 10.0f) {
        dominant_freq = last_valid_freq; 
    } else {
        last_valid_freq = dominant_freq;
    }

    free(cfg);
    free(out);
    ESP_LOGI("DOMINANT FREQ", "freq: %f", dominant_freq);
    return dominant_freq;
}


// Get closest musical note
char* get_note_from_freq(float freq) {
    float minDiff = 1000;
    char* bestNote = "n/a";
    float bestFreq = 0;
    
    for (int i = 0; i < sizeof(noteTable)/sizeof(Note); i++) {
        float diff = fabs(freq - noteTable[i].freq);
        if (diff < minDiff) {
            minDiff = diff;
            bestNote = noteTable[i].note;
            bestFreq = noteTable[i].freq;
        }
    }

    return bestNote;
}

// Drive DC motor from average frequencies of polling
static float frequency_to_motor_driver(float avg_freq, char* noteName) {
    float ref_freq = 0.0;
    
    // Get reference frequency from note table
    int n = sizeof(noteTable) / sizeof(noteTable[0]);
    for (int i = 0; i < n; i++) {
        if (strcmp(noteTable[i].note, noteName) == 0) {
            ref_freq = noteTable[i].freq;
        }
    }

    // Calculate difference between reference and average frequency
    float diff = avg_freq - ref_freq;
    float diff_mag = fabs(diff);
    int delay = diff_mag * DELAY_SCALE; // Proportional feedback

    ESP_LOGI("MOTOR DRIVER", "Frequency difference: %f Hz", diff);
    ESP_LOGI("MOTOR DRIVER", "Delay: %d ms", delay);
    ESP_LOGI("SPACER", "--------------------------------");

    // If within tolerance, stop motor and indicate successful tuning
    if (diff_mag < TUNING_TOLERANCE){
        display(diff, noteName, 0);
        ESP_LOGI("MOTOR DRIVER", "Tuning complete");
        ESP_LOGI("SPACER", "--------------------------------");
        gpio_set_level(MOTOR_IN1, 1);    
        gpio_set_level(MOTOR_IN2, 1);
        return 0;                         // indicates successful tuning
    }
    else {
        // If not within tolerance, drive motor in appropriate direction
        if (diff > 0){                      // CW and CCW depending on polarity of diff
            display(diff, noteName, 1);
            ESP_LOGI("MOTOR DRIVER", "Tuning in progress");
            ESP_LOGI("SPACER", "--------------------------------");
            gpio_set_level(MOTOR_IN1, 0);    
            gpio_set_level(MOTOR_IN2, 1);
            vTaskDelay(pdMS_TO_TICKS(delay));
            gpio_set_level(MOTOR_IN1, 1);    
            gpio_set_level(MOTOR_IN2, 1);
        }
        else{
            display(diff, noteName, 1);
            ESP_LOGI("MOTOR DRIVER", "Tuning in progress");
            ESP_LOGI("SPACER", "--------------------------------");
            gpio_set_level(MOTOR_IN1, 1);    
            gpio_set_level(MOTOR_IN2, 0);
            vTaskDelay(pdMS_TO_TICKS(delay));
            gpio_set_level(MOTOR_IN1, 1);    
            gpio_set_level(MOTOR_IN2, 1);
        }
        return diff;
    }
}

// Debounce timer callback
static void debounce_timer_callback(void *arg) {
    if (gpio_get_level(BUTTON_PIN) == 1) {
        float freq_sum = 0;
        float avg_freq = 0;
        float freq_diff = 0;
        char* detected_note = "";

        // Preliminary read to get detected note (which string is being tuned)
        for (int i = 0; i < FREQUENCY_SAMPLES; i++){
            vTaskDelay(100);  // Wait for next sample
            read_analog();  // Read microphone
            freq_sum += get_dominant_frequency();  // Get dominant frequency
        }

        avg_freq = freq_sum / (float)FREQUENCY_SAMPLES;  // Get average frequency
        detected_note = get_note_from_freq(avg_freq);  // Get note from frequency

        ESP_LOGI("INTERRUPT", "Average frequency: %f Hz", avg_freq);
        ESP_LOGI("INTERRUPT", "Detected note: %s", detected_note);
        ESP_LOGI("SPACER", "--------------------------------");

        // Tuning loop
        for (int i = 0; i < 10; i++){
            // Drive motor based on average frequency and reference note
            freq_diff = frequency_to_motor_driver(avg_freq, detected_note);

            // If within tolerance, break out of loop
            if (freq_diff == 0){
                break;
            }

            // Read again to get new average frequency
            freq_sum = 0;
            avg_freq = 0;
            freq_diff = 0;

            // Get new average frequency
            for (int i = 0; i < FREQUENCY_SAMPLES; i++){
                vTaskDelay(100);
                read_analog();
                freq_sum += get_dominant_frequency(); 
            }

            avg_freq = freq_sum / (float)FREQUENCY_SAMPLES;
            ESP_LOGI("INTERRUPT", "Average frequency: %f", avg_freq);
            ESP_LOGI("SPACER", "--------------------------------");
        }
    }   
}


// Interrupt
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    // Start 20ms debounce timer
    esp_timer_start_once(debounce_timer, 20000);  
}


void app_main(void) {
    // Configure GPIO input
    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&button_conf);

    // Create debounce timer
    esp_timer_create_args_t debounce_timer_args = {
        .callback = debounce_timer_callback,
        .name = "debounce_timer"
    };
    esp_timer_create(&debounce_timer_args, &debounce_timer);
    
    // Configure motor driver outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_12);

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

    // Initialize motor driver
    gpio_set_level(MOTOR_IN1, 1);    
    gpio_set_level(MOTOR_IN2, 1);

    // Install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, NULL);

}