#include "esp_camera.h"
#include <string.h>
#include <stdio.h>
#include "quirc.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

#define TAG "QR_TASK"

static camera_config_t camera_config = {
    .pin_pwdn  = 32,
    .pin_reset = -1,
    .pin_xclk = 0,
    .pin_sscb_sda = 26,
    .pin_sscb_scl = 27,

    .pin_d7 = 35,
    .pin_d6 = 34,
    .pin_d5 = 39,
    .pin_d4 = 36,
    .pin_d3 = 21,
    .pin_d2 = 19,
    .pin_d1 = 18,
    .pin_d0 = 5,
    .pin_vsync = 25,
    .pin_href = 23,
    .pin_pclk = 22,

    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,   // NON-NEGOTIABLE
    .frame_size   = FRAMESIZE_QVGA,
    .fb_count     = 1
};

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &cfg);
    uart_set_pin(UART_NUM_1, 4, 15, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void capture_and_decode_qr(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return;
    }

    struct quirc *qr = quirc_new();
    if (!qr) goto cleanup;

    if (quirc_resize(qr, fb->width, fb->height) < 0)
        goto qr_cleanup;

    uint8_t *image = quirc_begin(qr, NULL, NULL);
    memcpy(image, fb->buf, fb->width * fb->height);
    quirc_end(qr);

    if (quirc_count(qr) > 0) {
        struct quirc_code code;
        struct quirc_data data;

        quirc_extract(qr, 0, &code);
        if (quirc_decode(&code, &data) == 0) {
            char msg[160];
            snprintf(msg, sizeof(msg), "QR:%.150s\r\n", data.payload);
            uart_write_bytes(UART_NUM_1, msg, strlen(msg));
            ESP_LOGI(TAG, "QR decoded: %s", data.payload);
        } else {
            ESP_LOGW(TAG, "QR decode failed");
        }
    }

qr_cleanup:
    quirc_destroy(qr);
cleanup:
    esp_camera_fb_return(fb);
}

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        // We were explicitly woken to work

        uart_init();

        if (esp_camera_init(&camera_config) == ESP_OK) {
            capture_and_decode_qr();
            esp_camera_deinit();
        }

        // Give UART time to flush
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // --- Prepare GPIO13 for EXT0 wakeup ---
    rtc_gpio_init(GPIO_NUM_13);
    rtc_gpio_set_direction(GPIO_NUM_13, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en(GPIO_NUM_13);  // keep it LOW before sleep

    // Prepare to sleep again
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 1);

    // --- Optional: hold pin during sleep ---
    rtc_gpio_hold_en(GPIO_NUM_13); // only if needed

    // Important: GPIO13 must be LOW before sleeping
    //gpio_hold_en(GPIO_NUM_13);

    esp_deep_sleep_start();
}