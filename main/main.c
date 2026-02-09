#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "quirc.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define TAG "QR_TASK"

static camera_config_t camera_config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sccb_sda   = SIOD_GPIO_NUM,
    .pin_sccb_scl   = SIOC_GPIO_NUM,

    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,

    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,

    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,

    .pixel_format   = PIXFORMAT_GRAYSCALE,
    .frame_size     = FRAMESIZE_QVGA,

    .fb_count       = 1,
    .grab_mode      = CAMERA_GRAB_LATEST
};
static struct quirc *qr = NULL;
static struct quirc_code code;
static struct quirc_data data;

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
    if (!qr) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        vTaskDelay(pdMS_TO_TICKS(1000000));
    } else {
        ESP_LOGI(TAG, "Allocated memory");
    }

    if (quirc_resize(qr, 320, 240) < 0) {
        ESP_LOGE(TAG, "Failed to allocate video memory");
        vTaskDelay(pdMS_TO_TICKS(1000000));
    } else {
        ESP_LOGI(TAG, "Allocated video memory");
    }

    ESP_LOGI(TAG, "0");

    uint8_t *image;
    int w, h;

    ESP_LOGI(TAG, "Before quirc_begin");
    image = quirc_begin(qr, &w, &h);
    if (!image) {
        ESP_LOGE(TAG, "Failed to begin quirc");
        vTaskDelay(pdMS_TO_TICKS(1000000));
        return;
    }
    ESP_LOGI(TAG, "After quirc_begin");

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return;
    }

    size_t expected_len = w * h;

    ESP_LOGI(TAG, "fb->len = %d, expected = %d", fb->len, expected_len);

    if (fb->len != expected_len) {
        ESP_LOGE(TAG, "Framebuffer size mismatch!");
        esp_camera_fb_return(fb);
        quirc_destroy(qr);
        return;
    }

    memcpy(image, fb->buf, fb->len);

    if (fb->width != w || fb->height != h) {
        ESP_LOGE(TAG, "Resolution mismatch");
    } else {
        ESP_LOGI(TAG, "Resolution matches");
    }
    ESP_LOGI(TAG, "Copied image data to quirc buffer");

    quirc_end(qr);
    ESP_LOGI(TAG, "Finished quirc_end with %d bytes", fb->len);
    ESP_LOGI(TAG, "quirc_count = %d", quirc_count(qr));

    int count = quirc_count(qr);
    for (int i = 0; i < count; i++) {
        quirc_decode_error_t err;

        quirc_extract(qr, i, &code);
        ESP_LOGI(TAG, "Extracted code %d, now decoding", i);
        err = quirc_decode(&code, &data);

        if (err == QUIRC_SUCCESS) {
            ESP_LOGI(TAG, "QR: %s", data.payload);
        } else {
            ESP_LOGW(TAG, "QR decode failed: %d", err);
        }
    }
    ESP_LOGI(TAG, "Done processing QR code");
    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(100));
    quirc_destroy(qr);
}

static void cam_task(void *arg)
{
    ESP_LOGI(TAG, "Camera task started");
    ESP_LOGI(TAG, "Free stack: %d bytes",
         uxTaskGetStackHighWaterMark(NULL));
    uart_init();

    if (esp_camera_init(&camera_config) == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        s->set_gain_ctrl(s, 0);     // manual gain
        s->set_exposure_ctrl(s, 0);// manual exposure
        s->set_awb_gain(s, 0);     // disable auto white balance
        s->set_whitebal(s, 0);
        s->set_lenc(s, 0);         // lens correction off
        qr = quirc_new();
        assert(qr);
        vTaskDelay(pdMS_TO_TICKS(200));
        capture_and_decode_qr();
        esp_camera_deinit();
    } else {
        ESP_LOGE(TAG, "Camera init failed");
    }

    // Allow UART TX to finish
    vTaskDelay(pdMS_TO_TICKS(50));

    // Go back to deep sleep
    esp_deep_sleep_start();
}

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        // We were explicitly woken to work
        ESP_LOGI(TAG, "Woke up to scan QR code");

        xTaskCreate(
            cam_task,
            "cam_task",
            16384,      // REQUIRED: camera + quirc stack
            NULL,
            5,
            NULL
        );
        return;
    }

    // --- Prepare GPIO13 for EXT0 wakeup ---
    ESP_LOGI(TAG, "Configuring GPIO13 for wakeup");
    rtc_gpio_init(GPIO_NUM_13);
    rtc_gpio_set_direction(GPIO_NUM_13, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en(GPIO_NUM_13);  // keep it LOW before sleep

    // Prepare to sleep again
    ESP_LOGI(TAG, "Entering deep sleep, waiting for HIGH on GPIO13 to wake up");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 1);

    // --- Optional: hold pin during sleep ---
    ESP_LOGI(TAG, "Enabling hold on GPIO13");
    rtc_gpio_hold_en(GPIO_NUM_13); // only if needed

    // Important: GPIO13 must be LOW before sleeping
    //gpio_hold_en(GPIO_NUM_13);
    ESP_LOGI(TAG, "Going to sleep now");
    esp_deep_sleep_start();
}