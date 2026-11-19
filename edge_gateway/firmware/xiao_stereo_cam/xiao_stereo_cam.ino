#include "esp_camera.h"

const uint8_t SYNC_MAGIC[4] = {0xAA, 0x55, 0xAA, 0x55};
bool cam_ready = false;
esp_err_t init_err = ESP_OK;
bool led_on = false;

void setup() {
    Serial.begin(921600);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(800);
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = 15; c.pin_d1 = 17; c.pin_d2 = 18; c.pin_d3 = 16;
    c.pin_d4 = 14; c.pin_d5 = 12; c.pin_d6 = 11; c.pin_d7 = 48;
    c.pin_xclk = 10; c.pin_pclk = 13; c.pin_vsync = 38; c.pin_href = 47;
    c.pin_sccb_sda = 40; c.pin_sccb_scl = 39; c.pin_pwdn = -1; c.pin_reset = -1;
    c.xclk_freq_hz = 20000000; c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = FRAMESIZE_QVGA; c.jpeg_quality = 16;
    c.fb_count = 1; c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    init_err = esp_camera_init(&c);
    cam_ready = (init_err == ESP_OK);
}
void loop() {
    if (!cam_ready) {
        Serial.printf("CAM_ERR:0x%x\n", init_err);
        delay(500); return;
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { Serial.println("FB_ERR"); delay(200); return; }
    led_on = !led_on; digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
    uint32_t len = (uint32_t)fb->len;
    Serial.write(SYNC_MAGIC, 4); Serial.write((const uint8_t *)&len, 4);
    for (size_t s = 0; s < len; s += 512) {
        Serial.write(fb->buf + s, (len - s > 512) ? 512 : (len - s));
    }
    esp_camera_fb_return(fb); delay(45);
}
