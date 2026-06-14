#include "temp.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG_TEMP = "TEMP";
static gpio_num_t s_temp_pin = GPIO_NUM_NC;

/* ---------- 1-Wire low-level helpers ---------- */

static inline void ow_drive_low(void)
{
    gpio_set_direction(s_temp_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_temp_pin, 0);
}

static inline void ow_release_bus(void)
{
    // Release line so pull-up can bring it high
    gpio_set_direction(s_temp_pin, GPIO_MODE_INPUT);
}

static inline int ow_read_bus(void)
{
    return gpio_get_level(s_temp_pin);
}

static bool ow_reset_pulse(void)
{
    // Master reset pulse
    ow_drive_low();
    esp_rom_delay_us(480);

    // Release and wait for presence pulse
    ow_release_bus();
    esp_rom_delay_us(70);

    // DS18B20 should pull low here if present
    bool presence = (ow_read_bus() == 0);

    // Finish reset timeslot
    esp_rom_delay_us(410);

    return presence;
}

static void ow_write_bit(int bit)
{
    if (bit) {
        // Write '1'
        ow_drive_low();
        esp_rom_delay_us(6);
        ow_release_bus();
        esp_rom_delay_us(64);
    } else {
        // Write '0'
        ow_drive_low();
        esp_rom_delay_us(60);
        ow_release_bus();
        esp_rom_delay_us(10);
    }
}

static int ow_read_bit(void)
{
    int bit = 0;

    ow_drive_low();
    esp_rom_delay_us(6);

    ow_release_bus();
    esp_rom_delay_us(9);

    bit = ow_read_bus();

    esp_rom_delay_us(55);
    return bit;
}

static void ow_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t ow_read_byte(void)
{
    uint8_t value = 0;

    for (int i = 0; i < 8; i++) {
        value >>= 1;
        if (ow_read_bit()) {
            value |= 0x80;
        }
    }

    return value;
}

/* ---------- CRC8 for DS18B20 scratchpad ---------- */

static uint8_t ds18b20_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }

    return crc;
}

/* ---------- Public API ---------- */

esp_err_t temp_init(gpio_num_t pin)
{
    s_temp_pin = pin;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_temp_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (!ow_reset_pulse()) {
        ESP_LOGW(TAG_TEMP, "No DS18B20 detected on GPIO%d during init", s_temp_pin);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG_TEMP, "DS18B20 initialized on GPIO%d", s_temp_pin);
    return ESP_OK;
}

esp_err_t temp_read_celsius(float *temp_c)
{
    if (temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_temp_pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_STATE;
    }

    // 1) Reset + start temperature conversion
    if (!ow_reset_pulse()) {
        ESP_LOGE(TAG_TEMP, "Sensor not responding to reset");
        return ESP_ERR_NOT_FOUND;
    }

    ow_write_byte(0xCC); // SKIP ROM
    ow_write_byte(0x44); // CONVERT T

    // Wait for conversion to finish
    // Max conversion time at 12-bit is 750 ms
    vTaskDelay(pdMS_TO_TICKS(750));

    // 2) Reset + read scratchpad
    if (!ow_reset_pulse()) {
        ESP_LOGE(TAG_TEMP, "Sensor not responding before scratchpad read");
        return ESP_ERR_NOT_FOUND;
    }

    ow_write_byte(0xCC); // SKIP ROM
    ow_write_byte(0xBE); // READ SCRATCHPAD

    uint8_t scratchpad[9];
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = ow_read_byte();
    }

    // CRC check
    uint8_t crc = ds18b20_crc8(scratchpad, 8);
    if (crc != scratchpad[8]) {
        ESP_LOGE(TAG_TEMP, "CRC mismatch: calc=0x%02X, recv=0x%02X", crc, scratchpad[8]);
        return ESP_ERR_INVALID_CRC;
    }

    // Temperature is first two bytes, signed 16-bit
    int16_t raw_temp = (int16_t)((scratchpad[1] << 8) | scratchpad[0]);
    *temp_c = raw_temp / 16.0f;

    ESP_LOGI(TAG_TEMP, "Temperature: %.2f C", *temp_c);
    return ESP_OK;
}