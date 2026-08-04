#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "temp.h"
#include "pinout.h"
#include "config.h"
#include "webpage.h"
#include "audio.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"

// ---------- Compatibility aliases used by this repository ----------
#ifndef LP_PUMP
#define LP_PUMP LP_in
#endif
#ifndef HP_PUMP
#define HP_PUMP PUMP2_PIN
#endif
#ifndef HP_PUMP_DELAY_MS
#define HP_PUMP_DELAY_MS PUMP2_DELAY_MS
#endif
#ifndef NVS_KEY_LP_PUMP_ENABLE
#define NVS_KEY_LP_PUMP_ENABLE NVS_KEY_P1_ENABLE
#endif
#ifndef NVS_KEY_HP_PUMP_ENABLE
#define NVS_KEY_HP_PUMP_ENABLE NVS_KEY_P2_ENABLE
#endif

// ---------- ADC channel alias (battery) ----------
#define ADC_CHANNEL     BAT_ADC_CHANNEL

// ---------- Logging Tags ----------
static const char *TAG_MAIN = "MAIN";
static const char *TAG_BLINK = "BLINK";
static const char *TAG_ADC = "ADC";
static const char *TAG_OLED = "OLED";
static const char *TAG_BUTTON = "BUTTON";
static const char *TAG_ADMIN = "ADMIN";
static bool g_lp_pump_running = false;
static bool g_hp_pump_running = false;

// ---------- Display/System Mode ----------
typedef enum {
    MODE_HOME,
    MODE_WATER,
    MODE_SYSTEM,
    MODE_ADMIN_PREVIEW,
    MODE_AUTONOMOUS,
    MODE_WEBPAGE,
    MODE_ADMIN_EDIT
} display_mode_t;

// ---------- Joystick Direction ----------
typedef enum {
    JOY_NONE,
    JOY_UP,
    JOY_DOWN,
    JOY_LEFT,
    JOY_RIGHT
} joy_dir_t;

// ---------- Global Variables ----------
static float g_battery_voltage = 0.0f;
static int g_adc_raw = 0;

// Voltage thresholds
static float g_current_lvd = DEFAULT_LVD;
static float g_current_mvr = DEFAULT_MVR;
static float g_temp_lvd = DEFAULT_LVD;
static float g_temp_mvr = DEFAULT_MVR;

// Condition toggles (true = check is active)
static bool g_float_check = true;
static bool g_lvd_check   = true;
static bool g_temp_float_check = true;
static bool g_temp_lvd_check   = true;

// Pump 1 manual override (runtime only — resets to false on reboot)
static bool g_lp_pump_override      = false;
static bool g_temp_lp_pump_override = false;

// Pump enable/shutoff (persisted — when OFF pump cannot activate under any condition)
static bool g_lp_pump_enable      = true;
static bool g_hp_pump_enable      = true;
static bool g_temp_lp_pump_enable = true;
static bool g_temp_hp_pump_enable = true;

// Website Admin Mode (runtime only). When true, both pumps are forced off.
// OLED, sensors, Wi-Fi, and the web server continue running.
static bool g_web_admin_mode = false;

// Autonomous Mode (persisted). Toggling this on is a one-time preset that
// sets LP EN, HP EN, FLT CHK, and LVD CHK to true and clears the LP override;
// those four toggles remain independently editable afterward. This flag
// itself is only kept for status display and NVS persistence.
static bool g_autonomous_mode = false;

// Display mode state
static display_mode_t g_display_mode = MODE_HOME;
static int64_t g_last_admin_activity = 0;
static int g_admin_cursor = 0;
static int g_autonomous_cursor = 0;  // 0=Autonomous Mode, 1=LP Pump Enable

// Mutexes
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t voltage_mutex;

// ADC handle
static adc_oneshot_unit_handle_t adc_handle;

// ADC calibration (converts raw counts to mV using the chip's factory-trimmed
// eFuse curve, instead of an ideal-linear raw/4095*3300 assumption)
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_cali_enabled = false;

static void adc_calibration_init(void)
{
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        adc_cali_enabled = true;
        ESP_LOGI(TAG_ADC, "ADC calibration (line fitting) enabled");
    } else {
        ESP_LOGW(TAG_ADC, "ADC calibration init failed (%s); using uncalibrated conversion",
                 esp_err_to_name(ret));
    }
}

// Converts a raw ADC1 reading to millivolts using the calibration curve when
// available, falling back to a simple linear conversion otherwise.
static int adc_raw_to_mv(int raw)
{
    if (adc_cali_enabled) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv) == ESP_OK) {
            return mv;
        }
    }
    return (int)((raw / 4095.0f) * 3300.0f);
}

// OLED framebuffer
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// ---------- Forward Declarations ----------
static void enter_admin_edit(void);
static void exit_admin_edit(bool save);

// voltage_mutex must already be held before calling this helper.
// It copies webpage changes into the existing OLED Admin edit variables.
static void sync_existing_oled_admin_values_locked(void)
{
    g_temp_lvd = g_current_lvd;
    g_temp_mvr = g_current_mvr;
    g_temp_float_check = g_float_check;
    g_temp_lvd_check = g_lvd_check;
    g_temp_lp_pump_override = g_lp_pump_override;
    g_temp_lp_pump_enable = g_lp_pump_enable;
    g_temp_hp_pump_enable = g_hp_pump_enable;
}

// ---------- Temperature Variables ----------
static float g_temp_c = 0.0f;
static bool g_temp_valid = false;

// ---------- TDS variables ----------
static float g_tds_ppm = 0.0f;
static bool g_tds_valid = false;
static int g_tds_raw = 0;
static float g_tds_voltage = 0.0f;

// ---------- Current Sense Variables ----------
static float g_current_amps = 0.0f;
static bool g_current_valid = false;

// ---------- Flow Sensor Variables ----------
static float g_flow1_gpm = 0.0f;
static float g_flow2_gpm = 0.0f;

// ---------- Float Switch Variables ----------
static bool g_tank_full = false;

// True when the pump_task voltage hysteresis latch (LVD/MVR) currently
// allows the LP pump to run. Shared for display purposes only.
static bool g_pump_voltage_ok = false;

// ---------- NVS Functions ----------

float tds_calculate_ppm(int adc_raw, float temp_c);

static void nvs_init_storage(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG_ADMIN, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG_ADMIN, "NVS initialized");
}

static void nvs_load_voltages(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    
    if (err == ESP_OK) {
        // Load LVD (stored as uint32_t representation of float)
        uint32_t lvd_bits;
        err = nvs_get_u32(nvs_handle, NVS_KEY_LVD, &lvd_bits);
        if (err == ESP_OK) {
            memcpy(&g_current_lvd, &lvd_bits, sizeof(float));
            ESP_LOGI(TAG_ADMIN, "Loaded LVD: %.2fV", g_current_lvd);
        } else {
            g_current_lvd = DEFAULT_LVD;
            ESP_LOGI(TAG_ADMIN, "Using default LVD: %.2fV", g_current_lvd);
        }
        
        // Load MVR
        uint32_t mvr_bits;
        err = nvs_get_u32(nvs_handle, NVS_KEY_MVR, &mvr_bits);
        if (err == ESP_OK) {
            memcpy(&g_current_mvr, &mvr_bits, sizeof(float));
            ESP_LOGI(TAG_ADMIN, "Loaded MVR: %.2fV", g_current_mvr);
        } else {
            g_current_mvr = DEFAULT_MVR;
            ESP_LOGI(TAG_ADMIN, "Using default MVR: %.2fV", g_current_mvr);
        }
        
        // Load condition toggles (stored as uint8_t: 1=on, 0=off)
        uint8_t chk;
        g_float_check = (nvs_get_u8(nvs_handle, NVS_KEY_FLOAT_CHECK, &chk) == ESP_OK) ? (bool)chk : true;
        g_lvd_check   = (nvs_get_u8(nvs_handle, NVS_KEY_LVD_CHECK,   &chk) == ESP_OK) ? (bool)chk : true;
        g_lp_pump_enable   = (nvs_get_u8(nvs_handle, NVS_KEY_LP_PUMP_ENABLE,   &chk) == ESP_OK) ? (bool)chk : true;
        g_hp_pump_enable   = (nvs_get_u8(nvs_handle, NVS_KEY_HP_PUMP_ENABLE,   &chk) == ESP_OK) ? (bool)chk : true;
        g_autonomous_mode  = (nvs_get_u8(nvs_handle, NVS_KEY_AUTO_MODE,        &chk) == ESP_OK) ? (bool)chk : false;
        ESP_LOGI(TAG_ADMIN, "Checks - Float:%d LVD:%d LPEN:%d HPEN:%d Auto:%d",
                 g_float_check, g_lvd_check, g_lp_pump_enable, g_hp_pump_enable, g_autonomous_mode);

        nvs_close(nvs_handle);
    } else {
        g_current_lvd = DEFAULT_LVD;
        g_current_mvr = DEFAULT_MVR;
        g_float_check = true;
        g_lvd_check   = true;
        ESP_LOGI(TAG_ADMIN, "Using defaults - LVD: %.2fV, MVR: %.2fV",
                 g_current_lvd, g_current_mvr);
    }
}

static void nvs_save_voltages(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    
    if (err == ESP_OK) {
        // Save LVD (convert float to uint32_t for NVS)
        uint32_t lvd_bits;
        memcpy(&lvd_bits, &g_current_lvd, sizeof(float));
        err = nvs_set_u32(nvs_handle, NVS_KEY_LVD, lvd_bits);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_ADMIN, "Failed to save LVD: %s", esp_err_to_name(err));
        }
        
        // Save MVR
        uint32_t mvr_bits;
        memcpy(&mvr_bits, &g_current_mvr, sizeof(float));
        err = nvs_set_u32(nvs_handle, NVS_KEY_MVR, mvr_bits);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_ADMIN, "Failed to save MVR: %s", esp_err_to_name(err));
        }
        
        // Save condition toggles
        nvs_set_u8(nvs_handle, NVS_KEY_FLOAT_CHECK, (uint8_t)g_float_check);
        nvs_set_u8(nvs_handle, NVS_KEY_LVD_CHECK,   (uint8_t)g_lvd_check);
        nvs_set_u8(nvs_handle, NVS_KEY_LP_PUMP_ENABLE,   (uint8_t)g_lp_pump_enable);
        nvs_set_u8(nvs_handle, NVS_KEY_HP_PUMP_ENABLE,   (uint8_t)g_hp_pump_enable);
        nvs_set_u8(nvs_handle, NVS_KEY_AUTO_MODE,        (uint8_t)g_autonomous_mode);

        // Commit changes
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_ADMIN, "Voltages saved - LVD: %.2fV, MVR: %.2fV", 
                     g_current_lvd, g_current_mvr);
        } else {
            ESP_LOGE(TAG_ADMIN, "Failed to commit NVS: %s", esp_err_to_name(err));
        }
        
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG_ADMIN, "Failed to open NVS: %s", esp_err_to_name(err));
    }
}

// ---------- Voltage Validation ----------

static bool validate_voltages(void)
{
    if (g_temp_lvd < LVD_MIN || g_temp_lvd > LVD_MAX) return false;
    if (g_temp_mvr < MVR_MIN || g_temp_mvr > MVR_MAX) return false;
    if (g_temp_mvr < g_temp_lvd + MIN_HYSTERESIS) return false;
    return true;
}

static void adjust_lvd(float delta)
{
    g_temp_lvd += delta;
    
    // Round to nearest 0.1V to prevent floating point accumulation errors
    g_temp_lvd = roundf(g_temp_lvd * 10.0f) / 10.0f;
    
    // Constrain to valid range
    if (g_temp_lvd < LVD_MIN) g_temp_lvd = LVD_MIN;
    if (g_temp_lvd > LVD_MAX) g_temp_lvd = LVD_MAX;
    
    // Ensure hysteresis
    if (g_temp_lvd > g_temp_mvr - MIN_HYSTERESIS) {
        g_temp_lvd = g_temp_mvr - MIN_HYSTERESIS;
    }
    
    ESP_LOGI(TAG_ADMIN, "LVD adjusted to: %.2fV", g_temp_lvd);
}

static void adjust_mvr(float delta)
{
    g_temp_mvr += delta;
    
    // Round to nearest 0.1V to prevent floating point accumulation errors
    g_temp_mvr = roundf(g_temp_mvr * 10.0f) / 10.0f;
    
    // Constrain to valid range
    if (g_temp_mvr < MVR_MIN) g_temp_mvr = MVR_MIN;
    if (g_temp_mvr > MVR_MAX) g_temp_mvr = MVR_MAX;
    
    // Ensure hysteresis
    if (g_temp_mvr < g_temp_lvd + MIN_HYSTERESIS) {
        g_temp_mvr = g_temp_lvd + MIN_HYSTERESIS;
    }
    
    ESP_LOGI(TAG_ADMIN, "MVR adjusted to: %.2fV", g_temp_mvr);
}

// ---------- Admin Mode Functions ----------

static void enter_admin_edit(void)
{
    g_display_mode = MODE_ADMIN_EDIT;
    g_admin_cursor = 0;  // Start on LVD

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_temp_lvd             = g_current_lvd;
        g_temp_mvr             = g_current_mvr;
        g_temp_float_check     = g_float_check;
        g_temp_lvd_check       = g_lvd_check;
        g_temp_lp_pump_override  = g_lp_pump_override;
        g_temp_lp_pump_enable       = g_lp_pump_enable;
        g_temp_hp_pump_enable       = g_hp_pump_enable;
        xSemaphoreGive(voltage_mutex);
    }

    g_last_admin_activity = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG_ADMIN, "=== ENTERING ADMIN EDIT ===");
    ESP_LOGI(TAG_ADMIN, "Current LVD: %.2fV, MVR: %.2fV", g_temp_lvd, g_temp_mvr);
}

static void exit_admin_edit(bool save)
{
    if (save && validate_voltages()) {
        if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_current_lvd     = g_temp_lvd;
            g_current_mvr     = g_temp_mvr;
            g_float_check     = g_temp_float_check;
            g_lvd_check       = g_temp_lvd_check;
            g_lp_pump_override  = g_temp_lp_pump_override;
            g_lp_pump_enable       = g_temp_lp_pump_enable;
            g_hp_pump_enable       = g_temp_hp_pump_enable;
            xSemaphoreGive(voltage_mutex);
        }
        nvs_save_voltages();
        ESP_LOGI(TAG_ADMIN, "=== SETTINGS SAVED ===");
    } else {
        ESP_LOGI(TAG_ADMIN, "=== SETTINGS DISCARDED ===");
    }

    g_display_mode = MODE_HOME;
}

// Sets Autonomous Mode. voltage_mutex must already be held before calling.
// Turning it on is a one-time preset: LP EN, HP EN, FLT CHK, and LVD CHK are
// set true and the LP override is cleared (so the checks just enabled
// actually take effect). All four remain independently editable afterward —
// this flag is not continuously enforced.
static void apply_autonomous_mode_locked(bool enabled)
{
    g_autonomous_mode = enabled;
    if (enabled) {
        g_lp_pump_enable   = true;
        g_hp_pump_enable   = true;
        g_float_check      = true;
        g_lvd_check        = true;
        g_lp_pump_override = false;
    }
}

// Flips Autonomous Mode (OLED click handler).
static void toggle_autonomous_mode(void)
{
    bool new_state = false;
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        new_state = !g_autonomous_mode;
        apply_autonomous_mode_locked(new_state);
        xSemaphoreGive(voltage_mutex);
    }

    nvs_save_voltages();
    ESP_LOGI(TAG_ADMIN, "Autonomous mode -> %d", new_state);
    audio_play(new_state ? AUDIO_CLIP_AUTONOMOUS_ON : AUDIO_CLIP_AUTONOMOUS_OFF);
}

// Flips LP Pump Enable (OLED click handler, same flag as Admin Edit's LP EN
// and the website's Pump 1 Enable switch).
static void toggle_lp_pump_enable(void)
{
    bool new_state = false;
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_lp_pump_enable = !g_lp_pump_enable;
        new_state = g_lp_pump_enable;
        xSemaphoreGive(voltage_mutex);
    }

    nvs_save_voltages();
    ESP_LOGI(TAG_ADMIN, "LP pump enable -> %d", new_state);
}

// ---------- I2C Functions ----------

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_write_byte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_OLED, "i2c_write_byte failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t i2c_write_data(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_OLED, "i2c_write_data failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// ---------- OLED Functions ----------

static void oled_command(uint8_t cmd)
{
    i2c_write_byte(0x00, cmd);
}

static void oled_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    
    oled_command(0xAE); // Display off
    oled_command(0x20); oled_command(0x00); // Horizontal addressing mode
    oled_command(0xB0); // Set page start address
    oled_command(0xC8); // COM scan direction
    oled_command(0x00); oled_command(0x10); // Column address
    oled_command(0x40); // Start line address
    oled_command(0x81); oled_command(0xFF); // Contrast
    oled_command(0xA1); // Segment re-map
    oled_command(0xA6); // Normal display
    oled_command(0xA8); oled_command(0x3F); // Multiplex ratio
    oled_command(0xA4); // Display follows RAM
    oled_command(0xD3); oled_command(0x00); // Display offset
    oled_command(0xD5); oled_command(0x80); // Clock divide ratio
    oled_command(0xD9); oled_command(0xF1); // Pre-charge period
    oled_command(0xDA); oled_command(0x12); // COM pins configuration
    oled_command(0xDB); oled_command(0x40); // VCOMH deselect level
    oled_command(0x8D); oled_command(0x14); // Charge pump
    oled_command(0xAF); // Display ON
    
    vTaskDelay(pdMS_TO_TICKS(100));
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_clear_buffer(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    
    int byte_idx = x + (y / 8) * OLED_WIDTH;
    int bit_idx = y % 8;
    
    if (on) {
        oled_buffer[byte_idx] |= (1 << bit_idx);
    } else {
        oled_buffer[byte_idx] &= ~(1 << bit_idx);
    }
}

// Simple 5x7 font
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
    {0x08, 0x08, 0x2A, 0x08, 0x08}, // '+'
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // '=' (using O)
    {0x14, 0x14, 0x7F, 0x14, 0x14}, // '#'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '>'
    {0x00, 0x03, 0x05, 0x00, 0x00}, // '<'
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // '['
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // ']'
};

static void oled_draw_char_col(int x, int y, char c, bool on)
{
    int idx = -1;

    if (c == ' ') idx = 0;
    else if (c >= '0' && c <= '9') idx = c - '0' + 1;
    else if (c == ':') idx = 11;
    else if (c == '.') idx = 12;
    else if (c >= 'A' && c <= 'Z') idx = c - 'A' + 13;
    else if (c >= 'a' && c <= 'z') idx = (c - 'a') + 13;
    else if (c == '+') idx = 39;
    else if (c == '-') idx = 40;
    else if (c == '=') idx = 41;
    else if (c == '#') idx = 42;
    else if (c == '>') idx = 43;
    else if (c == '<') idx = 44;
    else if (c == '[') idx = 45;
    else if (c == ']') idx = 46;
    else idx = 0;

    if (idx < 0 || idx >= 47) return;

    for (int i = 0; i < 5; i++) {
        uint8_t col = font5x7[idx][i];
        for (int j = 0; j < 7; j++) {
            if (col & (1 << j)) {
                oled_set_pixel(x + i, y + j, on);
            }
        }
    }
}

static void oled_draw_char(int x, int y, char c)
{
    oled_draw_char_col(x, y, c, true);
}

static void oled_fill_rect(int x, int y, int w, int h)
{
    for (int py = y; py < y + h; py++)
        for (int px = x; px < x + w; px++)
            oled_set_pixel(px, py, true);
}

static void oled_draw_string_inv(int x, int y, const char *str)
{
    oled_fill_rect(0, y, OLED_WIDTH, 8);
    int xpos = x;
    while (*str) {
        oled_draw_char_col(xpos, y, *str, false);
        xpos += 6;
        str++;
    }
}

static void oled_draw_string(int x, int y, const char *str)
{
    int xpos = x;
    while (*str) {
        oled_draw_char(xpos, y, *str);
        xpos += 6;
        str++;
    }
}

static void oled_draw_circle(int cx, int cy, int r, bool filled)
{
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            int dist2 = x * x + y * y;
            if (filled) {
                if (dist2 <= r * r)
                    oled_set_pixel(cx + x, cy + y, true);
            } else {
                if (dist2 >= (r - 1) * (r - 1) && dist2 <= r * r)
                    oled_set_pixel(cx + x, cy + y, true);
            }
        }
    }
}

// 5 page dots centered at bottom: Home=0, Water=1, System=2, Admin=3, Manual=4
static void oled_draw_page_dots(int active_page)
{
    const int num_dots = 5;
    const int radius = 2;
    const int spacing = 12;
    const int y = 60;
    const int start_x = (OLED_WIDTH - (num_dots - 1) * spacing) / 2;

    for (int i = 0; i < num_dots; i++) {
        oled_draw_circle(start_x + i * spacing, y, radius, i == active_page);
    }
}

static void oled_update_display(void)
{
    oled_command(0x21); oled_command(0); oled_command(127);
    oled_command(0x22); oled_command(0); oled_command(7);
    
    const int chunk_size = 128;
    for (int i = 0; i < sizeof(oled_buffer); i += chunk_size) {
        uint8_t data[chunk_size + 1];
        data[0] = 0x40;
        memcpy(&data[1], &oled_buffer[i], chunk_size);
        i2c_write_data(data, chunk_size + 1);
    }
}

// ---------- Frog Splash Bitmap ----------
// 48x40 pixel frog outline (LSB first, 6 bytes per row)
static const uint8_t frog_bitmap[] = {
    // Row 0-3: Top of eyes
    0x00, 0x1E, 0x00, 0x00, 0x78, 0x00,
    0x00, 0x7F, 0x00, 0x00, 0xFE, 0x00,
    0x80, 0xFF, 0x00, 0x00, 0xFF, 0x01,
    0xC0, 0xC1, 0x01, 0x80, 0x83, 0x03,
    // Row 4-7: Eyes (open circles)
    0xC0, 0x80, 0x01, 0x80, 0x01, 0x03,
    0xE0, 0x80, 0x03, 0xC0, 0x01, 0x07,
    0xE0, 0x80, 0x03, 0xC0, 0x01, 0x07,
    0xC0, 0x80, 0x01, 0x80, 0x01, 0x03,
    // Row 8-11: Below eyes, head widens
    0xC0, 0xC1, 0x01, 0x80, 0x83, 0x03,
    0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
    0x00, 0xFE, 0xFF, 0xFF, 0x7F, 0x00,
    // Row 12-16: Head tapering
    0x00, 0xFC, 0xFF, 0xFF, 0x3F, 0x00,
    0x00, 0xF8, 0xFF, 0xFF, 0x1F, 0x00,
    0x00, 0xF0, 0xFF, 0xFF, 0x0F, 0x00,
    0x00, 0xE0, 0xFF, 0xFF, 0x07, 0x00,
    0x00, 0xE0, 0xFF, 0xFF, 0x07, 0x00,
    // Row 17-19: Smile
    0x00, 0xF0, 0x00, 0x00, 0x0F, 0x00,
    0x00, 0x70, 0x00, 0x00, 0x0E, 0x00,
    0x00, 0x38, 0x00, 0x00, 0x1C, 0x00,
    // Row 20-23: Chin to body
    0x00, 0x1C, 0x00, 0x00, 0x38, 0x00,
    0x00, 0x0E, 0x00, 0x00, 0x70, 0x00,
    0x00, 0x0F, 0x00, 0x00, 0xF0, 0x00,
    0x80, 0x07, 0x00, 0x00, 0xE0, 0x01,
    // Row 24-31: Body
    0xC0, 0x03, 0x00, 0x00, 0xC0, 0x03,
    0xE0, 0x01, 0x00, 0x00, 0x80, 0x07,
    0xE0, 0x01, 0x00, 0x00, 0x80, 0x07,
    0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F,
    0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F,
    0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F,
    0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F,
    0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F,
    // Row 32-35: Legs
    0xFC, 0x01, 0x00, 0x00, 0x80, 0x3F,
    0x9C, 0x03, 0x00, 0x00, 0xC0, 0x39,
    0x0E, 0x07, 0x00, 0x00, 0xE0, 0x70,
    0x0E, 0x0E, 0x00, 0x00, 0x70, 0x70,
    // Row 36-39: Webbed feet
    0x07, 0x1C, 0x00, 0x00, 0x38, 0xE0,
    0x47, 0x38, 0x00, 0x00, 0x1C, 0xE2,
    0xE7, 0x70, 0x00, 0x00, 0x0E, 0xE7,
    0xFE, 0xE0, 0x00, 0x00, 0x07, 0x7F,
};

#define FROG_BMP_WIDTH   48
#define FROG_BMP_HEIGHT  40

/**
 * Draw a 1-bit bitmap at (x_offset, y_offset).
 * Uses oled_set_pixel which is already bounds-checked.
 */
static void oled_draw_bitmap(int x_offset, int y_offset,
                             const uint8_t *bmp, int w, int h)
{
    int bytes_per_row = (w + 7) / 8;
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            int byte_idx = row * bytes_per_row + (col / 8);
            int bit_idx  = col % 8;
            if (bmp[byte_idx] & (1 << bit_idx)) {
                oled_set_pixel(x_offset + col, y_offset + row, true);
            }
        }
    }
}

/**
 * Show the F.R.O.G.S startup splash screen:
 *   Frame 1: Frog graphic + title (2.5s)
 *   Frame 2: "System Ready" confirmation (1.5s)
 */
static void frogs_show_splash(void)
{
    // --- Frame 1: Frog graphic + title ---
    oled_clear_buffer();

    // Center the 48x40 frog: x = (128-48)/2 = 40, y = 0
    oled_draw_bitmap(40, 0, frog_bitmap, FROG_BMP_WIDTH, FROG_BMP_HEIGHT);

    // "F.R.O.G.S" centered below frog
    oled_draw_string(37, 44, "F.R.O.G.S");

    // Subtitle
    oled_draw_string(16, 56, "Filtration Resource for Off-Grid Systems");

    oled_update_display();
    vTaskDelay(pdMS_TO_TICKS(2500));


}

// ========== CONTINUE TO PART 2 ==========
// ========== PART 2: DISPLAY AND TASK FUNCTIONS ==========

// ---------- Display Functions ----------

static void display_admin_mode(void)
{
    char line[32];

    oled_clear_buffer();

    // Yellow zone: title + timeout
    oled_draw_string(0, 4, "ADMIN EDIT");
    int64_t time_left = (ADMIN_TIMEOUT_MS - (esp_timer_get_time() / 1000 - g_last_admin_activity)) / 1000;
    if (time_left < 0) time_left = 0;
    snprintf(line, sizeof(line), "T:%llds", time_left);
    oled_draw_string(90, 4, line);

    // Scroll window: 5 visible rows, follows cursor
    // Items: 0=LVD 1=MVR 2=FLT CHK 3=LVD CHK 4=LP OVRD 5=LP EN 6=HP EN 7=RESET
    int scroll_top = (g_admin_cursor > 4) ? g_admin_cursor - 4 : 0;

    for (int i = 0; i < 5; i++) {
        int item = scroll_top + i;
        if (item > 7) break;
        int y = 16 + i * 9;
        bool selected = (item == g_admin_cursor);

        switch (item) {
            case 0: snprintf(line, sizeof(line), "LVD: %.1fV",  g_temp_lvd);                          break;
            case 1: snprintf(line, sizeof(line), "MVR: %.1fV",  g_temp_mvr);                          break;
            case 2: snprintf(line, sizeof(line), "FLT CHK: %s", g_temp_float_check  ? "ON" : "OFF"); break;
            case 3: snprintf(line, sizeof(line), "LVD CHK: %s", g_temp_lvd_check    ? "ON" : "OFF"); break;
            case 4: snprintf(line, sizeof(line), "LP OVRD: %s", g_temp_lp_pump_override ? "ON" : "OFF"); break;
            case 5: snprintf(line, sizeof(line), "LP EN:   %s", g_temp_lp_pump_enable    ? "ON" : "OFF"); break;
            case 6: snprintf(line, sizeof(line), "HP EN:   %s", g_temp_hp_pump_enable    ? "ON" : "OFF"); break;
            case 7: snprintf(line, sizeof(line), "RESET DFLTS");                                      break;
        }

        if (selected)
            oled_draw_string_inv(2, y, line);
        else
            oled_draw_string(2, y, line);
    }

    oled_update_display();
}

static void display_system_mode(void)
{
    char line[32];

    float lvd = 0.0f;
    float mvr = 0.0f;

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        lvd = g_current_lvd;
        mvr = g_current_mvr;
        xSemaphoreGive(voltage_mutex);
    }

    oled_clear_buffer();

    // Yellow zone: title
    oled_draw_string(40, 4, "SYSTEM");

    // Blue zone: content
    snprintf(line, sizeof(line), "LVD: %.1fV  MVR: %.1fV", lvd, mvr);
    oled_draw_string(0, 16, line);

    oled_draw_page_dots(2);
    oled_update_display();
}

static void display_water_mode(void)
{
    char line[32];
    float temp_c = 0.0f;
    float tds_ppm = 0.0f;
    float current_amps = 0.0f;
    float flow1 = 0.0f;
    float flow2 = 0.0f;
    bool temp_valid = false;
    bool tds_valid = false;
    bool current_valid = false;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        temp_c        = g_temp_c;
        temp_valid    = g_temp_valid;
        tds_ppm       = g_tds_ppm;
        tds_valid     = g_tds_valid;
        current_amps  = g_current_amps;
        current_valid = g_current_valid;
        flow1         = g_flow1_gpm;
        flow2         = g_flow2_gpm;
        xSemaphoreGive(state_mutex);
    }

    // Flow ratio
    float total_flow = flow1 + flow2;
    int ratio = (total_flow > 0.0f) ? (int)((flow1 / total_flow) * 100.0f) : 0;

    oled_clear_buffer();

    // Yellow zone: title
    oled_draw_string(16, 4, "WATER QUALITY");

    // Blue zone: content
    if (temp_valid)
        snprintf(line, sizeof(line), "TEMP: %.1fC", temp_c);
    else
        snprintf(line, sizeof(line), "TEMP: ERROR");
    oled_draw_string(0, 16, line);

    if (tds_valid)
        snprintf(line, sizeof(line), "TDS:  %.0fppm", tds_ppm);
    else
        snprintf(line, sizeof(line), "TDS:  ERROR");
    oled_draw_string(0, 25, line);
/*
    if (current_valid)
        snprintf(line, sizeof(line), "CURR: %.2fA", current_amps);
    else
        snprintf(line, sizeof(line), "CURR: ERROR");
    oled_draw_string(0, 34, line); 
*/
    snprintf(line, sizeof(line), "F1:%.1f F2:%.1f", flow1, flow2);
    oled_draw_string(0, 34, line);

    snprintf(line, sizeof(line), "RATIO: %d%%:%d%%", ratio, 100 - ratio);
    oled_draw_string(0, 43, line);

    oled_draw_page_dots(1);
    oled_update_display();
}

static void display_autonomous_mode(void)
{
    char line[32];
    bool auto_mode = false;
    bool lp_en = true, hp_en = true, flt_chk = true, lvd_chk = true;

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        auto_mode = g_autonomous_mode;
        lp_en     = g_lp_pump_enable;
        hp_en     = g_hp_pump_enable;
        flt_chk   = g_float_check;
        lvd_chk   = g_lvd_check;
        xSemaphoreGive(voltage_mutex);
    }

    oled_clear_buffer();

    oled_draw_string(4, 4, "AUTONOMOUS MODE");

    snprintf(line, sizeof(line), "STATUS: %s", auto_mode ? "ON" : "OFF");
    if (g_autonomous_cursor == 0) oled_draw_string_inv(0, 16, line);
    else                          oled_draw_string(0, 16, line);

    snprintf(line, sizeof(line), "LP PUMP: %s", lp_en ? "ON" : "OFF");
    if (g_autonomous_cursor == 1) oled_draw_string_inv(0, 26, line);
    else                          oled_draw_string(0, 26, line);

    snprintf(line, sizeof(line), "HP:%-3s F:%-3s L:%-3s",
             hp_en ? "ON" : "OFF", flt_chk ? "ON" : "OFF", lvd_chk ? "ON" : "OFF");
    oled_draw_string(0, 36, line);

    oled_draw_string(0, 46, "UP/DN:SEL CLK:TOGGLE");

    oled_draw_page_dots(4);
    oled_update_display();
}

static void display_webpage_preview(void)
{
    oled_clear_buffer();

    //yellow zone: header
    oled_draw_string(12, 4, "WEBPAGE CONNECTION");

    oled_draw_string(4, 20, "WIFI: FROGS");
    oled_draw_string(4, 30, "PASS:");
    oled_draw_string(4, 40, "frogspassword");
    oled_draw_string(4, 52, "192.168.4.1");

     oled_update_display();
}

static void display_admin_preview(void)
{
    oled_clear_buffer();

    // Yellow zone: title
    oled_draw_string(20, 4, "ADMIN MODE");

    // Blue zone: instructions
    oled_draw_string(6, 24, "CLICK STICK");
    oled_draw_string(6, 34, "TO ENTER");

    oled_draw_page_dots(3);
    oled_update_display();
}

static void display_home_mode(void)
{
    char line[32];
    float bv = 0.0f;
    bool lp_pump = false;
    bool hp_pump = false;
    bool tank_full = false;
    bool voltage_ok = false;

    bool lvd_chk = true, float_chk = true;
    bool lp_override = false, lp_enable = true;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bv        = g_battery_voltage;
        lp_pump      = g_lp_pump_running;
        hp_pump     = g_hp_pump_running;
        tank_full = g_tank_full;
        voltage_ok = g_pump_voltage_ok;
        xSemaphoreGive(state_mutex);
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        lvd_chk = g_lvd_check;
        float_chk = g_float_check;
        lp_override = g_lp_pump_override;
        lp_enable = g_lp_pump_enable;
        xSemaphoreGive(voltage_mutex);
    }

    oled_clear_buffer();

    // Yellow zone: title
    oled_draw_string(0, 4, "F.R.O.G.S");

    // Blue zone: content
    snprintf(line, sizeof(line), "BATT: %.2fV", bv);
    oled_draw_string(0, 16, line);

    snprintf(line, sizeof(line), "LP: %-3s HP: %-3s",
             lp_pump ? "ON" : "OFF", hp_pump ? "ON" : "OFF");
    oled_draw_string(0, 26, line);

    snprintf(line, sizeof(line), "TANK: %-4s",
             tank_full ? "LOW" : "FULL");
    oled_draw_string(0, 36, line);

    // LP pump turn-on conditions: "--" means the check is bypassed (toggled
    // off), so it can't block the pump regardless of the sensor reading.
    const char *v_str = !lvd_chk   ? "--" : (voltage_ok  ? "OK" : "NO");
    const char *f_str = !float_chk ? "--" : (!tank_full  ? "OK" : "NO");
    snprintf(line, sizeof(line), "V:%s F:%s EN:%s%s",
             v_str, f_str, lp_enable ? "ON" : "OFF",
             lp_override ? " OVR" : "");
    oled_draw_string(0, 46, line);
    oled_draw_page_dots(0);
    oled_update_display();
}
// ---------- Joystick Helper ----------

static joy_dir_t joy_read_direction(void)
{
    int vrx = 0, vry = 0;
    adc_oneshot_read(adc_handle, JOY_VRX_CHANNEL, &vrx);
    adc_oneshot_read(adc_handle, JOY_VRY_CHANNEL, &vry);

    // Prioritize the axis with the larger deflection
    int dx = vrx - JOY_CENTER;
    int dy = vry - JOY_CENTER;

    if (abs(dx) > abs(dy)) {
        if (dx < -JOY_DEADZONE) return JOY_LEFT;
        if (dx > JOY_DEADZONE)  return JOY_RIGHT;
    } else {
        if (dy < -JOY_DEADZONE) return JOY_UP;
        if (dy > JOY_DEADZONE)  return JOY_DOWN;
    }
    return JOY_NONE;
}

// ---------- Input Task (Joystick) ----------

void input_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG_BUTTON, "input_task started");

    // Joystick click (SW) state
    bool sw_current = true, sw_last = true;
    int64_t sw_debounce_time = 0;

    // Joystick direction state for repeat logic
    joy_dir_t last_dir = JOY_NONE;
    int64_t dir_start_time = 0;
    int64_t last_action_time = 0;
    bool first_action_done = false;

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;

        // --- Joystick SW (click) debounce ---
        bool sw_reading = gpio_get_level(JOY_SW_PIN);
        if (sw_reading != sw_last) sw_debounce_time = now;
        if ((now - sw_debounce_time) > DEBOUNCE_DELAY_MS) {
            if (sw_reading != sw_current) {
                sw_current = sw_reading;
                if (sw_current == 0) {  // Click detected
                    ESP_LOGI(TAG_BUTTON, "Joystick click");
                    if (g_display_mode == MODE_ADMIN_PREVIEW) {
                        enter_admin_edit();
                    } else if (g_display_mode == MODE_ADMIN_EDIT) {
                        exit_admin_edit(true);
                    } else if (g_display_mode == MODE_AUTONOMOUS) {
                        if (g_autonomous_cursor == 0) toggle_autonomous_mode();
                        else                          toggle_lp_pump_enable();
                    }
                }
            }
        }
        sw_last = sw_reading;

        // --- Joystick direction ---
        joy_dir_t dir = joy_read_direction();

        if (dir != last_dir) {
            // Direction changed — reset repeat state
            last_dir = dir;
            dir_start_time = now;
            first_action_done = false;

            // Immediate action on new direction
            if (dir != JOY_NONE) {
                first_action_done = true;
                last_action_time = now;
                goto handle_direction;
            }
        } else if (dir != JOY_NONE && first_action_done) {
            // Held in same direction — check for repeat
            int64_t held_ms = now - dir_start_time;
            if (held_ms >= JOY_INITIAL_DELAY_MS &&
                (now - last_action_time) >= JOY_REPEAT_RATE_MS) {
                last_action_time = now;
                goto handle_direction;
            }
        }
        goto skip_direction;

handle_direction:
        if (g_display_mode == MODE_AUTONOMOUS) {
            // Up/Down selects between Autonomous Mode and LP Pump Enable;
            // Left/Right still cycles to the adjacent top-level pages.
            if (dir == JOY_UP) {
                if (g_autonomous_cursor > 0) g_autonomous_cursor--;
            } else if (dir == JOY_DOWN) {
                if (g_autonomous_cursor < 1) g_autonomous_cursor++;
            } else if (dir == JOY_LEFT) {
                g_display_mode = MODE_ADMIN_PREVIEW;
                ESP_LOGI(TAG_BUTTON, "Mode -> %d", g_display_mode);
            } else if (dir == JOY_RIGHT) {
                g_display_mode = MODE_WEBPAGE;
                ESP_LOGI(TAG_BUTTON, "Mode -> %d", g_display_mode);
            }
        } else if (g_display_mode == MODE_HOME || g_display_mode == MODE_WATER ||
                   g_display_mode == MODE_SYSTEM || g_display_mode == MODE_ADMIN_PREVIEW ||
                   g_display_mode == MODE_WEBPAGE) {
            // Left/Right cycles: Home <-> Water <-> System <-> Admin Preview <-> Autonomous <-> Webpage <-> Home
            if (dir == JOY_LEFT) {
                if (g_display_mode == MODE_WATER)               g_display_mode = MODE_HOME;
                else if (g_display_mode == MODE_SYSTEM)         g_display_mode = MODE_WATER;
                else if (g_display_mode == MODE_ADMIN_PREVIEW)  g_display_mode = MODE_SYSTEM;
                else if (g_display_mode == MODE_WEBPAGE)        g_display_mode = MODE_AUTONOMOUS;
                else if (g_display_mode == MODE_HOME)           g_display_mode = MODE_WEBPAGE;
                ESP_LOGI(TAG_BUTTON, "Mode -> %d", g_display_mode);
            } else if (dir == JOY_RIGHT) {
                if (g_display_mode == MODE_HOME)                g_display_mode = MODE_WATER;
                else if (g_display_mode == MODE_WATER)          g_display_mode = MODE_SYSTEM;
                else if (g_display_mode == MODE_SYSTEM)         g_display_mode = MODE_ADMIN_PREVIEW;
                else if (g_display_mode == MODE_ADMIN_PREVIEW)  g_display_mode = MODE_AUTONOMOUS;
                else if (g_display_mode == MODE_WEBPAGE)        g_display_mode = MODE_HOME;
                ESP_LOGI(TAG_BUTTON, "Mode -> %d", g_display_mode);
            }
        } else if (g_display_mode == MODE_ADMIN_EDIT) {
            g_last_admin_activity = now;

            if (dir == JOY_UP) {
                if (g_admin_cursor > 0) g_admin_cursor--;
                ESP_LOGI(TAG_ADMIN, "Cursor -> %d", g_admin_cursor);
            } else if (dir == JOY_DOWN) {
                if (g_admin_cursor < 7) g_admin_cursor++;
                ESP_LOGI(TAG_ADMIN, "Cursor -> %d", g_admin_cursor);
            } else if (dir == JOY_LEFT || dir == JOY_RIGHT) {
                switch (g_admin_cursor) {
                    case 0:
                        if (dir == JOY_LEFT) adjust_lvd(-VOLTAGE_STEP);
                        else                 adjust_lvd(VOLTAGE_STEP);
                        break;
                    case 1:
                        if (dir == JOY_LEFT) adjust_mvr(-VOLTAGE_STEP);
                        else                 adjust_mvr(VOLTAGE_STEP);
                        break;
                    case 2:
                        g_temp_float_check = !g_temp_float_check;
                        ESP_LOGI(TAG_ADMIN, "Float check -> %d", g_temp_float_check);
                        break;
                    case 3:
                        g_temp_lvd_check = !g_temp_lvd_check;
                        ESP_LOGI(TAG_ADMIN, "LVD check -> %d", g_temp_lvd_check);
                        break;
                    case 4:
                        g_temp_lp_pump_override = !g_temp_lp_pump_override;
                        ESP_LOGI(TAG_ADMIN, "LP override -> %d", g_temp_lp_pump_override);
                        break;
                    case 5:
                        g_temp_lp_pump_enable = !g_temp_lp_pump_enable;
                        ESP_LOGI(TAG_ADMIN, "LP enable -> %d", g_temp_lp_pump_enable);
                        break;
                    case 6:
                        g_temp_hp_pump_enable = !g_temp_hp_pump_enable;
                        ESP_LOGI(TAG_ADMIN, "HP enable -> %d", g_temp_hp_pump_enable);
                        break;
                    case 7:
                        g_temp_lvd             = DEFAULT_LVD;
                        g_temp_mvr             = DEFAULT_MVR;
                        g_temp_float_check     = true;
                        g_temp_lvd_check       = true;
                        g_temp_lp_pump_override  = false;
                        g_temp_lp_pump_enable       = true;
                        g_temp_hp_pump_enable       = true;
                        ESP_LOGI(TAG_ADMIN, "Defaults restored");
                        break;
                }
            }
        }

skip_direction:
        // Admin edit timeout
        if (g_display_mode == MODE_ADMIN_EDIT) {
            if ((now - g_last_admin_activity) > ADMIN_TIMEOUT_MS) {
                ESP_LOGW(TAG_ADMIN, "Admin timeout - discarding changes");
                exit_admin_edit(false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms polling rate
    }
}

// ---------- Blink Task ----------

void blink_task(void *pvParameters)
{
    (void)pvParameters;
    
    bool led_state = false;
    
    ESP_LOGI(TAG_BLINK, "blink_task started");
    
    while (1) {
        led_state = !led_state;
        gpio_set_level(LED_PIN, led_state);
        
        // Read shared battery voltage
        float bv = 0.0f;
        int raw = 0;
        
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bv = g_battery_voltage;
            raw = g_adc_raw;
            xSemaphoreGive(state_mutex);
        }
        
        ESP_LOGI(TAG_BLINK, "LED: %s | Battery: %.2fV (ADC: %d) | Mode: %s", 
                 led_state ? "ON " : "OFF", bv, raw,
                 g_display_mode == MODE_ADMIN_EDIT ? "ADMIN" : "NORMAL");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// ---------- Temp Task ----------
void temp_task(void *pvParameters)
{
    (void)pvParameters;

    float temp_c = 0.0f;

    ESP_LOGI("TEMP_TASK", "temp_task started");

    while (1) {
        esp_err_t err = temp_read_celsius(&temp_c);

        if (err == ESP_OK) {
              temp_c -= 0.5f; 

            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_temp_c = temp_c;
                g_temp_valid = true;
                xSemaphoreGive(state_mutex);
            }

            ESP_LOGI("TEMP_TASK", "Water Temp: %.2f C", temp_c);
        } else {
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_temp_valid = false;
                xSemaphoreGive(state_mutex);
            }

            ESP_LOGE("TEMP_TASK", "Temp read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------- ADC Task ----------

void adc_task(void *pvParameters)
{
    (void)pvParameters;

    int tds_buffer[TDS_ADC_SAMPLES] = {0};
    int tds_buffer_index = 0;
    int tds_sum = 0;

    #define BAT_SCALE       5.7f
    #define BAT_CAL_FACTOR  1.0f  // was 1.048, tuned for the old uncalibrated ADC path;
                                  // redundant now that adc_cali corrects the raw reading
    #define BAT_AVG_SAMPLES 100  // 100 samples @ 50ms loop = ~5s rolling window

    TickType_t last_battery_publish = 0;

    static float battery_buffer[BAT_AVG_SAMPLES] = {0};
    static int battery_index = 0;
    static int battery_count = 0;
    static float battery_sum = 0.0f;
    static float battery_voltage = 0.0f;

    ESP_LOGI(TAG_ADC, "adc_task started");
    ESP_LOGI(TAG_ADC, "Battery: GPIO34 (ADC1_CH6)");
    ESP_LOGI(TAG_ADC, "Battery voltage range: %.1fV - %.1fV", BAT_V_MIN, BAT_V_MAX);
    ESP_LOGI(TAG_ADC, "Current Sense: GPIO32 (ADC1_CH4");

    while (1) {
        // --- Battery ADC ---
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG_ADC, "Battery ADC read failed: %s",
                     esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        float adc_voltage = adc_raw_to_mv(raw) / 1000.0f;
        float sample_voltage = adc_voltage * BAT_SCALE * BAT_CAL_FACTOR;

        // Feed every sample into a rolling buffer, but only publish
        // (update battery_voltage) once every 5 seconds for a stable reading.
        battery_sum -= battery_buffer[battery_index];
        battery_buffer[battery_index] = sample_voltage;
        battery_sum += sample_voltage;
        battery_index = (battery_index + 1) % BAT_AVG_SAMPLES;
        if (battery_count < BAT_AVG_SAMPLES) battery_count++;

        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_battery_publish) >= pdMS_TO_TICKS(5000)) {
            last_battery_publish = current_time;
            battery_voltage = battery_sum / battery_count;
        }
        // --- TDS ADC ---
        int tds_raw = 0;
        float temp_c_for_tds = 25.0f;   // fallback if temp is invalid
        float tds_voltage = 0.0f;
        float tds_ppm = 0.0f;
        bool tds_valid = false;

        
        ret = adc_oneshot_read(adc_handle, TDS_ADC_CHANNEL, &tds_raw);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG_ADC, "TDS ADC read failed: %s", esp_err_to_name(ret));
        } else {
            tds_sum -= tds_buffer[tds_buffer_index];
            tds_buffer[tds_buffer_index] = tds_raw;
            tds_sum += tds_raw;
            tds_buffer_index = (tds_buffer_index + 1) % TDS_ADC_SAMPLES;

            int tds_avg = tds_sum / TDS_ADC_SAMPLES;
            tds_voltage = adc_raw_to_mv(tds_avg) / 1000.0f;

            // Get latest temperature for compensation
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (g_temp_valid) {
                    temp_c_for_tds = g_temp_c;
                }
                xSemaphoreGive(state_mutex);
            }

            // Convert ADC reading to ppm using your method
            tds_ppm = tds_calculate_ppm(tds_avg, temp_c_for_tds);
            tds_valid = true;

            // Update TDS globals
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_tds_raw = tds_avg;
                g_tds_voltage = tds_voltage;
                g_tds_ppm = tds_ppm;
                g_tds_valid = tds_valid;
                xSemaphoreGive(state_mutex);
            }
        }
        
        /*
        // --- Current Sense ADC (IS 1+2, GPIO32) ---
        float current_amps = 0.0f;
        int is_raw = 0;
        bool current_valid = false;
        
        if (adc_oneshot_read(adc_handle, IS_1_2_CHANNEL, &is_raw) == ESP_OK)
        {
            // IS pin outputs ~1.2kA/A — with a 4.7kΩ sense resistor: V = I_sense * R
            // BTS700x kILIS = 22900 (typ), so I_load = (V_IS / R_sense) * kILIS
            float v_is = adc_raw_to_mv(is_raw) / 1000.0f;
            current_amps = (v_is/ 4730.0f) *22900.0f;
            current_valid = true; 
              ESP_LOGI(TAG_ADC, "Current = %.3f A (ADC=%d, V_IxS=%.3f V)", current_amps, is_raw, v_is);
        }  
        else {
            ESP_LOGE(TAG_ADC, "Current sense ADC read failed");
            }
        */
        // Update other global variables
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            g_adc_raw = raw;
            g_battery_voltage = battery_voltage;
            // g_current_amps = current_amps;
            // g_current_valid = current_valid;

            if (ret != ESP_OK) {
                g_tds_valid = false;
            }

            xSemaphoreGive(state_mutex);
        }

        // Log occasionally
        static int log_counter = 0;
        if (++log_counter >= 10) {
            if (tds_valid) {
                ESP_LOGI(TAG_ADC,
                         "Batt: %4d (%.2fV) | TDS: %4d (%.3fV, %.1f ppm, %.1fC)",
                         raw, battery_voltage,
                         tds_raw, tds_voltage, tds_ppm, temp_c_for_tds);
            } else {
                ESP_LOGI(TAG_ADC,
                         "Batt: %4d (%.2fV) | TDS: ERR",
                         raw, battery_voltage);
            }
            log_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------- OLED Task ----------

void oled_task(void *pvParameters)
{
    (void)pvParameters;
    
    ESP_LOGI(TAG_OLED, "oled_task started");
    
    // Startup splash screen with frog graphic
    frogs_show_splash();
    
    while (1) {
        // Display based on current mode
        switch (g_display_mode) {
            case MODE_WATER:          display_water_mode();    break;
           // case MODE_SYSTEM:         display_system_mode();   break;
            case MODE_ADMIN_PREVIEW:  display_admin_preview(); break;
            case MODE_WEBPAGE:        display_webpage_preview(); break;
            case MODE_AUTONOMOUS:     display_autonomous_mode(); break;
            case MODE_ADMIN_EDIT:     display_admin_mode();    break;
            default:                  display_home_mode();     break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void pump_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI("PUMP", "pump_task started");

    // Seed the hysteresis latch from the actual battery voltage at boot so the
    // pump doesn't have to climb all the way to MVR just because the system
    // restarted. MVR only gates re-arming after a real LVD trip during runtime.
    // adc_task only publishes its first averaged reading ~5s after boot, so
    // wait for a non-zero reading (rather than trusting the 0.0V startup
    // default) before seeding the latch.
    float boot_bv = 0.0f, boot_lvd = DEFAULT_LVD;
    for (int waited_ms = 0; waited_ms < 6000; waited_ms += 100) {
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            boot_bv = g_battery_voltage;
            xSemaphoreGive(state_mutex);
        }
        if (boot_bv > 0.0f) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        boot_lvd = g_current_lvd;
        xSemaphoreGive(voltage_mutex);
    }
    bool pump_enabled = (boot_bv >= boot_lvd);   // voltage-based enable (LVD/MVR hysteresis)
    ESP_LOGI("PUMP", "Startup voltage %.2fV, LVD %.1fV -> pump_enabled=%d", boot_bv, boot_lvd, pump_enabled);

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;  // ms

        float bv = 0.0f;
        float lvd = 0.0f;
        float mvr = 0.0f;

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bv = g_battery_voltage;
            xSemaphoreGive(state_mutex);
        }

        if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lvd = g_current_lvd;
            mvr = g_current_mvr;
            xSemaphoreGive(voltage_mutex);
        }

        // --- Voltage hysteresis (unchanged logic) ---
        if (pump_enabled && bv < lvd) {
            pump_enabled = false;
            ESP_LOGW("PUMP", "Battery %.2fV below LVD %.1fV - PUMP OFF", bv, lvd);
        } else if (!pump_enabled && bv >= mvr) {
            pump_enabled = true;
            ESP_LOGI("PUMP", "Battery %.2fV above MVR %.1fV - PUMP ON", bv, mvr);
        }

        // Read float switch and feed flow
        bool tank_full = false;
        float flow1 = 0.0f;
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            tank_full = g_tank_full;
            flow1 = g_flow1_gpm;
            xSemaphoreGive(state_mutex);
        }

        // Read condition toggles, override, and shutoffs
        bool float_chk = true, lvd_chk = true;
        bool lp_pump_override = false, lp_pump_enable = true, hp_pump_enable = true;
        bool web_admin_mode = false, autonomous_mode = false;
        if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            float_chk   = g_float_check;
            lvd_chk     = g_lvd_check;
            lp_pump_override = g_lp_pump_override;
            lp_pump_enable   = g_lp_pump_enable;
            hp_pump_enable   = g_hp_pump_enable;
            web_admin_mode   = g_web_admin_mode;
            autonomous_mode  = g_autonomous_mode;
            xSemaphoreGive(voltage_mutex);
        }

        // --- Condition-failure callouts (Autonomous Mode only, edge-triggered
        // so each failure is announced once, not every 500ms it stays failed) ---
        static bool prev_float_ok = true, prev_voltage_ok = true;
        bool float_ok_now   = float_chk ? !tank_full   : true;
        bool voltage_ok_now = lvd_chk   ? pump_enabled : true;
        if (autonomous_mode) {
            if (prev_float_ok && !float_ok_now) {
                audio_play(AUDIO_CLIP_FLOAT_CHECK_FAILED);
            }
            if (prev_voltage_ok && !voltage_ok_now) {
                audio_play(AUDIO_CLIP_VOLTAGE_CHECK_FAILED);
            }
        }
        prev_float_ok = float_ok_now;
        prev_voltage_ok = voltage_ok_now;

        // --- Final pump 1 decision ---
        // Shutoff (_enable) takes priority over everything including override
        bool conditions_met = lp_pump_override
                           || (float_ok_now && voltage_ok_now);
        bool active = !web_admin_mode
                   && lp_pump_enable
                   && conditions_met
                   && (g_display_mode != MODE_ADMIN_EDIT);

        gpio_set_level(LP_PUMP, active ? 1 : 0);

        // --- Pump 2: turns on after PUMP2_DELAY_MS of Pump 1 running, and
        // only once feed flow confirms water is actually moving. Along the
        // way, announce the countdown (assumes PUMP2_DELAY_MS == 60000). ---
        static int64_t lp_pump_start_time = 0;
        static bool announced_30s = false;
        static int last_announced_count = 0;  // last of 5..1 announced, 0 = none yet
        bool hp_pump_active = false;

        if (active) {
            if (lp_pump_start_time == 0) {
                lp_pump_start_time = now;
                announced_30s = false;
                last_announced_count = 0;
                ESP_LOGI("PUMP2", "Pump 1 started, waiting %ds before Pump 2",
                         HP_PUMP_DELAY_MS / 1000);
                audio_play(AUDIO_CLIP_COUNTDOWN_60S);
            }

            int64_t elapsed = now - lp_pump_start_time;
            int64_t remaining_ms = HP_PUMP_DELAY_MS - elapsed;

            if (!announced_30s && elapsed >= HP_PUMP_DELAY_MS / 2) {
                announced_30s = true;
                audio_play(AUDIO_CLIP_COUNTDOWN_30S);
            }

            if (remaining_ms > 0) {
                int remaining_s = (int)((remaining_ms + 999) / 1000);  // ceil to whole seconds
                int prev_threshold = (last_announced_count == 0) ? 6 : last_announced_count;
                if (remaining_s <= 5 && remaining_s < prev_threshold) {
                    static const audio_clip_id_t count_clips[5] = {
                        AUDIO_CLIP_COUNT_1, AUDIO_CLIP_COUNT_2, AUDIO_CLIP_COUNT_3,
                        AUDIO_CLIP_COUNT_4, AUDIO_CLIP_COUNT_5
                    };
                    audio_play(count_clips[remaining_s - 1]);
                    last_announced_count = remaining_s;
                }
            }

            if (hp_pump_enable
                && elapsed >= HP_PUMP_DELAY_MS
                && flow1 > HP_MIN_FLOW_GPM) {
                hp_pump_active = true;
            }
        } else {
            if (lp_pump_start_time != 0) {
                ESP_LOGI("HP_PUMP", "LP Pump stopped, HP Pump 2 OFF");
            }
            lp_pump_start_time = 0;
        }

        gpio_set_level(HP_PUMP, hp_pump_active ? 1 : 0);

        static int pump_log_counter = 0;
        if (++pump_log_counter >= 4) {
            ESP_LOGI("PUMP", "v_ok=%d lpen=%d hpen=%d ovrd=%d admin=%d flow1=%.2f -> lp_pump=%d hp_pump=%d",
                     pump_enabled, lp_pump_enable, hp_pump_enable, lp_pump_override,
                     web_admin_mode, flow1, active, hp_pump_active && hp_pump_enable);
            pump_log_counter = 0;
        }

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_lp_pump_running = active;
            g_hp_pump_running = hp_pump_active;
            g_pump_voltage_ok = pump_enabled;
            xSemaphoreGive(state_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------- Flow Sensor Counters ----------

static volatile uint32_t g_flow1_pulses = 0;
static volatile uint32_t g_flow2_pulses = 0;

static volatile int64_t g_flow1_last_us = 0;
static volatile int64_t g_flow2_last_us = 0;

// Protect shared pulse counters
static portMUX_TYPE flow1_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE flow2_mux = portMUX_INITIALIZER_UNLOCKED;

// Reject edges closer than 300 microseconds
#define FLOW_DEBOUNCE_US 1000

// Example:
// FLOW_CAL = pulses per second for 1 L/min

static void IRAM_ATTR flow1_isr_handler(void *arg)
{
    (void)arg;

    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&flow1_mux);
    
    if ((now - g_flow1_last_us) >= FLOW_DEBOUNCE_US) {
        g_flow1_pulses++;
        g_flow1_last_us = now;
    }
    portEXIT_CRITICAL_ISR(&flow1_mux);
}

static void IRAM_ATTR flow2_isr_handler(void *arg)
{
    (void)arg;

    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&flow2_mux);

    if ((now - g_flow2_last_us) >= FLOW_DEBOUNCE_US) {
        g_flow2_pulses++;
        g_flow2_last_us = now;
    }
    portEXIT_CRITICAL_ISR(&flow2_mux);
}

// ---------- Flow Sensor Task ----------

void flow_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI("FLOW", "flow_task started");

    int64_t window_start_us = esp_timer_get_time();

    while (1) {
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = now_us - window_start_us;

        if (elapsed_us >= ((int64_t)FLOW_SAMPLE_MS * 1000)) {
            uint32_t count1;
            uint32_t count2;

            // Atomically copy and reset the pulse counters
            portENTER_CRITICAL(&flow1_mux);
            count1 = g_flow1_pulses;
            g_flow1_pulses = 0;
            portEXIT_CRITICAL(&flow1_mux);

            portENTER_CRITICAL(&flow2_mux);
            count2 = g_flow2_pulses;
            g_flow2_pulses = 0;
            portEXIT_CRITICAL(&flow2_mux);

            // Convert the actual sample period to seconds
            float elapsed_seconds = (float)elapsed_us / 1000000.0f;

            // Convert pulse count to pulse frequency
            float frequency1_hz = (float)count1 / elapsed_seconds;
            float frequency2_hz = (float)count2 / elapsed_seconds;
            
            float lpm1 = frequency1_hz / FLOW_CAL;
            float lpm2 = frequency2_hz / FLOW_CAL;
            // FLOW_CAL is Hz per L/min
            float gpm1 = lpm1 * LITERS_TO_US_GALLONS;
            float gpm2 = lpm2 * LITERS_TO_US_GALLONS;

            // NC float switch: HIGH = open (float risen) = tank full
            bool tank_full = (gpio_get_level(FLOAT_SW_PIN) == 1);

            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_flow1_gpm = gpm1;
                g_flow2_gpm = gpm2;
                g_tank_full = tank_full;

                xSemaphoreGive(state_mutex);
            }

            ESP_LOGI(
                "FLOW",
                "F1: %.2f g/min (%lu pulses) | "
                "F2: %.2f g/min (%lu pulses) | "
                "Float: %s",
                gpm1,
                (unsigned long)count1,
                gpm2,
                (unsigned long)count2,
                tank_full ? "LOW" : "NOT LOW"
            );

            window_start_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// -----------TDS to ppm Conversion--------



float tds_calculate_ppm(int adc_raw, float temp_c)
{
    // Convert ADC to voltage (ESP32 is 12-bit)
    float voltage = (adc_raw / 4095.0f) * 3.3f;

    // Temperature compensation (from datasheet)
    float compensation_coefficient = 1.0f + 0.02f * (temp_c - 25.0f);
    float compensated_voltage = voltage / compensation_coefficient;

    // TDS conversion (empirical cubic fit)
    float tds_ppm = (133.42f * compensated_voltage * compensated_voltage * compensated_voltage
                   -255.86f * compensated_voltage * compensated_voltage
                   +857.39f * compensated_voltage) * 0.5f;

    return tds_ppm;
}

// ---------- New Addition Wi-Fi Access Point ----------

static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "FROGS_Controller",
            .ssid_len = strlen("FROGS_Controller"),
            .password = "frogspassword",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WIFI", "FROGS WiFi started");
    ESP_LOGI("WIFI", "SSID: FROGS_Controller");
    ESP_LOGI("WIFI", "Open browser to http://192.168.4.1");
}

// ---------- Web Server Handlers ----------

#define WEB_REQUEST_BODY_MAX 256

static esp_err_t send_json_response(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t send_json_error(httpd_req_t *req, const char *status, const char *message)
{
    char json[192];
    snprintf(json, sizeof(json), "{\"ok\":false,\"error\":\"%s\"}", message);
    httpd_resp_set_status(req, status);
    return send_json_response(req, json);
}

static esp_err_t read_request_body(httpd_req_t *req, char *body, size_t body_size)
{
    if (req->content_len <= 0 || (size_t)req->content_len >= body_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t received = 0;
    while (received < (size_t)req->content_len) {
        int result = httpd_req_recv(req, body + received, req->content_len - received);
        if (result == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (result <= 0) return ESP_FAIL;
        received += (size_t)result;
    }

    body[received] = '\0';
    return ESP_OK;
}

static const char *json_value_start(const char *json, const char *key)
{
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *value = strstr(json, pattern);
    if (value == NULL) return NULL;

    value = strchr(value + strlen(pattern), ':');
    if (value == NULL) return NULL;
    value++;

    while (*value == ' ' || *value == '\t' || *value == '\r' || *value == '\n') value++;
    return value;
}

static bool json_get_bool_value(const char *json, const char *key, bool *value)
{
    const char *start = json_value_start(json, key);
    if (start == NULL) return false;

    if (strncmp(start, "true", 4) == 0 || *start == '1') {
        *value = true;
        return true;
    }
    if (strncmp(start, "false", 5) == 0 || *start == '0') {
        *value = false;
        return true;
    }
    return false;
}

static bool json_get_float_value(const char *json, const char *key, float *value)
{
    const char *start = json_value_start(json, key);
    if (start == NULL) return false;

    char *end = NULL;
    float parsed = strtof(start, &end);
    if (end == start || !isfinite(parsed)) return false;

    *value = parsed;
    return true;
}

static bool json_get_string_value(const char *json, const char *key,
                                  char *value, size_t value_size)
{
    const char *start = json_value_start(json, key);
    if (start == NULL || *start != '"' || value_size == 0) return false;
    start++;

    const char *end = strchr(start, '"');
    if (end == NULL) return false;

    size_t length = (size_t)(end - start);
    if (length >= value_size) length = value_size - 1;
    memcpy(value, start, length);
    value[length] = '\0';
    return true;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI("WEB", "Serving webpage (%d bytes)", strlen(html_page));

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t data_get_handler(httpd_req_t *req)
{
    char json[640];

    float battery = 0.0f;
    float temp_c = 0.0f;
    float tds = 0.0f;
    float flow1 = 0.0f;
    float flow2 = 0.0f;
    bool lp_pump = false;
    bool hp_pump = false;
    bool lp_enabled = true;
    bool hp_enabled = true;
    bool temp_valid = false;
    bool tds_valid = false;
    bool tank_full = false;

    bool float_check = true;
    bool lvd_check = true;
    bool lp_override = false;
    bool autonomous_mode = false;
    float lvd = 0.0f;
    float mvr = 0.0f;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        battery = g_battery_voltage;
        temp_c = g_temp_c;
        tds = g_tds_ppm;
        flow1 = g_flow1_gpm;
        flow2 = g_flow2_gpm;
        lp_pump = g_lp_pump_running;
        hp_pump = g_hp_pump_running;
        temp_valid = g_temp_valid;
        tds_valid = g_tds_valid;
        tank_full = g_tank_full;
        xSemaphoreGive(state_mutex);
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lp_enabled = g_lp_pump_enable;
        hp_enabled = g_hp_pump_enable;
        lvd = g_current_lvd;
        mvr = g_current_mvr;
        float_check = g_float_check;
        lvd_check = g_lvd_check;
        lp_override = g_lp_pump_override;
        autonomous_mode = g_autonomous_mode;
        xSemaphoreGive(voltage_mutex);
    }

    float temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;
    float total_flow = flow1 + flow2;

    const char *blocked;
    if (battery < lvd) {
        blocked = "LOW V";
    } else if (tank_full) {
        blocked = "TANK";
    } else if (battery < mvr && !lp_pump) {
        blocked = "WAITING FOR MVR";
    } else {
        blocked = "NONE";
    }

    snprintf(json, sizeof(json),
             "{\"tds\":%.1f,\"temperature\":%.1f,\"flowRate\":%.2f,"
             "\"flow1\":%.2f,\"flow2\":%.2f,"
             "\"battery\":%.2f,\"pump\":\"LP:%s HP:%s\","
             "\"lpPumpEnabled\":%s,\"hpPumpEnabled\":%s,"
             "\"tempValid\":%s,\"tdsValid\":%s,"
             "\"floatUp\":%s,\"blocked\":\"%s\","
             "\"lvd\":%.2f,\"mvr\":%.2f,"
             "\"conditionChecks\":{\"float\":%s,\"lvd\":%s},"
             "\"lpOverride\":%s,\"autonomousMode\":%s}",
             tds,
             temp_f,
             total_flow,
             flow1,
             flow2,
             battery,
             lp_pump ? "ON" : "OFF",
             hp_pump ? "ON" : "OFF",
             lp_enabled ? "true" : "false",
             hp_enabled ? "true" : "false",
             temp_valid ? "true" : "false",
             tds_valid ? "true" : "false",
             tank_full ? "true" : "false",
             blocked,
             lvd,
             mvr,
             float_check ? "true" : "false",
             lvd_check ? "true" : "false",
             lp_override ? "true" : "false",
             autonomous_mode ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static void save_web_settings_locked(void)
{
    sync_existing_oled_admin_values_locked();
    g_last_admin_activity = esp_timer_get_time() / 1000;
    nvs_save_voltages();
}

static esp_err_t set_pump_enable_handler(httpd_req_t *req,
                                         bool low_pressure,
                                         bool enabled)
{
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    if (low_pressure) g_lp_pump_enable = enabled;
    else g_hp_pump_enable = enabled;

    save_web_settings_locked();
    xSemaphoreGive(voltage_mutex);
    return httpd_resp_sendstr(req, enabled ? "Pump enabled" : "Pump disabled");
}

static esp_err_t lp_enable_handler(httpd_req_t *req)
{
    return set_pump_enable_handler(req, true, true);
}

static esp_err_t lp_disable_handler(httpd_req_t *req)
{
    return set_pump_enable_handler(req, true, false);
}

static esp_err_t hp_enable_handler(httpd_req_t *req)
{
    return set_pump_enable_handler(req, false, true);
}

static esp_err_t hp_disable_handler(httpd_req_t *req)
{
    return set_pump_enable_handler(req, false, false);
}

static esp_err_t admin_mode_handler(httpd_req_t *req)
{
    char query[48];
    char value[12];
    bool requested_state = false;
    bool change_requested = false;

    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0) {
        if (query_len >= sizeof(query) ||
            httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
            httpd_query_key_value(query, "enabled", value, sizeof(value)) != ESP_OK) {
            return send_json_error(req, "400 Bad Request", "Expected enabled=1 or enabled=0");
        }

        if (strcmp(value, "1") == 0) {
            requested_state = true;
        } else if (strcmp(value, "0") == 0) {
            requested_state = false;
        } else {
            return send_json_error(req, "400 Bad Request", "Expected enabled=1 or enabled=0");
        }

        change_requested = true;
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    if (change_requested) {
        g_web_admin_mode = requested_state;
    }

    bool enabled = g_web_admin_mode;
    xSemaphoreGive(voltage_mutex);

    if (change_requested) {
        ESP_LOGW("WEB", "Website Admin Mode=%d; both pumps are %s",
                 enabled, enabled ? "forced OFF" : "returned to normal control");
    }

    char json[32];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"enabled\":%s}",
             enabled ? "true" : "false");
    return send_json_response(req, json);
}

static esp_err_t admin_settings_handler(httpd_req_t *req)
{
    char body[WEB_REQUEST_BODY_MAX];
    float lvd = 0.0f;
    float mvr = 0.0f;

    if (read_request_body(req, body, sizeof(body)) != ESP_OK ||
        !json_get_float_value(body, "lvd", &lvd) ||
        !json_get_float_value(body, "mvr", &mvr)) {
        return send_json_error(req, "400 Bad Request", "Expected numeric lvd and mvr values");
    }

    lvd = roundf(lvd * 10.0f) / 10.0f;
    mvr = roundf(mvr * 10.0f) / 10.0f;

    if (lvd < LVD_MIN || lvd > LVD_MAX ||
        mvr < MVR_MIN || mvr > MVR_MAX ||
        mvr < lvd + MIN_HYSTERESIS) {
        return send_json_error(req, "400 Bad Request", "Voltage values are outside the permitted range");
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    g_current_lvd = lvd;
    g_current_mvr = mvr;
    save_web_settings_locked();
    xSemaphoreGive(voltage_mutex);

    ESP_LOGI("WEB", "Web updated LVD=%.1fV MVR=%.1fV", lvd, mvr);
    return send_json_response(req, "{\"ok\":true}");
}

static esp_err_t admin_check_handler(httpd_req_t *req)
{
    char body[WEB_REQUEST_BODY_MAX];
    char check[16];
    bool enabled = false;

    if (read_request_body(req, body, sizeof(body)) != ESP_OK ||
        !json_get_string_value(body, "check", check, sizeof(check)) ||
        !json_get_bool_value(body, "enabled", &enabled)) {
        return send_json_error(req, "400 Bad Request", "Expected check and enabled values");
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    if (strcmp(check, "float") == 0) g_float_check = enabled;
    else if (strcmp(check, "lvd") == 0) g_lvd_check = enabled;
    else {
        xSemaphoreGive(voltage_mutex);
        return send_json_error(req, "400 Bad Request", "Unknown safety check");
    }

    save_web_settings_locked();
    xSemaphoreGive(voltage_mutex);

    ESP_LOGI("WEB", "Web updated %s check=%d", check, enabled);
    return send_json_response(req, "{\"ok\":true}");
}

static esp_err_t admin_override_handler(httpd_req_t *req)
{
    char body[WEB_REQUEST_BODY_MAX];
    char pump[16];
    bool enabled = false;

    if (read_request_body(req, body, sizeof(body)) != ESP_OK ||
        !json_get_string_value(body, "pump", pump, sizeof(pump)) ||
        !json_get_bool_value(body, "enabled", &enabled)) {
        return send_json_error(req, "400 Bad Request", "Expected pump and enabled values");
    }

    if (strcmp(pump, "lp") != 0) {
        return send_json_error(req, "403 Forbidden", "Only the low-pressure pump can be overridden");
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    g_lp_pump_override = enabled;
    sync_existing_oled_admin_values_locked();
    g_last_admin_activity = esp_timer_get_time() / 1000;
    xSemaphoreGive(voltage_mutex);

    // The original firmware keeps this override runtime-only.
    ESP_LOGW("WEB", "Web updated LP override=%d", enabled);
    return send_json_response(req, "{\"ok\":true}");
}

static esp_err_t admin_autonomous_handler(httpd_req_t *req)
{
    char body[WEB_REQUEST_BODY_MAX];
    bool enabled = false;

    if (read_request_body(req, body, sizeof(body)) != ESP_OK ||
        !json_get_bool_value(body, "enabled", &enabled)) {
        return send_json_error(req, "400 Bad Request", "Expected enabled value");
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    apply_autonomous_mode_locked(enabled);
    save_web_settings_locked();
    xSemaphoreGive(voltage_mutex);

    ESP_LOGI("WEB", "Web updated Autonomous mode=%d", enabled);
    audio_play(enabled ? AUDIO_CLIP_AUTONOMOUS_ON : AUDIO_CLIP_AUTONOMOUS_OFF);
    return send_json_response(req, "{\"ok\":true}");
}

static esp_err_t admin_reset_handler(httpd_req_t *req)
{
    char body[WEB_REQUEST_BODY_MAX];
    bool reset = false;

    if (read_request_body(req, body, sizeof(body)) != ESP_OK ||
        !json_get_bool_value(body, "reset", &reset) || !reset) {
        return send_json_error(req, "400 Bad Request", "Reset confirmation was not provided");
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    g_current_lvd = DEFAULT_LVD;
    g_current_mvr = DEFAULT_MVR;
    g_float_check = true;
    g_lvd_check = true;
    g_lp_pump_override = false;
    g_lp_pump_enable = true;
    g_hp_pump_enable = true;
    g_web_admin_mode = false;

    save_web_settings_locked();
    xSemaphoreGive(voltage_mutex);

    ESP_LOGW("WEB", "Web restored the original Admin defaults");
    return send_json_response(req, "{\"ok\":true}");
}

static void start_webserver(void)
{
    static httpd_handle_t server = NULL;

    if (server != NULL) {
        ESP_LOGW("WEB", "Web server already running");
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 4096;
    config.max_open_sockets = 2;
    config.max_uri_handlers = 13;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    const httpd_uri_t routes[] = {
        { .uri = "/",               .method = HTTP_GET,  .handler = root_get_handler,       .user_ctx = NULL },
        { .uri = "/data",           .method = HTTP_GET,  .handler = data_get_handler,       .user_ctx = NULL },
        { .uri = "/lp/enable",      .method = HTTP_GET,  .handler = lp_enable_handler,      .user_ctx = NULL },
        { .uri = "/lp/disable",     .method = HTTP_GET,  .handler = lp_disable_handler,     .user_ctx = NULL },
        { .uri = "/hp/enable",      .method = HTTP_GET,  .handler = hp_enable_handler,      .user_ctx = NULL },
        { .uri = "/hp/disable",     .method = HTTP_GET,  .handler = hp_disable_handler,     .user_ctx = NULL },
        { .uri = "/admin/mode",     .method = HTTP_GET,  .handler = admin_mode_handler,     .user_ctx = NULL },
        { .uri = "/admin/settings", .method = HTTP_POST, .handler = admin_settings_handler, .user_ctx = NULL },
        { .uri = "/admin/check",    .method = HTTP_POST, .handler = admin_check_handler,    .user_ctx = NULL },
        { .uri = "/admin/override", .method = HTTP_POST, .handler = admin_override_handler, .user_ctx = NULL },
        { .uri = "/admin/autonomous", .method = HTTP_POST, .handler = admin_autonomous_handler, .user_ctx = NULL },
        { .uri = "/admin/reset",    .method = HTTP_POST, .handler = admin_reset_handler,    .user_ctx = NULL },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI("WEB", "Web server started with %d URI handlers",
             (int)(sizeof(routes) / sizeof(routes[0])));
}


// ---------- Main Application ----------

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "=== Solar Charge Controller with Admin Mode ===");
    ESP_LOGI(TAG_MAIN, "LiFePO4 Battery Protection System");

    // Initialize NVS
    nvs_init_storage();

    // Load voltage settings
    nvs_load_voltages();

    // Create mutexes
    state_mutex = xSemaphoreCreateMutex();
    voltage_mutex = xSemaphoreCreateMutex();

    if (state_mutex == NULL || voltage_mutex == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create mutexes!");
        return;
    }
    ESP_LOGI(TAG_MAIN, "Mutexes created");

    // Start ESP32 Wi-Fi access point and web server
    wifi_init_ap();
    start_webserver();
    
    //ESP_ERROR_CHECK(ble_init());
    //ESP_LOGI(TAG_MAIN, "BLE initialized");
    // Configure LED GPIO
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    ESP_LOGI(TAG_MAIN, "LED GPIO configured");

    // Configure pump GPIO
    gpio_reset_pin(LP_PUMP);
    gpio_set_direction(LP_PUMP, GPIO_MODE_OUTPUT);
    gpio_set_level(LP_PUMP, 0);
    ESP_LOGI(TAG_MAIN, "Pump 1 configured: GPIO%d (U1)", LP_PUMP);

    gpio_reset_pin(HP_PUMP);
    gpio_set_direction(HP_PUMP, GPIO_MODE_OUTPUT);
    gpio_set_level(HP_PUMP, 0);
    ESP_LOGI(TAG_MAIN, "Pump 2 configured: GPIO%d (U2)", HP_PUMP);

    // Configure joystick SW (click) GPIO
    gpio_reset_pin(JOY_SW_PIN);
    gpio_set_direction(JOY_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOY_SW_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG_MAIN, "Joystick SW: GPIO%d", JOY_SW_PIN);


    ESP_LOGI(TAG_MAIN, "Joystick VRx: GPIO36 (ADC1_CH0)");
    ESP_LOGI(TAG_MAIN, "Joystick VRy: GPIO39 (ADC1_CH3)");

    // Configure ADC1
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    adc_oneshot_chan_cfg_t adc_chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_VRX_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_VRY_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TDS_ADC_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, IS_1_2_CHANNEL, &adc_chan_config));

    adc_calibration_init();
    ESP_LOGI(TAG_MAIN, "ADC1 configured: battery(GPIO34), JoyX(GPIO36), JoyY(GPIO39), TDS(GPIO33), IS(GPIO32)");
    

    // Configure DEN 1+3 pin (diagnosis enable for U1+U3)
    gpio_reset_pin(DEN_1_3_PIN);
    gpio_set_direction(DEN_1_3_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DEN_1_3_PIN, 1);
    ESP_LOGI(TAG_MAIN, "DEN 1+3 enabled on GPIO%d", DEN_1_3_PIN);

    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG_MAIN, "I2C initialized on SDA=%d, SCL=%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // Initialize OLED
    oled_init();
    ESP_LOGI(TAG_MAIN, "OLED initialized (128x64, 0x%02X)", OLED_I2C_ADDR);

    // Initialize temperature sensor
    esp_err_t temp_err = temp_init(TEMP_SENSOR_PIN);
    if (temp_err == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "Temperature sensor initialized on GPIO%d", TEMP_SENSOR_PIN);
    } else {
        ESP_LOGW(TAG_MAIN, "Temperature sensor init failed: %s", esp_err_to_name(temp_err));
    }

    // Initialize voice audio (LM386 via DAC1 / GPIO25)
    audio_init();

    // Configure flow sensor GPIOs (pull-up, active-low pulses, interrupt-driven counting)
    gpio_reset_pin(FEED_FLOW_PIN);
    gpio_set_direction(FEED_FLOW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(FEED_FLOW_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(FEED_FLOW_PIN, GPIO_INTR_NEGEDGE);
    ESP_LOGI(TAG_MAIN, "Flow sensor 1: GPIO%d", FEED_FLOW_PIN);

    gpio_reset_pin(PRODUCT_FLOW_PIN);
    gpio_set_direction(PRODUCT_FLOW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PRODUCT_FLOW_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(PRODUCT_FLOW_PIN, GPIO_INTR_NEGEDGE);
    ESP_LOGI(TAG_MAIN, "Flow sensor 2: GPIO%d", PRODUCT_FLOW_PIN);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(FEED_FLOW_PIN, flow1_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PRODUCT_FLOW_PIN, flow2_isr_handler, NULL));

    // Configure float switch GPIO (NC switch, pull-up: HIGH = open = tank full)
    gpio_reset_pin(FLOAT_SW_PIN);
    gpio_set_direction(FLOAT_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(FLOAT_SW_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG_MAIN, "Float switch: GPIO%d (HIGH=full, LOW=not full)", FLOAT_SW_PIN);

    // Initialize default sensor state
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_temp_c = 0.0f;
        g_temp_valid = false;

        g_tds_raw = 0;
        g_tds_voltage = 0.0f;
        g_tds_ppm = 0.0f;
        g_tds_valid = false;

        xSemaphoreGive(state_mutex);
    }

    // Create tasks
    xTaskCreate(blink_task, "blink_task", 2048, NULL, 1, NULL);
    xTaskCreate(temp_task, "temp_task", 4096, NULL, 2, NULL);
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 2, NULL);
    xTaskCreate(oled_task, "oled_task", 4096, NULL, 1, NULL);
    xTaskCreate(input_task, "input_task", 4096, NULL, 3, NULL);
    xTaskCreate(pump_task, "pump_task", 4096, NULL, 2, NULL);
    xTaskCreate(flow_task, "flow_task", 2048, NULL, 2, NULL);

    ESP_LOGI(TAG_MAIN, "All tasks started successfully");
    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "Joystick L/R: cycle modes (Normal > Metrics > Admin)");
    ESP_LOGI(TAG_MAIN, "Joystick click: enter Admin edit mode");
    ESP_LOGI(TAG_MAIN, "Admin: U/D select, L/R adjust, EXIT button saves");
    ESP_LOGI(TAG_MAIN, "Temperature sensor on GPIO13");
    ESP_LOGI(TAG_MAIN, "TDS sensor on GPIO33");
    ESP_LOGI(TAG_MAIN, "===========================================");

}
