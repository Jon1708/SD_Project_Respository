#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "temp.h"
#include "pinout.h"
#include "config.h"
#include "ble.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"

// ---------- ADC channel alias (battery) ----------
#define ADC_CHANNEL     BAT_ADC_CHANNEL

// ---------- Logging Tags ----------
static const char *TAG_MAIN = "MAIN";
static const char *TAG_BLINK = "BLINK";
static const char *TAG_ADC = "ADC";
static const char *TAG_OLED = "OLED";
static const char *TAG_BUTTON = "BUTTON";
static const char *TAG_RGB = "RGB";
static const char *TAG_ADMIN = "ADMIN";
static bool g_pump_running = false;
static bool g_pump2_running = false;

// ---------- Display/System Mode ----------
typedef enum {
    MODE_HOME,
    MODE_WATER,
    MODE_SYSTEM,
    MODE_ADMIN_PREVIEW,
    MODE_MANUAL_PREVIEW,
    MODE_MANUAL,
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
static int g_ldr_raw = 0;
static bool g_daylight_confirmed = false;  // true when light has been stable long enough

// Light timer state (shared from pump_task for metrics display)
static int64_t g_light_change_time = 0;
static bool g_light_is_bright = false;

// Voltage thresholds
static float g_current_lvd = DEFAULT_LVD;
static float g_current_mvr = DEFAULT_MVR;
static float g_temp_lvd = DEFAULT_LVD;
static float g_temp_mvr = DEFAULT_MVR;

// Condition toggles (true = check is active)
static bool g_ldr_check   = true;
static bool g_float_check = true;
static bool g_lvd_check   = true;
static bool g_temp_ldr_check   = true;
static bool g_temp_float_check = true;
static bool g_temp_lvd_check   = true;

// Pump 1 manual override (runtime only — resets to false on reboot)
static bool g_pump1_override      = false;
static bool g_temp_pump1_override = false;

// Pump enable/shutoff (persisted — when OFF pump cannot activate under any condition)
static bool g_p1_enable      = true;
static bool g_p2_enable      = true;
static bool g_temp_p1_enable = true;
static bool g_temp_p2_enable = true;

// Display mode state
static display_mode_t g_display_mode = MODE_HOME;
static int64_t g_last_admin_activity = 0;
static int g_admin_cursor = 0;
static int g_manual_subpage = 0;  // 0-2, changed by up/down in manual mode

// Mutexes
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t voltage_mutex;

// ADC handle
static adc_oneshot_unit_handle_t adc_handle;

// OLED framebuffer
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// ---------- Forward Declarations ----------
static void enter_admin_edit(void);
static void exit_admin_edit(bool save);

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
static float g_flow1_lpm = 0.0f;
static float g_flow2_lpm = 0.0f;

// ---------- Float Switch Variables ----------
static bool g_tank_full = false;

// ---------- NVS Functions ----------

float tds_calculate_ppm(int adc_raw, float temp_c);

//NEW ADDITION
static const char html_page[] =
"<!DOCTYPE html><html lang=\"en\"><head>"
"<meta charset=\"UTF-8\" />"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />"
"<title>F.R.O.G.S. Web App</title>"
"<style>"
"body{margin:0;background-color:#050505;color:white;font-family:Arial,sans-serif;}"
".app{padding:25px;}h1{text-align:center;color:#d4af37;letter-spacing:2px;}"
".status-card,.sensor-card,.switch-card{border:2px solid #d4af37;border-radius:18px;padding:20px;background-color:#121212;}"
".status-card{margin-bottom:25px;}.status-card h2,.sensor-card h3,.switch-card h3{color:#d4af37;}"
".sensor-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:18px;}"
".sensor-card{text-align:center;}.icon{font-size:32px;margin-bottom:10px;}"
".value{font-size:28px;font-weight:bold;}.label{color:#aaaaaa;}"
".switch-row{display:grid;grid-template-columns:1fr 1fr;gap:18px;margin-top:40px;}"
".switch-card{display:flex;justify-content:space-between;align-items:center;}"
".switch{position:relative;width:60px;height:34px;}.switch input{display:none;}"
".slider{position:absolute;cursor:pointer;inset:0;background-color:#444;border-radius:34px;transition:0.3s;}"
".slider:before{content:\"\";position:absolute;height:26px;width:26px;left:4px;bottom:4px;background-color:white;border-radius:50%;transition:0.3s;}"
"input:checked+.slider{background-color:#d4af37;}input:checked+.slider:before{transform:translateX(26px);}"
"@media(max-width:900px){.sensor-grid,.switch-row{grid-template-columns:1fr;}}"
".refresh-row{margin-top:25px;text-align:center;}.refresh-btn{background-color:#d4af37;color:black;border:none;border-radius:14px;padding:14px 28px;font-size:18px;font-weight:bold;cursor:pointer;}"
"</style></head><body>"
"<div class=\"app\"><h1>F.R.O.G.S. Water System</h1>"
"<div class=\"status-card\"><h2>System Status</h2><p id=\"connectionStatus\">❌ Device Disconnected</p><p id=\"wifiStatus\">⚠️ Waiting for ESP32 connection...</p></div>"
"<div class=\"sensor-grid\">"
"<div class=\"sensor-card\"><div class=\"icon\">💧</div><h3>TDS</h3><p class=\"value\" id=\"tdsValue\">-- ppm</p><p class=\"label\">Water Purity</p></div>"
"<div class=\"sensor-card\"><div class=\"icon\">🌡️</div><h3>Temperature</h3><p class=\"value\" id=\"tempValue\">-- °F</p><p class=\"label\">Water Temperature</p></div>"
"<div class=\"sensor-card\"><div class=\"icon\">🌊</div><h3>Flow Rate</h3><p class=\"value\" id=\"flowValue\">-- L/min</p><p class=\"label\">Water Flow</p></div>"
"<div class=\"sensor-card\"><div class=\"icon\">🔋</div><h3>Battery</h3><p class=\"value\" id=\"batteryValue\">-- V</p><p class=\"label\">Battery Voltage</p></div>"
"<div class=\"sensor-card\"><div class=\"icon\">⚙️</div><h3>Pump</h3><p class=\"value\" id=\"pumpValue\">OFF</p><p class=\"label\">Pump Status</p></div>"
"</div>"
"<div class=\"switch-row\">"
"<div class=\"switch-card\"><h3>Feed Pump Enable</h3><label class=\"switch\"><input type=\"checkbox\" id=\"pump1Switch\" onchange=\"setPump1(this.checked)\"><span class=\"slider\"></span></label></div>"
"<div class=\"switch-card\"><h3>Return Pump Enable</h3><label class=\"switch\"><input type=\"checkbox\" id=\"pump2Switch\" onchange=\"setPump2(this.checked)\"><span class=\"slider\"></span></label></div>"
"</div></div>"
"<div class=\"refresh-row\"><button class=\"refresh-btn\" onclick=\"getSensorData()\">Refresh Data</button></div>"
"<script>"
"async function getSensorData(){try{const response=await fetch('/data');const data=await response.json();"
"document.getElementById('tdsValue').innerText=data.tds+' ppm';"
"document.getElementById('tempValue').innerText=data.temperature+' °F';"
"document.getElementById('flowValue').innerText=data.flowRate+' L/min';"
"document.getElementById('batteryValue').innerText=data.battery+' V';"
"document.getElementById('pumpValue').innerText=data.pump;"
"document.getElementById('pump1Switch').checked=data.pump1Enabled;"
"document.getElementById('pump2Switch').checked=data.pump2Enabled;"
"document.getElementById('connectionStatus').innerText='✅ Device Connected';"
"document.getElementById('wifiStatus').innerText='ESP32 data received successfully.';"
"}catch(error){document.getElementById('connectionStatus').innerText='❌ Device Disconnected';document.getElementById('wifiStatus').innerText='Could not reach ESP32 /data route.';}}"
"async function setPump1(enabled){await fetch(enabled?'/pump1/enable':'/pump1/disable');getSensorData();}"
"async function setPump2(enabled){await fetch(enabled?'/pump2/enable':'/pump2/disable');getSensorData();}"
"setInterval(getSensorData,3000);getSensorData();"
"</script></body></html>";
// NEW_ADDITION



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
        g_ldr_check   = (nvs_get_u8(nvs_handle, NVS_KEY_LDR_CHECK,   &chk) == ESP_OK) ? (bool)chk : true;
        g_float_check = (nvs_get_u8(nvs_handle, NVS_KEY_FLOAT_CHECK, &chk) == ESP_OK) ? (bool)chk : true;
        g_lvd_check   = (nvs_get_u8(nvs_handle, NVS_KEY_LVD_CHECK,   &chk) == ESP_OK) ? (bool)chk : true;
        g_p1_enable   = (nvs_get_u8(nvs_handle, NVS_KEY_P1_ENABLE,   &chk) == ESP_OK) ? (bool)chk : true;
        g_p2_enable   = (nvs_get_u8(nvs_handle, NVS_KEY_P2_ENABLE,   &chk) == ESP_OK) ? (bool)chk : true;
        ESP_LOGI(TAG_ADMIN, "Checks - LDR:%d Float:%d LVD:%d P1EN:%d P2EN:%d",
                 g_ldr_check, g_float_check, g_lvd_check, g_p1_enable, g_p2_enable);

        nvs_close(nvs_handle);
    } else {
        g_current_lvd = DEFAULT_LVD;
        g_current_mvr = DEFAULT_MVR;
        g_ldr_check   = true;
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
        nvs_set_u8(nvs_handle, NVS_KEY_LDR_CHECK,   (uint8_t)g_ldr_check);
        nvs_set_u8(nvs_handle, NVS_KEY_FLOAT_CHECK, (uint8_t)g_float_check);
        nvs_set_u8(nvs_handle, NVS_KEY_LVD_CHECK,   (uint8_t)g_lvd_check);
        nvs_set_u8(nvs_handle, NVS_KEY_P1_ENABLE,   (uint8_t)g_p1_enable);
        nvs_set_u8(nvs_handle, NVS_KEY_P2_ENABLE,   (uint8_t)g_p2_enable);

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
        g_temp_ldr_check       = g_ldr_check;
        g_temp_float_check     = g_float_check;
        g_temp_lvd_check       = g_lvd_check;
        g_temp_pump1_override  = g_pump1_override;
        g_temp_p1_enable       = g_p1_enable;
        g_temp_p2_enable       = g_p2_enable;
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
            g_ldr_check       = g_temp_ldr_check;
            g_float_check     = g_temp_float_check;
            g_lvd_check       = g_temp_lvd_check;
            g_pump1_override  = g_temp_pump1_override;
            g_p1_enable       = g_temp_p1_enable;
            g_p2_enable       = g_temp_p2_enable;
            xSemaphoreGive(voltage_mutex);
        }
        nvs_save_voltages();
        ESP_LOGI(TAG_ADMIN, "=== SETTINGS SAVED ===");
    } else {
        ESP_LOGI(TAG_ADMIN, "=== SETTINGS DISCARDED ===");
    }

    g_display_mode = MODE_HOME;
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
    // Items: 0=LVD 1=MVR 2=LDR CHK 3=FLT CHK 4=LVD CHK 5=P1 OVRD 6=RESET
    int scroll_top = (g_admin_cursor > 4) ? g_admin_cursor - 4 : 0;

    for (int i = 0; i < 5; i++) {
        int item = scroll_top + i;
        if (item > 8) break;
        int y = 16 + i * 9;
        bool selected = (item == g_admin_cursor);

        switch (item) {
            case 0: snprintf(line, sizeof(line), "LVD: %.1fV",  g_temp_lvd);                          break;
            case 1: snprintf(line, sizeof(line), "MVR: %.1fV",  g_temp_mvr);                          break;
            case 2: snprintf(line, sizeof(line), "LDR CHK: %s", g_temp_ldr_check    ? "ON" : "OFF"); break;
            case 3: snprintf(line, sizeof(line), "FLT CHK: %s", g_temp_float_check  ? "ON" : "OFF"); break;
            case 4: snprintf(line, sizeof(line), "LVD CHK: %s", g_temp_lvd_check    ? "ON" : "OFF"); break;
            case 5: snprintf(line, sizeof(line), "P1 OVRD: %s", g_temp_pump1_override ? "ON" : "OFF"); break;
            case 6: snprintf(line, sizeof(line), "P1 EN:   %s", g_temp_p1_enable    ? "ON" : "OFF"); break;
            case 7: snprintf(line, sizeof(line), "P2 EN:   %s", g_temp_p2_enable    ? "ON" : "OFF"); break;
            case 8: snprintf(line, sizeof(line), "RESET DFLTS");                                      break;
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

    int ldr = 0;
    bool bright = false;
    bool daylight = false;
    int64_t change_time = 0;
    float lvd = 0.0f;
    float mvr = 0.0f;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ldr         = g_ldr_raw;
        bright      = g_light_is_bright;
        daylight    = g_daylight_confirmed;
        change_time = g_light_change_time;
        xSemaphoreGive(state_mutex);
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        lvd = g_current_lvd;
        mvr = g_current_mvr;
        xSemaphoreGive(voltage_mutex);
    }

    int64_t now = esp_timer_get_time() / 1000;
    int64_t elapsed = now - change_time;

    oled_clear_buffer();

    // Yellow zone: title
    oled_draw_string(40, 4, "SYSTEM");

    // Blue zone: content
    snprintf(line, sizeof(line), "LDR: %4d %s", ldr, bright ? "DAY" : "NIGHT");
    oled_draw_string(0, 16, line);

    snprintf(line, sizeof(line), "LVD: %.1fV  MVR: %.1fV", lvd, mvr);
    oled_draw_string(0, 26, line);

    if (bright && !daylight) {
        int64_t remaining = (LIGHT_ON_DELAY_MS - elapsed) / 1000;
        if (remaining < 0) remaining = 0;
        snprintf(line, sizeof(line), "ON TMR: %llds", remaining);
    } else if (!bright && daylight) {
        int64_t remaining = (LIGHT_OFF_DELAY_MS - elapsed) / 1000;
        if (remaining < 0) remaining = 0;
        snprintf(line, sizeof(line), "OFF TMR: %llds", remaining);
    } else if (daylight) {
        snprintf(line, sizeof(line), "LIGHT: CONFIRMED");
    } else {
        snprintf(line, sizeof(line), "LIGHT: WAITING");
    }
    oled_draw_string(0, 36, line);

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
        flow1         = g_flow1_lpm;
        flow2         = g_flow2_lpm;
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

    if (current_valid)
        snprintf(line, sizeof(line), "CURR: %.2fA", current_amps);
    else
        snprintf(line, sizeof(line), "CURR: ERROR");
    oled_draw_string(0, 34, line);

    snprintf(line, sizeof(line), "F1:%.1f F2:%.1f", flow1, flow2);
    oled_draw_string(0, 43, line);

    snprintf(line, sizeof(line), "RATIO: %d%%:%d%%", ratio, 100 - ratio);
    oled_draw_string(0, 52, line);

    oled_draw_page_dots(1);
    oled_update_display();
}

static void display_manual_preview(void)
{
    oled_clear_buffer();

    oled_draw_string(28, 4, "MANUAL");

    oled_draw_string(6, 24, "CLICK STICK");
    oled_draw_string(6, 34, "TO ENTER");

    oled_draw_page_dots(4);
    oled_update_display();
}

static void display_manual_mode(void)
{
    // 4 sub-pages: title (yellow zone) + 5 content lines (blue zone)
    static const char * const pages[4][6] = {
        {
            "VOLT SETTINGS 1/4",
            "LVD: LOW VOLT",
            "PUMP STOPS BELOW",
            "MVR: RECOVERY V",
            "PUMP STARTS ABOVE",
            "LVD CHK: USE LVD?",
        },
        {
            "SENSOR CHECKS 2/4",
            "LDR CHK: DAYLIGHT",
            "PUMP NEEDS SUN",
            "FLT CHK: TANK LVL",
            "PUMP NEEDS FULL",
            "DISABLE=BYPASS",
        },
        {
            "PUMP CONTROL  3/4",
            "P1 OVRD: FORCE ON",
            "BYPASSES ALL CHKS",
            "P1 EN: ALLOW P1",
            "P2 EN: ALLOW P2",
            "OFF=HARD SHUTOFF",
        },
        {
            "RESET DFLTS   4/4",
            "RESTORES:",
            "LVD=12.0 MVR=12.8",
            "ALL CHECKS: ON",
            "OVERRIDES: OFF",
            "PUMPS: ENABLED",
        },
    };

    oled_clear_buffer();

    oled_draw_string(0, 4, pages[g_manual_subpage][0]);

    for (int i = 0; i < 5; i++) {
        oled_draw_string(0, 16 + i * 9, pages[g_manual_subpage][i + 1]);
    }

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
    float lvd = 0.0f;
    float mvr = 0.0f;
    bool daylight = false;
    bool pump = false;
    bool pump2 = false;
    bool tank_full = false;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bv        = g_battery_voltage;
        daylight  = g_daylight_confirmed;
        pump      = g_pump_running;
        pump2     = g_pump2_running;
        tank_full = g_tank_full;
        xSemaphoreGive(state_mutex);
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        lvd = g_current_lvd;
        mvr = g_current_mvr;
        xSemaphoreGive(voltage_mutex);
    }

    oled_clear_buffer();

    // Yellow zone: title + daylight status
    oled_draw_string(0, 4, "F.R.O.G.S");
    oled_draw_string(78, 4, daylight ? "DAY" : "NIGHT");

    // Blue zone: content
    snprintf(line, sizeof(line), "BATT: %.2fV", bv);
    oled_draw_string(0, 16, line);

    snprintf(line, sizeof(line), "P1: %-3s  P2: %-3s",
             pump ? "ON" : "OFF", pump2 ? "ON" : "OFF");
    oled_draw_string(0, 26, line);

    snprintf(line, sizeof(line), "TANK: %-4s",
             tank_full ? "FULL" : "OK");
    oled_draw_string(0, 36, line);

    if (!daylight) {
        oled_draw_string(0, 46, "BLOCKED: NIGHT");
    } else if (bv < lvd) {
        oled_draw_string(0, 46, "BLOCKED: LOW V");
    } else if (tank_full) {
        oled_draw_string(0, 46, "BLOCKED: TANK");
    } else if (bv < mvr && !pump) {
        oled_draw_string(0, 46, "WAITING FOR MVR");
    } else {
        oled_draw_string(0, 46, "SYSTEM OK");
    }

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
                    } else if (g_display_mode == MODE_MANUAL_PREVIEW) {
                        g_display_mode = MODE_MANUAL;
                    } else if (g_display_mode == MODE_MANUAL) {
                        g_display_mode = MODE_MANUAL_PREVIEW;
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
        if (g_display_mode == MODE_MANUAL) {
            // Inside manual: up/down scrolls sub-pages only — click exits
            if (dir == JOY_UP   && g_manual_subpage > 0) g_manual_subpage--;
            if (dir == JOY_DOWN && g_manual_subpage < 3) g_manual_subpage++;
        } else if (g_display_mode == MODE_HOME || g_display_mode == MODE_WATER ||
                   g_display_mode == MODE_SYSTEM || g_display_mode == MODE_ADMIN_PREVIEW ||
                   g_display_mode == MODE_MANUAL_PREVIEW) {
            // Left/Right cycles: Home <-> Water <-> System <-> Admin Preview <-> Manual Preview <-> Home
            if (dir == JOY_LEFT) {
                if (g_display_mode == MODE_WATER)               g_display_mode = MODE_HOME;
                else if (g_display_mode == MODE_SYSTEM)         g_display_mode = MODE_WATER;
                else if (g_display_mode == MODE_ADMIN_PREVIEW)  g_display_mode = MODE_SYSTEM;
                else if (g_display_mode == MODE_MANUAL_PREVIEW) g_display_mode = MODE_ADMIN_PREVIEW;
                else if (g_display_mode == MODE_HOME)           g_display_mode = MODE_MANUAL_PREVIEW;
                ESP_LOGI(TAG_BUTTON, "Mode -> %d", g_display_mode);
            } else if (dir == JOY_RIGHT) {
                if (g_display_mode == MODE_HOME)                g_display_mode = MODE_WATER;
                else if (g_display_mode == MODE_WATER)          g_display_mode = MODE_SYSTEM;
                else if (g_display_mode == MODE_SYSTEM)         g_display_mode = MODE_ADMIN_PREVIEW;
                else if (g_display_mode == MODE_ADMIN_PREVIEW)  g_display_mode = MODE_MANUAL_PREVIEW;
                else if (g_display_mode == MODE_MANUAL_PREVIEW) g_display_mode = MODE_HOME;
                ESP_LOGI(TAG_BUTTON, "Mode -> %d", g_display_mode);
            }
        } else if (g_display_mode == MODE_ADMIN_EDIT) {
            g_last_admin_activity = now;

            if (dir == JOY_UP) {
                if (g_admin_cursor > 0) g_admin_cursor--;
                ESP_LOGI(TAG_ADMIN, "Cursor -> %d", g_admin_cursor);
            } else if (dir == JOY_DOWN) {
                if (g_admin_cursor < 8) g_admin_cursor++;
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
                        g_temp_ldr_check = !g_temp_ldr_check;
                        ESP_LOGI(TAG_ADMIN, "LDR check -> %d", g_temp_ldr_check);
                        break;
                    case 3:
                        g_temp_float_check = !g_temp_float_check;
                        ESP_LOGI(TAG_ADMIN, "Float check -> %d", g_temp_float_check);
                        break;
                    case 4:
                        g_temp_lvd_check = !g_temp_lvd_check;
                        ESP_LOGI(TAG_ADMIN, "LVD check -> %d", g_temp_lvd_check);
                        break;
                    case 5:
                        g_temp_pump1_override = !g_temp_pump1_override;
                        ESP_LOGI(TAG_ADMIN, "P1 override -> %d", g_temp_pump1_override);
                        break;
                    case 6:
                        g_temp_p1_enable = !g_temp_p1_enable;
                        ESP_LOGI(TAG_ADMIN, "P1 enable -> %d", g_temp_p1_enable);
                        break;
                    case 7:
                        g_temp_p2_enable = !g_temp_p2_enable;
                        ESP_LOGI(TAG_ADMIN, "P2 enable -> %d", g_temp_p2_enable);
                        break;
                    case 8:
                        g_temp_lvd             = DEFAULT_LVD;
                        g_temp_mvr             = DEFAULT_MVR;
                        g_temp_ldr_check       = true;
                        g_temp_float_check     = true;
                        g_temp_lvd_check       = true;
                        g_temp_pump1_override  = false;
                        g_temp_p1_enable       = true;
                        g_temp_p2_enable       = true;
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

// ---------- RGB LED Functions ----------

/**
 * Initialize RGB LED PWM channels
 */
static void rgb_init(void)
{
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Configure RED channel
    ledc_channel_config_t ledc_red = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_RED_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = RGB_RED_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_red));
    
    // Configure GREEN channel
    ledc_channel_config_t ledc_green = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_GREEN_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = RGB_GREEN_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_green));
    
    // Configure BLUE channel
    ledc_channel_config_t ledc_blue = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_BLUE_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = RGB_BLUE_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_blue));
    
    ESP_LOGI(TAG_RGB, "RGB LED PWM initialized");
    ESP_LOGI(TAG_RGB, "  RED:   GPIO%d (Channel %d)", RGB_RED_PIN, LEDC_RED_CHANNEL);
    ESP_LOGI(TAG_RGB, "  GREEN: GPIO%d (Channel %d)", RGB_GREEN_PIN, LEDC_GREEN_CHANNEL);
    ESP_LOGI(TAG_RGB, "  BLUE:  GPIO%d (Channel %d)", RGB_BLUE_PIN, LEDC_BLUE_CHANNEL);
}

/**
 * Set RGB LED to specific color
 * @param red   Red intensity (0-255)
 * @param green Green intensity (0-255)
 * @param blue  Blue intensity (0-255)
 */
static void rgb_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    ledc_set_duty(LEDC_MODE, LEDC_RED_CHANNEL,   red);
    ledc_update_duty(LEDC_MODE, LEDC_RED_CHANNEL);

    ledc_set_duty(LEDC_MODE, LEDC_GREEN_CHANNEL, green);
    ledc_update_duty(LEDC_MODE, LEDC_GREEN_CHANNEL);

    ledc_set_duty(LEDC_MODE, LEDC_BLUE_CHANNEL,  blue);
    ledc_update_duty(LEDC_MODE, LEDC_BLUE_CHANNEL);
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

    int adc_buffer[ADC_SAMPLES] = {0};
    int buffer_index = 0;
    int sum = 0;

    int ldr_buffer[LDR_ADC_SAMPLES] = {0};
    int ldr_buffer_index = 0;
    int ldr_sum = 0;

    ESP_LOGI(TAG_ADC, "adc_task started");
    ESP_LOGI(TAG_ADC, "Battery: GPIO34 (ADC1_CH6), LDR: GPIO35 (ADC1_CH7)");
    ESP_LOGI(TAG_ADC, "Battery voltage range: %.1fV - %.1fV", BAT_V_MIN, BAT_V_MAX);
    ESP_LOGI(TAG_ADC, "LDR threshold: %d (above = daylight)", LDR_THRESHOLD);

    while (1) {
        // --- Battery ADC ---
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG_ADC, "Battery ADC read failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        sum -= adc_buffer[buffer_index];
        adc_buffer[buffer_index] = raw;
        sum += raw;
        buffer_index = (buffer_index + 1) % ADC_SAMPLES;

        int avg_raw = sum / ADC_SAMPLES;

// Battery voltage divider: R1=68kΩ, R2=15kΩ (ratio = 83/15)
#define BAT_CAL 1.065f
float adc_voltage = (avg_raw / 4095.0f) * 3.3f;
float battery_voltage = adc_voltage * (83.0f / 15.0f) * BAT_CAL;

        // --- LDR ADC ---
        int ldr_raw = 0;
        ret = adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &ldr_raw);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG_ADC, "LDR ADC read failed: %s", esp_err_to_name(ret));
        } else {
            ldr_sum -= ldr_buffer[ldr_buffer_index];
            ldr_buffer[ldr_buffer_index] = ldr_raw;
            ldr_sum += ldr_raw;
            ldr_buffer_index = (ldr_buffer_index + 1) % LDR_ADC_SAMPLES;
        }

        int ldr_avg = ldr_sum / LDR_ADC_SAMPLES;

        // --- TDS ADC ---
        int tds_raw = 0;
        float temp_c_for_tds = 25.0f;   // fallback if temp is invalid
        float tds_voltage = 0.0f;
        float tds_ppm = 0.0f;
        bool tds_valid = false;

        /*
        ret = adc_oneshot_read(adc2_handle, TDS_ADC2_CHANNEL, &tds_raw);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG_ADC, "TDS ADC read failed: %s", esp_err_to_name(ret));
        } else {
            tds_sum -= tds_buffer[tds_buffer_index];
            tds_buffer[tds_buffer_index] = tds_raw;
            tds_sum += tds_raw;
            tds_buffer_index = (tds_buffer_index + 1) % TDS_ADC_SAMPLES;

            int tds_avg = tds_sum / TDS_ADC_SAMPLES;
            tds_voltage = (tds_avg / 4095.0f) * 3.3f;

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
        */

        // --- Current Sense ADC (IS 1+2, GPIO25) ---
        float current_amps = 0.0f;
        bool current_valid = false;
        /*
        if (adc_oneshot_read(adc2_handle, IS_3_4_CHANNEL, &is_raw) == ESP_OK) {
            // IS pin outputs ~1.2kA/A — with a 1kΩ sense resistor: V = I_sense * R
            // BTS700x kILIS = 1100 (typ), so I_load = (V_IS / R_sense) * kILIS
            float v_is = (is_raw / 4095.0f) * 3.3f;
            current_amps = (v_is / 1000.0f) * 1100.0f;
            current_valid = true;
        }
        */
        // Update other global variables
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_adc_raw = avg_raw;
            g_battery_voltage = battery_voltage;
            g_ldr_raw = ldr_avg;
            g_current_amps = current_amps;
            g_current_valid = current_valid;

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
                         "Batt: %4d (%.2fV) | LDR: %4d (%s) | TDS: %4d (%.3fV, %.1f ppm, %.1fC)",
                         avg_raw, battery_voltage,
                         ldr_avg, ldr_avg > LDR_THRESHOLD ? "DAY" : "NIGHT",
                         tds_raw, tds_voltage, tds_ppm, temp_c_for_tds);
            } else {
                ESP_LOGI(TAG_ADC,
                         "Batt: %4d (%.2fV) | LDR: %4d (%s) | TDS: ERR",
                         avg_raw, battery_voltage,
                         ldr_avg, ldr_avg > LDR_THRESHOLD ? "DAY" : "NIGHT");
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
            case MODE_SYSTEM:         display_system_mode();   break;
            case MODE_ADMIN_PREVIEW:  display_admin_preview(); break;
            case MODE_MANUAL_PREVIEW: display_manual_preview(); break;
            case MODE_MANUAL:         display_manual_mode();   break;
            case MODE_ADMIN_EDIT:     display_admin_mode();    break;
            default:                  display_home_mode();     break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ---------- RGB Test Task ----------

void rgb_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG_RGB, "rgb_task started");

    while (1) {
        if (g_display_mode == MODE_ADMIN_EDIT) {
            rgb_set_color(127, 0, 255);  // Purple in admin mode
        } else {
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

            if (bv < lvd) {
                rgb_set_color(255, 0, 0);    // Red - below LVD, critical
            } else if (bv < mvr) {
                rgb_set_color(255, 80, 0);   // Yellow/Orange - between LVD and MVR
            } else {
                rgb_set_color(0, 255, 0);    // Green - above MVR, healthy
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// BLE TASK
void ble_notify_task(void *pvParameters)
{
    char payload[256];

    while (1) {
        if (ble_is_connected()) {
            float bv = 0.0f;
            float temp_c = 0.0f;
            float tds = 0.0f;
            float current = 0.0f;
            float flow1 = 0.0f;
            float flow2 = 0.0f;
            bool pump1 = false;
            bool pump2 = false;
            bool temp_valid = false;
            bool tds_valid = false;
            bool current_valid = false;

            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                bv = g_battery_voltage;
                temp_c = g_temp_c;
                tds = g_tds_ppm;
                current = g_current_amps;
                flow1 = g_flow1_lpm;
                flow2 = g_flow2_lpm;
                pump1 = g_pump_running;
                pump2 = g_pump2_running;
                temp_valid = g_temp_valid;
                tds_valid = g_tds_valid;
                current_valid = g_current_valid;
                xSemaphoreGive(state_mutex);
            }

            snprintf(payload, sizeof(payload),
                     "{\"battery\":%.2f,\"temp\":%.2f,\"tds\":%.1f,"
                     "\"current\":%.2f,\"flow1\":%.2f,\"flow2\":%.2f,"
                     "\"pump1\":%d,\"pump2\":%d,\"tempValid\":%d,"
                     "\"tdsValid\":%d,\"currentValid\":%d}",
                     bv, temp_c, tds,
                     current, flow1, flow2,
                     pump1, pump2,
                     temp_valid, tds_valid, current_valid);

            ble_send(payload);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void pump_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI("PUMP", "pump_task started");
    ESP_LOGI("PUMP", "Light ON delay: %ds, OFF delay: %ds",
             LIGHT_ON_DELAY_MS / 1000, LIGHT_OFF_DELAY_MS / 1000);

    bool pump_enabled = false;          // voltage-based enable (LVD/MVR hysteresis)
    bool light_is_bright = false;       // raw light reading above threshold
    int64_t light_change_time = 0;      // timestamp when light state last changed
    bool daylight_stable = false;       // true after light confirmed stable

    // Initialize shared light state
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_light_change_time = 0;
        g_light_is_bright = false;
        xSemaphoreGive(state_mutex);
    }

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;  // ms

        float bv = 0.0f;
        float lvd = 0.0f;
        float mvr = 0.0f;
        int ldr = 0;

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bv = g_battery_voltage;
            ldr = g_ldr_raw;
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

        // --- Light debounce logic ---
        bool currently_bright = (ldr > LDR_THRESHOLD);

        if (currently_bright != light_is_bright) {
            // Light state changed — start the debounce timer
            light_is_bright = currently_bright;
            light_change_time = now;
            ESP_LOGI("PUMP", "Light changed to %s, waiting for confirmation...",
                     currently_bright ? "BRIGHT" : "DARK");

            // Share light timer state for metrics display
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_light_change_time = light_change_time;
                g_light_is_bright = light_is_bright;
                xSemaphoreGive(state_mutex);
            }
        }

        // Check if the light state has been stable long enough
        int64_t elapsed = now - light_change_time;

        if (light_is_bright && !daylight_stable) {
            if (elapsed >= LIGHT_ON_DELAY_MS) {
                daylight_stable = true;
                ESP_LOGI("PUMP", "Daylight confirmed after %llds", elapsed / 1000);
            }
        } else if (!light_is_bright && daylight_stable) {
            if (elapsed >= LIGHT_OFF_DELAY_MS) {
                daylight_stable = false;
                ESP_LOGW("PUMP", "Darkness confirmed after %llds - pump blocked",
                         elapsed / 1000);
            }
        }

        // Update shared daylight state
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_daylight_confirmed = daylight_stable;
            xSemaphoreGive(state_mutex);
        }

        // Read float switch
        bool tank_full = false;
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            tank_full = g_tank_full;
            xSemaphoreGive(state_mutex);
        }

        // Read condition toggles, override, and shutoffs
        bool ldr_check = true, float_chk = true, lvd_chk = true;
        bool p1_override = false, p1_enable = true, p2_enable = true;
        if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ldr_check   = g_ldr_check;
            float_chk   = g_float_check;
            lvd_chk     = g_lvd_check;
            p1_override = g_pump1_override;
            p1_enable   = g_p1_enable;
            p2_enable   = g_p2_enable;
            xSemaphoreGive(voltage_mutex);
        }

        // --- Final pump 1 decision ---
        // Shutoff (p1_enable) takes priority over everything including override
        bool conditions_met = p1_override
                           || ((lvd_chk   ? pump_enabled   : true)
                            && (ldr_check ? daylight_stable : true)
                            && (float_chk ? tank_full       : true));
        bool active = p1_enable && conditions_met && (g_display_mode != MODE_ADMIN_EDIT);

        gpio_set_level(FEED_PUMP_PIN, active ? 1 : 0);

        // --- Pump 2: turns on after PUMP2_DELAY_MS of Pump 1 running ---
        static int64_t pump1_start_time = 0;
        bool pump2_active = false;

        if (active) {
            if (pump1_start_time == 0) {
                pump1_start_time = now;
                ESP_LOGI("PUMP2", "Pump 1 started, waiting %ds before Pump 2",
                         PUMP2_DELAY_MS / 1000);
            }
            if (p2_enable && (now - pump1_start_time) >= PUMP2_DELAY_MS) {
                pump2_active = true;
            }
        } else {
            if (pump1_start_time != 0) {
                ESP_LOGI("PUMP2", "Pump 1 stopped, Pump 2 OFF");
            }
            pump1_start_time = 0;
        }

        gpio_set_level(PUMP2_PIN, pump2_active ? 1 : 0);

        static int pump_log_counter = 0;
        if (++pump_log_counter >= 4) {
            ESP_LOGI("PUMP", "v_ok=%d light=%d(%s) daylight=%d p1en=%d p2en=%d ovrd=%d -> pump1=%d pump2=%d",
                     pump_enabled, ldr, light_is_bright ? "B" : "D",
                     daylight_stable, p1_enable, p2_enable, p1_override,
                     active, pump2_active && p2_enable);
            pump_log_counter = 0;
        }

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_pump_running = active;
            g_pump2_running = pump2_active;
            xSemaphoreGive(state_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------- Flow Sensor Task ----------

void flow_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI("FLOW", "flow_task started");

    uint32_t count1 = 0, count2 = 0;
    bool last1 = true, last2 = true;

    int64_t window_start = esp_timer_get_time() / 1000;

    while (1) {
        bool cur1 = gpio_get_level(FLOW1_PIN);
        bool cur2 = gpio_get_level(FLOW2_PIN);

        if (!cur1 && last1) count1++;
        if (!cur2 && last2) count2++;
        last1 = cur1;
        last2 = cur2;

        int64_t now = esp_timer_get_time() / 1000;
        if ((now - window_start) >= FLOW_SAMPLE_MS) {
            float lpm1 = (count1 / FLOW_CAL);
            float lpm2 = (count2 / FLOW_CAL);

            bool tank_full = gpio_get_level(FLOAT_SW_PIN);

            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_flow1_lpm = lpm1;
                g_flow2_lpm = lpm2;
                g_tank_full = tank_full;
                xSemaphoreGive(state_mutex);
            }

            ESP_LOGI("FLOW", "F1: %.2f L/min  F2: %.2f L/min  Float: %s",
                     lpm1, lpm2, tank_full ? "FULL" : "NOT FULL");

            count1 = 0;
            count2 = 0;
            window_start = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
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

//NEW_WEBSITE_CODE
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

    ESP_LOGI("WIFI", "FROGS WiFi started. Connect to FROGS_Controller");
    ESP_LOGI("WIFI", "Open browser to http://192.168.4.1");
}


static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI("WEB", "Serving webpage (%d bytes)", strlen(html_page));

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t data_get_handler(httpd_req_t *req)
{
    char json[256];

    float battery = 0.0f;
    float temp_c = 0.0f;
    float tds = 0.0f;
    float flow1 = 0.0f;
    float flow2 = 0.0f;
    bool pump1 = false;
    bool pump2 = false;
    bool p1_enabled = true;
    bool p2_enabled = true;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        battery = g_battery_voltage;
        temp_c = g_temp_c;
        tds = g_tds_ppm;
        flow1 = g_flow1_lpm;
        flow2 = g_flow2_lpm;
        pump1 = g_pump_running;
        pump2 = g_pump2_running;
        xSemaphoreGive(state_mutex);
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        p1_enabled = g_p1_enable;
        p2_enabled = g_p2_enable;
        xSemaphoreGive(voltage_mutex);
    }

    float temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;
    float total_flow = flow1 + flow2;

    snprintf(json, sizeof(json),
             "{\"tds\":%.1f,\"temperature\":%.1f,\"flowRate\":%.2f,"
             "\"battery\":%.2f,\"pump\":\"Feed:%s Return:%s\","
             "\"pump1Enabled\":%s,\"pump2Enabled\":%s}",
             tds,
             temp_f,
             total_flow,
             battery,
             pump1 ? "ON" : "OFF",
             pump2 ? "ON" : "OFF",
             p1_enabled ? "true" : "false",
             p2_enabled ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t pump1_enable_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_p1_enable = true;
        xSemaphoreGive(voltage_mutex);
        nvs_save_voltages();
    }

    return httpd_resp_sendstr(req, "Feed Pump enabled");
}

static esp_err_t pump1_disable_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_p1_enable = false;
        xSemaphoreGive(voltage_mutex);
        nvs_save_voltages();
    }

    return httpd_resp_sendstr(req, "Pump 1 disabled");
}

static esp_err_t pump2_enable_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_p2_enable = true;
        xSemaphoreGive(voltage_mutex);
        nvs_save_voltages();
    }

    return httpd_resp_sendstr(req, "Pump 2 enabled");
}

static esp_err_t pump2_disable_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_p2_enable = false;
        xSemaphoreGive(voltage_mutex);
        nvs_save_voltages();
    }

    return httpd_resp_sendstr(req, "Pump 2 disabled");
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t pump1_enable_uri = {
        .uri = "/pump1/enable",
        .method = HTTP_GET,
        .handler = pump1_enable_handler,
        .user_ctx = NULL
    };

    httpd_uri_t pump1_disable_uri = {
        .uri = "/pump1/disable",
        .method = HTTP_GET,
        .handler = pump1_disable_handler,
        .user_ctx = NULL
    };

    httpd_uri_t pump2_enable_uri = {
        .uri = "/pump2/enable",
        .method = HTTP_GET,
        .handler = pump2_enable_handler,
        .user_ctx = NULL
    };

    httpd_uri_t pump2_disable_uri = {
        .uri = "/pump2/disable",
        .method = HTTP_GET,
        .handler = pump2_disable_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &data_uri);
    httpd_register_uri_handler(server, &pump1_enable_uri);
    httpd_register_uri_handler(server, &pump1_disable_uri);
    httpd_register_uri_handler(server, &pump2_enable_uri);
    httpd_register_uri_handler(server, &pump2_disable_uri);

    ESP_LOGI("WEB", "Web server started");
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

    
    //ADDITION
    wifi_init_ap();
    start_webserver();
    //

    
    ESP_ERROR_CHECK(ble_init());
    ESP_LOGI(TAG_MAIN, "BLE initialized");
    // Configure LED GPIO
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    ESP_LOGI(TAG_MAIN, "LED GPIO configured");

    // Configure pump GPIO
    gpio_reset_pin(FEED_PUMP_PIN);
    gpio_set_direction(FEED_PUMP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FEED_PUMP_PIN, 0);
    ESP_LOGI(TAG_MAIN, "Pump 1 configured: GPIO%d (U3)", FEED_PUMP_PIN);

    gpio_reset_pin(PUMP2_PIN);
    gpio_set_direction(PUMP2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP2_PIN, 0);
    ESP_LOGI(TAG_MAIN, "Pump 2 configured: GPIO%d (U1)", PUMP2_PIN);

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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_VRX_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_VRY_CHANNEL, &adc_chan_config));
    ESP_LOGI(TAG_MAIN, "ADC1 configured: battery(GPIO34), LDR(GPIO35), JoyX(GPIO36), JoyY(GPIO39)");

    // Configure ADC2 for TDS
    /*
    adc_oneshot_unit_init_cfg_t adc2_init_config = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_init_config, &adc2_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, TDS_ADC2_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, IS_3_4_CHANNEL, &adc_chan_config));
    ESP_LOGI(TAG_MAIN, "ADC2 configured: TDS(GPIO27, ADC2_CH7), IS(GPIO25, ADC2_CH8)");
    */

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

    // Initialize RGB LED
    rgb_init();
    ESP_LOGI(TAG_MAIN, "RGB LED initialized");

    // Initialize temperature sensor
    esp_err_t temp_err = temp_init(TEMP_SENSOR_PIN);
    if (temp_err == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "Temperature sensor initialized on GPIO%d", TEMP_SENSOR_PIN);
    } else {
        ESP_LOGW(TAG_MAIN, "Temperature sensor init failed: %s", esp_err_to_name(temp_err));
    }

    // Configure flow sensor GPIOs (pull-up, active-low pulses)
    gpio_reset_pin(FLOW1_PIN);
    gpio_set_direction(FLOW1_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(FLOW1_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG_MAIN, "Flow sensor 1: GPIO%d", FLOW1_PIN);

    gpio_reset_pin(FLOW2_PIN);
    gpio_set_direction(FLOW2_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(FLOW2_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG_MAIN, "Flow sensor 2: GPIO%d", FLOW2_PIN);

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
    xTaskCreate(rgb_task, "rgb_task", 2048, NULL, 1, NULL);
    xTaskCreate(pump_task, "pump_task", 4096, NULL, 2, NULL);
    xTaskCreate(ble_notify_task, "ble_notify_task", 4096, NULL, 2, NULL);
    xTaskCreate(flow_task, "flow_task", 2048, NULL, 2, NULL);

    ESP_LOGI(TAG_MAIN, "All tasks started successfully");
    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "Joystick L/R: cycle modes (Normal > Metrics > Admin)");
    ESP_LOGI(TAG_MAIN, "Joystick click: enter Admin edit mode");
    ESP_LOGI(TAG_MAIN, "Admin: U/D select, L/R adjust, EXIT button saves");
    ESP_LOGI(TAG_MAIN, "Temperature sensor on GPIO13");
    ESP_LOGI(TAG_MAIN, "TDS sensor on GPIO27 (ADC2)");
    ESP_LOGI(TAG_MAIN, "===========================================");
}
