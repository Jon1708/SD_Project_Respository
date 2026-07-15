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
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "temp.h"
#include "pinout.h"
#include "config.h"
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
static const char *TAG_RGB = "RGB";
static const char *TAG_ADMIN = "ADMIN";
static bool g_lp_pump_running = false;
static bool g_hp_pump_running = false;

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

// voltage_mutex must already be held before calling this helper.
// It copies webpage changes into the existing OLED Admin edit variables.
static void sync_existing_oled_admin_values_locked(void)
{
    g_temp_lvd = g_current_lvd;
    g_temp_mvr = g_current_mvr;
    g_temp_ldr_check = g_ldr_check;
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
static float g_flow1_lpm = 0.0f;
static float g_flow2_lpm = 0.0f;

// ---------- Float Switch Variables ----------
static bool g_tank_full = false;


// New_Addition
// ---------- Webpage Storage ----------
static const char html_page[] =
"<!DOCTYPE html>\n"
"<html lang=\"en\">\n"
"<head>\n"
"  <meta charset=\"UTF-8\" />\n"
"  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />\n"
"  <title>F.R.O.G.S. Web App</title>\n"
"\n"
"  <style>\n"
"    :root {\n"
"      --ucf-gold: #d4af37;\n"
"      --ucf-black: #050505;\n"
"      --panel: #121212;\n"
"      --muted: #aaaaaa;\n"
"      --good: #4fe08b;\n"
"      --warn: #ffd34d;\n"
"      --bad: #ff6a6a;\n"
"    }\n"
"\n"
"    * { box-sizing: border-box; }\n"
"\n"
"    body {\n"
"      margin: 0;\n"
"      background-color: var(--ucf-black);\n"
"      color: white;\n"
"      font-family: Arial, sans-serif;\n"
"      padding-bottom: 98px;\n"
"    }\n"
"\n"
"    .app {\n"
"      padding: 25px;\n"
"      max-width: 1400px;\n"
"      margin: 0 auto;\n"
"    }\n"
"\n"
"    h1 {\n"
"      text-align: center;\n"
"      color: var(--ucf-gold);\n"
"      letter-spacing: 2px;\n"
"      margin: 0 0 10px;\n"
"    }\n"
"\n"
"    .subtitle {\n"
"      text-align: center;\n"
"      color: var(--muted);\n"
"      margin: 0 0 25px;\n"
"    }\n"
"\n"
"    .status-card,\n"
"    .sensor-card,\n"
"    .switch-card,\n"
"    .info-card,\n"
"    .override-card {\n"
"      border: 2px solid var(--ucf-gold);\n"
"      border-radius: 18px;\n"
"      padding: 20px;\n"
"      background-color: var(--panel);\n"
"    }\n"
"\n"
"    .status-card {\n"
"      margin-bottom: 25px;\n"
"    }\n"
"\n"
"    .status-card h2,\n"
"    .sensor-card h3,\n"
"    .switch-card h3,\n"
"    .info-card h3,\n"
"    .override-card h3 {\n"
"      color: var(--ucf-gold);\n"
"      margin-top: 0;\n"
"    }\n"
"\n"
"    .sensor-grid {\n"
"      display: grid;\n"
"      grid-template-columns: repeat(4, 1fr);\n"
"      gap: 18px;\n"
"    }\n"
"\n"
"    .sensor-card {\n"
"      text-align: center;\n"
"    }\n"
"\n"
"    .icon {\n"
"      font-size: 32px;\n"
"      margin-bottom: 10px;\n"
"    }\n"
"\n"
"    .value {\n"
"      font-size: 28px;\n"
"      font-weight: bold;\n"
"      margin: 10px 0;\n"
"    }\n"
"\n"
"    .label {\n"
"      color: var(--muted);\n"
"      margin-bottom: 0;\n"
"    }\n"
"\n"
"    .switch-row,\n"
"    .info-grid {\n"
"      display: grid;\n"
"      grid-template-columns: 1fr 1fr;\n"
"      gap: 18px;\n"
"      margin-top: 25px;\n"
"    }\n"
"\n"
"    .switch-card {\n"
"      display: flex;\n"
"      justify-content: space-between;\n"
"      align-items: center;\n"
"      gap: 16px;\n"
"    }\n"
"\n"
"    .switch {\n"
"      position: relative;\n"
"      width: 60px;\n"
"      height: 34px;\n"
"      flex: 0 0 auto;\n"
"    }\n"
"\n"
"    .switch input {\n"
"      display: none;\n"
"    }\n"
"\n"
"    .slider {\n"
"      position: absolute;\n"
"      cursor: pointer;\n"
"      inset: 0;\n"
"      background-color: #444;\n"
"      border-radius: 34px;\n"
"      transition: 0.3s;\n"
"    }\n"
"\n"
"    .slider:before {\n"
"      content: \"\";\n"
"      position: absolute;\n"
"      height: 26px;\n"
"      width: 26px;\n"
"      left: 4px;\n"
"      bottom: 4px;\n"
"      background-color: white;\n"
"      border-radius: 50%;\n"
"      transition: 0.3s;\n"
"    }\n"
"\n"
"    input:checked + .slider {\n"
"      background-color: var(--ucf-gold);\n"
"    }\n"
"\n"
"    input:checked + .slider:before {\n"
"      transform: translateX(26px);\n"
"    }\n"
"\n"
"    .refresh-row {\n"
"      margin-top: 25px;\n"
"      text-align: center;\n"
"    }\n"
"\n"
"    .refresh-btn {\n"
"      background-color: var(--ucf-gold);\n"
"      color: black;\n"
"      border: none;\n"
"      border-radius: 14px;\n"
"      padding: 14px 28px;\n"
"      font-size: 18px;\n"
"      font-weight: bold;\n"
"      cursor: pointer;\n"
"    }\n"
"\n"
"    .refresh-btn:active {\n"
"      transform: scale(0.98);\n"
"    }\n"
"\n"
"    .page { display: none; }\n"
"    .page.active { display: block; }\n"
"\n"
"    .section-heading {\n"
"      color: var(--ucf-gold);\n"
"      margin: 0 0 8px;\n"
"    }\n"
"\n"
"    .section-note {\n"
"      color: var(--muted);\n"
"      margin: 0 0 22px;\n"
"    }\n"
"\n"
"    .state-good { color: var(--good); font-weight: bold; }\n"
"    .state-warn { color: var(--warn); font-weight: bold; }\n"
"    .state-bad { color: var(--bad); font-weight: bold; }\n"
"\n"
"    .setting-row {\n"
"      display: grid;\n"
"      grid-template-columns: 1.2fr 1fr;\n"
"      gap: 12px;\n"
"      align-items: center;\n"
"      border-top: 1px solid #343434;\n"
"      padding: 14px 0;\n"
"    }\n"
"\n"
"    .setting-row:first-of-type { border-top: 0; }\n"
"\n"
"    .setting-row input {\n"
"      width: 100%;\n"
"      padding: 10px 12px;\n"
"      background: #050505;\n"
"      color: white;\n"
"      border: 1px solid var(--ucf-gold);\n"
"      border-radius: 10px;\n"
"      font-size: 16px;\n"
"    }\n"
"\n"
"    .small-note {\n"
"      color: var(--muted);\n"
"      font-size: 14px;\n"
"      line-height: 1.45;\n"
"    }\n"
"\n"
"    .bottom-tabs {\n"
"      position: fixed;\n"
"      z-index: 20;\n"
"      left: 0;\n"
"      right: 0;\n"
"      bottom: 0;\n"
"      background: #0d0d0d;\n"
"      border-top: 2px solid var(--ucf-gold);\n"
"      display: flex;\n"
"      justify-content: center;\n"
"      gap: 4px;\n"
"      padding: 10px max(10px, env(safe-area-inset-right)) calc(10px + env(safe-area-inset-bottom)) max(10px, env(safe-area-inset-left));\n"
"      overflow-x: auto;\n"
"    }\n"
"\n"
"    .tab-btn {\n"
"      background: transparent;\n"
"      color: #d9d9d9;\n"
"      border: 1px solid transparent;\n"
"      border-radius: 12px;\n"
"      padding: 11px 15px;\n"
"      font-weight: bold;\n"
"      cursor: pointer;\n"
"      white-space: nowrap;\n"
"      min-width: 86px;\n"
"    }\n"
"\n"
"    .tab-btn.active {\n"
"      background: var(--ucf-gold);\n"
"      color: black;\n"
"      border-color: var(--ucf-gold);\n"
"    }\n"
"\n"
"    .override-actions {\n"
"      display: flex;\n"
"      gap: 12px;\n"
"      flex-wrap: wrap;\n"
"      margin-top: 14px;\n"
"    }\n"
"\n"
"    .override-btn {\n"
"      border: 1px solid var(--ucf-gold);\n"
"      background: #050505;\n"
"      color: white;\n"
"      border-radius: 12px;\n"
"      padding: 11px 15px;\n"
"      font-size: 15px;\n"
"      font-weight: bold;\n"
"      cursor: pointer;\n"
"    }\n"
"\n"
"    .override-btn.active {\n"
"      background: var(--ucf-gold);\n"
"      color: black;\n"
"    }\n"
"\n"
"    .override-btn.stop {\n"
"      border-color: #ff6a6a;\n"
"      color: #ff8d8d;\n"
"    }\n"
"\n"
"    .override-btn.stop.active {\n"
"      background: #ff6a6a;\n"
"      color: black;\n"
"    }\n"
"\n"
"    .admin-warning {\n"
"      border: 1px solid #ffbf47;\n"
"      background: #2b210c;\n"
"      border-radius: 12px;\n"
"      padding: 13px;\n"
"      margin-top: 18px;\n"
"      line-height: 1.45;\n"
"    }\n"
"\n"
"    @media (max-width: 1050px) {\n"
"      .sensor-grid { grid-template-columns: repeat(2, 1fr); }\n"
"    }\n"
"\n"
"    @media (max-width: 650px) {\n"
"      .app { padding: 18px; }\n"
"      .sensor-grid,\n"
"      .switch-row,\n"
"      .info-grid { grid-template-columns: 1fr; }\n"
"      .status-card,\n"
"      .sensor-card,\n"
"      .switch-card,\n"
"      .info-card,\n"
"      .override-card { padding: 17px; }\n"
"      h1 { font-size: 24px; }\n"
"      .tab-btn { min-width: 72px; padding: 10px 11px; font-size: 12px; }\n"
"    }\n"
"  </style>\n"
"</head>\n"
"\n"
"<body>\n"
"  <div class=\"app\">\n"
"    <h1>F.R.O.G.S. Water System</h1>\n"
"    <p class=\"subtitle\">Filtration Resource for Off-Grid Systems · Monitor on Home · Configure in Admin</p>\n"
"\n"
"    <section id=\"homePage\" class=\"page active\">\n"
"      <div class=\"status-card\">\n"
"        <h2>System Status</h2>\n"
"        <p id=\"connectionStatus\">❌ Device Disconnected</p>\n"
"        <p id=\"wifiStatus\">⚠️ Waiting for ESP32 connection...</p>\n"
"        <p id=\"systemBlock\">System state: <span class=\"state-warn\">WAITING FOR DATA</span></p>\n"
"      </div>\n"
"\n"
"      <div class=\"sensor-grid\">\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">💧</div>\n"
"          <h3>TDS</h3>\n"
"          <p class=\"value\" id=\"tdsValue\">-- ppm</p>\n"
"          <p class=\"label\">Water Purity</p>\n"
"        </div>\n"
"\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">🌡️</div>\n"
"          <h3>Temperature</h3>\n"
"          <p class=\"value\" id=\"tempValue\">-- °F</p>\n"
"          <p class=\"label\">Water Temperature</p>\n"
"        </div>\n"
"\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">🌊</div>\n"
"          <h3>Flow 1</h3>\n"
"          <p class=\"value\" id=\"flowValue\">-- L/min</p>\n"
"          <p class=\"label\">Input Water Flow</p>\n"
"        </div>\n"
"\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">🚰</div>\n"
"          <h3>Flow 2</h3>\n"
"          <p class=\"value\" id=\"homeFlow2Value\">-- L/min</p>\n"
"          <p class=\"label\">Output Water Flow</p>\n"
"        </div>\n"
"\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">📊</div>\n"
"          <h3>F1 : F2</h3>\n"
"          <p class=\"value\" id=\"homeRatioValue\">-- : --</p>\n"
"          <p class=\"label\">Direct Flow Values</p>\n"
"        </div>\n"
"\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">🔋</div>\n"
"          <h3>Battery</h3>\n"
"          <p class=\"value\" id=\"batteryValue\">-- V</p>\n"
"          <p class=\"label\">Battery Voltage</p>\n"
"        </div>\n"
"\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">☀️</div>\n"
"          <h3>LDR State</h3>\n"
"          <p class=\"value\" id=\"homeLdrState\">--</p>\n"
"          <p class=\"label\" id=\"homeLdrValue\">Light Sensor</p>\n"
"        </div>\n"
"\n"
"        <div class=\"sensor-card\">\n"
"          <div class=\"icon\">🪣</div>\n"
"          <h3>Tank Status</h3>\n"
"          <p class=\"value\" id=\"tankStatusValue\">--</p>\n"
"          <p class=\"label\" id=\"tankStatusDetail\">Float Switch: --</p>\n"
"        </div>\n"
"      </div>\n"
"\n"
"      <div class=\"switch-row\">\n"
"        <div class=\"switch-card\">\n"
"          <div>\n"
"            <h3>Pump 1 Enable</h3>\n"
"            <p class=\"small-note\">Requests low-pressure pump operation. Firmware safety rules remain active.</p>\n"
"          </div>\n"
"          <label class=\"switch\">\n"
"            <input type=\"checkbox\" id=\"pump1Switch\" onchange=\"setPump1(this.checked)\">\n"
"            <span class=\"slider\"></span>\n"
"          </label>\n"
"        </div>\n"
"\n"
"        <div class=\"switch-card\">\n"
"          <div>\n"
"            <h3>Pump 2 Enable</h3>\n"
"            <p class=\"small-note\">Requests high-pressure pump operation. Firmware safety rules remain active.</p>\n"
"          </div>\n"
"          <label class=\"switch\">\n"
"            <input type=\"checkbox\" id=\"pump2Switch\" onchange=\"setPump2(this.checked)\">\n"
"            <span class=\"slider\"></span>\n"
"          </label>\n"
"        </div>\n"
"      </div>\n"
"\n"
"      <div class=\"refresh-row\">\n"
"        <button class=\"refresh-btn\" onclick=\"getSensorData()\">Refresh Data</button>\n"
"      </div>\n"
"    </section>\n"
"\n"
"    <section id=\"adminPage\" class=\"page\">\n"
"      <h2 class=\"section-heading\">Admin Settings</h2>\n"
"      <p class=\"section-note\">Maintenance and protection settings. Readouts update from the ESP32 when the matching values are included in <code>/data</code>.</p>\n"
"\n"
"      <div class=\"status-card\">\n"
"        <h2>System Admin Mode</h2>\n"
"        <div class=\"switch-card\" style=\"margin-top:16px;\">\n"
"          <div>\n"
"            <h3 style=\"margin-bottom:6px;\">Stop Pump Operation</h3>\n"
"            <p class=\"small-note\">When enabled, both pumps are forced off. The OLED, sensors, Wi-Fi, and webpage remain active so the system can still be monitored and Admin Mode can be turned off.</p>\n"
"            <p id=\"adminModeStatus\" class=\"state-good\">Admin Mode: OFF — normal pump control</p>\n"
"          </div>\n"
"          <label class=\"switch\">\n"
"            <input type=\"checkbox\" id=\"adminModeSwitch\" onchange=\"setAdminMode(this.checked)\">\n"
"            <span class=\"slider\"></span>\n"
"          </label>\n"
"        </div>\n"
"        <p id=\"adminModeNotice\" class=\"small-note\" style=\"text-align:center; margin:14px 0 0;\"></p>\n"
"      </div>\n"
"\n"
"      <div class=\"info-grid\">\n"
"        <div class=\"info-card\">\n"
"          <h3>Current Protection Status</h3>\n"
"          <div class=\"setting-row\"><span>Battery Voltage Now</span><strong id=\"adminBatteryReadout\">-- V</strong></div>\n"
"          <div class=\"setting-row\"><span>Low-Voltage Disconnect (LVD)</span><strong id=\"lvdReadout\">-- V</strong></div>\n"
"          <div class=\"setting-row\"><span>Minimum Voltage Restart (MVR)</span><strong id=\"mvrReadout\">-- V</strong></div>\n"
"          <div class=\"setting-row\"><span>Daylight Threshold</span><strong id=\"lightThresholdReadout\">--</strong></div>\n"
"          <div class=\"setting-row\"><span>Minimum Flow</span><strong id=\"minFlowReadout\">-- L/min</strong></div>\n"
"        </div>\n"
"\n"
"        <div class=\"info-card\">\n"
"          <h3>Voltage Protection Control</h3>\n"
"          <p class=\"small-note\">LVD is the shutdown voltage. MVR is the voltage required before the system is allowed to restart.</p>\n"
"\n"
"          <div class=\"setting-row\">\n"
"            <label for=\"lvdSlider\">Shutdown Voltage (LVD)</label>\n"
"            <div>\n"
"              <input id=\"lvdSlider\" type=\"range\" min=\"10.0\" max=\"13.0\" step=\"0.1\" value=\"12.0\" oninput=\"syncVoltageInputs('lvd')\">\n"
"              <input id=\"lvdInput\" type=\"number\" min=\"10.0\" max=\"13.0\" step=\"0.1\" value=\"12.0\" oninput=\"syncVoltageInputs('lvd', true)\">\n"
"            </div>\n"
"          </div>\n"
"\n"
"          <div class=\"setting-row\">\n"
"            <label for=\"mvrSlider\">Restart Voltage (MVR)</label>\n"
"            <div>\n"
"              <input id=\"mvrSlider\" type=\"range\" min=\"11.0\" max=\"14.0\" step=\"0.1\" value=\"12.8\" oninput=\"syncVoltageInputs('mvr')\">\n"
"              <input id=\"mvrInput\" type=\"number\" min=\"11.0\" max=\"14.0\" step=\"0.1\" value=\"12.8\" oninput=\"syncVoltageInputs('mvr', true)\">\n"
"            </div>\n"
"          </div>\n"
"\n"
"          <div class=\"refresh-row\"><button class=\"refresh-btn\" onclick=\"saveProtectionSettings()\">Apply Voltage Settings</button></div>\n"
"          <p id=\"adminNotice\" class=\"small-note\" style=\"text-align:center;\"></p>\n"
"        </div>\n"
"\n"
"        <div class=\"info-card\">\n"
"          <h3>Daylight Safety Control</h3>\n"
"          <p class=\"small-note\">This maps to the existing OLED LDR CHK setting in firmware once the admin route is connected.</p>\n"
"\n"
"          <div class=\"switch-card\" style=\"margin-top:16px;\">\n"
"            <div>\n"
"              <h3 style=\"margin-bottom:6px;\">Daylight Threshold</h3>\n"
"              <p id=\"daylightControlStatus\" class=\"small-note\">Protection: ENABLED</p>\n"
"            </div>\n"
"            <label class=\"switch\">\n"
"              <input type=\"checkbox\" id=\"daylightEnableSwitch\" checked onchange=\"setDaylightControl(this.checked)\">\n"
"              <span class=\"slider\"></span>\n"
"            </label>\n"
"          </div>\n"
"\n"
"          <div class=\"setting-row\" style=\"margin-top:16px;\">\n"
"            <label for=\"daylightThresholdInput\">LDR Threshold</label>\n"
"            <input id=\"daylightThresholdInput\" type=\"number\" min=\"0\" max=\"4095\" step=\"1\" value=\"1500\" oninput=\"previewDaylightThreshold()\">\n"
"          </div>\n"
"\n"
"          <div class=\"refresh-row\">\n"
"            <button class=\"refresh-btn\" onclick=\"saveDaylightThreshold()\">Apply Daylight Settings</button>\n"
"          </div>\n"
"          <p id=\"daylightNotice\" class=\"small-note\" style=\"text-align:center;\"></p>\n"
"        </div>\n"
"      </div>\n"
"\n"
"      <div class=\"status-card\" style=\"margin-top:25px;\">\n"
"        <h2>Protection Rule</h2>\n"
"        <p id=\"voltageRuleText\" class=\"state-warn\">The system will shut down at -- V and may restart at -- V.</p>\n"
"        <p class=\"small-note\">The webpage is ready for the admin save routes, but read/display works independently through <code>/data</code>.</p>\n"
"      </div>\n"
"\n"
"      <div class=\"info-grid\">\n"
"        <div class=\"override-card\">\n"
"          <h3>Low-Pressure Pump Override</h3>\n"
"          <p id=\"lpOverrideState\" class=\"state-warn\">Override: OFF</p>\n"
"          <p class=\"small-note\">Use only for supervised maintenance and troubleshooting.</p>\n"
"          <div class=\"override-actions\">\n"
"            <button id=\"lpOverrideOnBtn\" class=\"override-btn\" onclick=\"setPumpOverride('lp', true)\">Enable Override</button>\n"
"            <button id=\"lpOverrideOffBtn\" class=\"override-btn stop active\" onclick=\"setPumpOverride('lp', false)\">Disable Override</button>\n"
"          </div>\n"
"        </div>\n"
"      </div>\n"
"\n"
"      <div class=\"admin-warning\">\n"
"        <strong>Important:</strong> only the low-pressure pump has a temporary maintenance override. The high-pressure pump remains protected from Admin override commands.\n"
"      </div>\n"
"      <p id=\"overrideNotice\" class=\"small-note\" style=\"text-align:center; margin-top:14px;\"></p>\n"
"\n"
"      <div class=\"status-card\" style=\"margin-top:25px;\">\n"
"        <h2 style=\"margin-bottom:8px;\">Safety Condition Checks</h2>\n"
"        <p class=\"small-note\">These match the OLED Admin Edit checks already in firmware, but are written out more clearly for the web app.</p>\n"
"\n"
"        <div class=\"info-grid\" style=\"margin-top:18px;\">\n"
"          <div class=\"switch-card\">\n"
"            <div>\n"
"              <h3>Light / Daylight Check</h3>\n"
"              <p id=\"ldrCheckStatus\" class=\"small-note\">Daylight safety check: ENABLED</p>\n"
"            </div>\n"
"            <label class=\"switch\">\n"
"              <input type=\"checkbox\" id=\"ldrCheckSwitch\" checked onchange=\"setConditionCheck('ldr', this.checked)\">\n"
"              <span class=\"slider\"></span>\n"
"            </label>\n"
"          </div>\n"
"\n"
"          <div class=\"switch-card\">\n"
"            <div>\n"
"              <h3>Float Switch Check</h3>\n"
"              <p id=\"floatCheckStatus\" class=\"small-note\">Tank float check: ENABLED</p>\n"
"            </div>\n"
"            <label class=\"switch\">\n"
"              <input type=\"checkbox\" id=\"floatCheckSwitch\" checked onchange=\"setConditionCheck('float', this.checked)\">\n"
"              <span class=\"slider\"></span>\n"
"            </label>\n"
"          </div>\n"
"\n"
"          <div class=\"switch-card\">\n"
"            <div>\n"
"              <h3>Voltage Protection Check</h3>\n"
"              <p id=\"lvdCheckStatus\" class=\"small-note\">Low-voltage check: ENABLED</p>\n"
"            </div>\n"
"            <label class=\"switch\">\n"
"              <input type=\"checkbox\" id=\"lvdCheckSwitch\" checked onchange=\"setConditionCheck('lvd', this.checked)\">\n"
"              <span class=\"slider\"></span>\n"
"            </label>\n"
"          </div>\n"
"        </div>\n"
"\n"
"        <div class=\"admin-warning\" style=\"margin-top:18px;\">\n"
"          <strong>Safety note:</strong> these switches map directly to <code>g_ldr_check</code>, <code>g_float_check</code>, and <code>g_lvd_check</code>. They are the same checks shown on the OLED Admin screen, just written out with clearer names.\n"
"        </div>\n"
"\n"
"        <p id=\"conditionNotice\" class=\"small-note\" style=\"text-align:center; margin:18px 0 0;\"></p>\n"
"      </div>\n"
"\n"
"      <div class=\"status-card\" style=\"margin-top:25px;\">\n"
"        <h2>Reset Defaults</h2>\n"
"        <p class=\"small-note\">Restores the Admin protection settings back to the firmware default values. This should match the OLED RESET DEFAULTS option.</p>\n"
"        <div class=\"refresh-row\">\n"
"          <button class=\"refresh-btn\" onclick=\"resetDefaults()\">Reset Admin Defaults</button>\n"
"        </div>\n"
"        <p id=\"resetNotice\" class=\"small-note\" style=\"text-align:center; margin-top:14px;\"></p>\n"
"      </div>\n"
"    </section>\n"
"  </div>\n"
"\n"
"  <nav class=\"bottom-tabs\" aria-label=\"FROGS app pages\">\n"
"    <button class=\"tab-btn active\" data-page=\"homePage\">HOME</button>\n"
"    <button class=\"tab-btn\" data-page=\"adminPage\">ADMIN</button>\n"
"  </nav>\n"
"\n"
"  <script>\n"
"    const demoData = {\n"
"      tds: 184,\n"
"      temperature: 74.3,\n"
"      flowRate: 1.25,\n"
"      flow1: 1.42,\n"
"      flow2: 1.18,\n"
"      battery: 12.6,\n"
"      pump1Enabled: false,\n"
"      pump2Enabled: false,\n"
"      daylight: \"NIGHT\",\n"
"      ldr: 0,\n"
"      tank: \"LOW\",\n"
"      blocked: \"NIGHT\",\n"
"      lvd: 12.0,\n"
"      mvr: 12.8,\n"
"      daylightThreshold: 1500,\n"
"      minFlow: 0.40,\n"
"      lpOverride: false,\n"
"      conditionChecks: {\n"
"        ldr: true,\n"
"        float: true,\n"
"        lvd: true\n"
"      },\n"
"      daylightSafetyEnabled: true,\n"
"      floatUp: false\n"
"    };\n"
"\n"
"    let lastData = {};\n"
"    let demoMode = false;\n"
"    let webAdminMode = false;\n"
"\n"
"    function setText(id, value) {\n"
"      const el = document.getElementById(id);\n"
"      if (el) el.innerText = value;\n"
"    }\n"
"\n"
"    function setChecked(id, checked) {\n"
"      const el = document.getElementById(id);\n"
"      if (el) el.checked = Boolean(checked);\n"
"    }\n"
"\n"
"    function setClass(id, className) {\n"
"      const el = document.getElementById(id);\n"
"      if (el) el.className = className;\n"
"    }\n"
"\n"
"    function boolText(value) {\n"
"      return value ? \"ON\" : \"OFF\";\n"
"    }\n"
"\n"
"    function firstDefined(data, names, fallback) {\n"
"      for (const name of names) {\n"
"        if (data[name] !== undefined && data[name] !== null) return data[name];\n"
"      }\n"
"      return fallback;\n"
"    }\n"
"\n"
"    function toBool(value, fallback = false) {\n"
"      if (value === undefined || value === null) return fallback;\n"
"      if (typeof value === \"boolean\") return value;\n"
"      if (typeof value === \"number\") return value !== 0;\n"
"      if (typeof value === \"string\") {\n"
"        const v = value.trim().toLowerCase();\n"
"        if ([\"true\", \"1\", \"on\", \"enabled\", \"yes\", \"day\", \"full\", \"up\"].includes(v)) return true;\n"
"        if ([\"false\", \"0\", \"off\", \"disabled\", \"no\", \"night\", \"low\", \"down\"].includes(v)) return false;\n"
"      }\n"
"      return fallback;\n"
"    }\n"
"\n"
"    function valueExists(value) {\n"
"      return value !== undefined && value !== null && value !== \"--\" && value !== \"UNKNOWN\";\n"
"    }\n"
"\n"
"    function formatValue(value, digits = 1) {\n"
"      const n = Number(value);\n"
"      return Number.isFinite(n) ? n.toFixed(digits) : \"--\";\n"
"    }\n"
"\n"
"    function normalizeDaylight(value) {\n"
"      if (!valueExists(value)) return \"UNKNOWN\";\n"
"      if (typeof value === \"boolean\") return value ? \"DAY\" : \"NIGHT\";\n"
"      if (typeof value === \"number\") return value !== 0 ? \"DAY\" : \"NIGHT\";\n"
"      const v = String(value).trim().toUpperCase();\n"
"      if ([\"DAY\", \"LIGHT\", \"BRIGHT\", \"SUN\", \"SUNLIGHT\", \"TRUE\", \"1\", \"ON\"].includes(v)) return \"DAY\";\n"
"      if ([\"NIGHT\", \"DARK\", \"FALSE\", \"0\", \"OFF\"].includes(v)) return \"NIGHT\";\n"
"      return v;\n"
"    }\n"
"\n"
"    function buildConditionChecks(data) {\n"
"      const nested = firstDefined(data, [\"conditionChecks\", \"checks\"], null);\n"
"      if (nested && typeof nested === \"object\") {\n"
"        return {\n"
"          ldr: toBool(firstDefined(nested, [\"ldr\", \"ldrCheck\", \"ldr_check\"], true), true),\n"
"          float: toBool(firstDefined(nested, [\"float\", \"floatCheck\", \"float_check\"], true), true),\n"
"          lvd: toBool(firstDefined(nested, [\"lvd\", \"lvdCheck\", \"lvd_check\"], true), true)\n"
"        };\n"
"      }\n"
"\n"
"      const oldNested = firstDefined(data, [\"sensorStates\", \"sensor_states\"], null);\n"
"      return {\n"
"        ldr: toBool(firstDefined(data, [\"ldrCheck\", \"ldr_check\", \"daylightSafetyEnabled\"], oldNested ? oldNested.ldr : true), true),\n"
"        float: toBool(firstDefined(data, [\"floatCheck\", \"float_check\"], oldNested ? oldNested.float : true), true),\n"
"        lvd: toBool(firstDefined(data, [\"lvdCheck\", \"lvd_check\"], oldNested ? oldNested.battery : true), true)\n"
"      };\n"
"    }\n"
"\n"
"    function updatePage(data) {\n"
"      const tds = firstDefined(data, [\"tds\", \"tdsPpm\"], \"--\");\n"
"      const temperature = firstDefined(data, [\"temperature\", \"temp\", \"tempF\"], \"--\");\n"
"      const flow1 = firstDefined(data, [\"flow1\", \"inputFlow\", \"flowRate\", \"flow\"], \"--\");\n"
"      const flow2 = firstDefined(data, [\"flow2\", \"outputFlow\"], \"--\");\n"
"      const battery = firstDefined(data, [\"battery\", \"batteryVoltage\", \"battery_voltage\"], \"--\");\n"
"\n"
"      const pump1 = toBool(firstDefined(data, [\"pump1Enabled\", \"lpPumpEnabled\", \"lpEnabled\", \"lpPumpEnable\", \"lp_enable\"], false));\n"
"      const pump2 = toBool(firstDefined(data, [\"pump2Enabled\", \"hpPumpEnabled\", \"hpEnabled\", \"hpPumpEnable\", \"hp_enable\"], false));\n"
"\n"
"      setText(\"tdsValue\", formatValue(tds, 1) + \" ppm\");\n"
"      setText(\"tempValue\", formatValue(temperature, 1) + \" °F\");\n"
"      setText(\"flowValue\", formatValue(flow1, 2) + \" L/min\");\n"
"      setText(\"homeFlow2Value\", formatValue(flow2, 2) + \" L/min\");\n"
"      setText(\"batteryValue\", formatValue(battery, 2) + \" V\");\n"
"\n"
"      const n1 = Number(flow1);\n"
"      const n2 = Number(flow2);\n"
"      const flowPair = (Number.isFinite(n1) && Number.isFinite(n2))\n"
"        ? n1.toFixed(2) + \" : \" + n2.toFixed(2)\n"
"        : formatValue(flow1, 2) + \" : \" + formatValue(flow2, 2);\n"
"      setText(\"homeRatioValue\", flowPair);\n"
"\n"
"      setChecked(\"pump1Switch\", pump1);\n"
"      setChecked(\"pump2Switch\", pump2);\n"
"\n"
"      const daylight = normalizeDaylight(firstDefined(data, [\"daylight\", \"lightState\", \"daylightConfirmed\", \"lightConfirmed\"], undefined));\n"
"      const ldr = firstDefined(data, [\"ldr\", \"lightValue\", \"ldrRaw\", \"ldr_raw\"], \"--\");\n"
"      setText(\"homeLdrState\", daylight);\n"
"      setText(\"homeLdrValue\", \"LDR: \" + (valueExists(ldr) ? ldr : \"--\"));\n"
"\n"
"      const floatRaw = firstDefined(data, [\"floatUp\", \"floatSwitchUp\", \"floatState\", \"tankFloat\", \"tankFull\", \"tank_full\"], undefined);\n"
"      const tankRaw = firstDefined(data, [\"tank\", \"tankStatus\", \"tank_status\"], undefined);\n"
"\n"
"      if (valueExists(floatRaw)) {\n"
"        const floatIsUp = toBool(floatRaw, false);\n"
"        setText(\"tankStatusValue\", floatIsUp ? \"FULL\" : \"LOW\");\n"
"        setText(\"tankStatusDetail\", \"Float Switch: \" + (floatIsUp ? \"UP\" : \"DOWN\"));\n"
"        setClass(\"tankStatusValue\", \"value \" + (floatIsUp ? \"state-good\" : \"state-warn\"));\n"
"      } else if (valueExists(tankRaw)) {\n"
"        const tankText = String(tankRaw).toUpperCase();\n"
"        setText(\"tankStatusValue\", tankText);\n"
"        setText(\"tankStatusDetail\", \"Float Switch: state not provided\");\n"
"        setClass(\"tankStatusValue\", \"value \" + ([\"EMPTY\", \"FULL\"].includes(tankText) ? \"state-good\" : \"state-warn\"));\n"
"      } else {\n"
"        setText(\"tankStatusValue\", \"UNKNOWN\");\n"
"        setText(\"tankStatusDetail\", \"Float Switch: no data\");\n"
"        setClass(\"tankStatusValue\", \"value state-warn\");\n"
"      }\n"
"\n"
"      const blocked = firstDefined(data, [\"blocked\", \"blockReason\", \"block_reason\"], \"NONE\");\n"
"      const blockedEl = document.getElementById(\"systemBlock\");\n"
"      const clearState = blocked === \"NONE\" || blocked === \"OK\" || blocked === false;\n"
"      blockedEl.innerHTML = \"System state: <span class='\" + (clearState ? \"state-good\" : \"state-warn\") + \"'>\" + (clearState ? \"SYSTEM OK\" : \"BLOCKED: \" + blocked) + \"</span>\";\n"
"\n"
"      const lvd = firstDefined(data, [\"lvd\", \"lowVoltageDisconnect\"], undefined);\n"
"      const mvr = firstDefined(data, [\"mvr\", \"minimumVoltageRestart\"], undefined);\n"
"      const daylightThreshold = firstDefined(data, [\"daylightThreshold\", \"ldrThreshold\", \"ldr_threshold\"], undefined);\n"
"      const minFlow = firstDefined(data, [\"minFlow\", \"minimumFlow\", \"min_flow\"], undefined);\n"
"      const daylightSafetyEnabled = toBool(firstDefined(data, [\"daylightSafetyEnabled\", \"daylight_enabled\", \"daylightControlEnabled\", \"ldrCheck\", \"ldr_check\"], true), true);\n"
"      const lpOverride = toBool(firstDefined(data, [\"lpOverride\", \"lp_override\", \"lpPumpOverride\"], false), false);\n"
"\n"
"      setText(\"adminBatteryReadout\", formatValue(battery, 2) + \" V\");\n"
"      setText(\"lvdReadout\", valueExists(lvd) ? formatValue(lvd, 2) + \" V\" : \"-- V\");\n"
"      setText(\"mvrReadout\", valueExists(mvr) ? formatValue(mvr, 2) + \" V\" : \"-- V\");\n"
"      setText(\"lightThresholdReadout\", valueExists(daylightThreshold) ? daylightThreshold : \"--\");\n"
"      setText(\"minFlowReadout\", valueExists(minFlow) ? formatValue(minFlow, 2) + \" L/min\" : \"-- L/min\");\n"
"\n"
"      updateDaylightControlDisplay(daylightSafetyEnabled);\n"
"      updateOverrideDisplay(\"lp\", lpOverride);\n"
"      updateConditionDisplays(buildConditionChecks(data));\n"
"\n"
"      const displayLvd = valueExists(lvd) ? Number(lvd).toFixed(1) : \"--\";\n"
"      const displayMvr = valueExists(mvr) ? Number(mvr).toFixed(1) : \"--\";\n"
"      setText(\"voltageRuleText\", \"The system will shut down at \" + displayLvd + \" V and may restart at \" + displayMvr + \" V.\");\n"
"\n"
"      if (valueExists(lvd) &&\n"
"          document.activeElement !== document.getElementById(\"lvdInput\") &&\n"
"          document.activeElement !== document.getElementById(\"lvdSlider\")) {\n"
"        document.getElementById(\"lvdInput\").value = Number(lvd).toFixed(1);\n"
"        document.getElementById(\"lvdSlider\").value = Number(lvd).toFixed(1);\n"
"      }\n"
"\n"
"      if (valueExists(mvr) &&\n"
"          document.activeElement !== document.getElementById(\"mvrInput\") &&\n"
"          document.activeElement !== document.getElementById(\"mvrSlider\")) {\n"
"        document.getElementById(\"mvrInput\").value = Number(mvr).toFixed(1);\n"
"        document.getElementById(\"mvrSlider\").value = Number(mvr).toFixed(1);\n"
"      }\n"
"\n"
"      if (valueExists(daylightThreshold) &&\n"
"          document.activeElement !== document.getElementById(\"daylightThresholdInput\")) {\n"
"        document.getElementById(\"daylightThresholdInput\").value = daylightThreshold;\n"
"      }\n"
"    }\n"
"\n"
"    function updateAdminModeDisplay(enabled) {\n"
"      webAdminMode = Boolean(enabled);\n"
"      setChecked(\"adminModeSwitch\", webAdminMode);\n"
"\n"
"      const statusEl = document.getElementById(\"adminModeStatus\");\n"
"      if (statusEl) {\n"
"        statusEl.textContent = webAdminMode\n"
"          ? \"Admin Mode: ON — both pumps are forced off\"\n"
"          : \"Admin Mode: OFF — normal pump control\";\n"
"        statusEl.className = webAdminMode ? \"state-bad\" : \"state-good\";\n"
"      }\n"
"\n"
"      const pump1Switch = document.getElementById(\"pump1Switch\");\n"
"      const pump2Switch = document.getElementById(\"pump2Switch\");\n"
"      if (pump1Switch) pump1Switch.disabled = webAdminMode;\n"
"      if (pump2Switch) pump2Switch.disabled = webAdminMode;\n"
"    }\n"
"\n"
"    async function getAdminMode() {\n"
"      if (demoMode) {\n"
"        updateAdminModeDisplay(webAdminMode);\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/mode\", { cache: \"no-store\" });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        const data = await response.json();\n"
"        updateAdminModeDisplay(toBool(data.enabled, false));\n"
"      } catch (error) {\n"
"        setText(\"adminModeNotice\", \"Unable to read Admin Mode from the ESP32.\");\n"
"      }\n"
"    }\n"
"\n"
"    async function setAdminMode(enabled) {\n"
"      const confirmationText = enabled\n"
"        ? \"Enable System Admin Mode? Both pumps will be forced off until Admin Mode is disabled.\"\n"
"        : \"Disable System Admin Mode and return the pumps to normal automatic control?\";\n"
"\n"
"      if (!window.confirm(confirmationText)) {\n"
"        setChecked(\"adminModeSwitch\", !enabled);\n"
"        return;\n"
"      }\n"
"\n"
"      if (demoMode) {\n"
"        updateAdminModeDisplay(enabled);\n"
"        setText(\"adminModeNotice\", enabled\n"
"          ? \"Demo mode: both pumps are shown as stopped.\"\n"
"          : \"Demo mode: normal pump control restored.\");\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/mode?enabled=\" + (enabled ? \"1\" : \"0\"), {\n"
"          cache: \"no-store\"\n"
"        });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        const data = await response.json();\n"
"        updateAdminModeDisplay(toBool(data.enabled, enabled));\n"
"        setText(\"adminModeNotice\", enabled\n"
"          ? \"Admin Mode enabled. Both pumps are forced off.\"\n"
"          : \"Admin Mode disabled. Normal pump control restored.\");\n"
"        getSensorData();\n"
"      } catch (error) {\n"
"        setChecked(\"adminModeSwitch\", !enabled);\n"
"        setText(\"adminModeNotice\", \"Firmware did not accept the Admin Mode command.\");\n"
"      }\n"
"    }\n"
"\n"
"    async function getSensorData() {\n"
"      try {\n"
"        const response = await fetch(\"/data\", { cache: \"no-store\" });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        const data = await response.json();\n"
"\n"
"        demoMode = false;\n"
"        lastData = { ...data };\n"
"        updatePage(lastData);\n"
"\n"
"        setText(\"connectionStatus\", \"✅ Device Connected\");\n"
"        setText(\"wifiStatus\", \"ESP32 data received successfully.\");\n"
"        getAdminMode();\n"
"      } catch (error) {\n"
"        demoMode = true;\n"
"        lastData = { ...demoData };\n"
"        updatePage(lastData);\n"
"\n"
"        setText(\"connectionStatus\", \"🟡 Demo Mode\");\n"
"        setText(\"wifiStatus\", \"Open this webpage through the ESP32 to use live /data values.\");\n"
"        updateAdminModeDisplay(webAdminMode);\n"
"      }\n"
"    }\n"
"\n"
"    async function setPump1(enabled) {\n"
"      if (demoMode) {\n"
"        lastData.pump1Enabled = enabled;\n"
"        updatePage(lastData);\n"
"        return;\n"
"      }\n"
"      try {\n"
"        await fetch(enabled ? \"/lp/enable\" : \"/lp/disable\");\n"
"      } finally {\n"
"        getSensorData();\n"
"      }\n"
"    }\n"
"\n"
"    async function setPump2(enabled) {\n"
"      if (demoMode) {\n"
"        lastData.pump2Enabled = enabled;\n"
"        updatePage(lastData);\n"
"        return;\n"
"      }\n"
"      try {\n"
"        await fetch(enabled ? \"/hp/enable\" : \"/hp/disable\");\n"
"      } finally {\n"
"        getSensorData();\n"
"      }\n"
"    }\n"
"\n"
"    function updateDaylightControlDisplay(enabled) {\n"
"      setChecked(\"daylightEnableSwitch\", enabled);\n"
"      const statusEl = document.getElementById(\"daylightControlStatus\");\n"
"      if (statusEl) {\n"
"        statusEl.textContent = \"Protection: \" + (enabled ? \"ENABLED\" : \"DISABLED\");\n"
"        statusEl.className = enabled ? \"state-good\" : \"state-warn\";\n"
"      }\n"
"    }\n"
"\n"
"    function previewDaylightThreshold() {\n"
"      const threshold = document.getElementById(\"daylightThresholdInput\").value;\n"
"      setText(\"daylightNotice\", \"New daylight threshold ready to apply: \" + threshold);\n"
"    }\n"
"\n"
"    async function setDaylightControl(enabled) {\n"
"      const confirmationText = enabled\n"
"        ? \"Enable daylight threshold protection?\"\n"
"        : \"Disable daylight threshold protection? Pumps may no longer be blocked based on the LDR state.\";\n"
"\n"
"      if (!window.confirm(confirmationText)) {\n"
"        setChecked(\"daylightEnableSwitch\", !enabled);\n"
"        return;\n"
"      }\n"
"\n"
"      if (demoMode) {\n"
"        lastData.daylightSafetyEnabled = enabled;\n"
"        updatePage(lastData);\n"
"        setText(\"daylightNotice\", enabled ? \"Demo mode: daylight protection enabled.\" : \"Demo mode: daylight protection disabled.\");\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/daylight\", {\n"
"          method: \"POST\",\n"
"          headers: { \"Content-Type\": \"application/json\" },\n"
"          body: JSON.stringify({ enabled })\n"
"        });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        setText(\"daylightNotice\", enabled ? \"Daylight protection enabled.\" : \"Daylight protection disabled.\");\n"
"        getSensorData();\n"
"      } catch (error) {\n"
"        setChecked(\"daylightEnableSwitch\", !enabled);\n"
"        setText(\"daylightNotice\", \"Firmware still needs the /admin/daylight route before this switch can control the real setting.\");\n"
"      }\n"
"    }\n"
"\n"
"    async function saveDaylightThreshold() {\n"
"      const threshold = Number(document.getElementById(\"daylightThresholdInput\").value);\n"
"      if (!Number.isFinite(threshold) || threshold < 0 || threshold > 4095) {\n"
"        setText(\"daylightNotice\", \"Enter a valid LDR threshold from 0 to 4095.\");\n"
"        return;\n"
"      }\n"
"\n"
"      if (demoMode) {\n"
"        lastData.daylightThreshold = threshold;\n"
"        updatePage(lastData);\n"
"        setText(\"daylightNotice\", \"Demo mode: daylight threshold saved as \" + threshold + \".\");\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/daylight\", {\n"
"          method: \"POST\",\n"
"          headers: { \"Content-Type\": \"application/json\" },\n"
"          body: JSON.stringify({ threshold })\n"
"        });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        setText(\"daylightNotice\", \"Daylight threshold updated to \" + threshold + \".\");\n"
"        getSensorData();\n"
"      } catch (error) {\n"
"        setText(\"daylightNotice\", \"Firmware still needs the /admin/daylight route before this value can be saved.\");\n"
"      }\n"
"    }\n"
"\n"
"    const conditionConfig = {\n"
"      ldr: {\n"
"        switchId: \"ldrCheckSwitch\",\n"
"        statusId: \"ldrCheckStatus\",\n"
"        name: \"Light / Daylight Check\",\n"
"        statusPrefix: \"Daylight safety check\"\n"
"      },\n"
"      float: {\n"
"        switchId: \"floatCheckSwitch\",\n"
"        statusId: \"floatCheckStatus\",\n"
"        name: \"Float Switch Check\",\n"
"        statusPrefix: \"Tank float check\"\n"
"      },\n"
"      lvd: {\n"
"        switchId: \"lvdCheckSwitch\",\n"
"        statusId: \"lvdCheckStatus\",\n"
"        name: \"Voltage Protection Check\",\n"
"        statusPrefix: \"Low-voltage check\"\n"
"      }\n"
"    };\n"
"\n"
"    function updateConditionDisplays(checks) {\n"
"      Object.entries(conditionConfig).forEach(([key, config]) => {\n"
"        const enabled = toBool(checks[key], true);\n"
"        setChecked(config.switchId, enabled);\n"
"        const statusEl = document.getElementById(config.statusId);\n"
"        if (statusEl) {\n"
"          statusEl.textContent = config.statusPrefix + \": \" + (enabled ? \"ENABLED\" : \"DISABLED\");\n"
"          statusEl.className = enabled ? \"state-good\" : \"state-warn\";\n"
"        }\n"
"      });\n"
"    }\n"
"\n"
"    async function setConditionCheck(check, enabled) {\n"
"      const config = conditionConfig[check];\n"
"      if (!config) return;\n"
"\n"
"      const confirmationText = enabled\n"
"        ? \"Enable \" + config.name + \"?\"\n"
"        : \"Disable \" + config.name + \"? This should only be done for supervised maintenance or troubleshooting.\";\n"
"\n"
"      if (!window.confirm(confirmationText)) {\n"
"        setChecked(config.switchId, !enabled);\n"
"        return;\n"
"      }\n"
"\n"
"      if (demoMode) {\n"
"        if (!lastData.conditionChecks) lastData.conditionChecks = buildConditionChecks(lastData);\n"
"        lastData.conditionChecks[check] = enabled;\n"
"        if (check === \"ldr\") lastData.daylightSafetyEnabled = enabled;\n"
"        updatePage(lastData);\n"
"        setText(\"conditionNotice\", config.name + \" is now \" + (enabled ? \"enabled.\" : \"disabled.\") + \" (demo mode)\");\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/check\", {\n"
"          method: \"POST\",\n"
"          headers: { \"Content-Type\": \"application/json\" },\n"
"          body: JSON.stringify({ check, enabled })\n"
"        });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        setText(\"conditionNotice\", config.name + \" is now \" + (enabled ? \"enabled.\" : \"disabled.\"));\n"
"        getSensorData();\n"
"      } catch (error) {\n"
"        setChecked(config.switchId, !enabled);\n"
"        setText(\"conditionNotice\", \"Firmware still needs the /admin/check route before this switch can control \" + config.name + \".\");\n"
"      }\n"
"    }\n"
"\n"
"    function updateOverrideDisplay(pump, enabled) {\n"
"      const stateEl = document.getElementById(pump + \"OverrideState\");\n"
"      const onBtn = document.getElementById(pump + \"OverrideOnBtn\");\n"
"      const offBtn = document.getElementById(pump + \"OverrideOffBtn\");\n"
"      if (!stateEl || !onBtn || !offBtn) return;\n"
"\n"
"      stateEl.textContent = \"Override: \" + (enabled ? \"ON\" : \"OFF\");\n"
"      stateEl.className = enabled ? \"state-bad\" : \"state-warn\";\n"
"      onBtn.classList.toggle(\"active\", enabled);\n"
"      offBtn.classList.toggle(\"active\", !enabled);\n"
"    }\n"
"\n"
"    async function setPumpOverride(pump, enabled) {\n"
"      if (pump !== \"lp\") {\n"
"        setText(\"overrideNotice\", \"High-pressure pump override is disabled for protection.\");\n"
"        return;\n"
"      }\n"
"\n"
"      const confirmationText = enabled\n"
"        ? \"Enable the low-pressure pump maintenance override? This should only be used while someone is actively supervising the system.\"\n"
"        : \"Disable the low-pressure pump override and return to normal safety control?\";\n"
"\n"
"      if (!window.confirm(confirmationText)) return;\n"
"\n"
"      if (demoMode) {\n"
"        lastData.lpOverride = enabled;\n"
"        updatePage(lastData);\n"
"        setText(\"overrideNotice\", \"Low-pressure pump override is \" + (enabled ? \"enabled in demo mode.\" : \"disabled in demo mode.\"));\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/override\", {\n"
"          method: \"POST\",\n"
"          headers: { \"Content-Type\": \"application/json\" },\n"
"          body: JSON.stringify({ pump, enabled })\n"
"        });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        setText(\"overrideNotice\", \"Low-pressure pump override is now \" + (enabled ? \"enabled.\" : \"disabled.\"));\n"
"        getSensorData();\n"
"      } catch (error) {\n"
"        setText(\"overrideNotice\", \"Firmware still needs the /admin/override route before it can apply pump overrides.\");\n"
"      }\n"
"    }\n"
"\n"
"    function syncVoltageInputs(which, fromNumber = false) {\n"
"      const slider = document.getElementById(which + \"Slider\");\n"
"      const number = document.getElementById(which + \"Input\");\n"
"      if (fromNumber) slider.value = number.value;\n"
"      else number.value = slider.value;\n"
"\n"
"      const lvd = Number(document.getElementById(\"lvdInput\").value);\n"
"      const mvr = Number(document.getElementById(\"mvrInput\").value);\n"
"      if (Number.isFinite(lvd) && Number.isFinite(mvr)) {\n"
"        setText(\"voltageRuleText\", \"The system will shut down at \" + lvd.toFixed(1) + \" V and may restart at \" + mvr.toFixed(1) + \" V.\");\n"
"      }\n"
"    }\n"
"\n"
"    async function saveProtectionSettings() {\n"
"      const lvd = Number(document.getElementById(\"lvdInput\").value);\n"
"      const mvr = Number(document.getElementById(\"mvrInput\").value);\n"
"\n"
"      if (!Number.isFinite(lvd) || !Number.isFinite(mvr)) {\n"
"        setText(\"adminNotice\", \"Enter valid voltage values first.\");\n"
"        return;\n"
"      }\n"
"\n"
"      if (mvr <= lvd) {\n"
"        setText(\"adminNotice\", \"Restart voltage must be higher than the shutdown voltage.\");\n"
"        return;\n"
"      }\n"
"\n"
"      if (demoMode) {\n"
"        lastData.lvd = lvd;\n"
"        lastData.mvr = mvr;\n"
"        updatePage(lastData);\n"
"        setText(\"adminNotice\", \"Demo mode: voltage protection values saved in this browser.\");\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/settings\", {\n"
"          method: \"POST\",\n"
"          headers: { \"Content-Type\": \"application/json\" },\n"
"          body: JSON.stringify({ lvd, mvr })\n"
"        });\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        setText(\"adminNotice\", \"Voltage protection settings applied to F.R.O.G.S.\");\n"
"        getSensorData();\n"
"      } catch (error) {\n"
"        setText(\"adminNotice\", \"Firmware still needs the /admin/settings route before voltage settings can be saved.\");\n"
"      }\n"
"    }\n"
"\n"
"\n"
"    async function resetDefaults() {\n"
"      const confirmed = window.confirm(\"Reset Admin settings to firmware defaults? This should restore LVD, MVR, condition checks, pump enable settings, and LP override to their default values.\");\n"
"      if (!confirmed) return;\n"
"\n"
"      if (demoMode) {\n"
"        lastData.lvd = demoData.lvd;\n"
"        lastData.mvr = demoData.mvr;\n"
"        lastData.conditionChecks = { ...demoData.conditionChecks };\n"
"        lastData.daylightSafetyEnabled = demoData.daylightSafetyEnabled;\n"
"        lastData.lpOverride = false;\n"
"        lastData.pump1Enabled = true;\n"
"        lastData.pump2Enabled = true;\n"
"        updateAdminModeDisplay(false);\n"
"        updatePage(lastData);\n"
"        setText(\"resetNotice\", \"Demo mode: Admin defaults restored.\");\n"
"        return;\n"
"      }\n"
"\n"
"      try {\n"
"        const response = await fetch(\"/admin/reset\", {\n"
"          method: \"POST\",\n"
"          headers: { \"Content-Type\": \"application/json\" },\n"
"          body: JSON.stringify({ reset: true })\n"
"        });\n"
"\n"
"        if (!response.ok) throw new Error(\"ESP32 returned HTTP \" + response.status);\n"
"        setText(\"resetNotice\", \"Admin defaults restored.\");\n"
"        getSensorData();\n"
"        getAdminMode();\n"
"      } catch (error) {\n"
"        setText(\"resetNotice\", \"Firmware still needs the /admin/reset route before the webpage can reset the real default values.\");\n"
"      }\n"
"    }\n"
"\n"
"    document.querySelectorAll(\".tab-btn\").forEach(button => {\n"
"      button.addEventListener(\"click\", () => {\n"
"        document.querySelectorAll(\".page\").forEach(page => page.classList.remove(\"active\"));\n"
"        document.querySelectorAll(\".tab-btn\").forEach(tab => tab.classList.remove(\"active\"));\n"
"        document.getElementById(button.dataset.page).classList.add(\"active\");\n"
"        button.classList.add(\"active\");\n"
"        window.scrollTo({ top: 0, behavior: \"smooth\" });\n"
"      });\n"
"    });\n"
"\n"
"    setInterval(getSensorData, 3000);\n"
"    getSensorData();\n"
"  </script>\n"
"</body>\n"
"</html>\n";


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
        g_ldr_check   = (nvs_get_u8(nvs_handle, NVS_KEY_LDR_CHECK,   &chk) == ESP_OK) ? (bool)chk : true;
        g_float_check = (nvs_get_u8(nvs_handle, NVS_KEY_FLOAT_CHECK, &chk) == ESP_OK) ? (bool)chk : true;
        g_lvd_check   = (nvs_get_u8(nvs_handle, NVS_KEY_LVD_CHECK,   &chk) == ESP_OK) ? (bool)chk : true;
        g_lp_pump_enable   = (nvs_get_u8(nvs_handle, NVS_KEY_LP_PUMP_ENABLE,   &chk) == ESP_OK) ? (bool)chk : true;
        g_hp_pump_enable   = (nvs_get_u8(nvs_handle, NVS_KEY_HP_PUMP_ENABLE,   &chk) == ESP_OK) ? (bool)chk : true;
        ESP_LOGI(TAG_ADMIN, "Checks - LDR:%d Float:%d LVD:%d LPEN:%d HPEN:%d",
                 g_ldr_check, g_float_check, g_lvd_check, g_lp_pump_enable, g_hp_pump_enable);

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
        nvs_set_u8(nvs_handle, NVS_KEY_LP_PUMP_ENABLE,   (uint8_t)g_lp_pump_enable);
        nvs_set_u8(nvs_handle, NVS_KEY_HP_PUMP_ENABLE,   (uint8_t)g_hp_pump_enable);

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
            g_ldr_check       = g_temp_ldr_check;
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
    // Items: 0=LVD 1=MVR 2=LDR CHK 3=FLT CHK 4=LVD CHK 5= OVRD 6=RESET
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
            case 5: snprintf(line, sizeof(line), "LP OVRD: %s", g_temp_lp_pump_override ? "ON" : "OFF"); break;
            case 6: snprintf(line, sizeof(line), "LP EN:   %s", g_temp_lp_pump_enable    ? "ON" : "OFF"); break;
            case 7: snprintf(line, sizeof(line), "HP EN:   %s", g_temp_hp_pump_enable    ? "ON" : "OFF"); break;
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
            "LP OVRD: FORCE ON",
            "BYPASSES ALL CHKS",
            "LP EN: ALLOW LP",
            "HP EN: ALLOW HP",
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
    bool lp_pump = false;
    bool hp_pump = false;
    bool tank_full = false;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bv        = g_battery_voltage;
        daylight  = g_daylight_confirmed;
        lp_pump      = g_lp_pump_running;
        hp_pump     = g_hp_pump_running;
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

    snprintf(line, sizeof(line), "LP: %-3s HP: %-3s",
             lp_pump ? "ON" : "OFF", hp_pump ? "ON" : "OFF");
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
    } else if (bv < mvr && !lp_pump) {
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
                        g_temp_lp_pump_override = !g_temp_lp_pump_override;
                        ESP_LOGI(TAG_ADMIN, "LP override -> %d", g_temp_lp_pump_override);
                        break;
                    case 6:
                        g_temp_lp_pump_enable = !g_temp_lp_pump_enable;
                        ESP_LOGI(TAG_ADMIN, "LP enable -> %d", g_temp_lp_pump_enable);
                        break;
                    case 7:
                        g_temp_hp_pump_enable = !g_temp_hp_pump_enable;
                        ESP_LOGI(TAG_ADMIN, "HP enable -> %d", g_temp_hp_pump_enable);
                        break;
                    case 8:
                        g_temp_lvd             = DEFAULT_LVD;
                        g_temp_mvr             = DEFAULT_MVR;
                        g_temp_ldr_check       = true;
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

    int tds_buffer[TDS_ADC_SAMPLES] = {0};
    int tds_buffer_index = 0;
    int tds_sum = 0;

    ESP_LOGI(TAG_ADC, "adc_task started");
    ESP_LOGI(TAG_ADC, "Battery: GPIO34 (ADC1_CH6), LDR: GPIO35 (ADC1_CH7)");
    ESP_LOGI(TAG_ADC, "Battery voltage range: %.1fV - %.1fV", BAT_V_MIN, BAT_V_MAX);
    ESP_LOGI(TAG_ADC, "LDR threshold: %d (above = daylight)", LDR_THRESHOLD);
    ESP_LOGI(TAG_ADC, "Current Sense: GPIO32 (ADC1_CH4");

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

// Battery voltage divider: R1=47kΩ, R2=10kΩ (ratio = 57/10)
#define BAT_SCALE 5.7f
float adc_voltage = (avg_raw / 4095.0f) * 3.3f;
float battery_voltage = adc_voltage * BAT_SCALE;

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

        
        ret = adc_oneshot_read(adc_handle, TDS_ADC_CHANNEL, &tds_raw);

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
        

        // --- Current Sense ADC (IS 1+2, GPIO32) ---
        float current_amps = 0.0f;
        int is_raw = 0;
        bool current_valid = false;
        
        if (adc_oneshot_read(adc_handle, IS_1_2_CHANNEL, &is_raw) == ESP_OK)
        {
            // IS pin outputs ~1.2kA/A — with a 4.7kΩ sense resistor: V = I_sense * R
            // BTS700x kILIS = 22900 (typ), so I_load = (V_IS / R_sense) * kILIS
            float v_is = (is_raw / 4095.0f) * 3.3f;
            current_amps = (v_is/ 4730.0f) *22900.0f;
            current_valid = true; 
              ESP_LOGI(TAG_ADC, "Current = %.3f A (ADC=%d, V_IxS=%.3f V)", current_amps, is_raw, v_is);
        }  
        else {
            ESP_LOGE(TAG_ADC, "Current sense ADC read failed");
            }
        
        // Update other global variables
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
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
        bool lp_pump_override = false, lp_pump_enable = true, hp_pump_enable = true;
        bool web_admin_mode = false;
        if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ldr_check   = g_ldr_check;
            float_chk   = g_float_check;
            lvd_chk     = g_lvd_check;
            lp_pump_override = g_lp_pump_override;
            lp_pump_enable   = g_lp_pump_enable;
            hp_pump_enable   = g_hp_pump_enable;
            web_admin_mode   = g_web_admin_mode;
            xSemaphoreGive(voltage_mutex);
        }

        // --- Final pump 1 decision ---
        // Shutoff (_enable) takes priority over everything including override
        bool conditions_met = lp_pump_override
                           || ((lvd_chk   ? pump_enabled   : true)
                            && (ldr_check ? daylight_stable : true)
                            && (float_chk ? !tank_full       : true));
        bool active = !web_admin_mode
                   && lp_pump_enable
                   && conditions_met
                   && (g_display_mode != MODE_ADMIN_EDIT);

        gpio_set_level(LP_PUMP, active ? 1 : 0);

        // --- Pump 2: turns on after PUMP2_DELAY_MS of Pump 1 running ---
        static int64_t lp_pump_start_time = 0;
        bool hp_pump_active = false;

        if (active) {
            if (lp_pump_start_time == 0) {
                lp_pump_start_time = now;
                ESP_LOGI("PUMP2", "Pump 1 started, waiting %ds before Pump 2",
                         HP_PUMP_DELAY_MS / 1000);
            }
            if (hp_pump_enable && (now - lp_pump_start_time) >= HP_PUMP_DELAY_MS) {
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
            ESP_LOGI("PUMP", "v_ok=%d light=%d(%s) daylight=%d lpen=%d hpen=%d ovrd=%d admin=%d -> lp_pump=%d hp_pump=%d",
                     pump_enabled, ldr, light_is_bright ? "B" : "D",
                     daylight_stable, lp_pump_enable, hp_pump_enable, lp_pump_override,
                     web_admin_mode, active, hp_pump_active && hp_pump_enable);
            pump_log_counter = 0;
        }

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_lp_pump_running = active;
            g_hp_pump_running = hp_pump_active;
            xSemaphoreGive(state_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------- Flow Sensor Task ----------

static volatile uint32_t g_flow1_pulses = 0;
static volatile uint32_t g_flow2_pulses = 0;
static volatile int64_t g_flow1_last_us = 0;
static volatile int64_t g_flow2_last_us = 0;

// Reject edges closer together than this (noise/ringing/floating-pin glitches).
// 300us allows real pulses up to ~3.3kHz, far above any realistic flow rate
// for this system (FLOW_CAL = 98Hz per L/min -> ~34 L/min before this would clip).
#define FLOW_DEBOUNCE_US 300

static void IRAM_ATTR flow1_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - g_flow1_last_us >= FLOW_DEBOUNCE_US) {
        g_flow1_pulses++;
        g_flow1_last_us = now;
    }
}

static void IRAM_ATTR flow2_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - g_flow2_last_us >= FLOW_DEBOUNCE_US) {
        g_flow2_pulses++;
        g_flow2_last_us = now;
    }
}

void flow_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI("FLOW", "flow_task started");

    int64_t window_start = esp_timer_get_time() / 1000;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));

        int64_t now = esp_timer_get_time() / 1000;
        if ((now - window_start) >= FLOW_SAMPLE_MS) {
            uint32_t count1 = g_flow1_pulses;
            g_flow1_pulses -= count1;
            uint32_t count2 = g_flow2_pulses;
            g_flow2_pulses -= count2;

            float lpm1 = (count1 / FLOW_CAL);
            float lpm2 = (count2 / FLOW_CAL);

            int raw_float = gpio_get_level(FLOAT_SW_PIN);
            bool tank_full = (raw_float == 0);

            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_flow1_lpm = lpm1;
                g_flow2_lpm = lpm2;
                g_tank_full = tank_full;
                xSemaphoreGive(state_mutex);
            }

            ESP_LOGI("FLOW", "F1: %.2f L/min  F2: %.2f L/min  Float: %s",
                     lpm1, lpm2, tank_full ? "FULL" : "NOT FULL");

            window_start = now;
        }
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
    bool daylight = false;
    bool tank_full = false;
    int ldr_raw = 0;

    bool ldr_check = true;
    bool float_check = true;
    bool lvd_check = true;
    bool lp_override = false;
    float lvd = 0.0f;
    float mvr = 0.0f;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        battery = g_battery_voltage;
        temp_c = g_temp_c;
        tds = g_tds_ppm;
        flow1 = g_flow1_lpm;
        flow2 = g_flow2_lpm;
        lp_pump = g_lp_pump_running;
        hp_pump = g_hp_pump_running;
        temp_valid = g_temp_valid;
        tds_valid = g_tds_valid;
        daylight = g_daylight_confirmed;
        tank_full = g_tank_full;
        ldr_raw = g_ldr_raw;
        xSemaphoreGive(state_mutex);
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lp_enabled = g_lp_pump_enable;
        hp_enabled = g_hp_pump_enable;
        lvd = g_current_lvd;
        mvr = g_current_mvr;
        ldr_check = g_ldr_check;
        float_check = g_float_check;
        lvd_check = g_lvd_check;
        lp_override = g_lp_pump_override;
        xSemaphoreGive(voltage_mutex);
    }

    float temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;
    float total_flow = flow1 + flow2;

    const char *blocked;
    if (!daylight) {
        blocked = "NIGHT";
    } else if (battery < lvd) {
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
             "\"daylight\":%s,\"ldr\":%d,\"floatUp\":%s,\"blocked\":\"%s\","
             "\"lvd\":%.2f,\"mvr\":%.2f,\"daylightThreshold\":%d,"
             "\"conditionChecks\":{\"ldr\":%s,\"float\":%s,\"lvd\":%s},"
             "\"lpOverride\":%s}",
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
             daylight ? "true" : "false",
             ldr_raw,
             tank_full ? "true" : "false",
             blocked,
             lvd,
             mvr,
             LDR_THRESHOLD,
             ldr_check ? "true" : "false",
             float_check ? "true" : "false",
             lvd_check ? "true" : "false",
             lp_override ? "true" : "false");

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

static esp_err_t admin_daylight_handler(httpd_req_t *req)
{
    char body[WEB_REQUEST_BODY_MAX];
    bool enabled = false;
    float unused_threshold = 0.0f;

    if (read_request_body(req, body, sizeof(body)) != ESP_OK) {
        return send_json_error(req, "400 Bad Request", "Invalid request body");
    }

    if (json_get_float_value(body, "threshold", &unused_threshold)) {
        return send_json_error(req, "501 Not Implemented",
                               "LDR threshold remains fixed in the original firmware");
    }

    if (!json_get_bool_value(body, "enabled", &enabled)) {
        return send_json_error(req, "400 Bad Request", "Expected enabled value");
    }

    if (xSemaphoreTake(voltage_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return send_json_error(req, "503 Service Unavailable", "Settings are busy");
    }

    g_ldr_check = enabled;
    save_web_settings_locked();
    xSemaphoreGive(voltage_mutex);

    ESP_LOGI("WEB", "Web updated LDR check=%d", enabled);
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

    if (strcmp(check, "ldr") == 0) g_ldr_check = enabled;
    else if (strcmp(check, "float") == 0) g_float_check = enabled;
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
    g_ldr_check = true;
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
    config.max_uri_handlers = 12;

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
        { .uri = "/admin/daylight", .method = HTTP_POST, .handler = admin_daylight_handler, .user_ctx = NULL },
        { .uri = "/admin/check",    .method = HTTP_POST, .handler = admin_check_handler,    .user_ctx = NULL },
        { .uri = "/admin/override", .method = HTTP_POST, .handler = admin_override_handler, .user_ctx = NULL },
        { .uri = "/admin/reset",    .method = HTTP_POST, .handler = admin_reset_handler,    .user_ctx = NULL },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI("WEB", "Web server started with 12 URI handlers");
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_VRX_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_VRY_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TDS_ADC_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, IS_1_2_CHANNEL, &adc_chan_config));
    ESP_LOGI(TAG_MAIN, "ADC1 configured: battery(GPIO34), LDR(GPIO35), JoyX(GPIO36), JoyY(GPIO39), TDS(GPIO33), IS(GPIO32)");
    

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
    xTaskCreate(rgb_task, "rgb_task", 2048, NULL, 1, NULL);
    xTaskCreate(pump_task, "pump_task", 4096, NULL, 2, NULL);
    //xTaskCreate(ble_notify_task, "ble_notify_task", 4096, NULL, 2, NULL);
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
