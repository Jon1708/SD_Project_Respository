#pragma once

// ---------- OLED ----------
#define OLED_I2C_ADDR       0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

// ---------- ADC ----------
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_SAMPLES         16
#define LDR_ADC_SAMPLES     16
#define TDS_ADC_SAMPLES     16

// ---------- Battery Voltage ----------
#define BAT_CAL             1.065f   // Calibration multiplier for voltage divider
#define BAT_V_MIN           9.0f
#define BAT_V_MAX           15.0f

// ---------- LVD / MVR (LiFePO4) ----------
#define DEFAULT_LVD         12.0f
#define DEFAULT_MVR         12.8f
#define LVD_MIN             10.0f
#define LVD_MAX             13.0f
#define MVR_MIN             11.0f
#define MVR_MAX             14.0f
#define MIN_HYSTERESIS      0.5f
#define VOLTAGE_STEP        0.1f

// ---------- Light Sensor ----------
#define LDR_THRESHOLD       1500     // ADC reading above this = daylight
#define LIGHT_ON_DELAY_MS   10000    // Sustained daylight before pump allowed
#define LIGHT_OFF_DELAY_MS  15000    // Sustained darkness before pump blocked

// ---------- Flow Sensors ----------
#define FLOW_CAL            98.0f    // F = 98 * Q (Hz per L/min)
#define FLOW_SAMPLE_MS      1000     // Pulse counting window

// ---------- Pumps ----------
#define PUMP2_DELAY_MS      10000    // Delay after pump 1 start before pump 2 turns on

// ---------- Joystick ----------
#define JOY_CENTER          2048
#define JOY_DEADZONE        500
#define JOY_INITIAL_DELAY_MS  400
#define JOY_REPEAT_RATE_MS    200

// ---------- RGB LED PWM ----------
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_RED_CHANNEL    LEDC_CHANNEL_0
#define LEDC_GREEN_CHANNEL  LEDC_CHANNEL_1
#define LEDC_BLUE_CHANNEL   LEDC_CHANNEL_2
#define LEDC_DUTY_RES       LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY      5000

// ---------- Admin / UI ----------
#define ADMIN_TIMEOUT_MS    60000
#define DEBOUNCE_DELAY_MS   50

// ---------- NVS ----------
#define NVS_NAMESPACE       "voltage"
#define NVS_KEY_LVD         "lvd"
#define NVS_KEY_MVR         "mvr"
#define NVS_KEY_LDR_CHECK   "ldr_chk"
#define NVS_KEY_FLOAT_CHECK "flt_chk"
#define NVS_KEY_LVD_CHECK   "lvd_chk"
#define NVS_KEY_P1_ENABLE   "p1_en"
#define NVS_KEY_P2_ENABLE   "p2_en"
