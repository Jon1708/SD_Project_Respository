#pragma once

// ---------- OLED ----------
#define OLED_I2C_ADDR       0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

// ---------- ADC ----------
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_SAMPLES         16
#define TDS_ADC_SAMPLES     16

// ---------- Battery Voltage ----------
#define BAT_CAL             1.065f   // Calibration multiplier for voltage divider
#define BAT_V_MIN           9.0f
#define BAT_V_MAX           15.0f

// ---------- LVD / MVR (LiFePO4) ----------
#define DEFAULT_LVD         12.0f
#define DEFAULT_MVR         12.8f
#define LVD_MIN             10.5f
#define LVD_MAX             14.6f
#define MVR_MIN             10.5f
#define MVR_MAX             14.6f

#define MIN_HYSTERESIS      0.5f
#define VOLTAGE_STEP        0.1f

// ---------- Flow Sensors ----------
#define FLOW_CAL            80.0f   // F = 98 * Q (Hz per L/min)
#define FLOW_SAMPLE_MS      1000     // Pulse counting window
#define LITERS_TO_US_GALLONS 0.264172f

// ---------- Pumps ----------
// HP (pump 2) requires both: LP running for PUMP2_DELAY_MS AND feed flow > HP_MIN_FLOW_GPM
#define PUMP2_DELAY_MS      60000    // Delay after pump 1 start before pump 2 turns on
#define HP_MIN_FLOW_GPM     0.0f     // Minimum feed flow (flow1) required before pump 2 turns on

// ---------- Joystick ----------
#define JOY_CENTER          2048
#define JOY_DEADZONE        500
#define JOY_INITIAL_DELAY_MS  400
#define JOY_REPEAT_RATE_MS    200

// ---------- Voice Audio (I2S built-in DAC on GPIO25) ----------
#define AUDIO_SAMPLE_RATE_HZ 8000    // Match the sample rate the WAV clips are exported at
#define AUDIO_DMA_BUF_COUNT  4
#define AUDIO_DMA_BUF_LEN    512     // Samples per DMA buffer

// ---------- Admin / UI ----------
#define ADMIN_TIMEOUT_MS    60000
#define DEBOUNCE_DELAY_MS   50

// ---------- NVS ----------
#define NVS_NAMESPACE       "voltage"
#define NVS_KEY_LVD         "lvd"
#define NVS_KEY_MVR         "mvr"
#define NVS_KEY_FLOAT_CHECK "flt_chk"
#define NVS_KEY_LVD_CHECK   "lvd_chk"
#define NVS_KEY_P1_ENABLE   "p1_en"
#define NVS_KEY_P2_ENABLE   "p2_en"
#define NVS_KEY_AUTO_MODE   "auto_mode"
