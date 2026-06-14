#pragma once

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

// ---------- Onboard LED ----------
#define LED_PIN             GPIO_NUM_2

// ---------- RGB LED ----------
#define RGB_RED_PIN         GPIO_NUM_32
#define RGB_GREEN_PIN       GPIO_NUM_33
#define RGB_BLUE_PIN        GPIO_NUM_4

// ---------- Temperature Sensor (DS18B20, 1-Wire) ----------
#define TEMP_SENSOR_PIN     GPIO_NUM_13

// ---------- Pumps (BTS7002 high-side switches) ----------
#define FEED_PUMP_PIN       GPIO_NUM_14  // Pump 1 - U3
#define PUMP2_PIN           GPIO_NUM_16  // Pump 2 - U1
#define DEN_1_3_PIN         GPIO_NUM_18  // Diagnosis enable U1+U3

// ---------- Flow Sensors ----------
#define FLOW1_PIN           GPIO_NUM_17
#define FLOW2_PIN           GPIO_NUM_19

// ---------- Float Switch ----------
#define FLOAT_SW_PIN        GPIO_NUM_23  // NC switch: HIGH = open = tank full

// ---------- Joystick ----------
#define JOY_SW_PIN          GPIO_NUM_26
#define JOY_VRX_CHANNEL     ADC_CHANNEL_0   // GPIO36 (ADC1)
#define JOY_VRY_CHANNEL     ADC_CHANNEL_3   // GPIO39 (ADC1)

// ---------- ADC1 Channels ----------
#define BAT_ADC_CHANNEL     ADC_CHANNEL_6   // GPIO34 - battery voltage
#define LDR_ADC_CHANNEL     ADC_CHANNEL_7   // GPIO35 - light sensor

// ---------- ADC2 Channels ----------
#define TDS_ADC2_CHANNEL    ADC_CHANNEL_7   // GPIO27
#define IS_3_4_CHANNEL      ADC_CHANNEL_8   // GPIO25 - current sense U3+U4

// ---------- I2C (OLED) ----------
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_TIMEOUT_MS      1000

// Compatibility aliases for older pump naming used in main.c
#define LP_PUMP FEED_PUMP_PIN
#define HP_PUMP PUMP2_PIN

#define IS_1_2_CHANNEL IS_3_4_CHANNEL
