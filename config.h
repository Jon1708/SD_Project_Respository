#pragma once

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

// ---------- Onboard LED ----------
#define LED_PIN             GPIO_NUM_2

// ---------- RGB LED ----------
#define RGB_GREEN_PIN       GPIO_NUM_19
#define RGB_BLUE_PIN        GPIO_NUM_18

// ---------- Temperature Sensor (DS18B20, 1-Wire) ----------
#define TEMP_SENSOR_PIN     GPIO_NUM_21

// ---------- Pumps (BTS7002 high-side switches) ----------
#define LP_in               GPIO_NUM_17  // Pump 1 (low-pressure) - U3
#define PUMP2_PIN           GPIO_NUM_16  // Pump 2 - U1
#define DEN_1_3_PIN         GPIO_NUM_13  // Diagnosis enable U1+U3

// ---------- Flow Sensors ----------
#define FEED_FLOW_PIN       GPIO_NUM_27  // Feed flow (input)
#define PRODUCT_FLOW_PIN    GPIO_NUM_26  // Product flow (output)

// ---------- Float Switch ----------
#define FLOAT_SW_PIN        GPIO_NUM_14  // NC switch: HIGH = open = tank full

// ---------- Joystick ----------
#define JOY_SW_PIN          GPIO_NUM_22
#define JOY_VRX_CHANNEL     ADC_CHANNEL_0   // GPIO36 (ADC1)
#define JOY_VRY_CHANNEL     ADC_CHANNEL_3   // GPIO39 (ADC1)

// ---------- ADC1 Channels ----------
#define BAT_ADC_CHANNEL     ADC_CHANNEL_6   // GPIO34 - battery voltage
#define LDR_ADC_CHANNEL     ADC_CHANNEL_7   // GPIO35 - light sensor
#define IS_1_2_CHANNEL      ADC_CHANNEL_4   // GPIO32 - current sense U1+U2
#define TDS_ADC_CHANNEL     ADC_CHANNEL_5   // GPIO33 - TDS sensor

// ---------- I2C (OLED) ----------
#define I2C_MASTER_SDA_IO   GPIO_NUM_23
#define I2C_MASTER_SCL_IO   GPIO_NUM_15
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_TIMEOUT_MS      1000
