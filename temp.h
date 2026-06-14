#ifndef TEMP_H
#define TEMP_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t temp_init(gpio_num_t pin);
esp_err_t temp_read_celsius(float *temp_c);

#endif