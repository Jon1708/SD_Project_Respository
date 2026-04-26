#ifndef BLE_H
#define BLE_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize BLE (NimBLE stack + services + advertising)
 */
esp_err_t ble_init(void);

/**
 * Send a JSON payload over BLE notification
 * Returns ESP_OK if sent, error otherwise
 */
esp_err_t ble_send(const char *payload);

/**
 * Check if a BLE client is connected
 */
bool ble_is_connected(void);

/**
 * Optional: call periodically if needed (not required for NimBLE RTOS mode)
 */
void ble_task(void *param);

#ifdef __cplusplus
}
#endif

#endif // BLE_H