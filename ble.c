// ble.c
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "ble.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE";

#define BLE_DEVICE_NAME "FROGS-System"

/*
 * Custom 128-bit UUIDs.
 * These can be any valid UUIDs as long as Flutter uses the same ones.
 */
static const ble_uuid128_t FROGS_SERVICE_UUID =
    BLE_UUID128_INIT(0x12, 0x34, 0x56, 0x78,
                     0x9A, 0xBC,
                     0xDE, 0xF0,
                     0x12, 0x34,
                     0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0);

static const ble_uuid128_t SENSOR_CHAR_UUID =
    BLE_UUID128_INIT(0xAB, 0xCD, 0xEF, 0x01,
                     0x23, 0x45,
                     0x67, 0x89,
                     0xAB, 0xCD,
                     0xEF, 0x01, 0x23, 0x45, 0x67, 0x89);

static uint16_t sensor_char_handle;
static uint16_t conn_handle;
static bool connected = false;

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static void ble_advertise(void);

static int sensor_access_cb(uint16_t conn_handle,
                            uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg)
{
    const char *msg = "{\"status\":\"ready\"}";

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            os_mbuf_append(ctxt->om, msg, strlen(msg));
            return 0;

        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &FROGS_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &SENSOR_CHAR_UUID.u,
                .access_cb = sensor_access_cb,
                .val_handle = &sensor_char_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        },
    },
    {0}
};

static void ble_advertise(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;

    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = BLE_DEVICE_NAME;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC,
                           NULL,
                           BLE_HS_FOREVER,
                           &adv_params,
                           ble_gap_event_cb,
                           NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as %s", BLE_DEVICE_NAME);
    }
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                connected = true;
                conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Client connected");
            } else {
                ESP_LOGW(TAG, "Connection failed; restarting advertising");
                ble_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            connected = false;
            ESP_LOGI(TAG, "Client disconnected");
            ble_advertise();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG,
                     "Subscribe event: attr_handle=%d notify=%d indicate=%d",
                     event->subscribe.attr_handle,
                     event->subscribe.cur_notify,
                     event->subscribe.cur_indicate);
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete; restarting");
            ble_advertise();
            return 0;

        default:
            return 0;
    }
}

static void ble_on_sync(void)
{
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer BLE address: %d", rc);
        return;
    }

    ble_advertise();
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE");

    int rc;

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed setting device name: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_count_cfg(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed counting GATT services: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed adding GATT services: %d", rc);
        return ESP_FAIL;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE initialized");
    return ESP_OK;
}

esp_err_t ble_send(const char *payload)
{
    if (payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!connected) {
        return ESP_ERR_INVALID_STATE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, strlen(payload));
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate BLE packet");
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gatts_notify_custom(conn_handle, sensor_char_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Notify failed: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BLE sent: %s", payload);
    return ESP_OK;
}

bool ble_is_connected(void)
{
    return connected;
}

void ble_task(void *param)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}