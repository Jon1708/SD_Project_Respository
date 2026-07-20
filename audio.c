#include "audio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/dac_continuous.h"
#include "esp_log.h"

#include "pinout.h"
#include "config.h"

static const char *TAG_AUDIO = "AUDIO";

static dac_continuous_handle_t s_dac_handle = NULL;
static QueueHandle_t s_clip_queue = NULL;

static void audio_task(void *pvParameters)
{
    (void)pvParameters;

    audio_clip_id_t clip;
    while (1) {
        if (xQueueReceive(s_clip_queue, &clip, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (clip >= AUDIO_CLIP_COUNT) {
            continue;
        }

        const audio_clip_t *c = &g_audio_clips[clip];
        if (c->data == NULL || c->len == 0) {
            continue;
        }

        esp_err_t err = dac_continuous_write(s_dac_handle, (uint8_t *)c->data, c->len, NULL, -1);
        if (err != ESP_OK) {
            ESP_LOGW(TAG_AUDIO, "Playback failed for clip %d: %s", clip, esp_err_to_name(err));
        }
    }
}

void audio_init(void)
{
    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,   // GPIO25 (AUDIO_DAC_PIN) only
        .desc_num  = AUDIO_DMA_BUF_COUNT,
        .buf_size  = AUDIO_DMA_BUF_LEN,
        .freq_hz   = AUDIO_SAMPLE_RATE_HZ,
        .offset    = 0,
        // APLL required: the default DAC digital clock only goes down to
        // ~19.6kHz on ESP32, which is above our voice clip sample rate.
        .clk_src   = DAC_DIGI_CLK_SRC_APLL,
        .chan_mode = DAC_CHANNEL_MODE_SIMUL,
    };

    esp_err_t err = dac_continuous_new_channels(&cont_cfg, &s_dac_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AUDIO, "Failed to init DAC: %s", esp_err_to_name(err));
        return;
    }
    ESP_ERROR_CHECK(dac_continuous_enable(s_dac_handle));

    s_clip_queue = xQueueCreate(8, sizeof(audio_clip_id_t));

    xTaskCreate(audio_task, "audio_task", 3072, NULL, 1, NULL);

    ESP_LOGI(TAG_AUDIO, "Audio initialized on GPIO%d (DAC1), %d Hz", AUDIO_DAC_PIN, AUDIO_SAMPLE_RATE_HZ);
}

void audio_play(audio_clip_id_t clip)
{
    if (s_clip_queue == NULL || clip >= AUDIO_CLIP_COUNT) {
        return;
    }
    if (xQueueSend(s_clip_queue, &clip, 0) != pdTRUE) {
        ESP_LOGW(TAG_AUDIO, "Clip queue full, dropped clip %d", clip);
    }
}
