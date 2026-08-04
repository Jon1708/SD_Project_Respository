#pragma once

#include <stddef.h>
#include <stdint.h>

// Voice clips played back through the LM386 module (fed from the ESP32's
// built-in DAC on GPIO25 - see AUDIO_DAC_PIN in pinout.h). Each clip is an
// 8-bit unsigned PCM buffer at AUDIO_SAMPLE_RATE_HZ (config.h), generated
// from a WAV file by tools/generate_audio_clips.py.
typedef enum {
    AUDIO_CLIP_AUTONOMOUS_ON,
    AUDIO_CLIP_AUTONOMOUS_OFF,
    AUDIO_CLIP_COUNTDOWN_60S,
    AUDIO_CLIP_COUNTDOWN_30S,
    AUDIO_CLIP_COUNT_5,
    AUDIO_CLIP_COUNT_4,
    AUDIO_CLIP_COUNT_3,
    AUDIO_CLIP_COUNT_2,
    AUDIO_CLIP_COUNT_1,
    AUDIO_CLIP_FLOAT_CHECK_FAILED,
    AUDIO_CLIP_VOLTAGE_CHECK_FAILED,
    AUDIO_CLIP_COUNT   // sentinel - number of clips
} audio_clip_id_t;

typedef struct {
    const uint8_t *data;
    size_t len;
} audio_clip_t;

// Defined in audio_clips.c (regenerate with tools/generate_audio_clips.py
// once real WAV recordings are available; ships with silent placeholders).
extern const audio_clip_t g_audio_clips[AUDIO_CLIP_COUNT];

// Sets up the DAC and starts the playback task. Call once from app_main().
void audio_init(void);

// Queues a clip for playback. Non-blocking: returns immediately, plays back
// on a dedicated task. If the queue is full the request is dropped (logged)
// rather than blocking the caller.
void audio_play(audio_clip_id_t clip);
