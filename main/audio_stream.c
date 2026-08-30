/*
 * Audio stream framing (device -> host, USB Serial/JTAG):
 *
 *   AUD,<t_us>,<n_bytes>\n<exactly n_bytes of PCM>
 *
 * t_us is the esp_timer timestamp taken when the complete chunk becomes
 * available. PCM is mono, 48000 Hz, signed 16-bit little-endian. Each normal
 * chunk contains 960 samples / 1920 bytes (20 ms). There is no delimiter or
 * trailing newline after the binary payload: the receiver must consume exactly
 * n_bytes before parsing the next AUD, IMU, CFG, or #-prefixed text line.
 * A shared serial mutex keeps each AUD header plus payload contiguous while
 * allowing complete IMU and CFG lines between audio chunks.
 */

#include "audio_stream.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AUDIO_SAMPLE_RATE_HZ 48000
#define AUDIO_BITS_PER_SAMPLE 16
#define AUDIO_CHANNEL_COUNT 1
#define AUDIO_CHUNK_MS 20
#define AUDIO_CHUNK_SAMPLES ((AUDIO_SAMPLE_RATE_HZ * AUDIO_CHUNK_MS) / 1000)
#define AUDIO_CHUNK_BYTES (AUDIO_CHUNK_SAMPLES * sizeof(int16_t))
#define AUDIO_SERIAL_WRITE_BYTES 128
#define AUDIO_INPUT_GAIN_DB 0.0f

/* Waveshare ESP32-S3-Touch-AMOLED-1.8 BSP v2.0.3 board definitions. */
#define AUDIO_I2S_BCLK GPIO_NUM_9
#define AUDIO_I2S_MCLK GPIO_NUM_16
#define AUDIO_I2S_WS GPIO_NUM_45
#define AUDIO_I2S_DOUT GPIO_NUM_8
#define AUDIO_I2S_DIN GPIO_NUM_10
#define AUDIO_I2S_PORT I2S_NUM_1
#define AUDIO_PA_ENABLE GPIO_NUM_46

static esp_codec_dev_handle_t microphone;
static SemaphoreHandle_t output_mutex;

static void print_audio_error(const char *operation, int error)
{
    xSemaphoreTake(output_mutex, portMAX_DELAY);
    printf("# audio %s failed: %d\n", operation, error);
    xSemaphoreGive(output_mutex);
}

static void audio_stream_task(void *context)
{
    (void)context;
    int16_t pcm[AUDIO_CHUNK_SAMPLES];

    for (;;) {
        const int result = esp_codec_dev_read(microphone, pcm, sizeof(pcm));
        if (result != ESP_CODEC_DEV_OK) {
            print_audio_error("read", result);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        const int64_t timestamp_us = esp_timer_get_time();
        xSemaphoreTake(output_mutex, portMAX_DELAY);
        printf("AUD,%" PRId64 ",%u\n", timestamp_us, (unsigned)sizeof(pcm));
        size_t written = 0;
        while (written < sizeof(pcm)) {
            const size_t remaining = sizeof(pcm) - written;
            const size_t block = remaining < AUDIO_SERIAL_WRITE_BYTES
                                     ? remaining
                                     : AUDIO_SERIAL_WRITE_BYTES;
            const size_t result =
                fwrite((const uint8_t *)pcm + written, 1, block, stdout);
            written += result;
            if (result != block) {
                break;
            }
            taskYIELD();
        }
        xSemaphoreGive(output_mutex);

        if (written != sizeof(pcm)) {
            print_audio_error("serial write", (int)written);
        }
    }
}

esp_err_t audio_stream_start(i2c_master_bus_handle_t i2c_bus,
                             SemaphoreHandle_t serial_mutex)
{
    if (i2c_bus == NULL || serial_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    output_mutex = serial_mutex;

    i2s_chan_handle_t tx_channel = NULL;
    i2s_chan_handle_t rx_channel = NULL;
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(AUDIO_I2S_PORT, I2S_ROLE_MASTER);
    channel_config.auto_clear = true;
    esp_err_t error = i2s_new_channel(&channel_config, &tx_channel, &rx_channel);
    if (error != ESP_OK) {
        return error;
    }

    const i2s_std_config_t i2s_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = AUDIO_I2S_MCLK,
            .bclk = AUDIO_I2S_BCLK,
            .ws = AUDIO_I2S_WS,
            .dout = AUDIO_I2S_DOUT,
            .din = AUDIO_I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    error = i2s_channel_init_std_mode(tx_channel, &i2s_config);
    if (error != ESP_OK) {
        return error;
    }
    error = i2s_channel_init_std_mode(rx_channel, &i2s_config);
    if (error != ESP_OK) {
        return error;
    }

    audio_codec_i2s_cfg_t codec_i2s_config = {
        .port = AUDIO_I2S_PORT,
        .tx_handle = tx_channel,
        .rx_handle = rx_channel,
    };
    const audio_codec_data_if_t *data_interface =
        audio_codec_new_i2s_data(&codec_i2s_config);
    if (data_interface == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const audio_codec_gpio_if_t *gpio_interface = audio_codec_new_gpio();
    if (gpio_interface == NULL) {
        return ESP_ERR_NO_MEM;
    }
    audio_codec_i2c_cfg_t codec_i2c_config = {
        .port = I2C_NUM_0,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t *control_interface =
        audio_codec_new_i2c_ctrl(&codec_i2c_config);
    if (control_interface == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_codec_dev_hw_gain_t hardware_gain = {
        .pa_voltage = 5.0f,
        .codec_dac_voltage = 3.3f,
    };
    es8311_codec_cfg_t es8311_config = {
        .ctrl_if = control_interface,
        .gpio_if = gpio_interface,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = AUDIO_PA_ENABLE,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = hardware_gain,
    };
    const audio_codec_if_t *codec_interface = es8311_codec_new(&es8311_config);
    if (codec_interface == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    esp_codec_dev_cfg_t microphone_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = codec_interface,
        .data_if = data_interface,
    };
    microphone = esp_codec_dev_new(&microphone_config);
    if (microphone == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_codec_dev_sample_info_t sample_info = {
        .bits_per_sample = AUDIO_BITS_PER_SAMPLE,
        .channel = AUDIO_CHANNEL_COUNT,
        .channel_mask = 0,
        .sample_rate = AUDIO_SAMPLE_RATE_HZ,
        .mclk_multiple = 256,
    };
    int result = esp_codec_dev_open(microphone, &sample_info);
    if (result != ESP_CODEC_DEV_OK) {
        return ESP_FAIL;
    }
    result = esp_codec_dev_set_in_gain(microphone, AUDIO_INPUT_GAIN_DB);
    if (result != ESP_CODEC_DEV_OK) {
        return ESP_FAIL;
    }

    if (xTaskCreate(audio_stream_task, "audio_stream", 6144, NULL, 1, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(output_mutex, portMAX_DELAY);
    printf("# audio ready: ES8311 mic, 48000 Hz mono s16le, 20 ms chunks\n");
    xSemaphoreGive(output_mutex);
    return ESP_OK;
}
