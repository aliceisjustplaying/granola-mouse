#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "proto_badge.h"
#include "proto_marker.h"
#include "qmi8658.h"

#ifdef GRANOLA_PROTO_AUDIO
#include "audio_stream.h"
#endif

#define PANEL_WIDTH 368
#define PANEL_HEIGHT 448
#define PANEL_GAP_X 0x10
#define STRIP_ROWS 16

#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_14
#define IO_EXPANDER_ADDRESS 0x20
#define IO_EXPANDER_OUTPUT_REGISTER 0x01
#define IO_EXPANDER_CONFIG_REGISTER 0x03
#define IO_EXPANDER_LCD_RESET (1U << 0U)
#define IO_EXPANDER_DISPLAY_POWER (1U << 1U)
#define IO_EXPANDER_TOUCH_RESET (1U << 2U)
#define IO_EXPANDER_SD_CHIP_SELECT (1U << 7U)
#define IO_EXPANDER_OUTPUTS                                                       \
    (IO_EXPANDER_LCD_RESET | IO_EXPANDER_DISPLAY_POWER |                         \
     IO_EXPANDER_TOUCH_RESET | IO_EXPANDER_SD_CHIP_SELECT)

#define QMI8658_RESET_REGISTER 0x60
#define QMI8658_RESET_COMMAND 0xB0
#define QMI8658_CTRL1_VALUE 0x60
#define GYRO_RANGE_DPS 2048
#define ACCEL_RANGE_G 8
#define IMU_ODR_HZ 250
#define IMU_PENDING_CAPACITY 32

static const co5300_lcd_init_cmd_t panel_init_commands[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x6F}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xBF}, 4, 0},
    {0x11, NULL, 0, 100},
    {0x29, NULL, 0, 0},
};

#define TELEMETRY_BAND_HEIGHT 56
#define TELEMETRY_BAR_HALF_WIDTH 180
#define TELEMETRY_CENTER_X 184
#define TELEMETRY_GYRO_SCALE_DPS 512.0f
#define TELEMETRY_ACCEL_SCALE_MG 2000.0f
#define TELEMETRY_CLIP_DPS 1843.0f
#define COLOR_GYRO 0x07E0   /* green */
#define COLOR_ACCEL 0x07FF  /* cyan */
#define COLOR_CLIP 0xF800   /* red */
#define COLOR_CENTER 0x39E7 /* dim gray */

/* Device badge MAC table — keep in sync with devices.conf */
static const uint8_t device_a_mac[6] = {0x1C, 0xDB, 0xD4, 0x7B, 0x7E, 0xE8};
static const uint8_t device_b_mac[6] = {0x1C, 0xDB, 0xD4, 0x7B, 0x85, 0xC8};

static DMA_ATTR uint16_t display_strip[PANEL_WIDTH * STRIP_ROWS];
static SemaphoreHandle_t display_transfer_done;
static esp_lcd_panel_handle_t display_panel;
static volatile float telemetry_values[6];
static qmi8658_dev_t imu;
static SemaphoreHandle_t serial_output_mutex;
static QueueHandle_t pending_imu;

typedef struct {
    int64_t timestamp_us;
    qmi8658_data_t data;
} pending_imu_sample_t;

static void check(esp_err_t error, const char *operation)
{
    if (error != ESP_OK) {
        printf("# FATAL %s: %s\n", operation, esp_err_to_name(error));
        abort();
    }
}

static i2c_master_bus_handle_t init_i2c(void)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    check(i2c_new_master_bus(&config, &bus), "create I2C bus");
    return bus;
}

static void reset_panel_power(i2c_master_bus_handle_t bus)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IO_EXPANDER_ADDRESS,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t device = NULL;
    check(i2c_master_bus_add_device(bus, &config, &device), "add IO expander");

    const uint8_t configure[] = {
        IO_EXPANDER_CONFIG_REGISTER, (uint8_t)~IO_EXPANDER_OUTPUTS};
    const uint8_t power_down[] = {
        IO_EXPANDER_OUTPUT_REGISTER, IO_EXPANDER_SD_CHIP_SELECT};
    const uint8_t power_up[] = {IO_EXPANDER_OUTPUT_REGISTER, IO_EXPANDER_OUTPUTS};

    check(i2c_master_transmit(device, configure, sizeof(configure), 100),
          "configure IO expander");
    check(i2c_master_transmit(device, power_down, sizeof(power_down), 100),
          "power down panel");
    vTaskDelay(pdMS_TO_TICKS(20));
    check(i2c_master_transmit(device, power_up, sizeof(power_up), 100),
          "power up panel");
    vTaskDelay(pdMS_TO_TICKS(150));
    check(i2c_master_bus_rm_device(device), "remove IO expander");
}

static bool on_display_transfer_done(esp_lcd_panel_io_handle_t io,
                                     esp_lcd_panel_io_event_data_t *event,
                                     void *context)
{
    (void)io;
    (void)event;
    (void)context;
    BaseType_t woke = pdFALSE;
    xSemaphoreGiveFromISR(display_transfer_done, &woke);
    return woke == pdTRUE;
}

static inline uint16_t byte_swap(uint16_t pixel)
{
    return (uint16_t)((pixel << 8) | (pixel >> 8));
}

static void draw_marker(esp_lcd_panel_handle_t panel)
{
    const int marker_x = (PANEL_WIDTH - PROTO_MARKER_WIDTH) / 2;
    const int marker_y = (PANEL_HEIGHT - PROTO_MARKER_HEIGHT) / 2;

    for (int strip_y = 0; strip_y < PANEL_HEIGHT; strip_y += STRIP_ROWS) {
        const int rows =
            strip_y + STRIP_ROWS <= PANEL_HEIGHT ? STRIP_ROWS : PANEL_HEIGHT - strip_y;
        for (int row = 0; row < rows; ++row) {
            const int panel_y = strip_y + row;
            for (int x = 0; x < PANEL_WIDTH; ++x) {
                uint16_t pixel = 0;
                if (x >= marker_x && x < marker_x + PROTO_MARKER_WIDTH &&
                    panel_y >= marker_y && panel_y < marker_y + PROTO_MARKER_HEIGHT) {
                    const int source_x = x - marker_x;
                    const int source_y = panel_y - marker_y;
                    pixel = proto_marker[source_y * PROTO_MARKER_WIDTH + source_x];
                }
                display_strip[row * PANEL_WIDTH + x] = byte_swap(pixel);
            }
        }
        check(esp_lcd_panel_draw_bitmap(panel, 0, strip_y, PANEL_WIDTH, strip_y + rows,
                                        display_strip),
              "draw marker strip");
        if (xSemaphoreTake(display_transfer_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
            printf("# FATAL display transfer timeout\n");
            abort();
        }
    }
}

static void push_strips(int y0, int height, const uint16_t *source, int source_width,
                        int x_offset)
{
    for (int strip_y = 0; strip_y < height; strip_y += STRIP_ROWS) {
        const int rows = strip_y + STRIP_ROWS <= height ? STRIP_ROWS : height - strip_y;
        for (int row = 0; row < rows; ++row) {
            for (int x = 0; x < PANEL_WIDTH; ++x) {
                uint16_t pixel = 0;
                const int sx = x - x_offset;
                if (sx >= 0 && sx < source_width) {
                    pixel = source[(strip_y + row) * source_width + sx];
                }
                display_strip[row * PANEL_WIDTH + x] = byte_swap(pixel);
            }
        }
        check(esp_lcd_panel_draw_bitmap(display_panel, 0, y0 + strip_y, PANEL_WIDTH,
                                        y0 + strip_y + rows, display_strip),
              "draw strip");
        if (xSemaphoreTake(display_transfer_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
            printf("# FATAL display transfer timeout\n");
            abort();
        }
    }
}

static void draw_badge(void)
{
    uint8_t mac[6] = {0};
    check(esp_read_mac(mac, ESP_MAC_WIFI_STA), "read MAC");
    const uint16_t *badge = badge_unknown;
    const char *label = "?";
    if (memcmp(mac, device_a_mac, 6) == 0) {
        badge = badge_a;
        label = "A";
    } else if (memcmp(mac, device_b_mac, 6) == 0) {
        badge = badge_b;
        label = "B";
    }
    push_strips(PANEL_HEIGHT - BADGE_HEIGHT, BADGE_HEIGHT, badge, BADGE_WIDTH, 0);
    printf("# device badge: %s (mac %02X:%02X:%02X:%02X:%02X:%02X)\n", label,
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void telemetry_task(void *context)
{
    (void)context;
    static uint16_t band[PANEL_WIDTH * STRIP_ROWS];
    for (;;) {
        float values[6];
        for (int i = 0; i < 6; ++i) {
            values[i] = telemetry_values[i];
        }
        for (int strip_y = 0; strip_y < TELEMETRY_BAND_HEIGHT; strip_y += STRIP_ROWS) {
            const int rows = strip_y + STRIP_ROWS <= TELEMETRY_BAND_HEIGHT
                                 ? STRIP_ROWS
                                 : TELEMETRY_BAND_HEIGHT - strip_y;
            for (int row = 0; row < rows; ++row) {
                const int y = strip_y + row;
                const int bar = (y - 2) / 9;
                const bool in_bar = y >= 2 && bar < 6 && ((y - 2) % 9) < 7;
                for (int x = 0; x < PANEL_WIDTH; ++x) {
                    uint16_t pixel = 0;
                    if (in_bar) {
                        const bool is_gyro = bar < 3;
                        const float value = values[bar];
                        const float scale = is_gyro ? TELEMETRY_GYRO_SCALE_DPS
                                                    : TELEMETRY_ACCEL_SCALE_MG;
                        const bool clipped =
                            is_gyro && (value >= TELEMETRY_CLIP_DPS ||
                                        value <= -TELEMETRY_CLIP_DPS);
                        float extent = value / scale;
                        if (extent > 1.0f) {
                            extent = 1.0f;
                        }
                        if (extent < -1.0f) {
                            extent = -1.0f;
                        }
                        const int length = (int)(extent * TELEMETRY_BAR_HALF_WIDTH);
                        const int lo = length < 0 ? TELEMETRY_CENTER_X + length
                                                  : TELEMETRY_CENTER_X;
                        const int hi = length < 0 ? TELEMETRY_CENTER_X
                                                  : TELEMETRY_CENTER_X + length;
                        if (x >= lo && x <= hi) {
                            pixel = clipped ? COLOR_CLIP
                                            : (is_gyro ? COLOR_GYRO : COLOR_ACCEL);
                        } else if (x >= TELEMETRY_CENTER_X - 1 &&
                                   x <= TELEMETRY_CENTER_X + 1) {
                            pixel = COLOR_CENTER;
                        }
                    }
                    band[row * PANEL_WIDTH + x] = byte_swap(pixel);
                }
            }
            check(esp_lcd_panel_draw_bitmap(display_panel, 0, strip_y, PANEL_WIDTH,
                                            strip_y + rows, band),
                  "draw telemetry strip");
            if (xSemaphoreTake(display_transfer_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
                printf("# FATAL telemetry transfer timeout\n");
                abort();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(66));
    }
}

static void init_display(i2c_master_bus_handle_t bus)
{
    reset_panel_power(bus);
    display_transfer_done = xSemaphoreCreateBinary();
    if (display_transfer_done == NULL) {
        printf("# FATAL create display semaphore\n");
        abort();
    }

    const spi_bus_config_t bus_config = {
        .sclk_io_num = GPIO_NUM_11,
        .data0_io_num = GPIO_NUM_4,
        .data1_io_num = GPIO_NUM_5,
        .data2_io_num = GPIO_NUM_6,
        .data3_io_num = GPIO_NUM_7,
        .max_transfer_sz = sizeof(display_strip),
    };
    check(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO), "create display SPI bus");

    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = GPIO_NUM_12,
        .dc_gpio_num = GPIO_NUM_NC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 1,
        .on_color_trans_done = on_display_transfer_done,
        .lcd_cmd_bits = 32,
        .lcd_param_bits = 8,
        .flags.quad_mode = true,
    };
    esp_lcd_panel_io_handle_t io = NULL;
    check(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io),
          "create panel IO");

    const co5300_vendor_config_t vendor_config = {
        .init_cmds = panel_init_commands,
        .init_cmds_size = sizeof(panel_init_commands) / sizeof(panel_init_commands[0]),
        .flags.use_qspi_interface = 1,
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
        .vendor_config = (void *)&vendor_config,
    };
    esp_lcd_panel_handle_t panel = NULL;
    check(esp_lcd_new_panel_co5300(io, &panel_config, &panel), "create CO5300 panel");
    display_panel = panel;
    check(esp_lcd_panel_reset(panel), "reset CO5300 panel");
    check(esp_lcd_panel_init(panel), "initialize CO5300 panel");
    check(esp_lcd_panel_set_gap(panel, PANEL_GAP_X, 0), "set CO5300 panel gap");
    check(esp_lcd_panel_disp_on_off(panel, true), "turn on CO5300 panel");
    draw_marker(panel);
    draw_badge();
    printf("# display ready: CO5300 368x448, marker centered\n");
}

static void init_imu(i2c_master_bus_handle_t bus)
{
    uint8_t address = 0;
    if (i2c_master_probe(bus, QMI8658_ADDRESS_HIGH, 100) == ESP_OK) {
        address = QMI8658_ADDRESS_HIGH;
    } else if (i2c_master_probe(bus, QMI8658_ADDRESS_LOW, 100) == ESP_OK) {
        address = QMI8658_ADDRESS_LOW;
    } else {
        printf("# FATAL QMI8658 not found at 0x6A or 0x6B\n");
        abort();
    }

    check(qmi8658_init(&imu, bus, address), "initialize QMI8658");
    check(qmi8658_write_register(&imu, QMI8658_RESET_REGISTER, QMI8658_RESET_COMMAND),
          "reset QMI8658");
    vTaskDelay(pdMS_TO_TICKS(20));
    check(qmi8658_write_register(&imu, QMI8658_CTRL1, QMI8658_CTRL1_VALUE),
          "configure QMI8658 CTRL1");
    check(qmi8658_set_accel_range(&imu, QMI8658_ACCEL_RANGE_8G),
          "set QMI8658 accel range");
    check(qmi8658_set_accel_odr(&imu, QMI8658_ACCEL_ODR_250HZ),
          "set QMI8658 accel ODR");
    check(qmi8658_set_gyro_range(&imu, QMI8658_GYRO_RANGE_2048DPS),
          "set QMI8658 gyro range");
    check(qmi8658_set_gyro_odr(&imu, QMI8658_GYRO_ODR_250HZ),
          "set QMI8658 gyro ODR");
    qmi8658_set_accel_unit_mg(&imu, true);
    qmi8658_set_gyro_unit_dps(&imu, true);
    check(qmi8658_enable_sensors(&imu, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO),
          "enable QMI8658");
    printf("# IMU ready: QMI8658 at 0x%02X\n", address);
}

static void print_config(void)
{
    xSemaphoreTake(serial_output_mutex, portMAX_DELAY);
    printf("CFG,gyro_range_dps=%d,accel_range_g=%d,odr_hz=%d\n",
           GYRO_RANGE_DPS, ACCEL_RANGE_G, IMU_ODR_HZ);
    xSemaphoreGive(serial_output_mutex);
}

static void imu_sample_task(void *context)
{
    (void)context;
    for (;;) {
        bool ready = false;
        if (qmi8658_is_data_ready(&imu, &ready) == ESP_OK && ready) {
            pending_imu_sample_t sample;
            if (qmi8658_read_sensor_data(&imu, &sample.data) == ESP_OK) {
                sample.timestamp_us = esp_timer_get_time();
                telemetry_values[0] = sample.data.gyroX;
                telemetry_values[1] = sample.data.gyroY;
                telemetry_values[2] = sample.data.gyroZ;
                telemetry_values[3] = sample.data.accelX;
                telemetry_values[4] = sample.data.accelY;
                telemetry_values[5] = sample.data.accelZ;
                if (xQueueSend(pending_imu, &sample, 0) != pdTRUE) {
                    pending_imu_sample_t discarded;
                    (void)xQueueReceive(pending_imu, &discarded, 0);
                    (void)xQueueSend(pending_imu, &sample, 0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    esp_log_level_set("*", ESP_LOG_ERROR);
    serial_output_mutex = xSemaphoreCreateMutex();
    if (serial_output_mutex == NULL) {
        printf("# FATAL create serial output mutex\n");
        abort();
    }

    i2c_master_bus_handle_t bus = init_i2c();
    init_display(bus);
    init_imu(bus);
    print_config();

    pending_imu = xQueueCreate(IMU_PENDING_CAPACITY, sizeof(pending_imu_sample_t));
    if (pending_imu == NULL ||
        xTaskCreate(imu_sample_task, "imu_sample", 4096, NULL, 2, NULL) != pdPASS) {
        printf("# FATAL start IMU sampling task\n");
        abort();
    }
    if (xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 1, NULL) != pdPASS) {
        printf("# FATAL start telemetry task\n");
        abort();
    }
#ifdef GRANOLA_PROTO_AUDIO
    check(audio_stream_start(bus, serial_output_mutex), "start audio stream");
#endif

    int64_t last_config_us = esp_timer_get_time();
    for (;;) {
        pending_imu_sample_t sample;
        if (xQueuePeek(pending_imu, &sample, 0) == pdTRUE &&
            xSemaphoreTake(serial_output_mutex, 0) == pdTRUE) {
            if (xQueueReceive(pending_imu, &sample, 0) == pdTRUE) {
                const qmi8658_data_t *data = &sample.data;
                printf("IMU,%" PRId64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                       sample.timestamp_us, data->gyroX, data->gyroY, data->gyroZ,
                       data->accelX / 1000.0f, data->accelY / 1000.0f,
                       data->accelZ / 1000.0f);
            }
            xSemaphoreGive(serial_output_mutex);
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_config_us >= 5000000) {
            print_config();
            last_config_us = now_us;
        }
        if (uxQueueMessagesWaiting(pending_imu) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            taskYIELD();
        }
    }
}
