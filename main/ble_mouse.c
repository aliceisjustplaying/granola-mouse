#include "ble_mouse.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "esp_hid_common.h"
#include "esp_hidd.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_store.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"

/* Air-mouse tuning. Axis signs are initial guesses and intentionally easy to flip. */
#define BLE_MOUSE_X_SIGN (-1.0f) /* dx from gyro Z */
#define BLE_MOUSE_Y_SIGN (-1.0f) /* dy from gyro X */
#define BLE_MOUSE_X_GAIN 0.08f
#define BLE_MOUSE_Y_GAIN 0.08f
#define BLE_MOUSE_DEADZONE_DPS 2.0f
#define BLE_MOUSE_REPORT_PERIOD_MS 12
#define BLE_MOUSE_BUTTON_GPIO GPIO_NUM_0
#define BLE_MOUSE_DEVICE_NAME "Granola Mouse"

#define HID_SERVICE_UUID 0x1812
#define HID_MOUSE_APPEARANCE 0x03C2

static const uint8_t mouse_report_map[] = {
    0x05, 0x01, /* Usage Page (Generic Desktop) */
    0x09, 0x02, /* Usage (Mouse) */
    0xA1, 0x01, /* Collection (Application) */
    0x09, 0x01, /*   Usage (Pointer) */
    0xA1, 0x00, /*   Collection (Physical) */
    0x05, 0x09, /*     Usage Page (Button) */
    0x19, 0x01, /*     Usage Minimum (Button 1) */
    0x29, 0x03, /*     Usage Maximum (Button 3) */
    0x15, 0x00, /*     Logical Minimum (0) */
    0x25, 0x01, /*     Logical Maximum (1) */
    0x95, 0x03, /*     Report Count (3) */
    0x75, 0x01, /*     Report Size (1) */
    0x81, 0x02, /*     Input (Data, Variable, Absolute) */
    0x95, 0x01, /*     Report Count (1) */
    0x75, 0x05, /*     Report Size (5) */
    0x81, 0x03, /*     Input (Constant) */
    0x05, 0x01, /*     Usage Page (Generic Desktop) */
    0x09, 0x30, /*     Usage (X) */
    0x09, 0x31, /*     Usage (Y) */
    0x09, 0x38, /*     Usage (Wheel) */
    0x15, 0x81, /*     Logical Minimum (-127) */
    0x25, 0x7F, /*     Logical Maximum (127) */
    0x75, 0x08, /*     Report Size (8) */
    0x95, 0x03, /*     Report Count (3) */
    0x81, 0x06, /*     Input (Data, Variable, Relative) */
    0xC0,       /*   End Collection */
    0xC0,       /* End Collection */
};

static esp_hid_raw_report_map_t report_maps[] = {{
    .data = mouse_report_map,
    .len = sizeof(mouse_report_map),
}};

static const esp_hid_device_config_t hid_config = {
    .vendor_id = 0x303A,
    .product_id = 0x0001,
    .version = 0x0100,
    .device_name = BLE_MOUSE_DEVICE_NAME,
    .manufacturer_name = "Granola",
    .serial_number = "Granola-Wand",
    .report_maps = report_maps,
    .report_maps_len = 1,
};

void ble_store_config_init(void);

static esp_hidd_dev_t *hid_device;
static SemaphoreHandle_t output_mutex;
static volatile bool connected;
static volatile float latest_gyro_x;
static volatile float latest_gyro_z;
static uint8_t own_address_type;
static ble_uuid16_t hid_service_uuid = BLE_UUID16_INIT(HID_SERVICE_UUID);

static void state_print(const char *message)
{
    if (output_mutex != NULL) {
        xSemaphoreTake(output_mutex, portMAX_DELAY);
    }
    printf("%s\n", message);
    if (output_mutex != NULL) {
        xSemaphoreGive(output_mutex);
    }
}

static void state_printf(const char *format, ...)
{
    if (output_mutex != NULL) {
        xSemaphoreTake(output_mutex, portMAX_DELAY);
    }
    va_list arguments;
    va_start(arguments, format);
    vprintf(format, arguments);
    va_end(arguments);
    putchar('\n');
    if (output_mutex != NULL) {
        xSemaphoreGive(output_mutex);
    }
}

static void start_advertising(void);

static int gap_event(struct ble_gap_event *event, void *context)
{
    (void)context;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            connected = false;
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        connected = false;
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (!connected) {
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == BLE_HS_ENOTCONN) {
            state_printf("# BLE encryption change status=%d (BLE_HS_ENOTCONN: not connected)",
                         event->enc_change.status);
        } else {
            state_printf("# BLE encryption change status=%d", event->enc_change.status);
        }
        if (event->enc_change.status == 0) {
            struct ble_gap_conn_desc description;
            if (ble_gap_conn_find(event->enc_change.conn_handle, &description) == 0 &&
                description.sec_state.bonded) {
                state_print("# BLE bonded");
            }
        }
        break;
    case BLE_GAP_EVENT_PARING_COMPLETE: {
        struct ble_gap_conn_desc description;
        const bool bonded =
            ble_gap_conn_find(event->pairing_complete.conn_handle, &description) == 0 &&
            description.sec_state.bonded;
        state_printf("# BLE auth complete status=%d bonded=%d",
                     event->pairing_complete.status, bonded);
        break;
    }
    case BLE_GAP_EVENT_SUBSCRIBE:
        state_printf("# BLE subscribe handle=%d", event->subscribe.attr_handle);
        break;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        state_print("# BLE repeat pairing");
        struct ble_gap_conn_desc description;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &description) == 0) {
            const int rc = ble_store_util_delete_peer(&description.peer_id_addr);
            if (rc != 0) {
                state_printf("# BLE bond delete failed status=%d", rc);
            }
        } else {
            state_print("# BLE bond lookup failed");
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    if (ble_gap_adv_active()) {
        return;
    }

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)BLE_MOUSE_DEVICE_NAME;
    fields.name_len = strlen(BLE_MOUSE_DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.appearance = HID_MOUSE_APPEARANCE;
    fields.appearance_is_present = 1;
    fields.uuids16 = &hid_service_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc == 0) {
        struct ble_gap_adv_params parameters = {0};
        parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
        parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
        parameters.itvl_min = BLE_GAP_ADV_ITVL_MS(30);
        parameters.itvl_max = BLE_GAP_ADV_ITVL_MS(50);
        rc = ble_gap_adv_start(own_address_type, NULL, BLE_HS_FOREVER, &parameters,
                               gap_event, NULL);
    }

    if (rc == 0) {
        state_print("# BLE advertising: " BLE_MOUSE_DEVICE_NAME);
    } else {
        state_print("# BLE advertising failed");
    }
}

static void hid_event(void *handler_context, esp_event_base_t base, int32_t id,
                      void *event_data)
{
    (void)handler_context;
    (void)base;
    (void)event_data;

    switch ((esp_hidd_event_t)id) {
    case ESP_HIDD_START_EVENT:
        if (ble_hs_id_infer_auto(0, &own_address_type) != 0) {
            state_print("# BLE address setup failed");
            return;
        }
        state_print("# BLE stack up");
        start_advertising();
        break;
    case ESP_HIDD_CONNECT_EVENT:
        connected = true;
        state_print("# BLE connected");
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        connected = false;
        state_print("# BLE disconnected");
        start_advertising();
        break;
    default:
        break;
    }
}

static int8_t motion_delta(float rate_dps, float sign, float gain)
{
    if (rate_dps > -BLE_MOUSE_DEADZONE_DPS && rate_dps < BLE_MOUSE_DEADZONE_DPS) {
        return 0;
    }

    float scaled = rate_dps * sign * gain;
    int delta = (int)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
    if (delta > 127) {
        delta = 127;
    } else if (delta < -127) {
        delta = -127;
    }
    return (int8_t)delta;
}

static void report_task(void *context)
{
    (void)context;
    bool previous_pressed = false;
    bool was_connected = false;
    TickType_t wake_time = xTaskGetTickCount();

    for (;;) {
        const bool pressed = gpio_get_level(BLE_MOUSE_BUTTON_GPIO) == 0;
        if (connected) {
            const int8_t dx = motion_delta(latest_gyro_z, BLE_MOUSE_X_SIGN,
                                           BLE_MOUSE_X_GAIN);
            const int8_t dy = motion_delta(latest_gyro_x, BLE_MOUSE_Y_SIGN,
                                           BLE_MOUSE_Y_GAIN);
            if (!was_connected || dx != 0 || dy != 0 || pressed != previous_pressed) {
                uint8_t report[4] = {
                    pressed ? 1 : 0,
                    (uint8_t)dx,
                    (uint8_t)dy,
                    0,
                };
                (void)esp_hidd_dev_input_set(hid_device, 0, 0, report,
                                             sizeof(report));
            }
        }
        previous_pressed = pressed;
        was_connected = connected;
        xTaskDelayUntil(&wake_time, pdMS_TO_TICKS(BLE_MOUSE_REPORT_PERIOD_MS));
    }
}

static void host_task(void *context)
{
    (void)context;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_mouse_update_gyro(float gyro_x_dps, float gyro_z_dps)
{
    latest_gyro_x = gyro_x_dps;
    latest_gyro_z = gyro_z_dps;
}

void ble_mouse_start(SemaphoreHandle_t serial_mutex)
{
    output_mutex = serial_mutex;

    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);

    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << BLE_MOUSE_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t controller_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&controller_config));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_nimble_init());

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_ENC;

    ESP_ERROR_CHECK(esp_hidd_dev_init(&hid_config, ESP_HID_TRANSPORT_BLE, hid_event,
                                      &hid_device));
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(BLE_MOUSE_DEVICE_NAME) == 0
                        ? ESP_OK
                        : ESP_FAIL);
    ESP_ERROR_CHECK(ble_svc_gap_device_appearance_set(HID_MOUSE_APPEARANCE) == 0
                        ? ESP_OK
                        : ESP_FAIL);
    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    state_printf("# BLE config SM_LVL=%d runtime=%d bonding=%d mitm=%d sc=%d",
                 CONFIG_BT_NIMBLE_SM_LVL, ble_hs_cfg.sm_sec_lvl,
                 ble_hs_cfg.sm_bonding, ble_hs_cfg.sm_mitm, ble_hs_cfg.sm_sc);
    state_printf("# BLE key distribution our=0x%02x (ENC|ID) their=0x%02x (ENC|ID)",
                 ble_hs_cfg.sm_our_key_dist, ble_hs_cfg.sm_their_key_dist);
    int bond_count;
    const int bond_count_status =
        ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &bond_count);
    if (bond_count_status == 0) {
        state_printf("# BLE stored bonds=%d", bond_count);
    } else {
        state_printf("# BLE stored bond count failed status=%d", bond_count_status);
    }

    nimble_port_freertos_init(host_task);

    if (xTaskCreate(report_task, "ble_mouse", 3072, NULL, 2, NULL) != pdPASS) {
        state_print("# FATAL start BLE mouse task");
        abort();
    }
}
