/*
 * ESP RainMaker integration for Heart Rate Monitor
 * Creates a custom "HeartSensor" device with BPM, SpO2 and health params.
 */

#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_ota.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_common_events.h>
#include <app_network.h>

#include "app_rainmaker.h"

static const char *TAG = "RMAKER";

/* RainMaker device and param handles */
static esp_rmaker_device_t *hr_device = NULL;

static esp_rmaker_param_t *param_bpm = NULL;
static esp_rmaker_param_t *param_spo2 = NULL;
static esp_rmaker_param_t *param_spo2_threshold = NULL;
static esp_rmaker_param_t *param_finger_status = NULL;
static esp_rmaker_param_t *param_health_status = NULL;

static esp_rmaker_param_t *oled_status_param = NULL;
static esp_rmaker_param_t *led_status_param = NULL;
static esp_rmaker_param_t *alarm_permission_param = NULL;
static esp_rmaker_param_t *buzzer_status_param = NULL;


/* Custom param names shown on ESP RainMaker app */
#define PARAM_NAME_BPM                 "Nhịp tim (BPM)"
#define PARAM_NAME_SPO2                "Nồng độ oxy SpO₂(%)"
#define PARAM_NAME_SPO2_THRESHOLD      "Ngưỡng cảnh báo SpO₂"
#define PARAM_NAME_FINGER_STATUS       "Trạng thái ngón tay"
#define PARAM_NAME_HEALTH_STATUS       "Tình trạng sức khỏe"

#define PARAM_NAME_OLED_STATUS         "OLED"
#define PARAM_NAME_LED_STATUS          "LED"
#define PARAM_NAME_ALARM_PERMISSION    "Quyền còi"
#define PARAM_NAME_BUZZER_STATUS       "Còi thực tế"

#define SPO2_THRESHOLD_DEFAULT         95
#define SPO2_THRESHOLD_MIN             80
#define SPO2_THRESHOLD_MAX             100
#define RMAKER_MIN_REPORT_INTERVAL_MS  2000
#define RMAKER_FORCE_REPORT_INTERVAL_MS 10000

static int spo2_alert_threshold = SPO2_THRESHOLD_DEFAULT;

/*
 * Cache trạng thái để tránh spam RainMaker.
 * Các hàm update_* chỉ cập nhật local param khi giá trị thay đổi.
 * app_rainmaker_update_data() sẽ flush tất cả param đã đổi bằng 1 MQTT message.
 */
static bool rmaker_param_dirty = false;
static char cached_oled_status[16] = "";
static char cached_led_status[16] = "";
static char cached_alarm_permission[16] = "";
static char cached_buzzer_status[32] = "";

static bool update_cached_string_param(esp_rmaker_param_t *param,
                                       char *cache,
                                       size_t cache_size,
                                       const char *value)
{
    if (!param || !value || !cache || cache_size == 0) {
        return false;
    }

    if (strcmp(cache, value) == 0) {
        return false;
    }

    esp_rmaker_param_update(param, esp_rmaker_str(value));
    strlcpy(cache, value, cache_size);
    rmaker_param_dirty = true;
    return true;
}

static void flush_rainmaker_updated_params_now(const char *reason)
{
    if (!hr_device || !rmaker_param_dirty) {
        return;
    }

    esp_err_t err = esp_rmaker_report_updated_params();
    if (err == ESP_OK) {
        rmaker_param_dirty = false;
        ESP_LOGI(TAG, "RainMaker status synced: %s", reason ? reason : "status");
    } else {
        ESP_LOGW(TAG, "RainMaker status sync failed: %s", esp_err_to_name(err));
    }
}


static int clamp_spo2_threshold(int value)
{
    if (value < SPO2_THRESHOLD_MIN) {
        return SPO2_THRESHOLD_MIN;
    }

    if (value > SPO2_THRESHOLD_MAX) {
        return SPO2_THRESHOLD_MAX;
    }

    return value;
}

int app_rainmaker_get_spo2_alert_threshold(void)
{
    return spo2_alert_threshold;
}

void app_rainmaker_set_spo2_alert_threshold(int threshold)
{
    spo2_alert_threshold = clamp_spo2_threshold(threshold);

    if (param_spo2_threshold) {
        esp_rmaker_param_update_and_report(
            param_spo2_threshold,
            esp_rmaker_int(spo2_alert_threshold)
        );
    }
}

void app_rainmaker_update_oled_status(bool oled_active)
{
    if (!hr_device || !oled_status_param) {
        return;
    }

    update_cached_string_param(
        oled_status_param,
        cached_oled_status,
        sizeof(cached_oled_status),
        oled_active ? "ON" : "OFF"
    );
}

void app_rainmaker_update_led_status(bool led_on)
{
    if (!hr_device || !led_status_param) {
        return;
    }

    update_cached_string_param(
        led_status_param,
        cached_led_status,
        sizeof(cached_led_status),
        led_on ? "ON" : "OFF"
    );
}

void app_rainmaker_update_alarm_permission(bool alarm_enabled)
{
    if (!hr_device || !alarm_permission_param) {
        return;
    }

    update_cached_string_param(
        alarm_permission_param,
        cached_alarm_permission,
        sizeof(cached_alarm_permission),
        alarm_enabled ? "Bật" : "Tắt"
    );
}

void app_rainmaker_update_buzzer_status(bool buzzer_on)
{
    if (!hr_device || !buzzer_status_param) {
        return;
    }

    update_cached_string_param(
        buzzer_status_param,
        cached_buzzer_status,
        sizeof(cached_buzzer_status),
        buzzer_on ? "Đang kêu" : "Không kêu"
    );
}

void app_rainmaker_update_alarm_status(bool alarm_on)
{
    app_rainmaker_update_buzzer_status(alarm_on);
}

/*
 * Đồng bộ nhóm trạng thái giống Web.
 * Hàm này report NGAY khi một trạng thái phụ thay đổi.
 * Vì có cache nên gọi hàm này liên tục trong vòng lặp vẫn không spam RainMaker.
 */
void app_rainmaker_update_runtime_status(bool oled_active,
                                         bool led_on,
                                         bool alarm_enabled,
                                         bool buzzer_on)
{
    if (!hr_device) {
        return;
    }

    bool changed = false;

    changed |= update_cached_string_param(
        oled_status_param,
        cached_oled_status,
        sizeof(cached_oled_status),
        oled_active ? "ON" : "OFF"
    );

    changed |= update_cached_string_param(
        led_status_param,
        cached_led_status,
        sizeof(cached_led_status),
        led_on ? "ON" : "OFF"
    );

    changed |= update_cached_string_param(
        alarm_permission_param,
        cached_alarm_permission,
        sizeof(cached_alarm_permission),
        alarm_enabled ? "Bật" : "Tắt"
    );

    changed |= update_cached_string_param(
        buzzer_status_param,
        cached_buzzer_status,
        sizeof(cached_buzzer_status),
        buzzer_on ? "Đang kêu" : "Không kêu"
    );


    if (changed) {
        flush_rainmaker_updated_params_now("runtime status changed");
    }
}

/* Write callback: chỉ ngưỡng cảnh báo SpO2 được phép chỉnh từ app */
static esp_err_t write_cb(const esp_rmaker_device_t *device,
                          const esp_rmaker_param_t *param,
                          const esp_rmaker_param_val_t val,
                          void *priv_data,
                          esp_rmaker_write_ctx_t *ctx)
{
    (void)device;
    (void)priv_data;

    const char *param_name = esp_rmaker_param_get_name(param);

    if (param_name && strcmp(param_name, PARAM_NAME_SPO2_THRESHOLD) == 0) {
        int new_threshold = spo2_alert_threshold;

        if (val.type == RMAKER_VAL_TYPE_INTEGER) {
            new_threshold = val.val.i;
        } else if (val.type == RMAKER_VAL_TYPE_FLOAT) {
            new_threshold = (int)(val.val.f + 0.5f);
        } else {
            ESP_LOGW(TAG, "Invalid type for %s", PARAM_NAME_SPO2_THRESHOLD);
            return ESP_ERR_INVALID_ARG;
        }

        new_threshold = clamp_spo2_threshold(new_threshold);
        spo2_alert_threshold = new_threshold;

        if (ctx && ctx->src == ESP_RMAKER_REQ_SRC_INIT) {
            esp_rmaker_param_update(param, esp_rmaker_int(new_threshold));
        } else {
            esp_rmaker_param_update_and_report(param, esp_rmaker_int(new_threshold));
        }

        ESP_LOGI(TAG, "SpO2 alert threshold updated: %d%%", spo2_alert_threshold);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Write request for read-only param ignored: %s",
             param_name ? param_name : "?");

    return ESP_OK;
}

static void rmaker_event_handler(void *arg,
                                 esp_event_base_t event_base,
                                 int32_t event_id,
                                 void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected to RainMaker cloud");
                break;

            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGW(TAG, "MQTT Disconnected from RainMaker cloud");
                break;

            default:
                break;
        }
    }
}

esp_err_t app_rainmaker_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized");

    esp_rmaker_console_init();

    app_network_init();
    ESP_LOGI(TAG, "Network initialized");

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            RMAKER_COMMON_EVENT,
            ESP_EVENT_ANY_ID,
            &rmaker_event_handler,
            NULL
        )
    );

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true,
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(
        &rainmaker_cfg,
        "Heart Rate Monitor",
        "Sensor"
    );

    if (!node) {
        ESP_LOGE(TAG, "Could not initialize node. Aborting!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        abort();
    }

    hr_device = esp_rmaker_device_create("HeartSensor", NULL, NULL);

    if (!hr_device) {
        ESP_LOGE(TAG, "Could not create device. Aborting!");
        abort();
    }

    esp_rmaker_device_add_cb(hr_device, write_cb, NULL);

    param_bpm = esp_rmaker_param_create(
        PARAM_NAME_BPM,
        NULL,
        esp_rmaker_int(0),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(param_bpm, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, param_bpm);

    param_spo2 = esp_rmaker_param_create(
        PARAM_NAME_SPO2,
        NULL,
        esp_rmaker_int(0),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(param_spo2, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, param_spo2);

    param_spo2_threshold = esp_rmaker_param_create(
        PARAM_NAME_SPO2_THRESHOLD,
        NULL,
        esp_rmaker_int(spo2_alert_threshold),
        PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST
    );
    esp_rmaker_param_add_ui_type(param_spo2_threshold, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(
        param_spo2_threshold,
        esp_rmaker_int(SPO2_THRESHOLD_MIN),
        esp_rmaker_int(SPO2_THRESHOLD_MAX),
        esp_rmaker_int(1)
    );
    esp_rmaker_device_add_param(hr_device, param_spo2_threshold);

    esp_rmaker_param_val_t *stored_threshold =
        esp_rmaker_param_get_val(param_spo2_threshold);

    if (stored_threshold &&
        stored_threshold->type == RMAKER_VAL_TYPE_INTEGER) {
        spo2_alert_threshold =
            clamp_spo2_threshold(stored_threshold->val.i);
    }

    param_finger_status = esp_rmaker_param_create(
        PARAM_NAME_FINGER_STATUS,
        NULL,
        esp_rmaker_str("Chưa phát hiện ngón tay"),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(param_finger_status, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, param_finger_status);

    param_health_status = esp_rmaker_param_create(
        PARAM_NAME_HEALTH_STATUS,
        NULL,
        esp_rmaker_str("Chưa đặt ngón tay"),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(param_health_status, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, param_health_status);

    oled_status_param = esp_rmaker_param_create(
        PARAM_NAME_OLED_STATUS,
        NULL,
        esp_rmaker_str("OFF"),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(oled_status_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, oled_status_param);

    led_status_param = esp_rmaker_param_create(
        PARAM_NAME_LED_STATUS,
        NULL,
        esp_rmaker_str("OFF"),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(led_status_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, led_status_param);

    alarm_permission_param = esp_rmaker_param_create(
        PARAM_NAME_ALARM_PERMISSION,
        NULL,
        esp_rmaker_str("Bật"),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(alarm_permission_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, alarm_permission_param);

    buzzer_status_param = esp_rmaker_param_create(
        PARAM_NAME_BUZZER_STATUS,
        NULL,
        esp_rmaker_str("Không kêu"),
        PROP_FLAG_READ
    );
    esp_rmaker_param_add_ui_type(buzzer_status_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(hr_device, buzzer_status_param);

    esp_rmaker_node_add_device(node, hr_device);
    ESP_LOGI(TAG, "HeartSensor device added with health monitor params");

    esp_rmaker_ota_enable_default();

    esp_rmaker_system_serv_config_t sys_cfg = {
        .flags = SYSTEM_SERV_FLAGS_ALL,
        .reboot_seconds = 2,
        .reset_seconds = 2,
        .reset_reboot_seconds = 2,
    };
    esp_rmaker_system_service_enable(&sys_cfg);

    err = esp_rmaker_start();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RainMaker agent: %s",
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "RainMaker agent started");

    app_rainmaker_update_oled_status(false);
    app_rainmaker_update_led_status(false);
    app_rainmaker_update_alarm_permission(true);
    app_rainmaker_update_buzzer_status(false);

    err = app_network_start(POP_TYPE_RANDOM);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start network: %s",
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Network started, waiting for provisioning/connection...");

    return ESP_OK;
}

void app_rainmaker_update_data(int bpm,
                               int spo2,
                               bool finger_detected,
                               const char *health_status)
{
    if (!hr_device) {
        return;
    }

    static int last_bpm = INT_MIN;
    static int last_spo2 = INT_MIN;
    static bool last_finger_detected = false;
    static char last_health_status[96] = "";
    static int64_t last_report_ms = 0;

    int64_t now_ms = esp_timer_get_time() / 1000;

    const char *finger_status = finger_detected
                                    ? "Đã đặt ngón tay"
                                    : "Chưa phát hiện ngón tay";

    bool force_update = (last_report_ms == 0) ||
                        (now_ms - last_report_ms >= RMAKER_FORCE_REPORT_INTERVAL_MS);

    if (param_bpm && (force_update || bpm != last_bpm)) {
        esp_rmaker_param_update(param_bpm, esp_rmaker_int(bpm));
        last_bpm = bpm;
        rmaker_param_dirty = true;
    }

    if (param_spo2 && (force_update || spo2 != last_spo2)) {
        esp_rmaker_param_update(param_spo2, esp_rmaker_int(spo2));
        last_spo2 = spo2;
        rmaker_param_dirty = true;
    }

    if (param_finger_status &&
        (force_update || finger_detected != last_finger_detected)) {
        esp_rmaker_param_update(param_finger_status, esp_rmaker_str(finger_status));
        last_finger_detected = finger_detected;
        rmaker_param_dirty = true;
    }

    if (param_health_status && health_status &&
        (force_update || strcmp(health_status, last_health_status) != 0)) {
        esp_rmaker_param_update(param_health_status, esp_rmaker_str(health_status));
        strlcpy(last_health_status, health_status, sizeof(last_health_status));
        rmaker_param_dirty = true;
    }

    bool can_report_now = (last_report_ms == 0) ||
                          (now_ms - last_report_ms >= RMAKER_MIN_REPORT_INTERVAL_MS);

    if (rmaker_param_dirty && can_report_now) {
        esp_err_t err = esp_rmaker_report_updated_params();
        if (err == ESP_OK) {
            last_report_ms = now_ms;
            rmaker_param_dirty = false;
        } else {
            ESP_LOGW(TAG, "RainMaker report failed: %s", esp_err_to_name(err));
        }
    }

    ESP_LOGD(TAG,
             "RMaker sync queued: BPM=%d, SpO2=%d, finger=%d, health=%s, dirty=%d",
             bpm,
             spo2,
             finger_detected,
             health_status ? health_status : "?",
             rmaker_param_dirty);
}
