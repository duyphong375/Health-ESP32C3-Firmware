/*
 * Heart Rate & SpO2 Monitor - ESP-IDF 6.0.1 (ESP32-C3)
 *
 * I2C: SDA=GPIO4, SCL=GPIO5
 * Devices: MAX30102 (0x57) + SSD1306 128x32 OLED (0x3C)
 *
 * Controls:
 *   Button A (GPIO2) - Chuyển màn hình: Waveform <-> Stats
 *   Button B (GPIO3) - Bật/tắt màn hình OLED
 *   Button C (GPIO6) - Tắt tiếng buzzer (Mute)
 *   Buzzer   (GPIO7) - Cảnh báo BPM/SpO2 (Low-level trigger module)
 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "max30102.h"
#include "ssd1306.h"
#include "pulse.h"
#include "app_rainmaker.h"

static const char *TAG = "MAIN";

/* ── I2C pins ── */
#define I2C_SDA_PIN   GPIO_NUM_4
#define I2C_SCL_PIN   GPIO_NUM_5

/* ── LED pin ── */
#define LED_PIN       GPIO_NUM_8

/* ── Button pins ── */
#define BTN_A_PIN     GPIO_NUM_2   /* Chuyển màn hình */
#define BTN_B_PIN     GPIO_NUM_3   /* Bật/tắt OLED    */
#define BTN_C_PIN     GPIO_NUM_6   /* Mute buzzer     */

/* ── Buzzer pin (Low-level trigger: LOW=BẬT, HIGH=TẮT) ── */
#define BUZZER_PIN    GPIO_NUM_7
#define BUZZER_ON     0
#define BUZZER_OFF    1

/* ── Chống dội phím 300ms (tăng để lọc bounce tốt hơn) ── */
#define DEBOUNCE_MS   300

/* ── Số nhịp bỏ qua lúc đầu (warmup) trước khi ghi max/min ── */
#define STATS_WARMUP_BEATS  8

/* MQTT dashboard: chỉnh URI broker ở đây hoặc truyền qua compiler flag. */
#define DEVICE_ID                "esp32c3_health_001"
#ifndef WEB_MQTT_BROKER_URI
#define WEB_MQTT_BROKER_URI      "mqtt://broker.emqx.io:1883"
#endif
#define MQTT_TELEMETRY_TOPIC       "device/" DEVICE_ID "/telemetry"
#define MQTT_CONTROL_THRESHOLD     "device/" DEVICE_ID "/control/spo2_threshold"
#define MQTT_CONTROL_RESET         "device/" DEVICE_ID "/control/reset_measurement"
#define MQTT_CONTROL_ALARM_MUTE    "device/" DEVICE_ID "/control/alarm_mute"
#define MQTT_CONTROL_OLED          "device/" DEVICE_ID "/control/oled"
#define WEB_TELEMETRY_MS           1000
#define SENSOR_STABLE_TIME_MS      5000     // Thời gian ổn định
/* ── Heart bitmap (16x16) ── */
static const uint8_t heart_bits[] = {
    0x00, 0x00, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe,
    0xfe, 0xff, 0xfe, 0xff, 0xfc, 0x7f, 0xf8, 0x3f,
    0xf0, 0x1f, 0xe0, 0x0f, 0xc0, 0x07, 0x80, 0x03,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* ── SpO2 lookup table ── */
static const uint8_t spo2_table[184] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
   100,100,100,100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
    97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
    90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
    80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
    66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
    28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10,  9,  7,  6,  5,
     3,  2,  1
};

/* ── Waveform recorder ── */
#define MAXWAVE 64

typedef struct {
    uint8_t waveform[MAXWAVE];
    uint8_t disp_wave[MAXWAVE];
    uint8_t wavep;
} waveform_t;

static void waveform_init(waveform_t *w) {
    memset(w, 0, sizeof(waveform_t));
}

static void waveform_record(waveform_t *w, int waveval) {
    /* Shift về vùng [0,255]. Clamp PHẢI nằm sau phép cộng 128 */
    waveval = waveval / 8 + 128;
    if (waveval < 0)   waveval = 0;
    if (waveval > 255) waveval = 255;
    w->waveform[w->wavep] = (uint8_t)waveval;
    w->wavep = (w->wavep + 1) % MAXWAVE;
}

static void waveform_scale(waveform_t *w) {
    uint8_t maxw = 0, minw = 255;
    for (int i = 0; i < MAXWAVE; i++) {
        if (w->waveform[i] > maxw) maxw = w->waveform[i];
        if (w->waveform[i] < minw) minw = w->waveform[i];
    }
    uint8_t scale8 = (maxw - minw) / 4 + 1;
    uint8_t index = w->wavep;
    for (int i = 0; i < MAXWAVE; i++) {
        w->disp_wave[i] = 31 - ((uint16_t)(w->waveform[index] - minw) * 8) / scale8;
        index = (index + 1) % MAXWAVE;
    }
}

static void waveform_draw(waveform_t *w, ssd1306_t *oled, uint8_t X) {
    for (int i = 0; i < MAXWAVE; i++) {
        uint8_t y = w->disp_wave[i];
        ssd1306_draw_pixel(oled, X + i, y);
        if (i < MAXWAVE - 1) {
            uint8_t nexty = w->disp_wave[i + 1];
            if (nexty > y) {
                for (uint8_t iy = y + 1; iy < nexty; ++iy)
                    ssd1306_draw_pixel(oled, X + i, iy);
            } else if (nexty < y) {
                for (uint8_t iy = nexty + 1; iy < y; ++iy)
                    ssd1306_draw_pixel(oled, X + i, iy);
            }
        }
    }
}

/* ── BPM Moving Average filter ── */
static ma_filter_t bpm_filter;

/* ── Global sensor state ── */
static ssd1306_t   oled;
static max30102_t  sensor;
static pulse_t     pulseIR;
static pulse_t     pulseRed;
static waveform_t  wave;

typedef struct {
    float bpm;
    float spo2;
    int waveform;
    int spo2_threshold;
    bool finger_detected;
    bool alarm_status;
    bool led_on;
    char finger_status[32];
    char health_status[64];
    char signal_quality[32];
} health_data_t;

/*
 * Nguồn dữ liệu trung tâm:
 * MAX30102 cập nhật struct này, OLED/RainMaker/MQTT/còi/LED đều đọc từ đây.
 */
static health_data_t health_data = {
    .bpm = 0,
    .spo2 = 0,
    .waveform = 0,
    .spo2_threshold = 95,
    .finger_detected = false,
    .alarm_status = false,
    .led_on = false,
    .finger_status = "Chưa phát hiện ngón tay",
    .health_status = "Chưa đặt ngón tay",
    .signal_quality = "Chưa có dữ liệu",
};

static esp_mqtt_client_handle_t web_mqtt_client = NULL;
static bool web_mqtt_connected = false;

static bool filter_for_graph = false;
static bool draw_Red         = false;

/* ── UI state ── */
static uint8_t cur_screen = 0;  /* 0 = waveform, 1 = stats */
static bool oled_on       = true; /* OLED power state */

/* ── Thống kê BPM (dùng cho màn hình stats) ── */
static long bpm_sum   = 0;
static int  bpm_count = 0;

/* ── Button queue & debounce timestamp ── */
static QueueHandle_t btn_queue;
static long btn_last_ms[3] = {0, 0, 0};  /* index: 0=A, 1=B, 2=C */

/* ── Buzzer state ── */
static bool buzzer_alerting    = false;
static bool buzzer_muted       = false;
static bool buzzer_phase       = false;
static long buzzer_last_toggle = 0;

/* ── Helper: millis() ── */
static inline long get_millis(void) {
    return (long)(esp_timer_get_time() / 1000);
}

static int health_bpm_i(void)
{
    return (int)(health_data.bpm + 0.5f);
}

static int health_spo2_i(void)
{
    return (int)(health_data.spo2 + 0.5f);
}

static void health_set_led(bool on)
{
    static bool last_led_on = false;
    static bool led_state_initialized = false;

    health_data.led_on = on;

    if (!led_state_initialized || last_led_on != on) {
        led_state_initialized = true;
        last_led_on = on;
        gpio_set_level(LED_PIN, on ? 1 : 0);
        app_rainmaker_update_led_status(on);
    }
}

/*
 * Đồng bộ thêm các trạng thái phụ lên ESP RainMaker.
 * OLED: Active / Unknown
 * LED: ON / OFF
 * Quyền còi: Bật / Tắt
 * Còi thực tế: Đang kêu / Không kêu
 */
static void sync_rainmaker_extra_status(void)
{
    bool alarm_permission = !buzzer_muted;
    bool buzzer_real_on = buzzer_alerting && !buzzer_muted;

    app_rainmaker_update_runtime_status(
        oled_on,
        health_data.led_on,
        alarm_permission,
        buzzer_real_on
    );
}

static bool is_spo2_valid(int spo2)
{
    return (spo2 > 0 && spo2 <= 100);
}

static bool is_bpm_valid(int bpm)
{
    return (bpm > 0);
}

static const char *signal_quality_from_ir(uint32_t ir_value)
{
    if (ir_value < 5000) {
        return "Chưa có dữ liệu";
    }
    if (ir_value < 20000) {
        return "Yếu";
    }
    if (ir_value < 50000) {
        return "Trung bình";
    }
    return "Tốt";
}

/*
 * Tình trạng sức khỏe gửi lên RainMaker.
 * Hàm này dùng trực tiếp health_data đang hiển thị trên OLED, không tạo nguồn đo riêng.
 */
static const char *update_health_status(bool finger_detected, int bpm, int spo2)
{
    int spo2_threshold = health_data.spo2_threshold;

    if (!finger_detected) {
        return "Chưa đặt ngón tay";
    }
    if (!is_bpm_valid(bpm) || !is_spo2_valid(spo2)) {
        return "Đang đo, vui lòng giữ yên tay";
    }

    bool spo2_low = (spo2 < spo2_threshold);
    bool bpm_low = (bpm < 60);
    bool bpm_high = (bpm > 100);

    if (spo2_low && (bpm_low || bpm_high)) {
        return "Cảnh báo sức khỏe bất thường";
    }
    if (spo2_low) {
        return "Cảnh báo: SpO₂ thấp";
    }
    if (bpm_low) {
        return "Nhịp tim thấp";
    }
    if (bpm_high) {
        return "Nhịp tim cao";
    }
    return "Bình thường";
}

static void health_update_text_status(void)
{
    strlcpy(health_data.finger_status,
            health_data.finger_detected ? "Đã đặt ngón tay" : "Chưa phát hiện ngón tay",
            sizeof(health_data.finger_status));
    strlcpy(health_data.health_status,
            update_health_status(health_data.finger_detected, health_bpm_i(), health_spo2_i()),
            sizeof(health_data.health_status));
}

static void health_reset_measurement_stats(void)
{
    bpm_sum = 0;
    bpm_count = 0;
}

static void make_iso_timestamp(char *buf, size_t len)
{
    time_t now_time = 0;
    struct tm timeinfo = {0};

    time(&now_time);
    localtime_r(&now_time, &timeinfo);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%S%z", &timeinfo);

    size_t used = strlen(buf);
    if (used == 24) {
        /* Đổi +0700 thành +07:00 để web đọc ISO dễ hơn. */
        buf[25] = '\0';
        buf[24] = buf[23];
        buf[23] = buf[22];
        buf[22] = ':';
    }
}

static void web_mqtt_publish_telemetry(void)
{
    if (!web_mqtt_client || !web_mqtt_connected) {
        return;
    }

    char timestamp[32];
    char payload[768];
    make_iso_timestamp(timestamp, sizeof(timestamp));

    snprintf(payload, sizeof(payload),
         "{\"device_id\":\"%s\",\"timestamp\":\"%s\",\"bpm\":%d,\"spo2\":%d,"
         "\"waveform\":%d,\"spo2_threshold\":%d,"
         "\"finger_status\":\"%s\",\"finger_detected\":%s,"
         "\"health_status\":\"%s\","
         "\"alarm_status\":%s,\"alarm_muted\":%s,"
         "\"led_status\":\"%s\",\"led\":%s,"
         "\"oled_status\":\"%s\",\"oled\":%s,"
         "\"signal_quality\":\"%s\"}",
         DEVICE_ID, timestamp,
         health_bpm_i(), health_spo2_i(),
         health_data.waveform, health_data.spo2_threshold,
         health_data.finger_status,
         health_data.finger_detected ? "true" : "false",
         health_data.health_status,
         health_data.alarm_status ? "true" : "false",
         buzzer_muted ? "true" : "false",
         health_data.led_on ? "Đang đo" : "Tắt",
         health_data.led_on ? "true" : "false",
         oled_on ? "ON" : "OFF",
         oled_on ? "true" : "false",
         health_data.signal_quality);
    ESP_LOGI("WEB_SYNC",
         "SEND WEB: BPM=%d | SpO2=%d | Wave=%d | Threshold=%d | Finger=%s | Health=%s",
         health_bpm_i(),
         health_spo2_i(),
         health_data.waveform,
         health_data.spo2_threshold,
         health_data.finger_status,
         health_data.health_status);
    esp_mqtt_client_publish(web_mqtt_client, MQTT_TELEMETRY_TOPIC, payload, 0, 0, 0);
}

static int parse_spo2_threshold_payload(const char *payload)
{
    int threshold = -1;
    const char *key = strstr(payload, "spo2_threshold");

    if (key) {
        key = strchr(key, ':');
        if (key) {
            sscanf(key + 1, "%d", &threshold);
        }
    } else {
        sscanf(payload, "%d", &threshold);
    }
    return threshold;
}

static bool parse_alarm_mute_payload(const char *payload, bool current_value)
{
    if (!payload) return current_value;

    /* Web gửi {"alarm_mute":true} để tắt còi
       và {"alarm_mute":false} để bật lại quyền còi. */
    if (strstr(payload, "false") || strstr(payload, "OFF") || strstr(payload, "off")) {
        return false;
    }
    if (strstr(payload, "true") || strstr(payload, "ON") || strstr(payload, "on")) {
        return true;
    }

    return current_value;
}

static bool parse_oled_payload(const char *payload, bool current_value)
{
    if (!payload) return current_value;

    if (strstr(payload, "false") ||
        strstr(payload, "OFF") ||
        strstr(payload, "off") ||
        strstr(payload, "\"oled\":false")) {
        return false;
    }

    if (strstr(payload, "true") ||
        strstr(payload, "ON") ||
        strstr(payload, "on") ||
        strstr(payload, "\"oled\":true")) {
        return true;
    }

    return current_value;
}

static void set_oled_state(bool next_oled_on, const char *source)
{
    if (oled_on == next_oled_on) {
        sync_rainmaker_extra_status();
        web_mqtt_publish_telemetry();
        return;
    }

    oled_on = next_oled_on;

    if (!oled_on) {
        ssd1306_fill(&oled, 0x00);
        ssd1306_off(&oled);
    } else {
        ssd1306_on(&oled);
        ssd1306_fill(&oled, 0x00);
    }

    /*
     * Đồng bộ:
     * OLED vật lý -> biến oled_on -> RainMaker -> Web telemetry
     */
    sync_rainmaker_extra_status();
    web_mqtt_publish_telemetry();

    ESP_LOGI(TAG, "OLED %s by %s", oled_on ? "ON" : "OFF", source ? source : "unknown");
}

static void set_buzzer_permission(bool enable_buzzer, const char *source)
{
    /*
     * enable_buzzer = true  => Quyền còi: Bật
     * enable_buzzer = false => Quyền còi: Tắt
     *
     * buzzer_muted = true   => người dùng đã tắt còi
     * buzzer_muted = false  => người dùng cho phép còi
     */
    buzzer_muted = !enable_buzzer;

    if (buzzer_muted) {
        /*
         * Người dùng tắt còi:
         * - Tắt chân buzzer vật lý ngay
         * - Không xóa trạng thái cảnh báo buzzer_alerting
         *   vì SpO2 có thể vẫn đang thấp
         * - Chỉ cho "Còi thực tế" = Không kêu
         */
        buzzer_phase = false;
        gpio_set_level(BUZZER_PIN, BUZZER_OFF);
        health_data.alarm_status = false;
    } else {
        /*
         * Người dùng bật lại còi:
         * Nếu hiện tại vẫn đang cảnh báo, cho còi kêu lại.
         */
        buzzer_phase = false;
        buzzer_last_toggle = get_millis();

        if (buzzer_alerting) {
            gpio_set_level(BUZZER_PIN, BUZZER_ON);
            health_data.alarm_status = true;
        } else {
            gpio_set_level(BUZZER_PIN, BUZZER_OFF);
            health_data.alarm_status = false;
        }
    }

    health_update_text_status();
    sync_rainmaker_extra_status();
    web_mqtt_publish_telemetry();

    ESP_LOGI(TAG,
             "Buzzer permission %s by %s, alerting=%d, muted=%d",
             enable_buzzer ? "ON" : "OFF",
             source ? source : "unknown",
             buzzer_alerting,
             buzzer_muted);
}

static void web_mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                   int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            web_mqtt_connected = true;
            ESP_LOGI(TAG, "Web MQTT connected");
            esp_mqtt_client_subscribe(web_mqtt_client, MQTT_CONTROL_THRESHOLD, 0);
            esp_mqtt_client_subscribe(web_mqtt_client, MQTT_CONTROL_RESET, 0);
            esp_mqtt_client_subscribe(web_mqtt_client, MQTT_CONTROL_ALARM_MUTE, 0);
            esp_mqtt_client_subscribe(web_mqtt_client, MQTT_CONTROL_OLED, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            web_mqtt_connected = false;
            ESP_LOGW(TAG, "Web MQTT disconnected");
            break;
        case MQTT_EVENT_DATA: {
            char topic[128] = {0};
            char data[128] = {0};
            int topic_len = event->topic_len < (int)sizeof(topic) - 1 ? event->topic_len : (int)sizeof(topic) - 1;
            int data_len = event->data_len < (int)sizeof(data) - 1 ? event->data_len : (int)sizeof(data) - 1;

            memcpy(topic, event->topic, topic_len);
            memcpy(data, event->data, data_len);

            if (strcmp(topic, MQTT_CONTROL_THRESHOLD) == 0) {
                int threshold = parse_spo2_threshold_payload(data);
                if (threshold >= 80 && threshold <= 100) {
                    health_data.spo2_threshold = threshold;
                    health_update_text_status();
                    app_rainmaker_set_spo2_alert_threshold(threshold);
                    web_mqtt_publish_telemetry();
                    ESP_LOGI(TAG, "Web updated SpO2 threshold: %d", threshold);
                }
            } else if (strcmp(topic, MQTT_CONTROL_RESET) == 0) {
                health_reset_measurement_stats();
                web_mqtt_publish_telemetry();
                ESP_LOGI(TAG, "Web requested measurement reset");
            } else if (strcmp(topic, MQTT_CONTROL_ALARM_MUTE) == 0) {
                bool next_muted = parse_alarm_mute_payload(data, buzzer_muted);

                /*
                * Web gửi:
                * alarm_mute = true  => tắt còi
                * alarm_mute = false => bật còi
                */
                set_buzzer_permission(!next_muted, "web");

            } else if (strcmp(topic, MQTT_CONTROL_OLED) == 0) {
                bool next_oled_on = parse_oled_payload(data, oled_on);
                set_oled_state(next_oled_on, "web");
            }
            break;
        }
        default:
            break;
    }
}

static void web_mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = WEB_MQTT_BROKER_URI,
    };

    web_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!web_mqtt_client) {
        ESP_LOGW(TAG, "Web MQTT init failed");
        return;
    }

    esp_mqtt_client_register_event(web_mqtt_client, ESP_EVENT_ANY_ID,
                                   web_mqtt_event_handler, NULL);
    esp_mqtt_client_start(web_mqtt_client);
    ESP_LOGI(TAG, "Web MQTT starting, broker=%s", WEB_MQTT_BROKER_URI);
}

/* ────────────────────────────────────────────────
 * ISR: chỉ đẩy GPIO number vào queue, KHÔNG xử lý
 * Debounce thực sự được làm trong sensor_task
 * ──────────────────────────────────────────────── */
static void IRAM_ATTR btn_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(btn_queue, &gpio_num, NULL);
}

/* ── Khởi tạo Button và Buzzer ── */
static void button_buzzer_init(void) {
    /* Buzzer: output, đặt TẮT ngay để tránh kêu lúc boot */
    gpio_config_t buz_conf = {
        .pin_bit_mask = 1ULL << BUZZER_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&buz_conf);
    gpio_set_level(BUZZER_PIN, BUZZER_OFF);

    /* Buttons: input pull-up nội, ngắt cạnh xuống (nhấn = GND) */
    btn_queue = xQueueCreate(5, sizeof(uint32_t));

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BTN_A_PIN) | (1ULL << BTN_B_PIN) | (1ULL << BTN_C_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_A_PIN, btn_isr_handler, (void *)BTN_A_PIN);
    gpio_isr_handler_add(BTN_B_PIN, btn_isr_handler, (void *)BTN_B_PIN);
    gpio_isr_handler_add(BTN_C_PIN, btn_isr_handler, (void *)BTN_C_PIN);

    ESP_LOGI(TAG, "Buttons A/B/C + Buzzer initialized");
}

/* ── Màn hình ID (thay magic number) ── */
typedef enum {
    SCREEN_DEVICE_ERROR = 0,
    SCREEN_NO_FINGER    = 1,
    SCREEN_WAVEFORM     = 2,
    SCREEN_SPLASH       = 3,
    SCREEN_INTRO        = 4,
    SCREEN_STATS        = 5,
} screen_msg_t;

/* ── Print digit ── */
static void print_digit(ssd1306_t *dev, int x, int y, long val, char c, uint8_t field, int big) {
    uint8_t ff = field;
    do {
        char ch = (val != 0) ? (val % 10 + '0') : c;
        ssd1306_draw_char(dev, x + big * (ff - 1) * 6, y, ch, big);
        val = val / 10;
        --ff;
    } while (ff > 0);
}

/* ── Draw OLED screen ── */
static void draw_oled(screen_msg_t msg) {
    ssd1306_first_page(&oled);
    do {
        switch (msg) {
            case SCREEN_DEVICE_ERROR:
                ssd1306_draw_str(&oled, 10, 0, "Device error", 1);
                break;

            case SCREEN_NO_FINGER:
                ssd1306_draw_rect(&oled, 0, 0, 128, 32);
                ssd1306_draw_str(&oled, 12, 12, "PLACE FINGER", 1);
                ssd1306_draw_xbmp(&oled, 100, 8, 16, 16, heart_bits);
                break;

            case SCREEN_WAVEFORM:
                /* Waveform trái + stats phải */
                waveform_draw(&wave, &oled, 0);
                ssd1306_draw_vline(&oled, 64, 0, 32);
                ssd1306_draw_xbmp(&oled, 68, 0, 16, 16, heart_bits);
                print_digit(&oled, 86, 4, health_bpm_i(), ' ', 3, 1);
                ssd1306_draw_str(&oled, 106, 4, "BPM", 1);
                ssd1306_draw_str(&oled, 68, 20, "SpO2:", 1);
                print_digit(&oled, 100, 20, health_spo2_i(), ' ', 3, 1);
                ssd1306_draw_char(&oled, 120, 20, '%', 1);
                break;

            case SCREEN_SPLASH:
                ssd1306_draw_rect(&oled, 0, 0, 128, 32);
                ssd1306_draw_str(&oled, 37, 4, "NHOM ODIN", 1);
                ssd1306_draw_hline(&oled, 15, 15, 98);
                ssd1306_draw_str(&oled, 25, 20, "Heart Monitor", 1);
                break;

            case SCREEN_INTRO:
                ssd1306_draw_str(&oled, 4, 6, "THIET BI DO NHIP TIM", 1);
                ssd1306_draw_str(&oled, 16, 18, "& NONG DO SpO2", 1);
                break;

            case SCREEN_STATS: {
                /* Màn hình thống kê (Button A) */
                int  avg = (bpm_count > STATS_WARMUP_BEATS) ? (int)(bpm_sum / (bpm_count - STATS_WARMUP_BEATS)) : 0;
                char buf[16];

                /* Dòng 1: Tiêu đề */
                ssd1306_draw_str(&oled, 30, 0, "THONG KE", 1);

                /* Dòng 2: BPM trung bình */
                ssd1306_draw_str(&oled, 0, 12, "AVG HR:", 1);
                if (avg > 0) {
                    snprintf(buf, sizeof(buf), "%d BPM", avg);
                } else {
                    snprintf(buf, sizeof(buf), "-- BPM");
                }
                ssd1306_draw_str(&oled, 48, 12, buf, 1);

                /* Dòng 3: SpO2 + trạng thái mute */
                ssd1306_draw_str(&oled, 0, 24, "SpO2:", 1);
                snprintf(buf, sizeof(buf), "%d%%", health_spo2_i());
                ssd1306_draw_str(&oled, 34, 24, buf, 1);
                if (buzzer_muted) {
                    ssd1306_draw_str(&oled, 90, 24, "[MUT]", 1);
                }
                break;
            }
        }
    } while (ssd1306_next_page(&oled));
}

/* ── Main sensor loop task ── */
static void sensor_task(void *pvParameters) {
    long lastBeat        = 0;
    long displaytime     = 0;
    long last_report_time = 0;
    long finger_start_time = 0;
    uint8_t sleep_counter = 0;
    bool firstBeat       = true;
    long last_sample_time = get_millis();

    while (1) {

        /* ════════════════════════════════════════
         * XỬ LÝ BUTTON (đầu mỗi vòng lặp)
         * Debounce: bỏ qua nếu < DEBOUNCE_MS so với lần nhấn trước
         * ════════════════════════════════════════ */
        uint32_t gpio_num;
        if (xQueueReceive(btn_queue, &gpio_num, 0) == pdTRUE) {
            long now_btn = get_millis();
            int idx = (gpio_num == BTN_A_PIN) ? 0 :
                    (gpio_num == BTN_B_PIN) ? 1 : 2;

            if (now_btn - btn_last_ms[idx] > DEBOUNCE_MS) {
                /*
                * Xác nhận chân vẫn ở mức LOW.
                * Tránh bounce lúc nhả nút.
                */
                if (gpio_get_level((gpio_num_t)gpio_num) == 0) {
                    btn_last_ms[idx] = now_btn;

                    if (gpio_num == BTN_A_PIN) {
                        /* Button A: chuyển màn hình */
                        cur_screen = (cur_screen == 0) ? 1 : 0;
                        ESP_LOGI(TAG, "BTN_A: screen=%d", cur_screen);

                    } else if (gpio_num == BTN_B_PIN) {
                        /*
                        * Button B:
                        * Tắt OLED thì giữ tắt mãi.
                        * Bật OLED thì giữ bật mãi.
                        * Đồng bộ Web + RainMaker.
                        */
                        set_oled_state(!oled_on, "button");
                        ESP_LOGI(TAG, "BTN_B: oled_on=%d", oled_on);

                    } else if (gpio_num == BTN_C_PIN) {
                        /*
                        * Button C:
                        * Nếu quyền còi đang Bật  => tắt còi
                        * Nếu quyền còi đang Tắt  => bật còi
                        * Đồng bộ Web + RainMaker.
                        */
                        set_buzzer_permission(buzzer_muted, "button");
                        ESP_LOGI(TAG, "BTN_C: buzzer permission=%s",
                                buzzer_muted ? "OFF" : "ON");
                    }

                    /* Xả hết bounce events còn trong queue */
                    {
                        uint32_t discard;
                        while (xQueueReceive(btn_queue, &discard, 0) == pdTRUE) {
                        }
                    }
                }
            }
        }

        /* ════════════════════════════════════════
         * BUZZER TICK: tạo âm thanh bíp bíp
         * Chỉ hoạt động khi đang alert và không bị mute
         * Tốc độ bíp: SpO2 thấp = 800ms, BPM bất thường = 400ms
         * ════════════════════════════════════════ */
        if (buzzer_alerting && !buzzer_muted) {
            long now_buz = get_millis();
            int interval = 800;

            health_data.alarm_status = true;

            if (now_buz - buzzer_last_toggle > interval) {
                buzzer_last_toggle = now_buz;
                buzzer_phase = !buzzer_phase;
                gpio_set_level(BUZZER_PIN, buzzer_phase ? BUZZER_ON : BUZZER_OFF);
            }
        } else {
            health_data.alarm_status = false;
            buzzer_phase = false;
            gpio_set_level(BUZZER_PIN, BUZZER_OFF);
        }

        /* ════════════════════════════════════════
         * SENSOR LOOP (giữ nguyên logic cũ)
         * ════════════════════════════════════════ */
        max30102_check(&sensor);
        long now = get_millis();
        health_data.spo2_threshold = app_rainmaker_get_spo2_alert_threshold();

        if (!max30102_available(&sensor)) {
            if (now - last_sample_time > 2000) {
                ESP_LOGW(TAG, "Sensor timeout, re-initializing...");
                max30102_setup(&sensor);
                last_sample_time = now;
            }
            taskYIELD();
            continue;
        }
        last_sample_time = now;

        uint32_t irValue  = max30102_get_ir(&sensor);
        uint32_t redValue = max30102_get_red(&sensor);
        max30102_next_sample(&sensor);

        if (irValue < 5000) {
            /* Không có ngón tay */
            firstBeat = true;
            health_data.bpm = 0;
            health_data.spo2 = 0;
            health_data.waveform = (int)irValue;
            health_data.finger_detected = false;
            health_data.alarm_status = false;
            strlcpy(health_data.signal_quality, signal_quality_from_ir(irValue),
                    sizeof(health_data.signal_quality));
            health_update_text_status();
            finger_start_time = 0;
            health_set_led(false); /* Tắt LED khi không đo */
            sync_rainmaker_extra_status();

            /* Tắt buzzer khi không đo */
            if (buzzer_alerting) {
                buzzer_alerting = false;
                health_data.alarm_status = false;
                gpio_set_level(BUZZER_PIN, BUZZER_OFF);
                sync_rainmaker_extra_status();
            }

            if (oled_on) {
                draw_oled(sleep_counter <= 50 ? SCREEN_NO_FINGER : SCREEN_INTRO);
            }

            if (now - last_report_time > WEB_TELEMETRY_MS) {
                last_report_time = now;
                app_rainmaker_update_data(health_bpm_i(), health_spo2_i(),
                                          health_data.finger_detected,
                                          health_data.health_status);
                web_mqtt_publish_telemetry();
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            ++sleep_counter;
            if (sleep_counter > 100) sleep_counter = 0;

        } else {
            if (finger_start_time == 0) {
                finger_start_time = now;
            }
            health_data.finger_detected = true;
            health_data.spo2_threshold = app_rainmaker_get_spo2_alert_threshold();
            strlcpy(health_data.signal_quality, signal_quality_from_ir(irValue),
                    sizeof(health_data.signal_quality));
            health_update_text_status();
            health_set_led(true); /* Sáng LED khi đang đo */
            sync_rainmaker_extra_status();
            sleep_counter = 0;
            int16_t IR_signal, Red_signal;
            bool beatRed, beatIR;

            if (!filter_for_graph) {
                IR_signal  = pulse_dc_filter(&pulseIR, irValue);
                Red_signal = pulse_dc_filter(&pulseRed, redValue);
                beatRed = pulse_is_beat(&pulseRed, pulse_ma_filter(&pulseRed, Red_signal));
                beatIR  = pulse_is_beat(&pulseIR,  pulse_ma_filter(&pulseIR, IR_signal));
            } else {
                IR_signal  = pulse_ma_filter(&pulseIR,  pulse_dc_filter(&pulseIR, irValue));
                Red_signal = pulse_ma_filter(&pulseRed, pulse_dc_filter(&pulseRed, redValue));
                beatRed = pulse_is_beat(&pulseRed, Red_signal);
                beatIR  = pulse_is_beat(&pulseIR,  IR_signal);
            }

            health_data.waveform = draw_Red ? (int)(-Red_signal) : (int)(-IR_signal);
            waveform_record(&wave, draw_Red ? -Red_signal : -IR_signal);

            if (draw_Red ? beatRed : beatIR) {
                if (firstBeat) {
                    firstBeat = false;
                    lastBeat  = now;
                } else {
                    long delta = now - lastBeat;
                    if (delta > 300 && delta < 2000) {
                        long btpm = 60000 / delta;
                        if (health_bpm_i() == 0) {
                            for (int i = 0; i < NSLOT; i++)
                                bpm_filter.buffer[i] = (int16_t)btpm;
                        }
                        health_data.bpm = ma_filter_apply(&bpm_filter, (int16_t)btpm);

                        /* Cập nhật thống kê
                         * Bỏ qua STATS_WARMUP_BEATS nhịp đầu
                         * vì những nhịp đầu thường chưa ổn định */
                        if (health_bpm_i() > 0) {
                            if (bpm_count < 100000) { /* chống tràn */
                                bpm_count++;
                                if (bpm_count > STATS_WARMUP_BEATS) {
                                    bpm_sum += health_bpm_i();
                                }
                            }
                        }
                    }
                    lastBeat = now;
                }

                /* SpO2 */
                int64_t ac_red = pulse_avg_ac(&pulseRed);
                int64_t dc_red = pulse_avg_dc(&pulseRed);
                int64_t ac_ir  = pulse_avg_ac(&pulseIR);
                int64_t dc_ir  = pulse_avg_dc(&pulseIR);

                int64_t numerator   = (ac_red * dc_ir) / 256;
                int64_t denominator = (dc_red * ac_ir) / 256;

                if (denominator > 0) {
                    int RX100 = (int)((numerator * 100) / denominator);
                    if (RX100 >= 0 && RX100 < 184)
                        health_data.spo2 = spo2_table[RX100];
                }

                health_update_text_status();
                ESP_LOGD(TAG, "BPM=%d SPO2=%d", health_bpm_i(), health_spo2_i());

                /* ── Kiểm tra ngưỡng cảnh báo ── */
                bool need_alert = false;
                /* Bỏ qua cảnh báo trong 8 giây đầu để chờ giá trị ổn định */
                if (now - finger_start_time > SENSOR_STABLE_TIME_MS) {
                    need_alert = is_spo2_valid(health_spo2_i()) &&
                                 (health_spo2_i() < health_data.spo2_threshold);
                }
            }

            /* Cập nhật màn hình mỗi 50ms */
            /* Cập nhật cảnh báo mỗi vòng lặp để ngưỡng chỉnh từ RainMaker có hiệu lực ngay. */
            bool need_spo2_alert = false;
            int spo2_threshold = app_rainmaker_get_spo2_alert_threshold();
            health_data.spo2_threshold = spo2_threshold;

            if (now - finger_start_time > SENSOR_STABLE_TIME_MS) {
                need_spo2_alert = is_spo2_valid(health_spo2_i()) && (health_spo2_i() < spo2_threshold);
            }

            if (need_spo2_alert && !buzzer_alerting) {
                buzzer_alerting = true;

                /*
                * Không tự bật lại quyền còi.
                * Nếu người dùng đã tắt còi, buzzer_muted vẫn giữ true.
                */
                buzzer_phase = false;
                buzzer_last_toggle = now;

                health_data.alarm_status = !buzzer_muted;

                health_update_text_status();
                sync_rainmaker_extra_status();

                ESP_LOGW(TAG, "ALERT! SPO2=%d threshold=%d, muted=%d",
                        health_spo2_i(), spo2_threshold, buzzer_muted);
            } else if (!need_spo2_alert && buzzer_alerting) {
                /*
                * Hết cảnh báo nhưng không đổi quyền còi.
                */
                buzzer_alerting = false;
                buzzer_phase = false;
                health_data.alarm_status = false;
                gpio_set_level(BUZZER_PIN, BUZZER_OFF);

                health_update_text_status();
                sync_rainmaker_extra_status();

                ESP_LOGI(TAG, "Alert cleared, buzzer permission=%s",
                        buzzer_muted ? "OFF" : "ON");
            }

            if (now - displaytime > 50) {
                displaytime = now;
                waveform_scale(&wave);
                if (oled_on) {
                    draw_oled(cur_screen == 0 ? SCREEN_WAVEFORM : SCREEN_STATS);
                }
            }

            /* Web cập nhật nhanh; RainMaker được giới hạn trong app_rainmaker.c để tránh MQTT Budget */
            if (now - last_report_time > WEB_TELEMETRY_MS) {
                last_report_time = now;
                app_rainmaker_update_data(health_bpm_i(), health_spo2_i(),
                                          health_data.finger_detected,
                                          health_data.health_status);
                web_mqtt_publish_telemetry();
            }
        }
    }
}

/* ── app_main ── */
void app_main(void) {
    ESP_LOGI(TAG, "Heart Rate & SpO2 Monitor starting...");
    setenv("TZ", "ICT-7", 1);
    tzset();

    /* Init LED */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);

    /* Init Button & Buzzer */
    button_buzzer_init();

    /* Init I2C */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = I2C_SDA_PIN,
        .scl_io_num        = I2C_SCL_PIN,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized (SDA=%d, SCL=%d)", I2C_SDA_PIN, I2C_SCL_PIN);

    /* Init OLED */
    ssd1306_init(&oled, bus_handle);
    ssd1306_fill(&oled, 0x00);

    /* Init filters */
    pulse_init(&pulseIR);
    pulse_init(&pulseRed);
    ma_filter_init(&bpm_filter);
    waveform_init(&wave);

    /* Splash screen */
    draw_oled(SCREEN_SPLASH);
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Init MAX30102 */
    if (!max30102_init(&sensor, bus_handle)) {
        ESP_LOGE(TAG, "MAX30102 not found!");
        draw_oled(SCREEN_DEVICE_ERROR);
        app_rainmaker_update_data(0, 0, false, "Lỗi cảm biến");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    max30102_setup(&sensor);
    ESP_LOGI(TAG, "MAX30102 initialized, starting measurement...");

    /* Tạo sensor task */
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    /* Init RainMaker */
    esp_err_t rm_err = app_rainmaker_init();
    if (rm_err != ESP_OK) {
        ESP_LOGW(TAG, "RainMaker init failed (%s), continuing without cloud",
                 esp_err_to_name(rm_err));
    } else {
        /* Đẩy trạng thái ban đầu lên RainMaker ngay sau khi cloud/device đã tạo xong.
           Nếu không có dòng này, các param phụ dễ bị kẹt ở giá trị mặc định OFF. */
        sync_rainmaker_extra_status();
        app_rainmaker_update_data(
            health_bpm_i(),
            health_spo2_i(),
            health_data.finger_detected,
            health_data.health_status
        );
    }

    /* MQTT dashboard chạy sau khi network/RainMaker đã khởi động. */
    web_mqtt_start();
}
