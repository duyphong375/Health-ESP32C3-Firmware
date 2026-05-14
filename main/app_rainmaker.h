#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t app_rainmaker_init(void);

void app_rainmaker_update_data(
    int bpm,
    int spo2,
    bool finger_detected,
    const char *health_status
);

int app_rainmaker_get_spo2_alert_threshold(void);
void app_rainmaker_set_spo2_alert_threshold(int threshold);

/* Trạng thái hiển thị trên RainMaker */
void app_rainmaker_update_oled_status(bool oled_active);
void app_rainmaker_update_led_status(bool led_on);

/* Quyền còi: Bật/Tắt */
void app_rainmaker_update_alarm_permission(bool alarm_enabled);

/* Còi thực tế: Đang kêu/Không kêu */
void app_rainmaker_update_buzzer_status(bool buzzer_on);

/* Hàm cũ, giữ lại để tránh lỗi nếu main.c còn gọi */
void app_rainmaker_update_alarm_status(bool alarm_on);

/* Hàm gom trạng thái để main.c gọi 1 lần */
void app_rainmaker_update_runtime_status(
    bool oled_on,
    bool led_on,
    bool alarm_permission,
    bool buzzer_real_on
);