// =======================================================================
// cloud_port.h - GIVEN helpers for Lab 08 (ESP RainMaker)
//
// Copy this file into the main/ folder of your project (next to rtc_log.h)
// and add
//     #include "cloud_port.h"
// to main.c, AFTER #include "rtc_log.h". You do NOT need to change it.
//
// It contains everything around the cloud that is not the topic of this lab:
//   - the event group g_cloud_events (cloud state for your tasks)
//   - the upload state s_cloud in RTC memory (survives deep sleep, like the log)
//   - the clock jump after the SNTP time sync (Unix timestamps for the log)
//   - one deadline for the cloud job (a missing Wi-Fi network must not drain the battery)
//   - the provisioning QR code on the display
// Header-only: include it in ONE .c file only (main.c).
// =======================================================================
#pragma once

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "app_network.h"
#include "esp_rmaker_common_events.h"
#include "rtc_log.h"            // measurement_t, now_us()

static const char *CLOUD_PORT_TAG = "cloud_port";

// ---------------------------------------------------------------- cloud state
// EVENT GROUP: several independent flags that tasks and event handlers set and wait for
#define MQTT_CONNECTED_BIT  BIT0    // set/cleared by the event handler below
#define CLOUD_BUSY_BIT      BIT1    // you set it: provisioning or upload running -> do not sleep
#define SYNC_DONE_BIT       BIT2    // you set it: cloud_task has finished its job
static EventGroupHandle_t g_cloud_events;

// Upload state in RTC memory: survives deep sleep (cleared on power-on / reset, like the log)
typedef struct {
    uint32_t uploaded_total;    // measurements 0 .. uploaded_total-1 are in the cloud
    uint32_t next_upload_at;    // next batch upload when rtc_log_total() reaches this value
    uint32_t epoch_offset_s;    // size of the clock jump at the time sync (0 = not known yet)
    uint8_t  cloud_ready;       // 1 = box was provisioned and online at least once
} cloud_state_t;
RTC_DATA_ATTR static cloud_state_t s_cloud;

// ---------------------------------------------------------------- clock jump
// When RainMaker sets the clock via SNTP, gettimeofday() jumps from "seconds since
// power-on" to Unix time (~1.7 billion s). Entries measured before the jump need the
// size of the jump to get a correct Unix timestamp.
#define EPOCH_VALID_S   1700000000LL        // Nov 2023: earlier values are "since power-on"
#define EPOCH_VALID_US  (EPOCH_VALID_S * 1000000LL)

static int64_t s_boot_rel_us;               // gettimeofday() at start-up
static int64_t s_boot_timer_us;             // esp_timer_get_time() at start-up (never jumps)

// Call it as the FIRST line of app_main().
static inline void cloud_port_init(void)
{
    s_boot_rel_us = now_us();
    s_boot_timer_us = esp_timer_get_time();
    g_cloud_events = xEventGroupCreate();
    assert(g_cloud_events != NULL);
}

// Stores the size of the clock jump once, in RTC memory. Call it before an upload and
// before deep sleep (s_boot_rel_us is lost in deep sleep).
static inline void cloud_time_update(void)
{
    if (s_cloud.epoch_offset_s != 0 || now_us() < EPOCH_VALID_US || s_boot_rel_us >= EPOCH_VALID_US) {
        return;     // already known, clock not synced yet, or the jump happened in an earlier boot
    }
    // where the clock would be without the jump: start-up time + monotonic time since start-up
    const int64_t expected_us = s_boot_rel_us + (esp_timer_get_time() - s_boot_timer_us);
    s_cloud.epoch_offset_s = (uint32_t) ((now_us() - expected_us) / 1000000);
    ESP_LOGI(CLOUD_PORT_TAG, "Clock synced: jump of %lu s stored", (unsigned long) s_cloud.epoch_offset_s);
}

// Unix timestamp of a log entry, or 0 if it cannot be determined (do not upload it then).
static inline uint32_t entry_unix_time(const measurement_t *m)
{
    if (m->timestamp_s >= EPOCH_VALID_S) {
        return m->timestamp_s;                  // measured after the sync: already Unix time
    }
    if (s_cloud.epoch_offset_s == 0) {
        return 0;                               // measured before a jump we do not know
    }
    return m->timestamp_s + s_cloud.epoch_offset_s;
}

// ---------------------------------------------------------------- deadline
// app_network_start() waits FOREVER for the Wi-Fi network. So the whole cloud job gets ONE
// deadline: cloud_task, power_task and app_main wait at most until then. A missing Wi-Fi
// network must not keep the box awake and drain the battery.
#if CONFIG_APP_NETWORK_PROV_TIMEOUT_PERIOD <= 0
#error "Set CONFIG_APP_NETWORK_PROV_TIMEOUT_PERIOD > 0 (sdkconfig.defaults), otherwise display mode has no deadline"
#endif
#define PROV_TIMEOUT_MS  (CONFIG_APP_NETWORK_PROV_TIMEOUT_PERIOD * 60 * 1000)  // provisioning time (sdkconfig)

static TickType_t s_cloud_deadline;         // tick count at which the cloud job gives up

// Sets the deadline to timeout_ms from now. Call it at the beginning of cloud_start().
static inline void cloud_deadline_set(uint32_t timeout_ms)
{
    s_cloud_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
}

// Remaining time until the deadline in ticks (0 = deadline passed).
// Use it as the timeout of every wait in the cloud job.
static inline TickType_t cloud_ticks_left(void)
{
    const TickType_t now = xTaskGetTickCount();
    // signed difference: still correct when the tick counter overflows
    return ((int32_t) (s_cloud_deadline - now) > 0) ? (s_cloud_deadline - now) : 0;
}

// Switches the radio off before deep sleep. Harmless if Wi-Fi was never started.
static inline void cloud_wifi_stop(void)
{
    esp_wifi_stop();
}

// ---------------------------------------------------------------- QR code + events
static bool s_cloud_port_display;           // QR code only if the display is running
static lv_obj_t *s_qr_screen;               // white overlay with the provisioning QR code

// app_network posts the QR payload as an event -> draw it with LVGL on top of the screen
// (the ASCII QR code in the serial monitor is easily missed after a reset)
static inline void cloud_port_qr_show(const char *payload)
{
    if (!s_cloud_port_display) {
        return;                             // timer mode: no display
    }
    bsp_display_lock(0);
    if (s_qr_screen == NULL) {
        s_qr_screen = lv_obj_create(lv_layer_top());    // top layer: display_task cannot overwrite it
        lv_obj_set_size(s_qr_screen, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(s_qr_screen, lv_color_white(), 0);
        lv_obj_set_style_border_width(s_qr_screen, 0, 0);
        lv_obj_set_style_radius(s_qr_screen, 0, 0);

        lv_obj_t *qr = lv_qrcode_create(s_qr_screen);
        lv_qrcode_set_size(qr, 180);
        lv_qrcode_set_dark_color(qr, lv_color_black());
        lv_qrcode_set_light_color(qr, lv_color_white());
        lv_qrcode_update(qr, payload, strlen(payload));
        lv_obj_align(qr, LV_ALIGN_TOP_MID, 0, 10);

        lv_obj_t *hint = lv_label_create(s_qr_screen);
        lv_label_set_text(hint, "Scan with the ESP RainMaker app");
        lv_obj_set_style_text_color(hint, lv_color_black(), 0);
        lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -5);
    }
    bsp_display_backlight_on();
    bsp_display_unlock();
}

// Removes the QR code (call it when app_network_start() has returned = online)
static inline void cloud_port_qr_hide(void)
{
    if (!s_cloud_port_display) {
        return;
    }
    bsp_display_lock(0);
    if (s_qr_screen != NULL) {
        lv_obj_delete(s_qr_screen);
        s_qr_screen = NULL;
    }
    bsp_display_unlock();
}

// Event handlers run in the default event loop task: keep them short
static void cloud_port_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == APP_NETWORK_EVENT && id == APP_NETWORK_EVENT_QR_DISPLAY) {
        ESP_LOGI(CLOUD_PORT_TAG, "Provisioning: QR code on the display");
        cloud_port_qr_show((const char *) data);
    } else if (base == APP_NETWORK_EVENT && id == APP_NETWORK_EVENT_PROV_TIMEOUT) {
        ESP_LOGW(CLOUD_PORT_TAG, "Provisioning timed out");
        cloud_port_qr_hide();
        xEventGroupClearBits(g_cloud_events, CLOUD_BUSY_BIT);   // allow deep sleep again
    } else if (base == RMAKER_COMMON_EVENT && id == RMAKER_MQTT_EVENT_CONNECTED) {
        ESP_LOGI(CLOUD_PORT_TAG, "RainMaker MQTT connected");
        xEventGroupSetBits(g_cloud_events, MQTT_CONNECTED_BIT);
    } else if (base == RMAKER_COMMON_EVENT && id == RMAKER_MQTT_EVENT_DISCONNECTED) {
        ESP_LOGW(CLOUD_PORT_TAG, "RainMaker MQTT disconnected");
        xEventGroupClearBits(g_cloud_events, MQTT_CONNECTED_BIT);
    }
}

// Registers the event handlers. Call it in cloud_start() AFTER app_network_init()
// (that creates the default event loop). with_display: may the QR code be drawn?
static inline void cloud_port_register_events(bool with_display)
{
    s_cloud_port_display = with_display;
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT, ESP_EVENT_ANY_ID, cloud_port_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, cloud_port_event_handler, NULL));
}
