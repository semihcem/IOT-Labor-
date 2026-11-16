// =======================================================================
// rtc_log.h - GIVEN measurement log in RTC memory for Lab 07 / Lab 08
//
// Copy this file into the main/ folder of your project and add
//     #include "rtc_log.h"
// to main.c. You do NOT need to change it. Read it to answer:
//   - Where is the log placed, and why does it survive deep sleep?
//   - How much memory does it need?
//   - What is the magic number good for?
//
// Header-only: everything is "static", so no change in CMakeLists.txt is
// necessary. Include it in ONE .c file only (main.c).
// =======================================================================
#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>         // memset()
#include <math.h>           // lroundf()
#include <sys/time.h>       // gettimeofday()
#include "esp_attr.h"       // RTC_DATA_ATTR
#include "esp_log.h"

#define RTC_LOG_SIZE   600              // ~5 h at 30 s intervals
#define RTC_LOG_MAGIC  0x4C414237       // "LAB7", any fixed value

// One measurement. Values are stored as scaled integers instead of float:
// 2 bytes instead of 4 per value -> more entries fit into the 8 KB RTC memory.
typedef struct {
    uint32_t timestamp_s;   // seconds since power-on (or Unix time once the clock is synced)
    int16_t  temp_x100;     // 23.67 °C -> 2367
    uint16_t hum_x100;      // 46.60 %  -> 4660
    uint16_t battery_mv;    // 0 if the ADC read failed
    uint8_t  motion;        // radar level during the measurement
} measurement_t;            // 12 bytes incl. padding

// Rolling (ring) buffer: when it is full, the oldest entry is overwritten.
typedef struct {
    uint32_t magic;         // == RTC_LOG_MAGIC if the content is valid
    uint32_t total;         // number of measurements since power-on (keeps counting)
    uint16_t head;          // next free slot (= next write position)
    uint16_t count;         // valid entries (<= RTC_LOG_SIZE)
    measurement_t entries[RTC_LOG_SIZE];
} rtc_log_t;                // 600 * 12 B + 12 B = 7212 B of 8192 B RTC slow memory

// RTC_DATA_ATTR places the variable in RTC slow memory: it stays powered in deep sleep.
// On power-on, reset or flashing, the bootloader sets it back to its initial value (all 0).
RTC_DATA_ATTR static rtc_log_t s_rtc_log;

static const char *RTC_LOG_TAG = "rtc_log";

// Current system time in µs. It is driven by the RTC timer and keeps running in deep sleep.
static inline int64_t now_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

// Call once at the start of app_main(). If the magic number does not match (power-on,
// reset, new firmware, or random content after a power loss), the log is cleared.
static inline void rtc_log_check(void)
{
    if (s_rtc_log.magic != RTC_LOG_MAGIC) {
        memset(&s_rtc_log, 0, sizeof(s_rtc_log));
        s_rtc_log.magic = RTC_LOG_MAGIC;
    }
}

// Stores one measurement with the current time.
static inline void rtc_log_add_values(float temperature, float humidity, int battery_mv, int motion)
{
    const measurement_t m = {
        .timestamp_s = (uint32_t) (now_us() / 1000000),
        .temp_x100 = (int16_t) lroundf(temperature * 100.0f),     // round, do not truncate
        .hum_x100 = (uint16_t) lroundf(humidity * 100.0f),
        .battery_mv = (uint16_t) battery_mv,
        .motion = (uint8_t) motion,
    };
    s_rtc_log.entries[s_rtc_log.head] = m;
    s_rtc_log.head = (s_rtc_log.head + 1) % RTC_LOG_SIZE;     // ring: back to 0 after the last slot
    if (s_rtc_log.count < RTC_LOG_SIZE) {
        s_rtc_log.count++;
    }
    s_rtc_log.total++;
}

// Number of measurements since power-on (also the ones that were already overwritten).
static inline uint32_t rtc_log_total(void)
{
    return s_rtc_log.total;
}

// Measurement number seq (0 = first measurement since power-on, total-1 = newest).
// Returns false if it does not exist (yet) or was already overwritten by newer ones.
static inline bool rtc_log_get(uint32_t seq, measurement_t *m)
{
    if (seq >= s_rtc_log.total || seq < s_rtc_log.total - s_rtc_log.count) {
        return false;
    }
    const uint32_t back = s_rtc_log.total - seq;                // 1 = newest entry
    *m = s_rtc_log.entries[(s_rtc_log.head - back + RTC_LOG_SIZE) % RTC_LOG_SIZE];
    return true;
}

// Prints the newest n entries, oldest of them first.
static inline void rtc_log_print(int n)
{
    if (n > s_rtc_log.count) {
        n = s_rtc_log.count;
    }
    ESP_LOGI(RTC_LOG_TAG, "%u entries stored, %lu measurements in total, newest %d:",
             s_rtc_log.count, (unsigned long) s_rtc_log.total, n);
    for (uint32_t seq = s_rtc_log.total - n; seq < s_rtc_log.total; seq++) {
        measurement_t m;
        rtc_log_get(seq, &m);
        ESP_LOGI(RTC_LOG_TAG, "  #%-5lu t=%10lu s  %5.2f degC  %5.2f %%  motion: %u  battery: %u mV",
                 (unsigned long) seq, (unsigned long) m.timestamp_s, m.temp_x100 / 100.0,
                 m.hum_x100 / 100.0, m.motion, m.battery_mv);
    }
}
