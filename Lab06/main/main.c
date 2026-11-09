/*
 * Lab 06 - Multitasking und Batteriemessung (Step 3)
 *
 * power_task   (Prio 5): Radar-Interrupt -> Backlight an, 10 s keine Bewegung -> aus
 * sensor_task  (Prio 4): jede 1 s AHT20 + Batterie lesen
 * display_task (Prio 3): alle 500 ms Werte anzeigen
 */

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"

#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "aht20.h"

#define I2C_SCL_PIN            40
#define I2C_SDA_PIN            41
#define I2C_FREQ_HZ            100000
#define RADAR_PIN              GPIO_NUM_21

#define BATTERY_ADC_UNIT       ADC_UNIT_1
#define BATTERY_ADC_CHANNEL    ADC_CHANNEL_9   // GPIO10
#define BATTERY_ADC_ATTEN      ADC_ATTEN_DB_12
#define BATTERY_DIVIDER        4
#define BATTERY_SAMPLES        16

#define SENSOR_PERIOD_MS       1000
#define DISPLAY_PERIOD_MS      500
#define INACTIVITY_TIMEOUT_MS  10000

#define TASK_STACK_SIZE        4096
#define POWER_TASK_PRIO        5
#define SENSOR_TASK_PRIO       4
#define DISPLAY_TASK_PRIO      3

// Das BSP belegt einen I2C-Port fuer Touch/Audio, der Dock-Bus (40/41) liegt auf dem anderen
#if BSP_I2C_NUM == 1
#define SENSOR_I2C_PORT        I2C_NUM_0
#else
#define SENSOR_I2C_PORT        I2C_NUM_1
#endif

static const char *TAG = "lab06";

typedef struct {
    float temperature;
    float humidity;
    bool valid;
    int battery_mv;
    int battery_percent;
    bool battery_valid;
} sensor_readings_t;

static sensor_readings_t g_readings;
static SemaphoreHandle_t g_readings_mutex;
static SemaphoreHandle_t g_motion_sem;

static i2c_bus_handle_t g_i2c_bus;
static aht20_dev_handle_t g_aht20;
static adc_oneshot_unit_handle_t g_adc;
static adc_cali_handle_t g_adc_cali;

static lv_obj_t *g_label;
static lv_obj_t *g_battery_label;

/* ---------- AHT20 ---------- */

static void aht20_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = true,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    g_i2c_bus = i2c_bus_create(SENSOR_I2C_PORT, &i2c_conf);
    assert(g_i2c_bus != NULL);

    const aht20_i2c_config_t aht_conf = {
        .bus_inst = g_i2c_bus,
        .i2c_addr = AHT20_ADDRRES_0,   // 0x38
    };
    ESP_ERROR_CHECK(aht20_new_sensor(&aht_conf, &g_aht20));
}

/* ---------- Batterie (ADC) ---------- */

static void battery_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc));

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc, BATTERY_ADC_CHANNEL, &chan_cfg));

    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &g_adc_cali));
}

static esp_err_t battery_read_mv(int *battery_mv)
{
    int sum = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
        int mv = 0;
        esp_err_t err = adc_oneshot_get_calibrated_result(g_adc, g_adc_cali, BATTERY_ADC_CHANNEL, &mv);
        if (err != ESP_OK) {
            return err;
        }
        sum += mv;
    }
    int avg_mv = sum / BATTERY_SAMPLES;   // Spannung am ADC-Pin (ca. 825-1050 mV)
    *battery_mv = avg_mv * BATTERY_DIVIDER;
    return ESP_OK;
}

// Li-ion discharge curve (approx.), linear interpolation between the points
static int battery_percent_from_mv(int mv)
{
    static const struct { int mv; int percent; } curve[] = {
        {4200, 100}, {4100, 90}, {4000, 80}, {3900, 65}, {3800, 50},
        {3700, 30}, {3600, 15}, {3500, 5}, {3300, 0},
    };
    const int n = sizeof(curve) / sizeof(curve[0]);

    if (mv >= curve[0].mv) {
        return 100;
    }
    for (int i = 1; i < n; i++) {
        if (mv >= curve[i].mv) {
            return curve[i].percent + (mv - curve[i].mv) * (curve[i - 1].percent - curve[i].percent)
                                      / (curve[i - 1].mv - curve[i].mv);
        }
    }
    return 0;
}

static const char *battery_symbol(int percent)
{
    if (percent > 80) {
        return LV_SYMBOL_BATTERY_FULL;
    } else if (percent > 55) {
        return LV_SYMBOL_BATTERY_3;
    } else if (percent > 30) {
        return LV_SYMBOL_BATTERY_2;
    } else if (percent > 10) {
        return LV_SYMBOL_BATTERY_1;
    }
    return LV_SYMBOL_BATTERY_EMPTY;
}

/* ---------- Radar-Interrupt ---------- */

static void IRAM_ATTR radar_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_motion_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

static void radar_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << RADAR_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // ISR-Service gibt es nur einmal pro Chip, evtl. schon vom BSP installiert
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADAR_PIN, radar_isr_handler, NULL));
}

/* ---------- Display ---------- */

static void ui_init(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_label_set_text(title, "Sensor Bay");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 8);

    g_battery_label = lv_label_create(scr);
    lv_obj_set_style_text_font(g_battery_label, &lv_font_montserrat_20, 0);
    lv_label_set_text(g_battery_label, LV_SYMBOL_BATTERY_EMPTY " --");
    lv_obj_align(g_battery_label, LV_ALIGN_TOP_RIGHT, -10, 12);

    g_label = lv_label_create(scr);
    lv_obj_set_style_text_font(g_label, &lv_font_montserrat_20, 0);
    lv_label_set_text(g_label, "Warte auf Sensor...");
    lv_obj_align(g_label, LV_ALIGN_LEFT_MID, 10, 15);

    bsp_display_unlock();
}

/* ---------- Tasks ---------- */

static void sensor_task(void *arg)
{
    while (1) {
        float temperature = 0, humidity = 0;
        uint32_t temp_raw, hum_raw;
        int battery_mv = 0;

        // langsame Messungen ausserhalb des Mutex
        esp_err_t aht_err = aht20_read_temperature_humidity(g_aht20, &temp_raw, &temperature,
                                                            &hum_raw, &humidity);
        esp_err_t bat_err = battery_read_mv(&battery_mv);
        int battery_percent = battery_percent_from_mv(battery_mv);

        // nur kopieren im Mutex
        xSemaphoreTake(g_readings_mutex, portMAX_DELAY);
        g_readings.temperature = temperature;
        g_readings.humidity = humidity;
        g_readings.valid = (aht_err == ESP_OK);
        g_readings.battery_mv = battery_mv;
        g_readings.battery_percent = battery_percent;
        g_readings.battery_valid = (bat_err == ESP_OK);
        xSemaphoreGive(g_readings_mutex);

        if (aht_err == ESP_OK) {
            ESP_LOGI(TAG, "T=%.2f C  H=%.2f %%  Bat=%d mV (%d %%)",
                     temperature, humidity, battery_mv, battery_percent);
        } else {
            ESP_LOGW(TAG, "AHT20 Fehler: %s", esp_err_to_name(aht_err));
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

static void display_task(void *arg)
{
    char text[96];
    char bat_text[32];

    while (1) {
        // Schnappschuss im Mutex
        xSemaphoreTake(g_readings_mutex, portMAX_DELAY);
        sensor_readings_t snap = g_readings;
        xSemaphoreGive(g_readings_mutex);

        int motion = gpio_get_level(RADAR_PIN);

        // Text ausserhalb formatieren
        if (snap.valid) {
            snprintf(text, sizeof(text), "Temperatur: %.1f °C\nFeuchte: %.1f %%\nMotion: %d",
                     snap.temperature, snap.humidity, motion);
        } else {
            snprintf(text, sizeof(text), "Temperatur: Error\nFeuchte: Error\nMotion: %d", motion);
        }

        if (snap.battery_valid) {
            snprintf(bat_text, sizeof(bat_text), "%s %d %%",
                     battery_symbol(snap.battery_percent), snap.battery_percent);
        } else {
            snprintf(bat_text, sizeof(bat_text), LV_SYMBOL_BATTERY_EMPTY " --");
        }

        bsp_display_lock(0);
        lv_label_set_text(g_label, text);
        lv_label_set_text(g_battery_label, bat_text);
        bsp_display_unlock();

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

static void power_task(void *arg)
{
    bool backlight_on = true;

    while (1) {
        if (xSemaphoreTake(g_motion_sem, pdMS_TO_TICKS(INACTIVITY_TIMEOUT_MS)) == pdTRUE) {
            // Flanke am Radar -> Aktivitaet
            ESP_LOGI(TAG, "Motion detected (Level %d)", gpio_get_level(RADAR_PIN));
            if (!backlight_on) {
                bsp_display_lock(0);
                bsp_display_backlight_on();
                bsp_display_unlock();
                backlight_on = true;
                ESP_LOGI(TAG, "Display an");
            }
        } else if (gpio_get_level(RADAR_PIN) == 1) {
            // Timeout, aber Person steht still vor der Box -> anlassen
        } else if (backlight_on) {
            bsp_display_lock(0);
            bsp_display_backlight_off();
            bsp_display_unlock();
            backlight_on = false;
            ESP_LOGI(TAG, "10 s keine Bewegung -> Display aus");
        }
    }
}

void app_main(void)
{
    bsp_display_start();
    bsp_display_backlight_on();
    ui_init();

    aht20_init();
    battery_init();

    g_readings_mutex = xSemaphoreCreateMutex();
    assert(g_readings_mutex != NULL);

    // Semaphore vor dem ISR-Handler anlegen
    g_motion_sem = xSemaphoreCreateBinary();
    assert(g_motion_sem != NULL);
    radar_init();

    BaseType_t ok;
    ok = xTaskCreate(power_task, "power_task", TASK_STACK_SIZE, NULL, POWER_TASK_PRIO, NULL);
    assert(ok == pdPASS);
    ok = xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIO, NULL);
    assert(ok == pdPASS);
    ok = xTaskCreate(display_task, "display_task", TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIO, NULL);
    assert(ok == pdPASS);
}
