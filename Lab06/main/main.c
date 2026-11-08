/*
 * Lab 06 - Multitasking und Batteriemessung (Step 1)
 *
 * sensor_task  (Prio 4): jede 1 s AHT20 lesen
 * display_task (Prio 3): alle 500 ms Werte anzeigen
 */

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "aht20.h"

#define I2C_SCL_PIN            40
#define I2C_SDA_PIN            41
#define I2C_FREQ_HZ            100000

#define SENSOR_PERIOD_MS       1000
#define DISPLAY_PERIOD_MS      500

#define TASK_STACK_SIZE        4096
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
} sensor_readings_t;

static sensor_readings_t g_readings;
static SemaphoreHandle_t g_readings_mutex;

static i2c_bus_handle_t g_i2c_bus;
static aht20_dev_handle_t g_aht20;

static lv_obj_t *g_label;

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

/* ---------- Display ---------- */

static void ui_init(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_label_set_text(title, "Sensor Bay");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 8);

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

        // langsame Messungen ausserhalb des Mutex
        esp_err_t aht_err = aht20_read_temperature_humidity(g_aht20, &temp_raw, &temperature,
                                                            &hum_raw, &humidity);

        // nur kopieren im Mutex
        xSemaphoreTake(g_readings_mutex, portMAX_DELAY);
        g_readings.temperature = temperature;
        g_readings.humidity = humidity;
        g_readings.valid = (aht_err == ESP_OK);
        xSemaphoreGive(g_readings_mutex);

        if (aht_err == ESP_OK) {
            ESP_LOGI(TAG, "T=%.2f C  H=%.2f %%", temperature, humidity);
        } else {
            ESP_LOGW(TAG, "AHT20 Fehler: %s", esp_err_to_name(aht_err));
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

static void display_task(void *arg)
{
    char text[96];

    while (1) {
        // Schnappschuss im Mutex
        xSemaphoreTake(g_readings_mutex, portMAX_DELAY);
        sensor_readings_t snap = g_readings;
        xSemaphoreGive(g_readings_mutex);

        // Text ausserhalb formatieren
        if (snap.valid) {
            snprintf(text, sizeof(text), "Temperatur: %.1f °C\nFeuchte: %.1f %%",
                     snap.temperature, snap.humidity);
        } else {
            snprintf(text, sizeof(text), "Temperatur: Error\nFeuchte: Error");
        }

        bsp_display_lock(0);
        lv_label_set_text(g_label, text);
        bsp_display_unlock();

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

void app_main(void)
{
    bsp_display_start();
    bsp_display_backlight_on();
    ui_init();

    aht20_init();

    g_readings_mutex = xSemaphoreCreateMutex();
    assert(g_readings_mutex != NULL);

    BaseType_t ok;
    ok = xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIO, NULL);
    assert(ok == pdPASS);
    ok = xTaskCreate(display_task, "display_task", TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIO, NULL);
    assert(ok == pdPASS);
}
