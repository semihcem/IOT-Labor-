/*
 * Lab 08 - Cloud-Anbindung mit ESP RainMaker (Step 4)
 *
 * Basis: Lab 07 (Deep Sleep, Timer-Wakeup alle 30 s, RTC-Log)
 * Step 1: RainMaker-Komponenten, Partitionstabelle, NVS
 * Step 2: Provisioning per QR-Code, Geraet "Climate", cloud_task
 * Step 3: RTC-Log mit Original-Zeitstempeln hochladen
 * Step 4: Store-and-Forward - WLAN nur alle 20 Messungen
 */

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>              // lroundf()
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
#include "esp_sleep.h"
#include "esp_bit_defs.h"     // BIT()
#include "driver/rtc_io.h"
#include "nvs_flash.h"

#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "aht20.h"
#include "rtc_log.h"
#include "cloud_port.h"           // nach rtc_log.h
#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_standard_params.h"
#include "app_network.h"
#include "app_reset.h"
#include "esp_rmaker_time_sync.h"

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
#define MEASURE_INTERVAL_S     30

#define MAX_UPLOAD_PER_SYNC   25      // 3 MQTT messages per entry -> 75 of 100 budget tokens
#define UPLOAD_EVERY_N        20      // Step 4: connect after 20 new measurements (= 10 min)
#define CLOUD_FLUSH_MS        2000    // Step 4: give the MQTT client time to send before sleeping
#define CLOUD_GRACE_MS        3000    // Step 4: an upload started just before the deadline may finish

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

// RainMaker-Params des Geraets "Climate" (in Step 3 fuer den Upload gebraucht)
static esp_rmaker_param_t *g_temp_param;
static esp_rmaker_param_t *g_hum_param;
static esp_rmaker_param_t *g_bat_param;

static int boot_count = 0;                      // normaler RAM
RTC_DATA_ATTR static int rtc_boot_count = 0;    // RTC slow memory

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

static void radar_gpio_init(void)
{
    // nach einem EXT0-Wakeup ist der Pin noch im RTC-Modus -> zurueck an den normalen GPIO
    rtc_gpio_deinit(RADAR_PIN);

    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << RADAR_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void radar_isr_init(void)
{
    // ISR-Service gibt es nur einmal pro Chip, evtl. schon vom BSP installiert
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADAR_PIN, radar_isr_handler, NULL));
}

/* ---------- Deep Sleep ---------- */

static void enter_deep_sleep(void)
{
    cloud_time_update();    // remember a clock jump (s_boot_rel_us is lost in deep sleep)
    cloud_wifi_stop();      // switch the radio off in an orderly way

    ESP_LOGI(TAG, "Gehe in Deep Sleep (Timer %d s oder Radar)", MEASURE_INTERVAL_S);

    esp_sleep_enable_timer_wakeup(MEASURE_INTERVAL_S * 1000000ULL);   // Zeit in us
    esp_sleep_enable_ext0_wakeup(RADAR_PIN, 1);                       // aufwachen bei HIGH

    // im Deep Sleep laeuft nur die RTC-Domaene -> Pin an RTC uebergeben und auf LOW ziehen
    gpio_intr_disable(RADAR_PIN);
    rtc_gpio_init(RADAR_PIN);
    rtc_gpio_pullup_dis(RADAR_PIN);
    rtc_gpio_pulldown_en(RADAR_PIN);

    fflush(stdout);
    esp_deep_sleep_start();     // kehrt nicht zurueck
}

// einmal messen und ins RTC-Log schreiben (Timer-Wakeup)
static void measure_once(void)
{
    float temperature = 0, humidity = 0;
    uint32_t temp_raw, hum_raw;
    int battery_mv = 0;

    esp_err_t aht_err = aht20_read_temperature_humidity(g_aht20, &temp_raw, &temperature,
                                                        &hum_raw, &humidity);
    if (aht_err != ESP_OK) {
        ESP_LOGW(TAG, "AHT20 Fehler, nichts gespeichert");
        return;
    }
    if (battery_read_mv(&battery_mv) != ESP_OK) {
        battery_mv = 0;
    }
    int motion = gpio_get_level(RADAR_PIN);

    rtc_log_add_values(temperature, humidity, battery_mv, motion);
}

/* ---------- Cloud (RainMaker) ---------- */

#define CLOUD_TIMEOUT_MS  30000     // timer mode: max. time for Wi-Fi + MQTT + time sync

// Newest measurement: start value of the params (instead of 0)
static measurement_t newest_measurement(void)
{
    measurement_t m = {0};
    if (rtc_log_get(rtc_log_total() - 1, &m)) {
        return m;                           // newest entry of the log
    }
    // log empty (after power-on / RESET): measure now.
    // cloud_start() runs in app_main() before sensor_task exists -> no mutex needed
    uint32_t temperature_raw, humidity_raw;
    float temperature, humidity;
    int battery_mv = 0;
    if (aht20_read_temperature_humidity(g_aht20, &temperature_raw, &temperature,
                                        &humidity_raw, &humidity) == ESP_OK) {
        m.temp_x100 = (int16_t) lroundf(temperature * 100.0f);
        m.hum_x100 = (uint16_t) lroundf(humidity * 100.0f);
    }
    if (battery_read_mv(&battery_mv) == ESP_OK) {
        m.battery_mv = (uint16_t) battery_mv;
    }
    return m;
}

// Uploads up to MAX_UPLOAD_PER_SYNC measurements that are not in the cloud yet, oldest first,
// with their original timestamps (RainMaker "simple time series").
// Returns the number of uploaded entries.
static int cloud_sync(void)
{
    cloud_time_update();                    // given: store the clock jump if it happened in this boot
    const uint32_t total = rtc_log_total();
    const uint32_t pending = total - s_cloud.uploaded_total;
    int uploaded = 0;

    // oldest first, at most MAX_UPLOAD_PER_SYNC entries (MQTT budget, the rest follows next time)
    while (s_cloud.uploaded_total < total && uploaded < MAX_UPLOAD_PER_SYNC) {
        measurement_t m;
        const bool exists = rtc_log_get(s_cloud.uploaded_total, &m);
        // Unix timestamp of the entry (0 = unknown). The RainMaker API expects an int.
        const int ts = exists ? (int) entry_unix_time(&m) : 0;
        if (ts == 0) {
            s_cloud.uploaded_total++;       // overwritten or no valid time: can never be uploaded
            continue;
        }

        esp_err_t err = ESP_OK;
        // die drei Werte mit dem Zeitstempel ts senden, den naechsten nur wenn der vorige ging
        err = esp_rmaker_param_report_simple_ts_data(g_temp_param, esp_rmaker_float(m.temp_x100 / 100.0f), ts, 0);
        if (err == ESP_OK) {
            err = esp_rmaker_param_report_simple_ts_data(g_hum_param, esp_rmaker_float(m.hum_x100 / 100.0f), ts, 0);
        }
        if (err == ESP_OK && m.battery_mv != 0) {      // 0 mV = nicht gemessen -> weglassen
            err = esp_rmaker_param_report_simple_ts_data(g_bat_param, esp_rmaker_int(m.battery_mv), ts, 0);
        }

        if (err != ESP_OK) {
            // e.g. MQTT disconnected or budget used up: do NOT count this entry,
            // it is retried at the next connection (never lose a measurement)
            ESP_LOGW(TAG, "Upload stopped: %s", esp_err_to_name(err));
            break;
        }
        s_cloud.uploaded_total++;           // only now the entry counts as "in the cloud"
        uploaded++;
    }
    if (uploaded > 0) {
        esp_rmaker_report_updated_params(); // dashboard: show the newest values in the app
    }
    ESP_LOGI(TAG, "Uploaded %d of %lu pending measurements", uploaded, (unsigned long) pending);
    return uploaded;
}

// One job: connect (or provision), upload the new log entries, end.
static void cloud_task(void *pvParameters)
{
    // blocks until the box is online: with the stored Wi-Fi credentials, or after provisioning
    if (app_network_start(POP_TYPE_RANDOM) == ESP_OK) {
        cloud_port_qr_hide();
        s_cloud.cloud_ready = 1;            // Wi-Fi credentials known: timer wake-ups may go online

        // wait for the MQTT connection, at most until the common deadline
        EventBits_t bits = xEventGroupWaitBits(g_cloud_events, MQTT_CONNECTED_BIT, pdFALSE, pdTRUE,
                                               cloud_ticks_left());

        // nur hochladen, wenn MQTT verbunden UND die Uhr synchronisiert ist
        if ((bits & MQTT_CONNECTED_BIT) && esp_rmaker_time_wait_for_sync(cloud_ticks_left()) == ESP_OK) {
            if (cloud_sync() > 0) {
                // (Step 4) RainMaker sends with QoS 1: give the broker time to confirm
                vTaskDelay(pdMS_TO_TICKS(CLOUD_FLUSH_MS));
            }
            // (Step 4) next batch upload after UPLOAD_EVERY_N further measurements
            s_cloud.next_upload_at = rtc_log_total() + UPLOAD_EVERY_N;
        } else {
            ESP_LOGW(TAG, "No MQTT connection or time sync before the deadline");
        }
    }
    xEventGroupClearBits(g_cloud_events, CLOUD_BUSY_BIT);  // only AFTER the upload: power_task may sleep again
    xEventGroupSetBits(g_cloud_events, SYNC_DONE_BIT);      // app_main may sleep (timer mode, Step 4)
    vTaskDelete(NULL);                      // job done (a task must never simply return)
}

static void cloud_start(bool with_display)
{
    xEventGroupSetBits(g_cloud_events, CLOUD_BUSY_BIT);    // do not sleep until cloud_task is done
    // one deadline for the whole cloud job: with display, the user may need the full
    // provisioning time; in timer mode nobody is waiting
    cloud_deadline_set(with_display ? PROV_TIMEOUT_MS + CLOUD_TIMEOUT_MS : CLOUD_TIMEOUT_MS);
    // every time-series report logs one INFO line with JSON (Step 3) -> warnings only
    esp_log_level_set("esp_rmaker_param", ESP_LOG_WARN);

    app_network_init();                     // Wi-Fi + provisioning, creates the default event loop
    // in former releases, app_network_init() was also called app_wifi_init()
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true,           // SNTP: Unix timestamps for the upload (Step 3)
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "IoT Lab Box", "Sensor");
    assert(node != NULL);

    // start values of the params = newest measurement (not 0, see the note above)
    const measurement_t last = newest_measurement();

    // Geraet "Climate" mit Name-Param und drei Params (nur lesen, Verlauf in der Cloud)
    esp_rmaker_device_t *device = esp_rmaker_device_create("Climate", ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    assert(device != NULL);
    esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Climate"));

    g_temp_param = esp_rmaker_param_create("Temperature", ESP_RMAKER_PARAM_TEMPERATURE,
                       esp_rmaker_float(last.temp_x100 / 100.0f),
                       PROP_FLAG_READ | PROP_FLAG_SIMPLE_TIME_SERIES);
    esp_rmaker_param_add_ui_type(g_temp_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(device, g_temp_param);

    g_hum_param = esp_rmaker_param_create("Humidity", "custom.param.humidity",
                      esp_rmaker_float(last.hum_x100 / 100.0f),
                      PROP_FLAG_READ | PROP_FLAG_SIMPLE_TIME_SERIES);
    esp_rmaker_param_add_ui_type(g_hum_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(device, g_hum_param);

    g_bat_param = esp_rmaker_param_create("Battery", "custom.param.battery-mv",
                      esp_rmaker_int(last.battery_mv),
                      PROP_FLAG_READ | PROP_FLAG_SIMPLE_TIME_SERIES);
    esp_rmaker_param_add_ui_type(g_bat_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(device, g_bat_param);

    esp_rmaker_device_assign_primary_param(device, g_temp_param);
    esp_rmaker_node_add_device(node, device);

    cloud_port_register_events(with_display);   // given: QR code, MQTT_CONNECTED_BIT
    if (with_display) {
        // BOOT button: hold 3 s -> forget Wi-Fi (new provisioning), hold 10 s -> factory reset
        app_reset_button_register(app_reset_button_create(BSP_BUTTON_CONFIG_IO, 0), 3, 10);
    }
    ESP_ERROR_CHECK(esp_rmaker_start());    // starts the RainMaker agent (connects once Wi-Fi is up)

    BaseType_t ok = xTaskCreate(cloud_task, "Cloud", 6144, NULL, 2, NULL);   // lowest priority
    assert(ok == pdPASS);
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
        } else if ((xEventGroupGetBits(g_cloud_events) & CLOUD_BUSY_BIT) && cloud_ticks_left() > 0) {
            // Provisioning oder Upload laeuft -> wach bleiben, aber nur bis zur Deadline
            ESP_LOGI(TAG, "Cloud busy (provisioning/upload) - staying awake for max. %lu s",
                     (unsigned long) (pdTICKS_TO_MS(cloud_ticks_left()) / 1000));
        } else if (backlight_on) {
            bsp_display_lock(0);
            bsp_display_backlight_off();
            bsp_display_unlock();
            backlight_on = false;
            ESP_LOGI(TAG, "10 s keine Bewegung -> Display aus");
            enter_deep_sleep();
        }
    }
}

void app_main(void)
{
    cloud_port_init();          // erste Zeile: Startzeit merken, Event-Group anlegen

    // Warum ist der Chip gestartet?
    const uint32_t wakeup_causes = esp_sleep_get_wakeup_causes();
    bool woken_by_radar = wakeup_causes & BIT(ESP_SLEEP_WAKEUP_EXT0);
    const bool woken_by_timer = wakeup_causes & BIT(ESP_SLEEP_WAKEUP_TIMER);
    // keins von beiden: Power-on oder Reset

    boot_count++;
    rtc_boot_count++;

    rtc_log_check();

    // NVS: hier speichert RainMaker WLAN-Zugangsdaten und Node-Daten
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());     // partition full or from an older IDF: start empty
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // I2C-Busse ueber das BSP anlegen (ohne Display), AHT20 benutzt den Dock-Bus mit
    ESP_ERROR_CHECK(bsp_i2c_init());
    aht20_init();
    battery_init();

    radar_gpio_init();

    // Timer wakeup: measure once and sleep again - display stays off, no tasks started
    if (woken_by_timer) {
        measure_once();
        if (gpio_get_level(RADAR_PIN) == 0) {
            // Lab 08: store-and-forward - only every UPLOAD_EVERY_N measurements switch on the radio
            if (s_cloud.cloud_ready && rtc_log_total() >= s_cloud.next_upload_at) {
                // set the next upload point already now: if this attempt fails (no Wi-Fi),
                // the next attempt comes only after UPLOAD_EVERY_N further measurements (back-off)
                s_cloud.next_upload_at = rtc_log_total() + UPLOAD_EVERY_N;
                ESP_LOGI(TAG, "Batch upload: %lu measurements pending",
                         (unsigned long) (rtc_log_total() - s_cloud.uploaded_total));
                cloud_start(false);         // no QR code, no BOOT button, deadline CLOUD_TIMEOUT_MS

                // auf das Ende des Uploads warten - nie ohne Timeout
                EventBits_t bits = xEventGroupWaitBits(g_cloud_events, SYNC_DONE_BIT, pdFALSE, pdTRUE,
                                       cloud_ticks_left() + pdMS_TO_TICKS(CLOUD_FLUSH_MS + CLOUD_GRACE_MS));
                if (!(bits & SYNC_DONE_BIT)) {
                    ESP_LOGW(TAG, "Upload timeout - retrying after %d measurements", UPLOAD_EVERY_N);
                }
            }
            enter_deep_sleep();             // does not return
        }
        // radar is active right now: continue as if woken up by motion
        ESP_LOGI(TAG, "Motion during measurement - starting display");
        woken_by_radar = true;
    }

    g_readings_mutex = xSemaphoreCreateMutex();
    assert(g_readings_mutex != NULL);

    // Semaphore vor dem ISR-Handler anlegen
    g_motion_sem = xSemaphoreCreateBinary();
    assert(g_motion_sem != NULL);
    if (woken_by_radar) {
        xSemaphoreGive(g_motion_sem);   // die Bewegung, die geweckt hat, zaehlt als erste
    }

    bsp_display_start();
    bsp_display_backlight_on();
    ui_init();

    // nach Power-on/RESET einmal online gehen (QR-Code, falls noch nicht eingerichtet)
    if (!s_cloud.cloud_ready) {
        cloud_start(true);
    }

    radar_isr_init();

    BaseType_t ok;
    ok = xTaskCreate(power_task, "power_task", TASK_STACK_SIZE, NULL, POWER_TASK_PRIO, NULL);
    assert(ok == pdPASS);
    ok = xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIO, NULL);
    assert(ok == pdPASS);
    ok = xTaskCreate(display_task, "display_task", TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIO, NULL);
    assert(ok == pdPASS);

    // USB-Konsole braucht nach dem Aufwachen 1-2 s, sonst fehlen die Zeilen
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Wakeup: %s | boot_count=%d | rtc_boot_count=%d",
             woken_by_timer ? "Timer (Radar HIGH)" : (woken_by_radar ? "Radar (EXT0)" : "Power-on/Reset"),
             boot_count, rtc_boot_count);
    rtc_log_print(10);
    ESP_LOGI(TAG, "Cloud: %lu of %lu measurements uploaded",
             (unsigned long) s_cloud.uploaded_total, (unsigned long) rtc_log_total());
}
