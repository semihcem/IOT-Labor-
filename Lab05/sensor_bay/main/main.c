/*
 * Lab 05 – Sensor Bay der ESP32-S3-BOX-3
 *
 *  - AHT-Sensor (Temperatur + Luftfeuchte) ueber I2C (SCL 40, SDA 41, Adresse 0x38)
 *  - Radarsensor AT581X: Bewegung als 0/1 an GPIO 21
 *  - Anzeige auf dem Display mit LVGL + Ausgabe im seriellen Monitor
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "bsp/esp-bsp.h"   // espressif/esp-box-3
#include "lvgl.h"
#include "aht20.h"         // espressif/aht20 (bringt i2c_bus mit)

/* ---------- Einstellungen ---------- */
#define I2C_SCL_PIN      40
#define I2C_SDA_PIN      41
#define I2C_FREQ_HZ      100000            // 100 kHz (Standard-Mode)
#define RADAR_PIN        GPIO_NUM_21

#define LOOP_MS          100               // Radar alle 100 ms pruefen
#define AHT_JEDE_N_LOOPS 20                // AHT alle 20 * 100 ms = 2 s lesen

/*
 * Die BOX-3 hat zwei I2C-Controller (Port 0 und 1).
 * Das BSP benutzt BSP_I2C_NUM fuer Touch/Audio (Pins 18/8) und legt beim
 * Display-Start auf dem ANDEREN Port schon den Dock-Bus (Pins 40/41) an.
 * Wir nehmen deshalb genau diesen anderen Port – i2c_bus_create() erkennt den
 * vorhandenen Bus und verwendet ihn mit, statt einen Fehler zu werfen.
 */
#if BSP_I2C_NUM == 1
#define SENSOR_I2C_PORT  I2C_NUM_0
#else
#define SENSOR_I2C_PORT  I2C_NUM_1
#endif

static const char *TAG = "sensor_bay";

/* ---------- Globale Objekte ---------- */
static i2c_bus_handle_t   i2c_bus = NULL;
static aht20_dev_handle_t aht20   = NULL;

static lv_obj_t *temp_label, *temp_bar;
static lv_obj_t *hum_label,  *hum_bar;
static lv_obj_t *radar_box,  *radar_label;

#if LVGL_VERSION_MAJOR >= 9
#define AKTIVER_SCREEN() lv_screen_active()
#else
#define AKTIVER_SCREEN() lv_scr_act()
#endif

/* ================= Sensoren ================= */

/* I2C-Bus konfigurieren und AHT-Sensor anlegen */
static esp_err_t aht_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = true,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(SENSOR_I2C_PORT, &i2c_conf);
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C-Bus konnte nicht erstellt werden");
        return ESP_FAIL;
    }

    // Kontrolle: welche Geraete antworten auf dem Bus? (AHT sollte 0x38 sein)
    uint8_t adressen[8];
    uint8_t anzahl = i2c_bus_scan(i2c_bus, adressen, sizeof(adressen));
    for (int i = 0; i < anzahl && i < (int)sizeof(adressen); i++) {
        ESP_LOGI(TAG, "I2C-Geraet gefunden: 0x%02X", adressen[i]);
    }

    const aht20_i2c_config_t aht_conf = {
        .bus_inst = i2c_bus,
        .i2c_addr = AHT20_ADDRRES_0,     // 0x38 (Schreibfehler "ADDRRES" stammt aus der Bibliothek)
    };
    return aht20_new_sensor(&aht_conf, &aht20);
}

/* Radar-Pin als Eingang einrichten */
static void radar_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << RADAR_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   // ohne Signal sicher 0
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

/* ================= Oberflaeche ================= */

static lv_obj_t *label_erstellen(lv_obj_t *parent, int y, const lv_font_t *font, const char *text)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0x0B5D3B), 0);
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, y);
    return label;
}

static lv_obj_t *balken_erstellen(lv_obj_t *parent, int y, int min, int max)
{
    lv_obj_t *bar = lv_bar_create(parent);
    lv_obj_set_size(bar, 260, 18);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, y);
    lv_bar_set_range(bar, min, max);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x3CC88C), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xB4D232), LV_PART_INDICATOR);
    return bar;
}

static void ui_erstellen(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = AKTIVER_SCREEN();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xA8DDF5), 0);

    label_erstellen(scr, 4, &lv_font_montserrat_28, "Sensor Bay");

    temp_label = label_erstellen(scr, 45, &lv_font_montserrat_20, "Temperatur: --");
    temp_bar   = balken_erstellen(scr, 72, 0, 50);

    hum_label  = label_erstellen(scr, 100, &lv_font_montserrat_20, "Luftfeuchte: --");
    hum_bar    = balken_erstellen(scr, 127, 0, 100);

    // Radar-Anzeige: farbige Box mit Text (grau = keine Bewegung, rot = Bewegung)
    radar_box = lv_obj_create(scr);
    lv_obj_set_size(radar_box, 260, 56);
    lv_obj_align(radar_box, LV_ALIGN_TOP_MID, 0, 165);
    lv_obj_set_style_radius(radar_box, 12, 0);
    lv_obj_set_style_border_width(radar_box, 0, 0);
    lv_obj_set_style_bg_color(radar_box, lv_color_hex(0x9E9E9E), 0);
    lv_obj_clear_flag(radar_box, LV_OBJ_FLAG_SCROLLABLE);

    radar_label = lv_label_create(radar_box);
    lv_obj_set_style_text_font(radar_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(radar_label, lv_color_white(), 0);
    lv_label_set_text(radar_label, "Keine Bewegung");
    lv_obj_center(radar_label);

    bsp_display_unlock();
}

static void ui_aht_anzeigen(float temperatur, float feuchte)
{
    char buf[40];
    bsp_display_lock(0);

    snprintf(buf, sizeof(buf), "Temperatur: %.1f °C", temperatur);
    lv_label_set_text(temp_label, buf);
    lv_bar_set_value(temp_bar, (int)temperatur, LV_ANIM_ON);

    snprintf(buf, sizeof(buf), "Luftfeuchte: %.1f %%", feuchte);
    lv_label_set_text(hum_label, buf);
    lv_bar_set_value(hum_bar, (int)feuchte, LV_ANIM_ON);

    bsp_display_unlock();
}

static void ui_aht_fehler(void)
{
    bsp_display_lock(0);
    lv_label_set_text(temp_label, "Temperatur: Fehler");
    lv_label_set_text(hum_label, "Luftfeuchte: Fehler");
    bsp_display_unlock();
}

static void ui_radar_anzeigen(bool bewegung)
{
    bsp_display_lock(0);
    lv_obj_set_style_bg_color(radar_box, lv_color_hex(bewegung ? 0xE53935 : 0x9E9E9E), 0);
    lv_label_set_text(radar_label, bewegung ? "Bewegung erkannt!" : "Keine Bewegung");
    bsp_display_unlock();
}

/* ================= Hauptprogramm ================= */

void app_main(void)
{
    /* --- Setup --- */
    bsp_display_start();          // Display + LVGL; legt auch beide I2C-Busse an
    bsp_display_backlight_on();
    ui_erstellen();

    if (aht_init() != ESP_OK) {
        ESP_LOGE(TAG, "AHT-Sensor nicht gefunden - Sensor-Dock richtig aufgesteckt?");
    }
    radar_init();

    int  zaehler = 0;
    int  letzter_radar = -1;      // -1 = noch nie gelesen, damit die erste Anzeige sicher kommt

    /* --- Schleife --- */
    while (1) {
        /* Radar: jede Runde (100 ms) */
        int radar = gpio_get_level(RADAR_PIN);   // 0 = keine Bewegung, 1 = Bewegung
        if (radar != letzter_radar) {            // nur bei Aenderung neu zeichnen/loggen
            ESP_LOGI(TAG, "Radar: %d (%s)", radar, radar ? "Bewegung" : "keine Bewegung");
            ui_radar_anzeigen(radar);
            letzter_radar = radar;
        }

        /* AHT: jede 20. Runde (2 s) */
        if (zaehler % AHT_JEDE_N_LOOPS == 0) {
            float temperatur = 0, feuchte = 0;
            uint32_t temp_raw = 0, hum_raw = 0;

            esp_err_t err = ESP_FAIL;
            if (aht20 != NULL) {
                err = aht20_read_temperature_humidity(aht20, &temp_raw, &temperatur, &hum_raw, &feuchte);
            }

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Temperatur: %.2f °C | Luftfeuchte: %.2f %%", temperatur, feuchte);
                ui_aht_anzeigen(temperatur, feuchte);
            } else {
                ESP_LOGW(TAG, "AHT-Lesefehler: %s", esp_err_to_name(err));
                ui_aht_fehler();
            }
        }

        zaehler++;
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
