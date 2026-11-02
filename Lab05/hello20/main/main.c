/*
 * Lab 05 – 1.2: Schleife 20x "Hello World", danach Neustart
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"   // fuer esp_restart()

void app_main(void)
{
    for (int i = 1; i <= 20; i++) {
        printf("Hello World %d/20\n", i);
        vTaskDelay(pdMS_TO_TICKS(500));   // 0,5 s Pause
    }

    printf("Fertig - Neustart...\n");
    fflush(stdout);       // Text sicher ausgeben, bevor neu gestartet wird
    esp_restart();        // Software-Reset
}
