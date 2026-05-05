// ListFiles example - ESP-IDF port of examples/ListFiles/ListFiles.ino
//
// Adjust the four pin assignments and clock speed to match your wiring.
// Defaults below target an ESP32-S3-DevKitC-1 with the W25Qxx flash on SPI2.

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "SerialFlash.h"

namespace {

constexpr char TAG[] = "list_files";

constexpr gpio_num_t PIN_SCLK = GPIO_NUM_12;
constexpr gpio_num_t PIN_MOSI = GPIO_NUM_11;
constexpr gpio_num_t PIN_MISO = GPIO_NUM_13;
constexpr gpio_num_t PIN_CS   = GPIO_NUM_10;
constexpr int        SPI_HZ   = 20 * 1000 * 1000; // 20 MHz; chip can usually do 50

void list_files_task(void *) {
    SerialFlashConfig cfg;
    cfg.host           = SPI2_HOST;
    cfg.sclk_pin       = PIN_SCLK;
    cfg.mosi_pin       = PIN_MOSI;
    cfg.miso_pin       = PIN_MISO;
    cfg.cs_pin         = PIN_CS;
    cfg.clock_speed_hz = SPI_HZ;
    cfg.init_bus       = true;

    if (!SerialFlash.begin(cfg)) {
        ESP_LOGE(TAG, "Unable to access SPI Flash chip");
        vTaskDelete(nullptr);
        return;
    }

    uint8_t id[5] = {};
    SerialFlash.readID(id);
    ESP_LOGI(TAG, "Flash ID: %02X %02X %02X (capacity %u bytes)",
             id[0], id[1], id[2], (unsigned)SerialFlash.capacity(id));

    ESP_LOGI(TAG, "All files on SPI flash chip:");
    SerialFlash.opendir();
    while (true) {
        char filename[64];
        uint32_t filesize = 0;
        if (!SerialFlash.readdir(filename, sizeof(filename), filesize)) break;
        ESP_LOGI(TAG, "  %-20s  %u bytes", filename, (unsigned)filesize);
    }

    vTaskDelete(nullptr);
}

} // namespace

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(list_files_task, "list_files",
                            4096, nullptr,
                            tskIDLE_PRIORITY + 3, nullptr, 0);
}
