// Internal-only helpers shared between SerialFlashChip.cpp and
// SerialFlashDirectory.cpp. Lives at component root (NOT under include/) so
// it's not exposed to consumers via INCLUDE_DIRS.

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace serial_flash_priv {

extern SemaphoreHandle_t mutex;  // recursive; created lazily in SerialFlashChip::begin

class Lock {
public:
    Lock()  { xSemaphoreTakeRecursive(mutex, portMAX_DELAY); }
    ~Lock() { xSemaphoreGiveRecursive(mutex); }
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
};

} // namespace serial_flash_priv
