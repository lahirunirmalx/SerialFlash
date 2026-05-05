// CopyFromSerial - ESP-IDF receiver for extras/rawfile-uploader.py.
//
// Wire-compatible with the upstream Arduino sketch's framing protocol:
//
//     0x7E START      start-of-frame, end-of-file marker
//     0x7D ESCAPE     body-byte escape prefix (next byte is XOR'd with 0x20)
//     0x7C SEPARATOR  separates filename / length / body fields
//
//     Per file:
//         START
//         <filename, raw 7-bit ASCII>
//         SEPARATOR
//         <4 bytes file length, big-endian>
//         SEPARATOR
//         <body, with START and ESCAPE bytes escaped; SEPARATOR passes through>
//         START                       (end-of-file marker)
//
// Sketch reads bytes from a dedicated protocol UART (UART1 by default)
// rather than the console UART, so log output and binary data stay
// separated. Connect the host PC to the protocol UART via a USB-UART
// adapter; keep the board's console USB cable for monitoring.
//
// Adjust the pin/UART/clock constants below to match your wiring.

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "SerialFlash.h"

namespace {

constexpr char TAG[] = "copy_from_serial";

constexpr spi_host_device_t FLASH_HOST  = SPI2_HOST;
constexpr gpio_num_t        PIN_SCLK    = GPIO_NUM_12;
constexpr gpio_num_t        PIN_MOSI    = GPIO_NUM_11;
constexpr gpio_num_t        PIN_MISO    = GPIO_NUM_13;
constexpr gpio_num_t        PIN_CS      = GPIO_NUM_10;
constexpr int               FLASH_HZ    = 20 * 1000 * 1000;

constexpr uart_port_t  PROTO_UART = UART_NUM_1;
constexpr gpio_num_t   PIN_PROTO_RX = GPIO_NUM_18;
constexpr gpio_num_t   PIN_PROTO_TX = GPIO_NUM_17;
constexpr int          PROTO_BAUD   = 115200;
constexpr size_t       PROTO_RX_BUF = 4096;

// 3 s of silence on the protocol UART = uploads complete, list contents.
constexpr TickType_t   IDLE_TIMEOUT = pdMS_TO_TICKS(3000);

constexpr uint8_t BYTE_START     = 0x7E;
constexpr uint8_t BYTE_ESCAPE    = 0x7D;
constexpr uint8_t BYTE_SEPARATOR = 0x7C;

constexpr size_t MAX_FILENAME_LEN = 63;
constexpr size_t PAGE_BUF_BYTES   = 256; // matches SerialFlash page size

enum class State {
    IDLE,           // wait for 0x7E (start of next file)
    FILENAME,       // accumulate name until 0x7C
    LENGTH,         // 4 raw bytes, big-endian
    EXPECT_LEN_SEP, // expect 0x7C after length
    BODY,           // streaming body to flash, escape-aware
    SKIP_BODY,      // create() failed; consume body bytes without writing
    EXPECT_END,     // expect 0x7E end-of-file marker
};

struct Receiver {
    State           state            = State::IDLE;
    char            filename[MAX_FILENAME_LEN + 1] = {};
    size_t          filename_len     = 0;
    uint8_t         length_buf[4]    = {};
    size_t          length_bytes     = 0;
    uint32_t        expected_length  = 0;
    uint32_t        body_received    = 0;
    bool            escape_active    = false;
    SerialFlashFile file;
    uint8_t         page_buf[PAGE_BUF_BYTES] = {};
    size_t          page_buf_len     = 0;
    uint32_t        files_ok         = 0;
    uint32_t        files_failed     = 0;
};

void flush_page(Receiver& r) {
    if (r.page_buf_len == 0) return;
    r.file.write(r.page_buf, r.page_buf_len);
    r.page_buf_len = 0;
}

void abort_current_file(Receiver& r, const char* reason) {
    ESP_LOGE(TAG, "abort '%s': %s", r.filename, reason);
    r.files_failed++;
    r.state = State::IDLE;
    r.filename_len = 0;
    r.escape_active = false;
}

void on_byte(Receiver& r, uint8_t b) {
    switch (r.state) {

    case State::IDLE:
        if (b == BYTE_START) {
            r.filename_len = 0;
            r.state = State::FILENAME;
        }
        // else: stray byte; stay idle until a START is seen
        break;

    case State::FILENAME:
        if (b == BYTE_SEPARATOR) {
            r.filename[r.filename_len] = '\0';
            r.length_bytes = 0;
            r.state = State::LENGTH;
        } else if (r.filename_len < MAX_FILENAME_LEN) {
            r.filename[r.filename_len++] = static_cast<char>(b);
        } else {
            abort_current_file(r, "filename too long");
        }
        break;

    case State::LENGTH:
        r.length_buf[r.length_bytes++] = b;
        if (r.length_bytes == 4) {
            r.expected_length =
                (static_cast<uint32_t>(r.length_buf[0]) << 24) |
                (static_cast<uint32_t>(r.length_buf[1]) << 16) |
                (static_cast<uint32_t>(r.length_buf[2]) <<  8) |
                (static_cast<uint32_t>(r.length_buf[3]));
            r.state = State::EXPECT_LEN_SEP;
        }
        break;

    case State::EXPECT_LEN_SEP:
        if (b != BYTE_SEPARATOR) {
            abort_current_file(r, "missing separator after length");
            break;
        }
        r.body_received  = 0;
        r.escape_active  = false;
        r.page_buf_len   = 0;
        if (!SerialFlash.create(r.filename, r.expected_length)) {
            ESP_LOGW(TAG, "create failed (already exists / no space): %s — skipping",
                     r.filename);
            r.state = State::SKIP_BODY;
            break;
        }
        r.file = SerialFlash.open(r.filename);
        if (!r.file) {
            abort_current_file(r, "open after create returned no file");
            r.state = State::SKIP_BODY;
            break;
        }
        ESP_LOGI(TAG, "creating '%s' (%u bytes)",
                 r.filename, static_cast<unsigned>(r.expected_length));
        r.state = State::BODY;
        break;

    case State::BODY: {
        uint8_t value;
        if (r.escape_active) {
            value = b ^ 0x20;
            r.escape_active = false;
        } else if (b == BYTE_ESCAPE) {
            r.escape_active = true;
            break;
        } else {
            value = b;
        }
        r.page_buf[r.page_buf_len++] = value;
        r.body_received++;
        if (r.page_buf_len == PAGE_BUF_BYTES ||
            r.body_received == r.expected_length) {
            flush_page(r);
        }
        if (r.body_received == r.expected_length) {
            ESP_LOGI(TAG, "wrote '%s' (%u bytes)",
                     r.filename, static_cast<unsigned>(r.expected_length));
            r.files_ok++;
            r.state = State::EXPECT_END;
        }
        break;
    }

    case State::SKIP_BODY: {
        if (r.escape_active) {
            r.escape_active = false;
            r.body_received++;
        } else if (b == BYTE_ESCAPE) {
            r.escape_active = true;
            break;
        } else {
            r.body_received++;
        }
        if (r.body_received == r.expected_length) {
            r.state = State::EXPECT_END;
        }
        break;
    }

    case State::EXPECT_END:
        if (b == BYTE_START) {
            r.state = State::IDLE;
        } else {
            abort_current_file(r, "missing end-of-file marker");
        }
        break;
    }
}

void list_contents() {
    ESP_LOGI(TAG, "--- chip contents ---");
    SerialFlash.opendir();
    while (true) {
        char     name[64] = {};
        uint32_t size = 0;
        if (!SerialFlash.readdir(name, sizeof(name), size)) break;
        ESP_LOGI(TAG, "  %-32s  %u bytes", name, static_cast<unsigned>(size));
    }
}

void receiver_task(void*) {
    SerialFlashConfig flash_cfg;
    flash_cfg.host           = FLASH_HOST;
    flash_cfg.sclk_pin       = PIN_SCLK;
    flash_cfg.mosi_pin       = PIN_MOSI;
    flash_cfg.miso_pin       = PIN_MISO;
    flash_cfg.cs_pin         = PIN_CS;
    flash_cfg.clock_speed_hz = FLASH_HZ;
    flash_cfg.init_bus       = true;

    if (!SerialFlash.begin(flash_cfg)) {
        ESP_LOGE(TAG, "SerialFlash.begin() failed - check wiring");
        vTaskDelete(nullptr);
        return;
    }

    uint8_t id[5] = {};
    SerialFlash.readID(id);
    ESP_LOGI(TAG, "flash ID %02X %02X %02X, capacity %u bytes",
             id[0], id[1], id[2],
             static_cast<unsigned>(SerialFlash.capacity(id)));

    uart_config_t uc = {};
    uc.baud_rate  = PROTO_BAUD;
    uc.data_bits  = UART_DATA_8_BITS;
    uc.parity     = UART_PARITY_DISABLE;
    uc.stop_bits  = UART_STOP_BITS_1;
    uc.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uc.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(PROTO_UART, PROTO_RX_BUF, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(PROTO_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(PROTO_UART, PIN_PROTO_TX, PIN_PROTO_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "ready - listening on UART%d (RX=%d TX=%d) @ %d baud",
             PROTO_UART, PIN_PROTO_RX, PIN_PROTO_TX, PROTO_BAUD);

    Receiver r;
    bool reported_done = true;
    uint8_t rx[256];

    while (true) {
        const int n = uart_read_bytes(PROTO_UART, rx, sizeof(rx), IDLE_TIMEOUT);
        if (n > 0) {
            reported_done = false;
            for (int i = 0; i < n; i++) {
                on_byte(r, rx[i]);
            }
        } else if (!reported_done) {
            // 3 s of silence after some traffic -> uploads complete
            if (r.state != State::IDLE) {
                ESP_LOGW(TAG, "stream ended mid-frame (state=%d)",
                         static_cast<int>(r.state));
                if (r.state == State::BODY) flush_page(r);
                r.state = State::IDLE;
            }
            ESP_LOGI(TAG, "upload complete: %u ok, %u failed",
                     static_cast<unsigned>(r.files_ok),
                     static_cast<unsigned>(r.files_failed));
            list_contents();
            reported_done = true;
        }
    }
}

} // namespace

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(receiver_task, "rx_proto",
                            8192, nullptr,
                            tskIDLE_PRIORITY + 3, nullptr, 0);
}
