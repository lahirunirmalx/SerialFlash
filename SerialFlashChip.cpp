/* SerialFlash Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/PaulStoffregen/SerialFlash
 * Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * ESP-IDF port: backed by spi_master with hardware CS framing.
 *
 * MIT-licensed; see SerialFlash.h for the full notice.
 */

#include "SerialFlash.h"
#include "serial_flash_priv.h"

#include <cstring>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_memory_utils.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "serial_flash";

constexpr uint8_t FLAG_32BIT_ADDR    = 0x01; // larger than 16 MByte address
constexpr uint8_t FLAG_STATUS_CMD70  = 0x02; // requires special busy flag check
constexpr uint8_t FLAG_DIFF_SUSPEND  = 0x04; // uses 2 different suspend commands
constexpr uint8_t FLAG_MULTI_DIE     = 0x08; // multiple die, don't read across 32M barrier
constexpr uint8_t FLAG_256K_BLOCKS   = 0x10; // has 256K erase blocks
constexpr uint8_t FLAG_DIE_MASK      = 0xC0; // top 2 bits count during multi-die erase

constexpr uint8_t ID0_WINBOND  = 0xEF;
constexpr uint8_t ID0_SPANSION = 0x01;
constexpr uint8_t ID0_MICRON   = 0x20;
constexpr uint8_t ID0_MACRONIX = 0xC2;
constexpr uint8_t ID0_SST      = 0xBF;
constexpr uint8_t ID0_ADESTO   = 0x1F;

// Largest data-phase byte count per spi_device_polling_transmit. Must be
// <= the max_transfer_sz passed to spi_bus_initialize().
constexpr size_t SF_MAX_RX_CHUNK = 4096;

// Bounce buffer size for users whose buffers aren't DMA-capable (e.g. PSRAM
// or unaligned). Smaller than SF_MAX_RX_CHUNK to keep internal RAM cost low;
// the tradeoff is more transactions per non-DMA read.
constexpr size_t SF_BOUNCE_BYTES = 1024;

spi_device_handle_t s_dev       = nullptr;
spi_host_device_t   s_host      = SPI2_HOST;
bool                s_owns_bus  = false;
uint8_t            *s_bounce    = nullptr; // DMA-capable; lazily allocated
StaticSemaphore_t   s_mutex_storage;

bool ensure_bounce() {
    if (s_bounce == nullptr) {
        s_bounce = static_cast<uint8_t *>(
            heap_caps_malloc(SF_BOUNCE_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
        if (s_bounce == nullptr) {
            ESP_LOGE(TAG, "bounce alloc failed (%u bytes)",
                     static_cast<unsigned>(SF_BOUNCE_BYTES));
            return false;
        }
    }
    return true;
}

// One-shot SPI transaction with separate cmd / address / data phases. Hardware
// CS frames the whole sequence (driven by spics_io_num).
//
// `len` is the byte count of the data phase; pass tx for write, rx for read,
// or both for full-duplex. Cmd/addr/dummy_bits are sent before the data phase.
esp_err_t spi_xfer(uint8_t cmd, uint64_t addr, uint8_t addr_bits,
                   const void *tx, void *rx, size_t len,
                   uint8_t dummy_bits = 0)
{
    spi_transaction_ext_t t = {};
    t.base.flags     = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR
                     | SPI_TRANS_VARIABLE_DUMMY;
    t.command_bits   = 8;
    t.address_bits   = addr_bits;
    t.dummy_bits     = dummy_bits;
    t.base.cmd       = cmd;
    t.base.addr      = addr;
    t.base.length    = len * 8;
    t.base.rxlength  = rx ? len * 8 : 0;
    t.base.tx_buffer = tx;
    t.base.rx_buffer = rx;
    return spi_device_polling_transmit(s_dev, &t.base);
}

inline uint8_t status_cmd_byte(uint8_t f) {
    return (f & FLAG_STATUS_CMD70) ? 0x70 : 0x05;
}

// Returns true when the chip's busy bit is clear under the appropriate
// status protocol for this part. Callers hold the mutex.
bool poll_busy_clear(uint8_t f) {
    uint8_t status = 0;
    spi_xfer(status_cmd_byte(f), 0, 0, nullptr, &status, 1);
    return (f & FLAG_STATUS_CMD70) ? ((status & 0x80) != 0)
                                   : ((status & 0x01) == 0);
}

// Wait for the chip to finish its current operation. Spins for ~50 polls
// (~2-3ms, covering page-program latency at typical clocks) before backing
// off to vTaskDelay(1) so long erases (sector ~50ms, chip ~30s) don't starve
// other tasks. Caller holds the mutex.
void wait_busy_clear(uint8_t f) {
    int spin = 50;
    while (!poll_busy_clear(f)) {
        if (spin > 0) {
            spin--;
            // Each poll itself takes ~30-50us at SPI clocks, which is the
            // natural pacing - no extra delay needed inside the spin window.
        } else {
            vTaskDelay(1);
        }
    }
}

} // namespace

namespace serial_flash_priv {
SemaphoreHandle_t mutex = nullptr;
}
using serial_flash_priv::Lock;
using serial_flash_priv::mutex;

uint16_t SerialFlashChip::dirindex = 0;
uint8_t  SerialFlashChip::flags    = 0;
uint8_t  SerialFlashChip::busy     = 0;

SerialFlashChip SerialFlash;


void SerialFlashChip::wait(void)
{
    Lock lock;
    wait_busy_clear(flags);
    busy = 0;
}


void SerialFlashChip::read(uint32_t addr, void *buf, uint32_t len)
{
    Lock lock;
    uint8_t *p = static_cast<uint8_t *>(buf);
    uint8_t b, f, cmd;

    // Match upstream behavior: zero the user buffer first so that any
    // SPI failure leaves zeros rather than uninitialized memory.
    std::memset(p, 0, len);

    f = flags;
    b = busy;
    if (b) {
        if (poll_busy_clear(f)) {
            b = 0;
            busy = 0;
        } else if (b < 3) {
            // TODO: this may not work on Spansion chips which apparently
            // have 2 different suspend commands, for program vs erase.
            spi_xfer(0x06, 0, 0, nullptr, nullptr, 0); // write enable (Micron req'd)
            esp_rom_delay_us(1);
            cmd = 0x75; // suspend program/erase for almost all chips
            // but Spansion just has to be different for program suspend!
            if ((f & FLAG_DIFF_SUSPEND) && (b == 1)) cmd = 0x85;
            spi_xfer(cmd, 0, 0, nullptr, nullptr, 0); // suspend
            // Micron chips don't actually suspend until flags are read.
            // Suspend completes in ~20us on real parts so a tight yield
            // loop is appropriate here (no need for the wait_busy_clear
            // backoff which targets ms-scale operations).
            while (!poll_busy_clear(f)) {
                vTaskDelay(0);
            }
        } else {
            // Chip is busy with an operation that can't be suspended; wait
            // it out with full backoff (sector erase, multi-die erase, etc.)
            wait_busy_clear(f);
            busy = 0;
            b = 0;
        }
    }

    const uint8_t addr_bits = (f & FLAG_32BIT_ADDR) ? 32 : 24;
    const bool dma_ok = esp_ptr_dma_capable(p);
    do {
        uint32_t rdlen = len;
        if (f & FLAG_MULTI_DIE) {
            // Don't cross the 32 MByte die boundary in a single transaction.
            if ((addr & 0xFE000000) != ((addr + len - 1) & 0xFE000000)) {
                rdlen = 0x2000000 - (addr & 0x1FFFFFF);
            }
        }

        void *dst;
        bool used_bounce = false;
        if (dma_ok) {
            if (rdlen > SF_MAX_RX_CHUNK) rdlen = SF_MAX_RX_CHUNK;
            dst = p;
        } else {
            // Non-DMA-capable user buffer (e.g. PSRAM, cached region):
            // route through the bounce buffer so spi_master's DMA path stays
            // happy.
            if (!ensure_bounce()) {
                ESP_LOGE(TAG, "read aborted - bounce alloc failed");
                return;
            }
            if (rdlen > SF_BOUNCE_BYTES) rdlen = SF_BOUNCE_BYTES;
            dst = s_bounce;
            used_bounce = true;
        }

        spi_xfer(0x03, addr, addr_bits, nullptr, dst, rdlen);
        if (used_bounce) std::memcpy(p, s_bounce, rdlen);

        p    += rdlen;
        addr += rdlen;
        len  -= rdlen;
    } while (len > 0);

    if (b) {
        spi_xfer(0x06, 0, 0, nullptr, nullptr, 0); // write enable (Micron req'd)
        esp_rom_delay_us(1);
        cmd = 0x7A;
        if ((f & FLAG_DIFF_SUSPEND) && (b == 1)) cmd = 0x8A;
        spi_xfer(cmd, 0, 0, nullptr, nullptr, 0); // resume program/erase
    }
}


void SerialFlashChip::write(uint32_t addr, const void *buf, uint32_t len)
{
    Lock lock;
    const uint8_t *p = static_cast<const uint8_t *>(buf);
    uint32_t max, pagelen;

    const uint8_t addr_bits = (flags & FLAG_32BIT_ADDR) ? 32 : 24;
    const bool dma_ok = esp_ptr_dma_capable(p);

    do {
        if (busy) wait();
        spi_xfer(0x06, 0, 0, nullptr, nullptr, 0); // write enable
        max = 256 - (addr & 0xFF);
        pagelen = (len <= max) ? len : max;
        esp_rom_delay_us(1); // TODO: reduce this, but prefer safety first

        const void *src;
        if (dma_ok) {
            src = p;
        } else {
            if (!ensure_bounce()) {
                ESP_LOGE(TAG, "write aborted - bounce alloc failed");
                return;
            }
            // pagelen <= 256, always fits in the bounce buffer
            std::memcpy(s_bounce, p, pagelen);
            src = s_bounce;
        }

        // 0x02 = page program; cmd + 24/32-bit addr + payload
        spi_xfer(0x02, addr, addr_bits, src, nullptr, pagelen);
        p    += pagelen;
        addr += pagelen;
        len  -= pagelen;
        busy = 4;
    } while (len > 0);
}


void SerialFlashChip::eraseAll()
{
    Lock lock;
    if (busy) wait();
    uint8_t id[5];
    readID(id);
    if (id[0] == 0x20 && id[2] >= 0x20 && id[2] <= 0x22) {
        // Micron's multi-die chips require special die erase commands
        //  N25Q512A   20 BA 20  2 dies  32 Mbyte/die   65 nm transitors
        //  N25Q00AA   20 BA 21  4 dies  32 Mbyte/die   65 nm transitors
        //  MT25QL02GC 20 BA 22  2 dies  128 Mbyte/die  45 nm transitors
        uint8_t die_count = 2;
        if (id[2] == 0x21) die_count = 4;
        uint8_t die_index = flags >> 6;
        flags &= 0x3F;
        if (die_index >= die_count) return; // all dies erased
        uint8_t die_size = 2;  // in 16 Mbyte units
        if (id[2] == 0x22) die_size = 8;
        spi_xfer(0x06, 0, 0, nullptr, nullptr, 0); // write enable
        esp_rom_delay_us(1);
        // die erase command, 32-bit address selecting the die
        uint32_t die_addr = static_cast<uint32_t>(die_index) * die_size * 0x01000000UL;
        spi_xfer(0xC4, die_addr, 32, nullptr, nullptr, 0);
        flags |= (die_index + 1) << 6;
    } else {
        spi_xfer(0x06, 0, 0, nullptr, nullptr, 0); // write enable
        esp_rom_delay_us(1);
        spi_xfer(0xC7, 0, 0, nullptr, nullptr, 0); // bulk erase
    }
    busy = 3;
}


void SerialFlashChip::eraseBlock(uint32_t addr)
{
    Lock lock;
    uint8_t f = flags;
    if (busy) wait();
    spi_xfer(0x06, 0, 0, nullptr, nullptr, 0); // write enable
    esp_rom_delay_us(1);
    // 0xD8 = sector/block erase
    spi_xfer(0xD8, addr, (f & FLAG_32BIT_ADDR) ? 32 : 24,
             nullptr, nullptr, 0);
    busy = 2;
}


bool SerialFlashChip::ready()
{
    Lock lock;
    if (!busy) return true;
    if (!poll_busy_clear(flags)) return false;
    busy = 0;
    if (flags & FLAG_DIE_MASK) {
        // continue a multi-die erase
        eraseAll();
        return false;
    }
    return true;
}


bool SerialFlashChip::begin(const SerialFlashConfig& cfg)
{
    if (mutex == nullptr) {
        mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutex_storage);
    }
    Lock lock;

    // Validate required pins. spi_bus_initialize accepts -1 / GPIO_NUM_NC as
    // "no pin" and reports success, masking the real cause when the chip
    // never responds.
    if (cfg.init_bus) {
        if (cfg.sclk_pin == GPIO_NUM_NC ||
            cfg.mosi_pin == GPIO_NUM_NC ||
            cfg.miso_pin == GPIO_NUM_NC) {
            ESP_LOGE(TAG, "begin: sclk/mosi/miso must all be set when init_bus=true");
            return false;
        }
    }
    if (cfg.cs_pin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "begin: cs_pin must be set");
        return false;
    }

    // If begin() is called again, tear down the previous device cleanly.
    if (s_dev != nullptr) {
        spi_bus_remove_device(s_dev);
        s_dev = nullptr;
        if (s_owns_bus) {
            spi_bus_free(s_host);
            s_owns_bus = false;
        }
    }
    s_host = cfg.host;

    if (cfg.init_bus) {
        spi_bus_config_t bus = {};
        bus.mosi_io_num     = cfg.mosi_pin;
        bus.miso_io_num     = cfg.miso_pin;
        bus.sclk_io_num     = cfg.sclk_pin;
        bus.quadwp_io_num   = -1;
        bus.quadhd_io_num   = -1;
        bus.max_transfer_sz = SF_MAX_RX_CHUNK;
        esp_err_t err = spi_bus_initialize(s_host, &bus, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
            return false;
        }
        s_owns_bus = true;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits   = 0; // overridden per-transaction via SPI_TRANS_VARIABLE_CMD
    devcfg.address_bits   = 0;
    devcfg.mode           = 0;
    devcfg.clock_speed_hz = cfg.clock_speed_hz;
    devcfg.spics_io_num   = cfg.cs_pin;
    devcfg.queue_size     = 1;
    devcfg.flags          = 0;
    esp_err_t err = spi_bus_add_device(s_host, &devcfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        if (s_owns_bus) {
            spi_bus_free(s_host);
            s_owns_bus = false;
        }
        return false;
    }

    uint8_t id[5];
    uint8_t f;
    uint32_t size;

    readID(id);
    if ((id[0] == 0   && id[1] == 0   && id[2] == 0) ||
        (id[0] == 255 && id[1] == 255 && id[2] == 255)) {
        return false;
    }
    f = 0;
    size = capacity(id);
    if (size > 16777216) {
        // more than 16 Mbyte requires 32 bit addresses
        f |= FLAG_32BIT_ADDR;
        if (id[0] == ID0_SPANSION) {
            // spansion uses MSB of bank register: write 0x80 to bank reg via 0x17
            uint8_t bank = 0x80;
            spi_xfer(0x17, 0, 0, &bank, nullptr, 1);
        } else {
            // micron & winbond & macronix use command sequence
            spi_xfer(0x06, 0, 0, nullptr, nullptr, 0); // write enable
            esp_rom_delay_us(1);
            spi_xfer(0xB7, 0, 0, nullptr, nullptr, 0); // enter 4 byte addr mode
        }
        if (id[0] == ID0_MICRON) f |= FLAG_MULTI_DIE;
    }
    if (id[0] == ID0_SPANSION) {
        // Spansion has separate suspend commands
        f |= FLAG_DIFF_SUSPEND;
        if (!id[4]) {
            // Spansion chips with id[4] == 0 use 256K sectors
            f |= FLAG_256K_BLOCKS;
        }
    }
    if (id[0] == ID0_MICRON) {
        // Micron requires busy checks with a different command
        f |= FLAG_STATUS_CMD70; // TODO: all or just multi-die chips?
    }
    flags = f;
    readID(id);
    return true;
}


void SerialFlashChip::end()
{
    if (mutex == nullptr) return;
    Lock lock;
    if (s_dev != nullptr) {
        spi_bus_remove_device(s_dev);
        s_dev = nullptr;
    }
    if (s_owns_bus) {
        spi_bus_free(s_host);
        s_owns_bus = false;
    }
    if (s_bounce != nullptr) {
        heap_caps_free(s_bounce);
        s_bounce = nullptr;
    }
    flags = 0;
    busy  = 0;
}


void SerialFlashChip::sleep()
{
    Lock lock;
    if (busy) wait();
    spi_xfer(0xB9, 0, 0, nullptr, nullptr, 0); // deep power down
}


void SerialFlashChip::wakeup()
{
    Lock lock;
    spi_xfer(0xAB, 0, 0, nullptr, nullptr, 0); // wake up from deep power down
}


void SerialFlashChip::readID(uint8_t *buf) // caller must provide 5 bytes
{
    Lock lock;
    if (busy) wait();
    // Mfr / memory type / capacity (+ Spansion ID-CFI / sector size).
    // Always read 5 bytes; non-Spansion chips simply return don't-care for [3..4].
    // ID is small enough to skip the bounce-buffer dance: <=64 byte transfers
    // go through the SPI FIFO without DMA on all targets.
    spi_xfer(0x9F, 0, 0, nullptr, buf, 5);
}


void SerialFlashChip::readSerialNumber(uint8_t *buf) // needs room for 8 bytes
{
    Lock lock;
    if (busy) wait();
    // 0x4B + 4 dummy bytes + 8 data bytes. Encode the 4 dummy bytes as a
    // zero address phase so the SPI peripheral clocks them out for free.
    // 8 bytes fits in the FIFO; no bounce needed.
    spi_xfer(0x4B, 0, 32, nullptr, buf, 8);
}


uint32_t SerialFlashChip::capacity(const uint8_t *id)
{
    uint32_t n = 1048576; // unknown chips, default to 1 MByte

    if (id[0] == ID0_ADESTO && id[1] == 0x89) {
        n = 1048576 * 16; // 16 MB
    } else if (id[2] >= 16 && id[2] <= 31) {
        n = 1ul << id[2];
    } else if (id[2] >= 32 && id[2] <= 37) {
        n = 1ul << (id[2] - 6);
    } else if ((id[0] == 0   && id[1] == 0   && id[2] == 0) ||
               (id[0] == 255 && id[1] == 255 && id[2] == 255)) {
        n = 0;
    }
    return n;
}


uint32_t SerialFlashChip::blockSize()
{
    // Spansion chips >= 512 mbit use 256K sectors
    if (flags & FLAG_256K_BLOCKS) return 262144;
    // everything else seems to have 64K sectors
    return 65536;
}


/*
Chip                Uniform Sector Erase
                    20/21   52      D8/DC
                    -----   --      -----
W25Q64CV            4       32      64
W25Q128FV           4       32      64
S25FL127S                           64
N25Q512A            4               64
N25Q00AA            4               64
S25FL512S                           256
SST26VF032          4
AT25SF128A                  32      64
*/

//                  size    sector      ID bytes        busy    pgm/erase   chip
// Part             Mbyte   kbyte                       cmd     suspend     erase
// ----             ----    -----       --------        ---     -------     -----
// Winbond W25Q64CV     8   64          EF 40 17
// Winbond W25Q128FV    16  64          EF 40 18        05      single      60 & C7
// Winbond W25Q256FV    32  64          EF 40 19
// Spansion S25FL064A   8   ?           01 02 16
// Spansion S25FL127S   16  64          01 20 18        05
// Spansion S25FL128P   16  64          01 20 18
// Spansion S25FL256S   32  64          01 02 19        05                  60 & C7
// Spansion S25FL512S   64  256         01 02 20
// Macronix MX25L12805D 16  ?           C2 20 18
// Macronix MX66L51235F 64              C2 20 1A
// Numonyx M25P128      16  ?           20 20 18
// Micron M25P80        1   ?           20 20 14
// Micron N25Q128A      16  64          20 BA 18
// Micron N25Q512A      64  ?           20 BA 20        70      single      C4 x2
// Micron N25Q00AA      128 64          20 BA 21                single      C4 x4
// Micron MT25QL02GC    256 64          20 BA 22        70                  C4 x2
// SST  SST25WF010      1/8 ?           BF 25 02
// SST  SST25WF020      1/4 ?           BF 25 03
// SST  SST25WF040      1/2 ?           BF 25 04
// SST  SST25VF016B     1   ?           BF 25 41
// SST26VF016               ?           BF 26 01
// SST26VF032               ?           BF 26 02
// SST25VF032           4   64          BF 25 4A
// SST26VF064           8   ?           BF 26 43
// LE25U40CMC           1/2 64          62 06 13
// Adesto AT25SF128A    16              1F 89 01
