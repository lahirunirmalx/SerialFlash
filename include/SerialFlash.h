/* SerialFlash Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/PaulStoffregen/SerialFlash
 * Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * ESP-IDF port: drop-in compatible API, backed by ESP-IDF spi_master driver.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SerialFlash_h_
#define SerialFlash_h_

#include <stdint.h>
#include <stddef.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"

class SerialFlashFile;

struct SerialFlashConfig {
    spi_host_device_t host;
    gpio_num_t        sclk_pin;
    gpio_num_t        mosi_pin;
    gpio_num_t        miso_pin;
    gpio_num_t        cs_pin;
    int               clock_speed_hz;
    bool              init_bus;

    SerialFlashConfig()
        : host(SPI2_HOST),
          sclk_pin(GPIO_NUM_NC),
          mosi_pin(GPIO_NUM_NC),
          miso_pin(GPIO_NUM_NC),
          cs_pin(GPIO_NUM_NC),
          clock_speed_hz(20 * 1000 * 1000),
          init_bus(true) {}
};

class SerialFlashChip
{
public:
    static bool begin(const SerialFlashConfig& cfg);
    static void end();

    static uint32_t capacity(const uint8_t *id);
    static uint32_t blockSize();
    static void sleep();
    static void wakeup();
    static void readID(uint8_t *buf);
    static void readSerialNumber(uint8_t *buf);
    static void read(uint32_t addr, void *buf, uint32_t len);
    static bool ready();
    static void wait();
    static void write(uint32_t addr, const void *buf, uint32_t len);
    static void eraseAll();
    static void eraseBlock(uint32_t addr);

    static SerialFlashFile open(const char *filename);
    static bool create(const char *filename, uint32_t length, uint32_t align = 0);
    static bool createErasable(const char *filename, uint32_t length) {
        return create(filename, length, blockSize());
    }
    static bool exists(const char *filename);
    static bool remove(const char *filename);
    static bool remove(SerialFlashFile &file);
    static void opendir() { dirindex = 0; }
    static bool readdir(char *filename, uint32_t strsize, uint32_t &filesize);

private:
    static uint16_t dirindex; // current position for readdir()
    static uint8_t  flags;    // chip features
    static uint8_t  busy;     // 0 = ready
                              // 1 = suspendable program operation
                              // 2 = suspendable erase operation
                              // 3 = busy for realz!!
};

extern SerialFlashChip SerialFlash;


class SerialFlashFile
{
public:
    constexpr SerialFlashFile() {}
    operator bool() {
        return address > 0;
    }
    uint32_t read(void *buf, uint32_t rdlen) {
        if (offset + rdlen > length) {
            if (offset >= length) return 0;
            rdlen = length - offset;
        }
        SerialFlash.read(address + offset, buf, rdlen);
        offset += rdlen;
        return rdlen;
    }
    uint32_t write(const void *buf, uint32_t wrlen) {
        if (offset + wrlen > length) {
            if (offset >= length) return 0;
            wrlen = length - offset;
        }
        SerialFlash.write(address + offset, buf, wrlen);
        offset += wrlen;
        return wrlen;
    }
    void seek(uint32_t n) { offset = n; }
    uint32_t position() { return offset; }
    uint32_t size() { return length; }
    uint32_t available() {
        if (offset >= length) return 0;
        return length - offset;
    }
    void erase();
    void flush() {}
    void close() {}
    uint32_t getFlashAddress() { return address; }

protected:
    friend class SerialFlashChip;
    uint32_t address  = 0;
    uint32_t length   = 0;
    uint32_t offset   = 0;
    uint16_t dirindex = 0;
};

#endif
