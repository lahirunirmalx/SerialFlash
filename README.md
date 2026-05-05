# SerialFlash for ESP-IDF

ESP-IDF / PlatformIO port of [Paul Stoffregen's SerialFlash](https://github.com/PaulStoffregen/SerialFlash) library — filesystem-like access to SPI NOR flash chips (Winbond W25Qxx, Spansion S25FLxx, Micron N25Qxx, Macronix MX25Lxx, SST/Adesto, etc.).

In-progress file write and erase operations do NOT block read access on other files. SerialFlash automatically allocates files with flash page and sector awareness, and supports suspending in-progress write and erase operations to minimize read latency even while the flash memory is busy.

Performance-oriented design imposes some usage limitations: files are created with a fixed size that can never change or grow; once created, files cannot be renamed or deleted (except by erasing the entire chip); files begin with all bytes erased (`0xFF`); each byte may be written only once. Files created as erasable may be fully erased to allow new data to be written. Best performance is achieved by writing in 256-byte chunks.

## Hardware compatibility

![W25Q128FV chip](doc/w25q128fv.jpg)

Tested chip families (compatibility list inherited from upstream):

    Winbond W25Q80BV / W25Q64FV / W25Q128FV / W25Q256FV
    Micron  N25Q512A / N25Q00AA
    Spansion S25FL127S / S25FL256S / S25FL512S
    Macronix MX25L… (autodetected)
    Adesto / SST 25-series

The library auto-detects chip type and capacity at `begin()` time, including 16 MByte / 24-bit and >16 MByte / 32-bit address modes, multi-die Micron parts, and 256K-sector Spansion parts.

## Targets

`esp32`, `esp32s2`, `esp32s3`, `esp32c3`, `esp32c6`, `esp32h2`. Requires ESP-IDF ≥ 5.0.

## Adding the component to your project

### As a managed component (local path)

In `main/idf_component.yml`:

    dependencies:
      serial_flash:
        path: "path/to/SerialFlash"

### As a git submodule

    git submodule add https://github.com/PaulStoffregen/SerialFlash components/SerialFlash

ESP-IDF will discover it automatically (the repo root is itself a component directory with `CMakeLists.txt` + `idf_component.yml`).

## Usage

    #include "SerialFlash.h"

    SerialFlashConfig cfg;
    cfg.host           = SPI2_HOST;
    cfg.sclk_pin       = GPIO_NUM_12;
    cfg.mosi_pin       = GPIO_NUM_11;
    cfg.miso_pin       = GPIO_NUM_13;
    cfg.cs_pin         = GPIO_NUM_10;
    cfg.clock_speed_hz = 20 * 1000 * 1000;
    cfg.init_bus       = true;   // false if your app already called spi_bus_initialize()

    if (!SerialFlash.begin(cfg)) {
        // chip not detected — check wiring, pull-ups, voltage
    }

### Open / read

    SerialFlashFile f = SerialFlash.open("settings.bin");
    if (f) {
        char buf[128];
        f.read(buf, sizeof(buf));
    }

### File size and position

    f.size();
    f.position();
    f.seek(123);

### Write

    f.write(buf, len);

Writes can only target previously unwritten bytes within the file's original size; file size never changes after creation.

### Erase

    f.erase();   // only valid for files created via createErasable()

### Create / exists / remove

    SerialFlash.create("data.bin", 65536);
    SerialFlash.createErasable("log.bin", 1 << 20);
    SerialFlash.exists("data.bin");
    SerialFlash.remove("data.bin");   // marks slot deleted; does not reclaim space

### Directory listing

    SerialFlash.opendir();
    char     filename[64];
    uint32_t filesize;
    while (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
        printf("%-20s %u bytes\n", filename, (unsigned)filesize);
    }

### Full erase

    SerialFlash.eraseAll();
    while (!SerialFlash.ready()) {
        vTaskDelay(pdMS_TO_TICKS(100));   // 30 seconds to 2 minutes typical
    }

## Pin selection

- Avoid the on-package flash-SPI pins (esp32: GPIO6–11; WROVER also adds 16–17).
- Avoid GPIO12 on the original esp32 — driving it high at boot forces 1.8 V flash and can brick 3.3 V modules.
- Use `SPI2_HOST` on all variants; `SPI3_HOST` is also available on `esp32`/`esp32s3`.

## Performance and memory notes

- **Chunking.** Read and write paths split the requested transfer into ≤ 4096-byte SPI transactions internally. You can pass any `len` to `read()` / `write()`; the component handles chunking, page boundaries, and the 32 MByte multi-die boundary on Micron parts.
- **Bounce buffer.** If you pass a non-DMA-capable buffer (e.g. PSRAM via `MALLOC_CAP_SPIRAM`), the component lazily allocates a 1 KB DMA-capable scratch buffer in internal RAM and routes the transfer through it. No allocation happens unless a non-DMA buffer is actually used. The bounce buffer is freed by `end()`.
- **Wait backoff.** `wait()` busy-polls the chip status for ~50 iterations (~2-3 ms — enough to cover typical page-program latency) before backing off to `vTaskDelay(1)`. This keeps write throughput close to the hardware limit while still yielding the CPU during long erases (sector ~50 ms, chip ~30-120 s).
- **Thread safety.** Every public `SerialFlash.*` and `SerialFlashFile.*` entry point takes a recursive mutex, including `open()` / `create()` / `remove()` / `readdir()`. Concurrent calls from multiple FreeRTOS tasks are safe.

## Differences from the upstream Arduino library

| Concern         | Arduino library                                | This ESP-IDF port                                  |
|-----------------|------------------------------------------------|----------------------------------------------------|
| SPI driver      | `SPIClass` + manual CS via direct port writes  | `spi_master` + hardware CS (`spics_io_num`)        |
| CS framing      | `CSASSERT()` / `CSRELEASE()` macros            | One `spi_transaction_ext_t` per logical operation  |
| Address phase   | Sent as discrete `transfer16` calls            | Native cmd/addr/dummy phases of the SPI peripheral |
| Wait loops      | Spin                                           | Spin then `vTaskDelay(1)` backoff                  |
| Microsec delay  | `delayMicroseconds(1)`                         | `esp_rom_delay_us(1)`                              |
| Thread safety   | None                                           | Recursive mutex around every public method         |
| `begin()`       | `begin(SPIClass&, cs)` / `begin(cs)`           | `begin(const SerialFlashConfig&)`                  |

The on-flash directory layout is byte-for-byte compatible with the Arduino library — chips provisioned by either side can be read by the other.

## Examples

- [`examples/esp-idf/ListFiles/`](examples/esp-idf/ListFiles/) — opens the chip, prints the directory. Mirrors upstream `ListFiles.ino`.
- [`examples/esp-idf/CopyFromSerial/`](examples/esp-idf/CopyFromSerial/) — receives framed files from a host PC over a dedicated UART (default UART1 on RX=18 / TX=17 @ 115200) and writes them into the chip via `SerialFlash.create()` + `write()`. Pairs with the host-side uploader below. Mirrors upstream `CopyFromSerial.ino`.

Adjust the pin assignments at the top of each example's `main/main.cpp`, then `pio run -t upload monitor`.

## Tools

[`extras/rawfile-uploader.py`](extras/rawfile-uploader.py) — host-side uploader that streams files over USB serial to the `CopyFromSerial` firmware. Wire-compatible with the upstream Arduino script's framing protocol, so it works against either an Arduino or ESP-IDF receiver. Python 3, requires `pyserial`.

    pip install pyserial
    extras/rawfile-uploader.py /dev/ttyUSB0 audio1.raw audio2.raw

The protocol UART is intentionally separate from the ESP32 console UART so log output and binary data don't share a channel — connect the host PC to the protocol UART via a USB-UART adapter (CP2102, FT232, CH340, …); keep the board's native USB cable for `idf.py monitor` / `pio device monitor`.

## License

MIT (same as upstream). See `include/SerialFlash.h` for the notice.
