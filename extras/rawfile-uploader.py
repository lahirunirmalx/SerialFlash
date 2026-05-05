#!/usr/bin/env python3
"""Upload raw files into a SerialFlash chip over USB serial.

This is a Python 3 port of the upstream `rawfile-uploader.py` that shipped
with the Arduino library, kept wire-compatible with that script's framing
protocol so any matching device-side receiver — Arduino *or* ESP-IDF — can
consume the bytes.

The receiving firmware is expected to:

  1. Format / pre-erase the chip on first boot.
  2. Read framed records from its USB-CDC / UART, and for each one call
     `SerialFlash.create(filename, length)` followed by
     `SerialFlashFile.write(...)` for the body bytes.
  3. Treat ~3 s of serial silence as "uploads complete".

Wire framing
------------

    Special bytes:
        0x7E (~)   START          start-of-frame, end-of-file marker
        0x7D (})   ESCAPE         body-byte escape prefix
        0x7C (|)   SEPARATOR      separates filename / length / body fields

    Per file:
        START
        <filename, raw 7-bit ASCII>
        SEPARATOR
        <4 bytes file length, big-endian>
        SEPARATOR
        <body bytes, with START and ESCAPE bytes XOR'd with 0x20 after a
         literal ESCAPE byte; SEPARATOR bytes pass through unmodified>
        START                       (acts as end-of-file marker)

Caveats inherited from upstream:
- Filenames must not contain 0x7E or 0x7C (no header escaping).
- File-length bytes are not escaped either, so files in the unrealistic
  > ~2 GB / high-byte-collides-with-START range can confuse the receiver.

Usage
-----

    rawfile-uploader.py /dev/ttyUSB0 audio1.raw audio2.raw ...
    rawfile-uploader.py --baud 921600 --flash-mb 128 COM3 *.raw

USB-CDC ignores the baud setting on the host side, but the option is here
for native UART bridges (CP2102, CH340, etc.).
"""

import argparse
import os
import sys
import time

import serial

BYTE_START     = 0x7E
BYTE_ESCAPE    = 0x7D
BYTE_SEPARATOR = 0x7C


def encode_file(path: str) -> bytes:
    """Build the framed byte stream for one file."""
    body = bytearray()
    body.append(BYTE_START)

    name = os.path.basename(path).encode("ascii", errors="strict")
    if BYTE_START in name or BYTE_SEPARATOR in name:
        raise ValueError(
            f"filename {os.path.basename(path)!r} contains a reserved "
            f"framing byte (0x7E / 0x7C)"
        )
    body.extend(name)
    body.append(BYTE_SEPARATOR)

    file_length = os.path.getsize(path)
    body.append((file_length >> 24) & 0xFF)
    body.append((file_length >> 16) & 0xFF)
    body.append((file_length >>  8) & 0xFF)
    body.append( file_length        & 0xFF)
    body.append(BYTE_SEPARATOR)

    with open(path, "rb") as f:
        for b in f.read():
            if b == BYTE_START or b == BYTE_ESCAPE:
                body.append(BYTE_ESCAPE)
                body.append(b ^ 0x20)
            else:
                body.append(b)

    body.append(BYTE_START)  # end-of-file marker
    return bytes(body)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Upload files into a SerialFlash chip over USB serial.",
    )
    p.add_argument("port", help="serial port (e.g. /dev/ttyUSB0, COM3)")
    p.add_argument("files", nargs="+", help="raw files to upload")
    p.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="baud rate; ignored by USB-CDC, used by UART bridges (default: 115200)",
    )
    p.add_argument(
        "--flash-mb",
        type=int,
        default=16,
        help="flash chip capacity in MByte, used as a sanity check (default: 16)",
    )
    p.add_argument(
        "--write-timeout",
        type=float,
        default=30.0,
        help="serial write timeout in seconds (default: 30)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    flash_bytes = args.flash_mb * 1024 * 1024
    total_size  = sum(os.path.getsize(f) for f in args.files)
    if total_size > flash_bytes:
        print(
            "Selected files exceed flash capacity:\n"
            f"  flash size: {flash_bytes:>14,} bytes\n"
            f"  total size: {total_size:>14,} bytes",
            file=sys.stderr,
        )
        return 1

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=args.write_timeout,
        write_timeout=args.write_timeout,
    )

    print(f"Uploading {len(args.files)} file(s) to {args.port} @ {args.baud} baud")
    for i, path in enumerate(args.files, start=1):
        size = os.path.getsize(path)
        sys.stdout.write(f"  [{i}/{len(args.files)}] {path} ({size:,} bytes) ... ")
        sys.stdout.flush()

        encoded = encode_file(path)
        start_t = time.monotonic()
        ser.write(encoded)
        ser.flush()
        elapsed = time.monotonic() - start_t

        kbps = (size / 1024) / elapsed if elapsed > 0 else float("inf")
        print(f"done ({kbps:.2f} KB/s)")

    print("All files uploaded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
