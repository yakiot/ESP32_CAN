# Yakiot ESP32 CAN Firmware

Open-source vehicle-telematics firmware for the Yakiot OBD + GNSS hardware
(ESP32-based). It reads the vehicle CAN bus and streams parsed telemetry to
your app or gateway over BLE — no vendor lock-in, every line is yours to fork.

- **OBD-II mode** — standard 11-bit CAN PID polling (RPM, speed, odometer,
  DTCs) for 12V passenger vehicles
- **J1939 mode** — parses broadcast 29-bit CAN PGNs (engine hours, axle
  weights, brake pressures, VIN) for 24V trucks and heavy machinery
- **BLE GATT API** — telemetry notify characteristic
  (`7e140002-…-51fa952a6a01`) pushes frames like `v=<VIN>`, RPM, speed and
  fuel data to any BLE client; ideal for custom Android apps or
  BLE-to-cloud gateways
- **Raw ESP32 access** — unrestricted peripheral access (TWAI/CAN, UART,
  GPIO) alongside the stock firmware, no proprietary wrappers

## Hardware

Runs on the **Yakiot YAK-01** dev kit:

- ESP32 + TWAI (CAN 2.0B) transceiver
- OBD-II connector, **12V/24V input with load-dump protection** —
  safe on passenger cars and commercial truck networks
- Multi-constellation GNSS, LTE Cat-M/NB-IoT, BLE
- Buy: **https://yakiot.com** — $120 USD, no minimum order, volume pricing
  from 10 units

The firmware is plain ESP32 Arduino, so it also runs on generic
ESP32 + CAN-transceiver boards for bench development.

## Flashing

Requires [PlatformIO](https://platformio.org/) (`pip install platformio`
or the VS Code extension).

```bash
# clone
git clone https://github.com/yakiot/ESP32_CAN.git
cd ESP32_CAN

# build for your vehicle class — pick exactly one env
pio run -e heavy_duty_j1939   # 24V trucks / heavy machinery (SAE J1939)
pio run -e passenger_obd2     # 12V passenger cars (OBD-II)

# flash over USB serial (auto-detects the port)
pio run -e passenger_obd2 -t upload

# watch the serial console at 115200 baud
pio run -e passenger_obd2 -t monitor
```

If auto-detection fails with multiple devices connected, uncomment
`upload_port` in `platformio.ini` (e.g. `/dev/tty.usbserial-0001` on macOS,
`/dev/ttyUSB0` on Linux, `COM3` on Windows).

## Documentation

See `docs/` for the BLE API and protocol notes.

## License

See `LICENSE`.
