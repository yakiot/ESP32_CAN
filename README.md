# Peko - Yakiot ESP32 CAN Firmware

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

Requires [PlatformIO](https://platformio.org/) — either the CLI (`pio`) or
the VS Code extension.

### Install PlatformIO

**macOS**

```bash
brew install platformio-core        # Homebrew (recommended)
# or with pip:
pip3 install platformio
```

**Ubuntu / Debian**

```bash
sudo apt update && sudo apt install -y python3-pip
pip3 install platformio
# serial-port access without sudo:
sudo usermod -a -G dialout $USER    # then log out and back in
# udev rules so the USB serial adapter is detected correctly:
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify with `pio --version`. Alternatively install the
[PlatformIO IDE extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
in VS Code — it bundles the CLI and adds Build/Upload/Monitor buttons.

### Build & flash

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

## Debugging

Connect the ESP32 to your computer using a USB data cable. From the project
folder, list serial devices and open the console at **115200 baud**:

```bash
pio device list
pio device monitor --baud 115200
```

If multiple serial devices are attached, select the ESP32 port explicitly:

```bash
# macOS: replace with the port shown by pio device list
pio device monitor --port /dev/cu.usbserial-0001 --baud 115200

# Ubuntu: replace with the port shown by pio device list
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

With the monitor open, press the board's **EN / RESET** button. After BLE
initializes, the startup log includes lines like these (address is an example):

```text
BLE address: aa:bb:cc:dd:ee:ff
BLE service: 7e140001-8c55-4a82-b345-51fa952a6a01
BLE telemetry: 7e140002-8c55-4a82-b345-51fa952a6a01
```

Copy your board's **BLE address**, not its Wi-Fi MAC, into Peko_demo's
`AppConstant.BLE_MAC_ADDRESS`. Use uppercase hex for Android's address
validation, for example `AA:BB:CC:DD:EE:FF`. This lets the app connect directly
without running a BLE scan. The firmware advertises as **Yak-ELD**.

The address prints once per boot; reset the board if you opened the monitor
after startup. CAN frames and decoded readings appear afterward when vehicle
data is received. Close the monitor with **Ctrl+C** before uploading firmware
or opening another serial program on the same port.

## Documentation

See `docs/` for the BLE API and protocol notes.

## License

See `LICENSE`.
