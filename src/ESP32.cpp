#include <Arduino.h>

#include <ESP32-TWAI-CAN.hpp>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>

#if defined(PROTOCOL_J1939) == defined(PROTOCOL_OBD2)
#error "Select exactly one protocol: PROTOCOL_J1939 or PROTOCOL_OBD2"
#endif

static constexpr char SERVICE_UUID[] = "7e140001-8c55-4a82-b345-51fa952a6a01";
static constexpr char TELEMETRY_UUID[] = "7e140002-8c55-4a82-b345-51fa952a6a01";
static BLECharacteristic* telemetry = nullptr;
static std::atomic<bool> bleConnected{false};
static std::atomic<bool> restartAdvertising{false};
static uint32_t lastRpmNotify = 0;
static uint32_t lastSpeedNotify = 0;
static uint32_t lastFuelNotify = 0;
static uint32_t lastOdoNotify = 0;
static char vehicleVin[18] = {};
static uint32_t lastVinNotify = 0;

static void acceptVin(const uint8_t* data, size_t length) {
  if (length < 17) return;
  for (size_t i = 0; i < 17; ++i) {
    const char c = data[i];
    if (!((c >= 'A' && c <= 'Z' && c != 'I' && c != 'O' && c != 'Q') ||
          (c >= '0' && c <= '9'))) return;
  }
  memcpy(vehicleVin, data, 17);
  vehicleVin[17] = '\0';
}

static void publishVin() {
  if (!vehicleVin[0] || !bleConnected.load() || millis() - lastVinNotify < 5000) return;
  lastVinNotify = millis();
  char payload[21];
  snprintf(payload, sizeof(payload), "v=%s", vehicleVin);
  telemetry->setValue(payload);
  telemetry->notify();
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected.store(true);
    Serial.println("BLE connected");
  }

  void onDisconnect(BLEServer*) override {
    bleConnected.store(false);
    restartAdvertising.store(true);
    Serial.println("BLE disconnected");
  }
};

static ServerCallbacks serverCallbacks;

static void startBle() {
  BLEDevice::init("Yak-ELD");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(&serverCallbacks);
  BLEService* service = server->createService(SERVICE_UUID);
  telemetry = service->createCharacteristic(
      TELEMETRY_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  telemetry->addDescriptor(new BLE2902());
  telemetry->setValue("ready");
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();
  Serial.printf("BLE address: %s\n", BLEDevice::getAddress().toString().c_str());
  Serial.printf("BLE service: %s\nBLE telemetry: %s\n", SERVICE_UUID, TELEMETRY_UUID);
}

static void publishMetric(const char* name, float value, uint32_t& lastNotify) {
  if (!bleConnected.load() || millis() - lastNotify < 200) return;
  lastNotify = millis();
  char payload[20];
  snprintf(payload, sizeof(payload), "%s=%.1f", name, value);
  telemetry->setValue(payload);
  telemetry->notify();
}

// TWAI RX Pins
#define CAN_TX 5
#define CAN_RX 4

CanFrame rxFrame;
static bool canReady = false;

#ifdef PROTOCOL_OBD2
static uint16_t obdEcu = 0;
static uint8_t pendingMode = 0;
static uint8_t pendingPid = 0;
static uint32_t requestSentAt = 0;
static uint32_t lastObdRequest = 0;
static uint32_t lastRpmRequest = 0;
static uint32_t lastSpeedRequest = 0;
static uint32_t lastAuxRequest = 0;
static bool supported[256] = {};
static int supportPage = 0;
static bool vinSupportChecked = false;
static bool vinSupported = false;
static uint8_t auxSlot = 0;
static uint8_t isoBuffer[32] = {};
static size_t isoLength = 0;
static size_t isoReceived = 0;
static uint8_t isoSequence = 1;
static uint32_t isoAt = 0;

static uint32_t bigEndian32(const uint8_t* bytes) {
  return (uint32_t(bytes[0]) << 24) | (uint32_t(bytes[1]) << 16) |
         (uint32_t(bytes[2]) << 8) | bytes[3];
}

static void finishObd(const uint8_t* data, size_t length, uint16_t source) {
  if (length >= 3 && data[0] == 0x7F && data[1] == pendingMode) {
    Serial.printf("OBD-II negative response mode=%02X code=%02X\n", pendingMode, data[2]);
    return;
  }
  if (length < 2 || data[0] != pendingMode + 0x40 || data[1] != pendingPid) return;
  if (pendingMode == 1 && supportPage >= 0 && pendingPid == supportPage) {
    if (length != 6) return;
    if (!obdEcu) obdEcu = source;
    const uint32_t mask = bigEndian32(data + 2);
    for (int bit = 0; bit < 32 && supportPage + bit + 1 < 256; ++bit) {
      supported[supportPage + bit + 1] = (mask & (uint32_t(1) << (31 - bit))) != 0;
    }
    supportPage = (mask & 1) && supportPage < 0xA0 ? supportPage + 0x20 : -1;
  } else if (pendingMode == 9 && pendingPid == 0) {
    if (length != 6) return;
    vinSupported = (bigEndian32(data + 2) & 0x40000000UL) != 0;
    vinSupportChecked = true;
  } else if (pendingMode == 9 && pendingPid == 2) {
    if (length != 20 || data[2] != 1) return;
    acceptVin(data + 3, 17);
  } else if (pendingMode == 1) {
    if (pendingPid == 0x0C && length == 4) {
      publishMetric("rpm", ((uint16_t(data[2]) << 8) | data[3]) / 4.0f, lastRpmNotify);
    } else if (pendingPid == 0x0D && length == 3) {
      publishMetric("speed", data[2], lastSpeedNotify);
    } else if (pendingPid == 0x2F && length == 3) {
      publishMetric("fuel", data[2] * 100.0f / 255, lastFuelNotify);
    } else if (pendingPid == 0xA6 && length == 6) {
      publishMetric("odo", bigEndian32(data + 2) * 0.1f, lastOdoNotify);
    } else return;
  }
  pendingMode = 0;
  isoLength = 0;
}

static void pollObd() {
  const uint32_t now = millis();
  if (pendingMode) {
    if (now - requestSentAt < (isoLength ? 1000UL : 250UL) &&
        (!isoLength || now - isoAt < 250)) return;
    Serial.printf("OBD-II timeout mode=%02X pid=%02X\n", pendingMode, pendingPid);
    pendingMode = 0;
    isoLength = 0;
  }
  if (now - lastObdRequest < 100) return;
  uint8_t mode = 1;
  uint8_t pid = 0;
  if (supportPage >= 0) pid = uint8_t(supportPage);
  else if (supported[0x0C] && now - lastRpmRequest >= 1000) pid = 0x0C;
  else if (supported[0x0D] && now - lastSpeedRequest >= 1000) pid = 0x0D;
  else if (now - lastAuxRequest >= 1500) {
    lastAuxRequest = now;
    if (!vinSupportChecked) { mode = 9; pid = 0; }
    else {
      const uint8_t slot = auxSlot++ % 3;
      if (slot == 0 && supported[0x2F]) pid = 0x2F;
      else if (slot == 1 && supported[0xA6]) pid = 0xA6;
      else if (slot == 2 && vinSupported && !vehicleVin[0]) { mode = 9; pid = 2; }
      else return;
    }
  } else return;
  CanFrame request = {};
  request.identifier = obdEcu ? obdEcu - 8 : 0x7DF;
  request.data_length_code = 8;
  request.data[0] = 2;
  request.data[1] = mode;
  request.data[2] = pid;
  lastObdRequest = now;
  if (!ESP32Can.writeFrame(request)) {
    Serial.println("OBD-II CAN transmit failed");
    return;
  }
  if (pid == 0x0C && mode == 1) lastRpmRequest = now;
  if (pid == 0x0D && mode == 1) lastSpeedRequest = now;
  pendingMode = mode;
  pendingPid = pid;
  requestSentAt = now;
}

static void receiveObd(const CanFrame& frame) {
  if (frame.extd || frame.rtr || frame.identifier < 0x7E8 || frame.identifier > 0x7EF ||
      (obdEcu && frame.identifier != obdEcu) || frame.data_length_code < 2 ||
      frame.data_length_code > 8 || !pendingMode) return;
  const uint8_t type = frame.data[0] >> 4;
  if (type == 0) {
    const size_t length = frame.data[0] & 15;
    if (length && length + 1 <= frame.data_length_code) finishObd(frame.data + 1, length, frame.identifier);
  } else if (type == 1 && pendingMode == 9 && pendingPid == 2 && frame.data_length_code == 8) {
    const size_t length = ((frame.data[0] & 15) << 8) | frame.data[1];
    if (length != 20 || frame.data[2] != 0x49 || frame.data[3] != 2) return;
    isoLength = length;
    isoReceived = 6;
    isoSequence = 1;
    isoAt = millis();
    memcpy(isoBuffer, frame.data + 2, 6);
    CanFrame flow = {};
    flow.identifier = frame.identifier - 8;
    flow.data_length_code = 8;
    flow.data[0] = 0x30;
    flow.data[2] = 10;
    if (!ESP32Can.writeFrame(flow)) { isoLength = 0; pendingMode = 0; }
  } else if (type == 2 && isoLength) {
    if ((frame.data[0] & 15) != isoSequence || millis() - isoAt >= 250) {
      isoLength = 0;
      pendingMode = 0;
      return;
    }
    const size_t count = min(isoLength - isoReceived, size_t(frame.data_length_code - 1));
    memcpy(isoBuffer + isoReceived, frame.data + 1, count);
    isoReceived += count;
    isoSequence = (isoSequence + 1) & 15;
    isoAt = millis();
    if (isoReceived == isoLength) finishObd(isoBuffer, isoLength, frame.identifier);
  }
}
#endif

#ifdef PROTOCOL_J1939
static uint8_t vinSource = 0xFF;
static uint8_t vinSequence = 1;
static uint8_t vinBuffer[32] = {};
static size_t vinLength = 0;
static size_t vinReceived = 0;
static uint32_t vinAt = 0;

static void receiveJ1939Vin(uint32_t pgn, const CanFrame& frame) {
  const uint8_t source = frame.identifier & 0xFF;
  const uint8_t destination = (frame.identifier >> 8) & 0xFF;
  if (vinLength && millis() - vinAt > 1500) vinLength = 0;
  if (pgn == 0xEC00 && destination == 0xFF && frame.data_length_code == 8 &&
      frame.data[0] == 0x20) {
    const uint32_t target = frame.data[5] | (uint32_t(frame.data[6]) << 8) | (uint32_t(frame.data[7]) << 16);
    if (target != 0xFEEC) return;
    const size_t length = frame.data[1] | (uint16_t(frame.data[2]) << 8);
    if (length < 17 || length > sizeof(vinBuffer) || frame.data[3] != (length + 6) / 7) return;
    vinSource = source;
    vinLength = length;
    vinReceived = 0;
    vinSequence = 1;
    vinAt = millis();
  } else if (pgn == 0xEB00 && destination == 0xFF && source == vinSource && vinLength &&
             frame.data_length_code == 8) {
    if (frame.data[0] != vinSequence++) { vinLength = 0; return; }
    const size_t count = min(size_t(7), vinLength - vinReceived);
    memcpy(vinBuffer + vinReceived, frame.data + 1, count);
    vinReceived += count;
    vinAt = millis();
    if (vinReceived == vinLength) { acceptVin(vinBuffer, vinLength); vinLength = 0; }
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);
  startBle();

  // Initialize CAN at 250kbps (Standard J1939 Speed)
  ESP32Can.setPins(CAN_TX, CAN_RX);
#ifdef PROTOCOL_J1939
  canReady = ESP32Can.begin(TWAI_SPEED_250KBPS);
  if (canReady) Serial.println("J1939 Receiver Ready at 250kbps!");
#else
  canReady = ESP32Can.begin(TWAI_SPEED_500KBPS);
  if (canReady) Serial.println("OBD-II polling ready: 500kbps, 11-bit, Mode 01 RPM/speed");
#endif
  if (!canReady) Serial.println("CAN Init Failed!");
}

void loop() {
  if (restartAdvertising.exchange(false)) BLEDevice::startAdvertising();
  publishVin();
  if (!canReady) { delay(10); return; }
#ifdef PROTOCOL_OBD2
  pollObd();
#endif
  // Try to read a frame (10ms timeout to keep loop responsive)
  if (ESP32Can.readFrame(rxFrame, 10)) {
#ifdef PROTOCOL_OBD2
    receiveObd(rxFrame);
#else
    if (!rxFrame.extd || rxFrame.rtr || rxFrame.data_length_code > 8) return;
    // 1. Extract PGN (Bits 8-25 of the 29-bit ID)
    uint32_t pgn = (rxFrame.identifier >> 8) & 0x3FFFF;
    if (((rxFrame.identifier >> 16) & 0xFF) < 240) pgn &= 0x3FF00;
    receiveJ1939Vin(pgn, rxFrame);

    Serial.printf("ID: 0x%08X | PGN: %-5u | Data: ", rxFrame.identifier, pgn);
    for (int i = 0; i < rxFrame.data_length_code; i++) {
      Serial.printf("%02X ", rxFrame.data[i]);
    }
    Serial.println();

    if (rxFrame.data_length_code < 5) return;

    // 2. Decode J1939 PGNs
    switch (pgn) {
      case 61444: {  // Engine Speed (EEC1)
        // Little Endian: Byte 3 LSB, Byte 4 MSB
        uint16_t rawRpm = (rxFrame.data[4] << 8) | rxFrame.data[3];
        float rpm = rawRpm * 0.125;
        publishMetric("rpm", rpm, lastRpmNotify);
        Serial.printf("   [DECODED] Engine Speed: %.0f RPM\n", rpm);
        break;
      }

      case 65265: {  // Vehicle Speed (CCVS1)
        // Little Endian: Byte 1 LSB, Byte 2 MSB
        uint16_t rawSpeed = (rxFrame.data[2] << 8) | rxFrame.data[1];
        float speedKmh = rawSpeed / 256.0;
        publishMetric("speed", speedKmh, lastSpeedNotify);

        // Ignition: Byte 4, Bits 1-2
        uint8_t ignitionBits = (rxFrame.data[4]) & 0x03;
        const char* ignition = (ignitionBits == 0x01) ? "ON" : "OFF";

        Serial.printf("   [DECODED] Speed: %.1f km/h | Ignition: %s\n",
                      speedKmh, ignition);
        break;
      }

      case 65217: {  // Odometer (High Res Distance)
        uint32_t rawOdo = (uint32_t)rxFrame.data[0] |
                          ((uint32_t)rxFrame.data[1] << 8) |
                          ((uint32_t)rxFrame.data[2] << 16) |
                          ((uint32_t)rxFrame.data[3] << 24);
        if (rawOdo >= 0xFA000000UL) break;
        float odoKm = rawOdo * 0.005;
        publishMetric("odo", odoKm, lastOdoNotify);
        Serial.printf("   [DECODED] Odometer: %.2f km\n", odoKm);
        break;
      }

      case 0xFEE0: {
        if (rxFrame.data_length_code != 8 || millis() - lastOdoNotify < 1000) break;
        const uint32_t raw = uint32_t(rxFrame.data[4]) | (uint32_t(rxFrame.data[5]) << 8) |
                             (uint32_t(rxFrame.data[6]) << 16) | (uint32_t(rxFrame.data[7]) << 24);
        if (raw < 0xFA000000UL) publishMetric("odo", raw * 0.125f, lastOdoNotify);
        break;
      }
      case 0xFEFC: {
        if (rxFrame.data[1] <= 250) publishMetric("fuel", rxFrame.data[1] * 0.4f, lastFuelNotify);
        break;
      }

      case 65253: {  // Engine Hours
        uint32_t rawHours = (uint32_t)rxFrame.data[0] |
                            ((uint32_t)rxFrame.data[1] << 8) |
                            ((uint32_t)rxFrame.data[2] << 16) |
                            ((uint32_t)rxFrame.data[3] << 24);
        float hours = rawHours * 0.05;
        Serial.printf("   [DECODED] Engine Hours: %.2f hrs\n", hours);
        break;
      }
    }
#endif
  }
}
