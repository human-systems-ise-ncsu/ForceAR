#include "BLEManager.h"
#include <Arduino.h>

BLEService svc(BLE_SERVICE_UUID);
// Buffer size: 37 floats * 4 bytes = 148 bytes. 256 is safe.
// BLECharacteristic chr(BLE_CHARACTERISTIC_UUID, BLERead | BLENotify, 256);
BLECharacteristic chrRaw(UUID_RAW,   BLERead | BLENotify, 48);
BLECharacteristic chrProc(UUID_PROC, BLERead | BLENotify, 48);
BLECharacteristic chrFilt(UUID_FILT, BLERead | BLENotify, 48);

static LedMode ledMode = LedMode::Off;
static unsigned long ledLast = 0;
static bool ledPinsReady = false;

static void initLedPins() {
  if (ledPinsReady) return;
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  ledPinsReady = true;
}

void setupBLE(const char* deviceName) {
  initLedPins();

  if (!BLE.begin()) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    while (1); // Halt if BLE fails
  }

  BLE.setLocalName(deviceName);
  BLE.setAdvertisedService(svc);
  
  // svc.addCharacteristic(chr);
  // Add characteristics to service
  svc.addCharacteristic(chrRaw);
  svc.addCharacteristic(chrProc);
  svc.addCharacteristic(chrFilt);

  BLE.addService(svc);

  BLE.advertise();
  setLedMode(LedMode::BlinkBlueSlow);
}

void handleBLE() {
  BLE.poll();
  updateLED();
}

void bleSendRaw(const uint8_t* data, size_t size) {
  if (BLE.connected()) chrRaw.writeValue(data, size);
}

void bleSendProc(const uint8_t* data, size_t size) {
  if (BLE.connected()) chrProc.writeValue(data, size);
}

void bleSendFilt(const uint8_t* data, size_t size) {
  if (BLE.connected()) chrFilt.writeValue(data, size);
}

bool isConnected() {
  return BLE.connected();
}

void setLedMode(LedMode m) {
  initLedPins();
  ledMode = m;
  // Initialize LED pins state immediately if needed
}

void updateLED() {
  initLedPins();

  // Simple LED logic for Arduino GIGA (Active LOW usually, implies LOW=ON)
  // Adjust HIGH/LOW based on your specific wiring/board definition
  unsigned long now = millis();
  
  switch (ledMode) {
    case LedMode::BlinkBlueSlow:
      if (now - ledLast >= 500) {
        ledLast = now;
        static bool on = false; on = !on;
        digitalWrite(LED_BLUE, on ? LOW : HIGH);
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, HIGH);
      }
      break;
    case LedMode::BlinkRedFast:
      if (now - ledLast >= 150) {
        ledLast = now;
        static bool on = false; on = !on;
        digitalWrite(LED_RED, on ? LOW : HIGH);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, HIGH);
      }
      break;
    case LedMode::SolidGreen:
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      break;
    case LedMode::SolidRed:
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, HIGH);
      break;
    case LedMode::SolidYellow:
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, HIGH);
      break;
    case LedMode::Off:
    default:
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      break;
  }
}
