#ifndef BLEMANAGER_H
#define BLEMANAGER_H

#include <ArduinoBLE.h>

// BLE UUIDs
static const char* BLE_SERVICE_UUID = "12345678-1234-5678-9abc-def012345678";
// Three separate Characteristics
static const char* UUID_RAW  = "12345678-1234-5678-9abc-def012345678"; 
static const char* UUID_PROC = "12345678-1234-5678-9abc-def012345679";
static const char* UUID_FILT = "12345678-1234-5678-9abc-def01234567A";
// Transmission interval
static const uint16_t TX_INTERVAL_MS = 100; // 100ms = 10Hz

// --- LED Modes definition ---
enum class LedMode {
  Off,
  BlinkBlueSlow, // Advertising
  BlinkRedFast,  // Error
  SolidGreen,     // Connected
  SolidRed,
  SolidYellow
};

// --- Function Prototypes ---
void setupBLE(const char* deviceName);
void handleBLE();
// void sendData(const uint8_t* data, size_t size);
bool isConnected();
void updateLED();
void setLedMode(LedMode mode); // Helper to change LED state

void bleSendRaw(const uint8_t* data, size_t size);
void bleSendProc(const uint8_t* data, size_t size);
void bleSendFilt(const uint8_t* data, size_t size);

#endif
