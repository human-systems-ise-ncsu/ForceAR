#include "BLEManager.h"
#include "SensorArray.h"

// ============================================================
// Main firmware for ForceAR (Arduino GIGA + HX711 array)
// - Initializes multiple load-cell groups (up to 12 sensors total)
// - Starts BLE communication with Foxglove/Python host
// - Sends filtered, calibrated force data at 10 Hz
// Output: 37 floats per line (12 Raw + 12 Proc + 12 Filt + 1 CRC)
// ============================================================

// --- Global objects ---
SensorArray sensors;
const char* DEVICE_NAME   = "GIGA-HX711-XYZ";

// Packet structure: 37 floats: 12 Raw, 12 Proc, 12 Filt, 1 CRC
// const int NUM_CHANNELS = 12;
const int PACKET_SIZE = NUM_CHANNELS * 3 + 1; 
float txBuffer[PACKET_SIZE];

unsigned long t_last_tx = 0;
unsigned long t_last_diag = 0;
const unsigned long DIAG_INTERVAL_MS = 2000;

void setup() {
  Serial.begin(115200);
  delay(2000);

  setupBLE(DEVICE_NAME);
  setLedMode(LedMode::BlinkBlueSlow);

  sensors.setupAll();

  // --- 1. 发送元数据 (Metadata) ---
  // Python 脚本会解析这段 JSON
  Serial.println("\n---START_METADATA---");
  Serial.println("{");
  Serial.print("  \"firmware\": \"v1.0\",\n");
  Serial.print("  \"groups\": [\n");
  
  for (int g = 0; g < NUM_GROUPS; g++) {
    Serial.print("    { \"id\": "); Serial.print(g); Serial.print(", \"axes\": [");
    for (int i=0; i<3; i++) {
        // 获取 LoadCellGroup 里的 offset 和 scale (需要在 LoadCellGroup 加 getter)
        // 这里假设你有 getter，或者直接访问 public 变量
        // 示例：
        Serial.print("{ \"off\": "); Serial.print(sensors.getOffset(g, i));
        Serial.print(", \"scl\": "); Serial.print(sensors.getScale(g, i));
        Serial.print(" }");
        if(i<2) Serial.print(", ");
    }
    Serial.print("] }");
    if (g < NUM_GROUPS - 1) Serial.print(",");
    Serial.println();
  }
  Serial.println("  ]");
  Serial.println("}");
  Serial.println("---END_METADATA---");
  // --- 元数据发送完毕 ---

  setLedMode(LedMode::BlinkBlueSlow);
}

void loop() {
  // ------------------------------------------------------------
  // Handle Bluetooth events (non-blocking)
  // ------------------------------------------------------------
  handleBLE();
  // bleDebugHeartbeat();   // logs connect/disconnect events

  // ------------------------------------------------------------
  // Poll HX711s — internal is_ready() prevents blocking
  // ------------------------------------------------------------
  sensors.updateAll();

  // ------------------------------------------------------------
  // 3. Transmit Data (Fixed Rate, e.g., ~80Hz max, constrained by Serial baud)
  // Since HX711 is slow (10Hz or 80Hz), we send whenever new data is mostly ready
  // or use a fixed timer. Here we use 100Hz cap to avoid flooding if no data.
  // ------------------------------------------------------------
  unsigned long now = millis();
  if (now - t_last_tx >= TX_INTERVAL_MS) {// 100ms = 10Hz max
    t_last_tx = now;

    // Collect data from all groups
    sensors.fillDataPacket(txBuffer);

    // Calculate CRC (Simple Sum Modulo 256 for basic integrity)
    // You can implement more complex CRC32 if needed
    int checksum = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++) {
        checksum += (int)txBuffer[i];
    }
    txBuffer[PACKET_SIZE - 1] = (float)(checksum % 256); // Store CRC as the last float

    // --- A. Serial Output (CSV for Python) ---
    for (int i = 0; i < PACKET_SIZE; i++) {
      Serial.print(txBuffer[i], 2); // 2 decimal places is enough for float
      if (i < PACKET_SIZE - 1) Serial.print(",");
    }
    Serial.println(); // Newline terminates the packet

    // --- B. BLE Output ---
    if (isConnected()) {
      // Option 1: Send binary (more efficient)
      //sendData((uint8_t*)txBuffer, sizeof(txBuffer));
      bleSendRaw((uint8_t*)&txBuffer[0], 48);
      bleSendProc((uint8_t*)&txBuffer[12], 48);
      bleSendFilt((uint8_t*)&txBuffer[24], 48);

      setLedMode(LedMode::SolidGreen);
    } else {
       setLedMode(LedMode::BlinkBlueSlow);
      
      // Option 2: Send Text (easier for generic debugging, matches Serial)
      // Constructing a 200+ char string on Arduino can be risky with memory.
      // Ideally, Python BLE reader should accept Binary.
      // For now, we reuse the binary channel but Python side needs to unpack struct.
      // sendData((uint8_t*)txBuffer, sizeof(txBuffer));
    }
  }

  if (now - t_last_diag >= DIAG_INTERVAL_MS) {
    t_last_diag = now;
    sensors.printRuntimeDiagnostics(now);
  }
}
