// PERSONAL NOTES 
// 10/12/25 — Updated by Jordan (finalized)
// - Removed Serial.printf (unsupported on GIGA) → now uses Serial.print()
// - Replaced private HX711 access with isXReady(), isYReady(), isZReady()
// - Added missing closing braces
// - Fully tested for clean compile on Arduino GIGA

#include "SensorArray.h"
#include "BLEManager.h"
#include <string.h>
#include <math.h>

static void delayWithBleService(uint32_t delayMs) {
  unsigned long start = millis();
  while (millis() - start < delayMs) {
    handleBLE();
    delay(1);
  }
}

// --- MAIN INITIALIZATION FUNCTION ---
SensorArray::SensorArray() : groups{
    LoadCellGroup(GROUP1_X_DOUT, GROUP1_X_SCK, GROUP1_Y_DOUT, GROUP1_Y_SCK, GROUP1_Z_DOUT, GROUP1_Z_SCK),
    LoadCellGroup(GROUP2_X_DOUT, GROUP2_X_SCK, GROUP2_Y_DOUT, GROUP2_Y_SCK, GROUP2_Z_DOUT, GROUP2_Z_SCK),
    LoadCellGroup(GROUP3_X_DOUT, GROUP3_X_SCK, GROUP3_Y_DOUT, GROUP3_Y_SCK, GROUP3_Z_DOUT, GROUP3_Z_SCK),
    LoadCellGroup(GROUP4_X_DOUT, GROUP4_X_SCK, GROUP4_Y_DOUT, GROUP4_Y_SCK, GROUP4_Z_DOUT, GROUP4_Z_SCK)
} {
    resetWindows();
}
// This runs once at startup to initialize all load cell groups
// and check which sensors are working or failed
void SensorArray::setupAll() {
  //Serial.println("Initializing all load cell groups...");

  delayWithBleService(1000); // short delay to make sure everything powers on cleanly

  int totalSensors = NUM_GROUPS * 3;   // total possible sensors (4 groups * 3 sensors = 12)
  int totalWorking = 0;                // keeps track of how many sensors actually respond

  // --- LOOP THROUGH EACH GROUP OF 3 LOAD CELLS ---
  for (int g = 0; g < NUM_GROUPS; ++g) {
    //Serial.print("Group "); Serial.print(g + 1);
    //Serial.println(" setup... ");

    groups[g].setup(ALL_GROUPS_CONFIG[g][0].scale, ALL_GROUPS_CONFIG[g][1].scale, ALL_GROUPS_CONFIG[g][2].scale);

    //groups[g].setup();    // runs setup() for that LoadCellGroup (each has 3 HX711s)
    int working = groups[g].tare();     // tare that group, counter for this specific group

    totalWorking += working;

    //Serial.print("Group "); Serial.print(g + 1);
    //Serial.print(" active sensors: "); Serial.print(working);
    //Serial.println();
  }

  // --- PRINT TOTAL SYSTEM STATUS ---
  //Serial.print("Total working sensors: ");
  //Serial.print(totalWorking);
  //Serial.print(" / "); Serial.println(totalSensors);
  //Serial.println();

  // --- LED COLOR FEEDBACK (uses setLEDColor() from BLEManager.cpp) ---
  if (totalWorking == 0) {
    setLedMode(LedMode::SolidRed);
  } else if (totalWorking < totalSensors) {
    setLedMode(LedMode::SolidYellow);
  } else {
    setLedMode(LedMode::BlinkBlueSlow); // sensors OK, waiting for BLE connection
  }

  // --- WARM-UP DELAY (IMPORTANT FOR STABILITY) ---
  //Serial.println("Stabilizing sensors (2 secondes delay)...");
  delayWithBleService(2000);
  //Serial.println("Initialization complete!");

  resetWindows();
}

// --- TARE FUNCTION ---
// This zeros out all groups when called manually
void SensorArray::tareAll() {
  //Serial.println("Taring all sensors...");
  for (int g = 0; g < NUM_GROUPS; ++g) {
    groups[g].tare();
  }
  resetWindows();
}

// --- MAIN UPDATE FUNCTION ---
// Reads live data from each group and applies filters
void SensorArray::updateAll() {
  for (int g = 0; g < NUM_GROUPS; ++g) {
    groups[g].read(); // read from each load cell group
  }
}

float SensorArray::getOffset(int g, int i){return static_cast<float>(groups[g].getOffset(i));}

float SensorArray::getScale(int g, int i){return groups[g].getScale(i);}

void SensorArray::printRuntimeDiagnostics(unsigned long now) {
    Serial.print("#DIAG ms=");
    Serial.print(now);

    Serial.print(" tare=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            Serial.print(groups[g].getTareReady(axis) ? 1 : 0);
        }
    }

    Serial.print(" updates=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            Serial.print(groups[g].getUpdateCount(axis));
        }
    }

    Serial.print(" age_ms=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            uint32_t last = groups[g].getLastUpdateMs(axis);
            long age = last == 0 ? -1 : static_cast<long>(now - last);
            Serial.print(age);
        }
    }

    Serial.print(" dout=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            Serial.print(groups[g].getDataPinState(axis));
        }
    }

    Serial.print(" sck=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            Serial.print(groups[g].getClockPinState(axis));
        }
    }

    Serial.print(" raw=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            Serial.print(groups[g].raw[axis], 0);
        }
    }

    Serial.print(" offset=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            Serial.print(groups[g].getOffset(axis));
        }
    }

    Serial.print(" scale=");
    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            if (g > 0 || axis > 0) Serial.print(",");
            Serial.print(groups[g].getScale(axis), 3);
        }
    }

    Serial.println();
}

// --- Helpers ---
float SensorArray::clampOutlier(int channelIndex, float val) {
    // 简单的统计滤波器实现
    // 更新窗口
    window[channelIndex][wIndex[channelIndex]] = val;
    wIndex[channelIndex] = (wIndex[channelIndex] + 1) % OUTLIER_WINDOW;
    if (wCount[channelIndex] < OUTLIER_WINDOW) wCount[channelIndex]++;

    if (wCount[channelIndex] < 3) return val; // 数据太少不处理

    // 计算均值和方差
    float sum = 0, sumSq = 0;
    for(int i=0; i<wCount[channelIndex]; i++) {
        float v = window[channelIndex][i];
        sum += v;
        sumSq += v*v;
    }
    float mean = sum / wCount[channelIndex];
    float variance = (sumSq / wCount[channelIndex]) - (mean * mean);
    float stddev = sqrt(variance > 0 ? variance : 0);

    // 判定异常
    if (abs(val - mean) > OUTLIER_SIGMA * stddev) {
        return mean; // 用均值替代异常值
    }
    return val;
}

void SensorArray::fillDataPacket(float* buffer) {
    int raw_offset = 0;
    int proc_offset = 12;
    int filt_offset = 24;

    for (int g = 0; g < NUM_GROUPS; g++) {
        for (int axis = 0; axis < 3; axis++) {
            int chIdx = g * 3 + axis;
            
            // 1. 获取新产生的数据
            float r = groups[g].raw[axis]; // 修正：使用数组访问
            float p = groups[g].proc[axis];
            float f = groups[g].filt[axis];

            // 2. 可选：在这里对 Raw 数据做 Outlier Clamping
            // r = clampOutlier(chIdx, r); 
            // p = clampOutlier(chIdx, p); //或者对scale之后有物理意义的数据进行clamping
            // 注意：通常 clamp 应该在 filter 之前，或者只针对显示
            // 这里我们保持 buffer 原样输出

            buffer[raw_offset + chIdx]  = r;
            buffer[proc_offset + chIdx] = p;
            buffer[filt_offset + chIdx] = f;
        }
    }
}

void SensorArray::resetWindows() {
  memset(wCount, 0, sizeof(wCount));
  memset(wIndex, 0, sizeof(wIndex));
  for(int i=0; i<NUM_CHANNELS; i++) {
    for(int j=0; j<OUTLIER_WINDOW; j++) window[i][j] = 0.0f;
  }
}

void SensorArray::printCalibrationJSON() {
    for (int g = 0; g < NUM_GROUPS; g++) {
        Serial.print("    { \"id\": "); Serial.print(g); Serial.print(", \"axes\": [");
        for (int i=0; i<3; i++) {
            long off = groups[g].getOffset(i); 
            float scl = groups[g].getScale(i);

            Serial.print("{ \"off\": "); Serial.print(off);
            Serial.print(", \"scl\": "); Serial.print(scl);
            Serial.print(" }");
            if(i<2) Serial.print(", ");
        }
        Serial.print("] }");
        if (g < NUM_GROUPS - 1) Serial.print(",");
        Serial.println();
    }
}
