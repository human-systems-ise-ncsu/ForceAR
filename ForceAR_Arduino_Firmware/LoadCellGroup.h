#ifndef LOADCELLGROUP_H
#define LOADCELLGROUP_H

#include <Arduino.h>
#include "HX711.h"
#include "Filters.h"
#include "CalibrationConfig.h"  // includes calibration offsets/scales

#define TARE_SAMPLES 9 //
// ============================================================
// LoadCellGroup Class
// Represents one group of three HX711 load cells (X, Y, Z axes).
// Each group can be initialized with its own DOUT/SCK pin pairs.
// ============================================================
class LoadCellGroup {
public:
  // ------------------------------------------------------------
  // Constructor
  // Pass in 3 pairs of pins (DOUT, SCK) for X, Y, Z load cells.
  // ------------------------------------------------------------
  LoadCellGroup(int dx, int sx, int dy, int sy, int dz, int sz);

  // ------------------------------------------------------------
  // Setup
  // Initialize HX711 modules, apply calibration values, and set gain.
  // ------------------------------------------------------------
  void setup(float scaleX, float scaleY, float scaleZ);

  // ------------------------------------------------------------
  // Tare
  // Zero out the current readings for all three sensors.
  // ------------------------------------------------------------
  int tare();
  bool safeTare(HX711& sensor, const char* axisName);

  // ------------------------------------------------------------
  // Read
  // Fetch latest force readings from each HX711 and apply smoothing.
  // ------------------------------------------------------------
  void read();

  long getOffset(int axis);
  float getScale(int axis);
  bool getTareReady(int axis);
  uint32_t getUpdateCount(int axis);
  uint32_t getLastUpdateMs(int axis);
  int getDataPinState(int axis);
  int getClockPinState(int axis);


  // ------------------------------------------------------------
  // Public Data
  // These variables hold the latest filtered readings.
  // ------------------------------------------------------------
  // 1. Raw (Direct from ADC, no scale, no offset)
  float raw[3]; 
  
  // 2. Processed (Raw - Offset) / Scale
  float proc[3];

  // 3. Filtered (Processed passed through Butterworth LPF)
  float filt[3];

  bool isXReady;
  bool isYReady;
  bool isZReady;

private:
  int pinDX;
  int pinSX;
  int pinDY;
  int pinSY;
  int pinDZ;
  int pinSZ;

  // ------------------------------------------------------------
  // HX711 Sensor Objects
  // Each axis has its own amplifier board.
  // ------------------------------------------------------------
  HX711 sensorX;
  HX711 sensorY;
  HX711 sensorZ;

  // Filters
  SOSFilter2 filterX;
  SOSFilter2 filterY;
  SOSFilter2 filterZ;

  uint32_t updateCount[3];
  uint32_t lastUpdateMs[3];

  int getDataPin(int axis);
  int getClockPin(int axis);

  // Internal helper to reduce code duplication
  // Returns tuple-like update: updates raw, proc, filt in place
  void updateChannel(HX711& sensor, SOSFilter2& filter, int idx);
};

#endif
