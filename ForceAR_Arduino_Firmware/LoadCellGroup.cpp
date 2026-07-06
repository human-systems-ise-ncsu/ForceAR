#include "LoadCellGroup.h"
#include "BLEManager.h"

static bool waitReadyWithBleService(HX711& sensor, uint32_t timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (sensor.is_ready()) return true;
    handleBLE();
    delay(1);
  }
  return sensor.is_ready();
}

static void setupSensor(HX711& sensor, float scale) {
  if (waitReadyWithBleService(sensor, 100)) {
    sensor.set_gain(128);
  }
  sensor.set_scale(scale);
}

static bool readAverageWithBleService(HX711& sensor, int samples, long& value) {
  long sum = 0;
  int count = 0;

  for (int i = 0; i < samples; ++i) {
    if (!waitReadyWithBleService(sensor, 500)) break;
    sum += sensor.read();
    count++;
    handleBLE();
  }

  if (count == 0) return false;
  value = sum / count;
  return true;
}

// ===============================================
// Constructor
// Each LoadCellGroup represents a 3-axis set of HX711 sensors.
// ===============================================
LoadCellGroup::LoadCellGroup(int dx, int sx, int dy, int sy, int dz, int sz)
: raw{0.0f, 0.0f, 0.0f},
  proc{0.0f, 0.0f, 0.0f},
  filt{0.0f, 0.0f, 0.0f},
  isXReady(false),
  isYReady(false),
  isZReady(false),
  pinDX(dx),
  pinSX(sx),
  pinDY(dy),
  pinSY(sy),
  pinDZ(dz),
  pinSZ(sz),
  sensorX(),
  sensorY(),
  sensorZ(),
  filterX(),
  filterY(),
  filterZ(),
  updateCount{0, 0, 0},
  lastUpdateMs{0, 0, 0} {
}

// ===============================================
// Setup - Gain and Calibration
// Called once during initialization.
// ===============================================
void LoadCellGroup::setup(float scaleX, float scaleY, float scaleZ) {
  // Use the default HX711 timing used by the earlier post-pin-reassignment code.
  sensorX.begin(pinDX, pinSX);
  handleBLE();
  sensorY.begin(pinDY, pinSY);
  handleBLE();
  sensorZ.begin(pinDZ, pinSZ);
  handleBLE();

  // Initialize Filters
  filterX.initDefault();
  filterY.initDefault();
  filterZ.initDefault();

  setupSensor(sensorX, scaleX);
  setupSensor(sensorY, scaleY);
  setupSensor(sensorZ, scaleZ);

  // Set offest， not necessary here since we tare later
}

// ===============================================
// Internal Helper: Safely performs tare on a single sensor
// Returns true if tare succeeded (sensor ready and zeroed).
// ===============================================
bool LoadCellGroup::safeTare(HX711& sensor, const char* axisName) {
    const uint32_t TIMEOUT_MS = 3000; // 3 seconds timeout
    
    //Serial.print("  ["); Serial.print(axisName); Serial.print("] Checking readiness... ");
    
    // 1. wait for ready， prevent from blocking at read
    long offset = 0;
    if (waitReadyWithBleService(sensor, TIMEOUT_MS) &&
        readAverageWithBleService(sensor, TARE_SAMPLES, offset)) {
      sensor.set_offset(offset);
        //sensor.tare();
        //Serial.println("OK. Tared.");
      return true;
    } else {
        //Serial.println("FAILED (Timeout).");
        return false;
    }
}


// ===============================================
// Tare Function (Revised)
// Zeros all three sensors for this group if they are ready.
// Returns the count of successfully tared sensors.
// ===============================================
int LoadCellGroup::tare() {
    int workingCount = 0;
    
    // X Axis
    if (safeTare(sensorX, "X")) {
        workingCount++;
        isXReady = true;
    } else {
        isXReady = false;
    }

    // Y Axis
    if (safeTare(sensorY, "Y")) {
        workingCount++;
        isYReady = true;
    } else {
        isYReady = false;
    }

    // Z Axis
    if (safeTare(sensorZ, "Z")) {
        workingCount++;
        isZReady = true;
    } else {
        isZReady = false;
    }
    
    return workingCount;
}

// ===============================================
// Sanitize Function (internal helper)
// Ensures values are valid (not NaN or infinite).
// ===============================================
static inline float sanitizeSample(float v, float mean, bool has_mean) {
  if (isnan(v) || isinf(v)) {
    return has_mean ? mean : 0.f;
  }
  return v;
}

void LoadCellGroup::updateChannel(HX711& sensor, SOSFilter2& filter, int idx) {
  if (sensor.is_ready()) {
    // 1. Get Raw (Direct ADC count)
    // read() returns long, we cast to float for uniformity
    long r = sensor.read(); 
    raw[idx] = (float)r;

    // 2. Calculate Processed
    // Formula: (Raw - Offset) / Scale
    // We need to access the calibration constants. 
    // Ideally, pass these into updateChannel or look them up.
    // For simplicity, let's assume HX711 library holds offset/scale:
    // sensor.set_scale() / set_offset() should be called in setup().
    // If using the config file defines directly:
    
    double offset = sensor.get_offset(); 
    float scale = sensor.get_scale();
    if (scale == 0) scale = 1.0f; // Prevent div by zero

    float p = (float)(r - offset) / scale;
    proc[idx] = p;

    // 3. Calculate Filtered
    filt[idx] = filter.process(p);

    updateCount[idx]++;
    lastUpdateMs[idx] = millis();
  }
}

// ===============================================
// Read Function
// Reads forces from each sensor and applies EMA smoothing.
// ===============================================
void LoadCellGroup::read() {
  // X Axis (Index 0)
  updateChannel(sensorX, filterX, 0);
  
  // Y Axis (Index 1)
  updateChannel(sensorY, filterY, 1);
  
  // Z Axis (Index 2)
  updateChannel(sensorZ, filterZ, 2);
}

// ===============================================
// Get Offset
// Helper to retrieve the current tare offset for metadata
// axis: 0=X, 1=Y, 2=Z
// ===============================================
long LoadCellGroup::getOffset(int axis) {
    switch (axis) {
        case 0: return sensorX.get_offset();
        case 1: return sensorY.get_offset();
        case 2: return sensorZ.get_offset();
        default: return 0;
    }
}

// ===============================================
// Get Scale
// Helper to retrieve the current scale factor for metadata
// axis: 0=X, 1=Y, 2=Z
// ===============================================
float LoadCellGroup::getScale(int axis) {
    switch (axis) {
        case 0: return sensorX.get_scale();
        case 1: return sensorY.get_scale();
        case 2: return sensorZ.get_scale();
        default: return 1.0f;
    }
}

bool LoadCellGroup::getTareReady(int axis) {
    switch (axis) {
        case 0: return isXReady;
        case 1: return isYReady;
        case 2: return isZReady;
        default: return false;
    }
}

uint32_t LoadCellGroup::getUpdateCount(int axis) {
    if (axis < 0 || axis >= 3) return 0;
    return updateCount[axis];
}

uint32_t LoadCellGroup::getLastUpdateMs(int axis) {
    if (axis < 0 || axis >= 3) return 0;
    return lastUpdateMs[axis];
}

int LoadCellGroup::getDataPinState(int axis) {
    int pin = getDataPin(axis);
    if (pin < 0) return -1;
    return digitalRead(pin);
}

int LoadCellGroup::getClockPinState(int axis) {
    int pin = getClockPin(axis);
    if (pin < 0) return -1;
    return digitalRead(pin);
}

int LoadCellGroup::getDataPin(int axis) {
    switch (axis) {
        case 0: return pinDX;
        case 1: return pinDY;
        case 2: return pinDZ;
        default: return -1;
    }
}

int LoadCellGroup::getClockPin(int axis) {
    switch (axis) {
        case 0: return pinSX;
        case 1: return pinSY;
        case 2: return pinSZ;
        default: return -1;
    }
}
