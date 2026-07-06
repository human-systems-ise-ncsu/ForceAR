#ifndef CALIBRATION_CONFIG_H
#define CALIBRATION_CONFIG_H

// 🧠 Jordan's Notes
// This file defines HX711 calibration constants (offset and scale)
// for each axis (X, Y, Z) of every load cell group.
// You can calibrate each sensor individually using HX_calibration.ino
// and update the values below accordingly.
//
// - OFFSET = reading from sensor when unloaded (zero reference)
// - SCALE  = factor that converts raw readings to grams or newtons
//   (can be negative depending on wiring orientation)
//
// Example: if you add new groups, copy Group 1’s structure and rename as needed.
// The firmware will loop through all defined groups dynamically.

// ============================================
// 🔧 GROUP 1 CALIBRATION (currently in use)
// ============================================

// X Axis
#define HX711_GROUP1_X_OFFSET    270667
#define HX711_GROUP1_X_SCALE     -387.680023f

// Y Axis
#define HX711_GROUP1_Y_OFFSET   -150114
#define HX711_GROUP1_Y_SCALE    -392.593536f

// Z Axis
#define HX711_GROUP1_Z_OFFSET   -157430//-259549
#define HX711_GROUP1_Z_SCALE    401.030457f

#define HX711_GROUP2_X_OFFSET   452862
#define HX711_GROUP2_X_SCALE    -442.573456f
#define HX711_GROUP2_Y_OFFSET   -5014
#define HX711_GROUP2_Y_SCALE    -389.728516f
#define HX711_GROUP2_Z_OFFSET   -491383
#define HX711_GROUP2_Z_SCALE    386.6587525f

#define HX711_GROUP3_X_OFFSET   210001
#define HX711_GROUP3_X_SCALE    -409.392029f
#define HX711_GROUP3_Y_OFFSET   497833
#define HX711_GROUP3_Y_SCALE    -384.706268f
#define HX711_GROUP3_Z_OFFSET   -311284
#define HX711_GROUP3_Z_SCALE    393.661224f

#define HX711_GROUP4_X_OFFSET   34880
#define HX711_GROUP4_X_SCALE    -391.326874f
#define HX711_GROUP4_Y_OFFSET   -38319
#define HX711_GROUP4_Y_SCALE    -364.161499f
#define HX711_GROUP4_Z_OFFSET   -38766
#define HX711_GROUP4_Z_SCALE    384.035492f

// ============================================
// ⚙️ STRUCT ACCESS (for cleaner code in C++)
// ============================================
struct AxisConfig {
    long offset;
    float scale;
};

static const AxisConfig ALL_GROUPS_CONFIG[4][3] = {
    // Group 1 (X, Y, Z)
    { 
      {HX711_GROUP1_X_OFFSET, HX711_GROUP1_X_SCALE}, 
      {HX711_GROUP1_Y_OFFSET, HX711_GROUP1_Y_SCALE}, 
      {HX711_GROUP1_Z_OFFSET, HX711_GROUP1_Z_SCALE} 
    },
    // Group 2...
    { {HX711_GROUP2_X_OFFSET, HX711_GROUP2_X_SCALE}, 
      {HX711_GROUP2_Y_OFFSET, HX711_GROUP2_Y_SCALE}, 
      {HX711_GROUP2_Z_OFFSET, HX711_GROUP2_Z_SCALE} },
    // Group 3...
    { {HX711_GROUP3_X_OFFSET, HX711_GROUP3_X_SCALE}, 
      {HX711_GROUP3_Y_OFFSET, HX711_GROUP3_Y_SCALE}, 
      {HX711_GROUP3_Z_OFFSET, HX711_GROUP3_Z_SCALE} },
    // Group 4...
    { {HX711_GROUP4_X_OFFSET, HX711_GROUP4_X_SCALE}, 
      {HX711_GROUP4_Y_OFFSET, HX711_GROUP4_Y_SCALE}, 
      {HX711_GROUP4_Z_OFFSET, HX711_GROUP4_Z_SCALE} }
};
#endif
