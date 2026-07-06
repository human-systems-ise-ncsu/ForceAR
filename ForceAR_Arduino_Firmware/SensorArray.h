#ifndef SENSORARRAY_H
#define SENSORARRAY_H
#pragma once
#include "LoadCellGroup.h"

// --- PIN DEFINITIONS ---
// Group 1 (X,Y,Z)
#define GROUP1_X_DOUT  22
#define GROUP1_X_SCK   23
#define GROUP1_Y_DOUT  24
#define GROUP1_Y_SCK   25
#define GROUP1_Z_DOUT  26
#define GROUP1_Z_SCK   27

// Group 2 (X,Y,Z)
#define GROUP2_X_DOUT  30
#define GROUP2_X_SCK   31
#define GROUP2_Y_DOUT  32
#define GROUP2_Y_SCK   33
#define GROUP2_Z_DOUT  34
#define GROUP2_Z_SCK   35

// Group 3 (X,Y,Z)
#define GROUP3_X_DOUT  36
#define GROUP3_X_SCK   37
#define GROUP3_Y_DOUT  38
#define GROUP3_Y_SCK   39
#define GROUP3_Z_DOUT  40
#define GROUP3_Z_SCK   41

// Group 4 (X,Y,Z)
#define GROUP4_X_DOUT  42
#define GROUP4_X_SCK   43
#define GROUP4_Y_DOUT  44
#define GROUP4_Y_SCK   45
#define GROUP4_Z_DOUT  46
#define GROUP4_Z_SCK   47

const int NUM_GROUPS = 4;
const int NUM_CHANNELS = 12; // 4 groups * 3 axes

// Outlier clamp parameters
static constexpr int   OUTLIER_WINDOW = 8;      // sliding window size per channel
static constexpr float OUTLIER_SIGMA  = 3.0f;   // clamp if |x-mean| > k * stddev

class SensorArray {
public:
    // --- Constructor initializes each group with its pin set ---
    SensorArray();
    // --- Functions ---
    void setupAll();
    void tareAll();
    void updateAll();
    void resetWindows();
    void printCalibrationJSON();
    void printRuntimeDiagnostics(unsigned long now);

    // Fills the 37-float buffer: [Raw_12, Proc_12, Filt_12, CRC_placeholder]
    void fillDataPacket(float* buffer);

    float getOffset(int g, int i);
    float getScale(int g, int i);

private:
    LoadCellGroup groups[NUM_GROUPS];
    // Helper for outlier removal
    float clampOutlier(int channelIndex, float val);

    // Buffers for outlier filtering
    float window[NUM_CHANNELS][OUTLIER_WINDOW];
    uint8_t wCount[NUM_CHANNELS];
    uint8_t wIndex[NUM_CHANNELS];
};
#endif // SENSORARRAY_H
