#pragma once
#include <Arduino.h>

// ============================================================
// Purpose:
// - Smooths noisy HX711 sensor readings using a 4th-order Butterworth low-pass filter.
// - Based on a two-section (biquad) cascade structure (Direct Form I, transposed).
//
// Notes:
// - This filter reduces noise while keeping low-frequency force signals responsive.
// - You can adjust cutoff frequency by recalculating coefficients (if needed).
// ============================================================


// ------------------------------------------------------------
// 🧠 Biquad Filter Section
// Each "biquad" handles one 2nd-order filter section.
// Two of these chained = 4th-order total filter.
// ------------------------------------------------------------
struct Biquad {
  float b0{}, b1{}, b2{}, a1{}, a2{};  // filter coefficients
  float z1{0.f}, z2{0.f};              // filter state variables (delay elements)

  // Single-step filtering function
  inline float process(float x) {
    // Direct Form I (transposed) — efficient for embedded systems
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};


// ------------------------------------------------------------
// ⚙️ SOSFilter2 — Second-Order Sections Filter (2-stage cascade)
// Provides smooth, stable low-pass filtering for sensor signals.
// ------------------------------------------------------------
class SOSFilter2 {
public:
  // ----------------------------------------------------------
  // initDefault()
  // Initializes coefficients for a 4th-order Butterworth filter.
  // Cutoff ≈ 10 Hz (sampling ≈ 100 Hz typical HX711 read rate)
  // ----------------------------------------------------------
  void initDefault() {
    // Derived coefficients for Butterworth 4th-order LPF
    // Computed for 10 Hz passband; stable for typical HX711 frequencies.

    gain_ = 0.0466f;

    // --- Section 1 ---
    s1_.b0 = 1.0000f;  s1_.b1 = 2.0003f;  s1_.b2 = 1.0003f;
    s1_.a1 = -0.3290f; s1_.a2 = 0.0646f;

    // --- Section 2 ---
    s2_.b0 = 1.0000f;  s2_.b1 = 1.9997f;  s2_.b2 = 0.9997f;
    s2_.a1 = -0.4531f; s2_.a2 = 0.4663f;
  }

  // ----------------------------------------------------------
  // process(x)
  // Runs one sample through both sections of the filter.
  // Returns smoothed value.
  // ----------------------------------------------------------
  inline float process(float x) {
    float y = s1_.process(x);  // pass through section 1
    y = s2_.process(y);        // then section 2
    return gain_ * y;
  }

  // ----------------------------------------------------------
  // reset()
  // Clears internal state (use when restarting or re-taring)
  // ----------------------------------------------------------
  void reset() { 
    s1_.z1 = s1_.z2 = 0.f;
    s2_.z1 = s2_.z2 = 0.f;
  }

private:
  Biquad s1_, s2_;    // two cascaded biquad sections
  float  gain_{0.0466f};
};
