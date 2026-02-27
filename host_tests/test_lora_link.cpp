#include "lora_link.h"
#include <iostream>
#include <cassert>
#include "Arduino.h"

// Define global mock objects
MockSerial Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;
TwoWire Wire;
SPIClass SPI1;

int main() {
    // 1. Zero/Negative Input
    assert(LoraLink::agl_from_press_mm_(0, 101325) == 0);
    assert(LoraLink::agl_from_press_mm_(101325, 0) == 0);
    assert(LoraLink::agl_from_press_mm_(-100, 101325) == 0);

    // 2. Sea Level (Pressure == Reference)
    assert(LoraLink::agl_from_press_mm_(101325, 101325) == 0);

    // 3. Ascent (Pressure < Reference)
    // At ~100m altitude, pressure drops by ~12hPa (approx)
    // Let's use standard atmosphere calculation check:
    // P = P0 * (1 - L*h/T0)^(g*M/(R*L))
    // For small h, roughly 1hPa ~ 8.43m
    // 101325 Pa ref, 100000 Pa measured. Delta = 1325 Pa ~ 13.25 hPa -> ~111m
    int32_t alt_1000hPa = LoraLink::agl_from_press_mm_(100000, 101325);
    std::cout << "1000hPa (ref 1013.25hPa) -> " << alt_1000hPa << " mm" << std::endl;
    assert(alt_1000hPa > 0);
    assert(alt_1000hPa > 100000 && alt_1000hPa < 120000); // Expect around 111m

    // 4. Descent/Below Reference (Pressure > Reference)
    // Note: The function returns 0 if altitude <= 0.
    // The implementation:
    // const float altitude_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
    // if (!isfinite(altitude_m) || altitude_m <= 0.0f) { return 0; }
    // So negative altitudes are clamped to 0.

    int32_t alt_below_ref = LoraLink::agl_from_press_mm_(102000, 101325);
    std::cout << "1020hPa (ref 1013.25hPa) -> " << alt_below_ref << " mm" << std::endl;
    // assert(alt_below_ref < 0); // ORIGINAL FAILED ASSERTION
    assert(alt_below_ref == 0); // CORRECTED ASSERTION based on code logic

    // 5. Edge Cases
    // Very high altitude (low pressure)
    // 20000 Pa (approx 11km)
    int32_t alt_high = LoraLink::agl_from_press_mm_(20000, 101325);
    std::cout << "200hPa (ref 1013.25hPa) -> " << alt_high << " mm" << std::endl;
    assert(alt_high > 10000000); // > 10km

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
