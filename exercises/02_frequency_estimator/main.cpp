// =============================================================================
//  Challenge 02 — Frequency Estimator
// =============================================================================
//
//  Virtual hardware:
//    ADC Ch 0  →  io.analog_read(0)      Process sensor signal (0–4095)
//    OUT reg 3 →  io.write_reg(3, …)     Frequency estimate in centiHz
//                                        e.g. write_reg(3, 4733) = 47.33 Hz
//
//  Goal:
//    Measure the frequency of the signal on ADC channel 0 and publish your
//    estimate continuously via register 3.
//
//  Rationale:
//    Raw analog signals are subject to extreme mechanical and electrical noise.
//    To achieve both ±0.5 Hz accuracy and immunity to transients, I implement a 3-layer defense mechanism:
//     1. IIR Low-Pass Filter on the raw ADC to smooth high-frequency noise.
//     2. Dynamic Envelope Tracking to set Schmitt Trigger thresholds proportionally (25% and 75%), guaranteeing immunity to amplitude shifts.
//     3. The "3-Strike" Outlier Rejection: If a measured period deviates massively from the EMA, it is quarantined. Only if it persists for 3 consecutive cycles is it accepted as a true "step change" (<1s convergence).
//
// =============================================================================

#include <trac_fw_io.hpp>
#include <cstdint>
#include <cmath>

constexpr uint32_t SAMPLE_RATE_MS = 1;
constexpr uint32_t TIMEOUT_MS = 2000;

int main() {
    trac_fw_io_t io;

    uint32_t last_sample_time = io.millis();
    uint32_t last_edge_time = 0;
    bool is_high = false;

    // 1. Signal Tracking (IIR & Envelope)
    uint32_t filtered_adc_x16 = 2048 << 4; // Fast IIR filter base
    int32_t sig_max = 2048;
    int32_t sig_min = 2048;

    // 2. Period Tracking
    uint32_t filtered_period_x1000 = 0;
    uint32_t last_published_freq = 0xFFFFFFFF;
    
    // 3. Disturbance Rejection State
    uint8_t strike_count = 0;

    while (true) {
        uint32_t current_time = io.millis();

        if (current_time - last_sample_time >= SAMPLE_RATE_MS) {
            last_sample_time = current_time;
            
            uint16_t raw_adc = io.analog_read(0);

            // Layer 1: IIR Low-Pass Filter
            filtered_adc_x16 = filtered_adc_x16 - (filtered_adc_x16 >> 4) + raw_adc;
            uint16_t adc_val = filtered_adc_x16 >> 4;

            // Layer 2: Dynamic Envelope Tracking 
            if (adc_val > sig_max) sig_max = adc_val;
            else if (sig_max > 0) sig_max--;

            if (adc_val < sig_min) sig_min = adc_val;
            else if (sig_min < 4095) sig_min++;

            int32_t amplitude = sig_max - sig_min;
            if (amplitude < 100) amplitude = 100; // Prevent division by zero on flatline

            uint16_t threshold_high = sig_min + (amplitude * 3) / 4; // Trigger at 75% height
            uint16_t threshold_low  = sig_min + (amplitude * 1) / 4; // Reset at 25% height

            // Schmitt Trigger Edge Detection
            bool edge_detected = false;
            if (!is_high && adc_val > threshold_high) {
                is_high = true;
                edge_detected = true;
            } else if (is_high && adc_val < threshold_low) {
                is_high = false;
            }

            // Layer 3: Period Calculation & Disturbance Rejection
            if (edge_detected) {
                if (last_edge_time > 0) {
                    uint32_t raw_period_ms = current_time - last_edge_time;
                    uint32_t raw_period_x1000 = raw_period_ms * 1000;

                    if (filtered_period_x1000 == 0) {
                        filtered_period_x1000 = raw_period_x1000; // Initialization
                    } else {
                        // Check if the new period is a massive anomaly (> 25% shift)
                        int32_t deviation = std::abs((int32_t)raw_period_x1000 - (int32_t)filtered_period_x1000);
                        
                        if (deviation > (filtered_period_x1000 / 4)) {
                            // Anomaly detected! Do NOT update the estimate yet.
                            strike_count++;
                            
                            // If anomaly persists for 3 waves, it's a real step change.
                            if (strike_count >= 3) {
                                filtered_period_x1000 = raw_period_x1000;
                                strike_count = 0;
                            }
                        } else {
                            // Normal behavior: Reset strikes and apply smooth EMA (for ±0.5 Hz precision)
                            strike_count = 0;
                            filtered_period_x1000 = (filtered_period_x1000 * 7 + raw_period_x1000) / 8;
                        }
                    }

                    // Publish to Output
                    if (strike_count == 0 && filtered_period_x1000 > 0) {
                        uint32_t freq_cHz = 100000000 / filtered_period_x1000;
                        if (freq_cHz != last_published_freq) {
                            io.write_reg(3, freq_cHz);
                            last_published_freq = freq_cHz;
                        }
                    }
                }
                last_edge_time = current_time;
            }
        }

        // Signal Loss / Machine Shutdown Guard
        if (last_edge_time > 0 && (current_time - last_edge_time > TIMEOUT_MS)) {
            if (last_published_freq != 0) {
                io.write_reg(3, 0);
                last_published_freq = 0;
                filtered_period_x1000 = 0;
                strike_count = 0;
            }
            last_edge_time = 0; 
        }
    }
}