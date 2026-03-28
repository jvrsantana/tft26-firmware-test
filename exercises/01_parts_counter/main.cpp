// =============================================================================
//  Exercise 01 — Parts Counter
// =============================================================================
//
//  Virtual hardware:
//    SW 0        →  io.digital_read(0)        Inductive sensor input
//    Display     →  io.write_reg(6, …)        LCD debug (see README for format)
//                   io.write_reg(7, …)
//
//  Goal:
//    Count every part that passes the sensor and show the total on the display.
//
//  Rationale:
//    Industrial inductive sensors are prone to oscillations due to electrical noise and physical vibration on the conveyor belt. 
//    This characteristic can lead to false positive counts (duplicate readings). 
//    To ensure robust signal integrity, my solution implements a non-blocking Finite State Machine (FSM) for software oscillation elimination.
//
// =============================================================================

#include <trac_fw_io.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>

constexpr uint8_t SENSOR_PORT = 0;
constexpr uint32_t DEBOUNCE_TIME_MS = 30; // 30ms is a standard industrial window to filter out mechanical vibration/chatter

enum class SensorState {
    IDLE,               // Waiting for a part (Sensor LOW)
    DEBOUNCE_RISING,    // Part detected, validating stability (Sensor HIGH)
    PART_PRESENT,       // Part validated and counted (Sensor HIGH)
    DEBOUNCE_FALLING    // Part leaving, validating stability (Sensor LOW)
};

// Updates the display registers only when explicitly called (on change).
void update_display(trac_fw_io_t& io, uint32_t count) {
    char buf[9] = {};
    std::snprintf(buf, sizeof(buf), "%8u", count);  // right-aligned, 8 chars
    
    uint32_t r6, r7;
    std::memcpy(&r6, buf + 0, 4);
    std::memcpy(&r7, buf + 4, 4);
    
    io.write_reg(6, r6);
    io.write_reg(7, r7);
}

int main() {
    trac_fw_io_t io;

    uint32_t count = 0;
    SensorState state = SensorState::IDLE;
    uint32_t state_timer = 0;

    // Initial output state (show the running total starting at 0)
    update_display(io, count);

    while (true) {
        bool sensor_high = io.digital_read(SENSOR_PORT);
        uint32_t current_time = io.millis();

        switch (state) {
            case SensorState::IDLE:
                if (sensor_high) { // Potential edge detected, start validation timer
                    state = SensorState::DEBOUNCE_RISING; 
                    state_timer = current_time;
                }
                break;

            case SensorState::DEBOUNCE_RISING:
                if (!sensor_high) { // Glitch/Noise detected: Signal dropped before timeout. Abort.
                    state = SensorState::IDLE;
                } else if (current_time - state_timer >= DEBOUNCE_TIME_MS) {  // Validation successful: Signal stable. Increment and update
                    count++;
                    update_display(io, count);
                    state = SensorState::PART_PRESENT;
                }
                break;

            case SensorState::PART_PRESENT:
                if (!sensor_high) { // Part might be leaving, start validation timer
                    state = SensorState::DEBOUNCE_FALLING;
                    state_timer = current_time;
                }
                break;

            case SensorState::DEBOUNCE_FALLING:
                if (sensor_high) { // Glitch/Bounce detected while leaving. Return to present state.
                    state = SensorState::PART_PRESENT;
                } else if (current_time - state_timer >= DEBOUNCE_TIME_MS) { // Validation successful: Part has completely left the sensor zone.
                    state = SensorState::IDLE;
                }
                break;
        }
    }
}