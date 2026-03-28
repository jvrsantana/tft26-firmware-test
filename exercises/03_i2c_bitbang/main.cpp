// =============================================================================
//  Exercise 03 — I2C Sensors (Bit-bang)
// =============================================================================
//
//  Virtual hardware:
//    P8 (SCL)  →  io.digital_write(8, …) / io.digital_read(8)
//    P9 (SDA)  →  io.digital_write(9, …) / io.digital_read(9)
//
//  PART 1 — TMP64 temperature sensor at I2C address 0x48
//    Register 0x0F  WHO_AM_I   — 1 byte  (expected: 0xA5)
//    Register 0x00  TEMP_RAW   — 4 bytes, big-endian int32_t, milli-Celsius
//
//  PART 2 — Unknown humidity sensor (same register layout, address unknown)
//    Register 0x0F  WHO_AM_I   — 1 byte
//    Register 0x00  HUM_RAW    — 4 bytes, big-endian int32_t, milli-percent
//
//  Goal (Part 1):
//    1. Implement an I2C master via bit-bang on P8/P9.
//    2. Read WHO_AM_I from TMP64 and confirm the sensor is present.
//    3. Read TEMP_RAW in a loop and print the temperature in °C every second.
//    4. Update display registers 6–7 with the formatted temperature string.
//
//  Goal (Part 2):
//    5. Scan the I2C bus (addresses 0x08–0x77) and print every responding address.
//    6. For each unknown device found, read its WHO_AM_I and print it.
//    7. Add the humidity sensor to the 1 Hz loop: read HUM_RAW and print %RH.
//
//  Rationale:
//    To achieve robust communication without I2C hardware, I implemented a custom Bit-Bang protocol that strictly adheres to the Open-Drain bus architecture.
//
//  Architecture:
//    1. HAL Layer: Micro-delay and raw pin manipulation.
//    2. Protocol Layer: I2C primitives (Start, Stop, WriteByte, ReadByte).
//    3. Driver Layer: Repeated-Start register reading for TMP64 and HMD10.
//    4. App Layer: Non-blocking 1Hz loop for sampling and display update.
// =============================================================================

#include <trac_fw_io.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>

// --- I2C Configuration ---
constexpr uint8_t SCL_PIN = 8;
constexpr uint8_t SDA_PIN = 9;

constexpr uint8_t TMP64_ADDR = 0x48; // Known Temp Sensor Address
constexpr uint8_t REG_TEMP_RAW = 0x00;
constexpr uint8_t REG_WHO_AM_I = 0x0F;

// =============================================================================
//  1. HAL LAYER 
// =============================================================================
// Minimal delay for I2C clock timing. In a 400kHz bus, half-period is ~1.25us.
inline void i2c_delay() {
    for (volatile int i = 0; i < 5; ++i); 
}

// Open-Drain logic: Write 0 to pull low, write 1 to release (float high).
inline void sda_high(trac_fw_io_t& io) { io.digital_write(SDA_PIN, 1); i2c_delay(); }
inline void sda_low(trac_fw_io_t& io)  { io.digital_write(SDA_PIN, 0); i2c_delay(); }
inline void scl_high(trac_fw_io_t& io) { io.digital_write(SCL_PIN, 1); i2c_delay(); }
inline void scl_low(trac_fw_io_t& io)  { io.digital_write(SCL_PIN, 0); i2c_delay(); }
inline bool sda_read(trac_fw_io_t& io) { return io.digital_read(SDA_PIN); }

// =============================================================================
//  2. PROTOCOL LAYER
// =============================================================================
void i2c_start(trac_fw_io_t& io) {
    sda_high(io);
    scl_high(io);
    sda_low(io);  // SDA falls while SCL is high
    scl_low(io);
}

void i2c_stop(trac_fw_io_t& io) {
    sda_low(io);
    scl_low(io);
    scl_high(io);
    sda_high(io); // SDA rises while SCL is high
}

// Returns true if ACK received, false if NACK
bool i2c_write_byte(trac_fw_io_t& io, uint8_t byte) {
    for (int i = 7; i >= 0; --i) {
        if ((byte >> i) & 1) sda_high(io);
        else sda_low(io);
        scl_high(io);
        scl_low(io);
    }
    // Read ACK/NACK bit
    sda_high(io); // Release SDA
    scl_high(io);
    bool ack = !sda_read(io); // 0 means ACK, 1 means NACK
    scl_low(io);
    return ack;
}

uint8_t i2c_read_byte(trac_fw_io_t& io, bool send_ack) {
    uint8_t byte = 0;
    sda_high(io); // Release SDA to read
    for (int i = 7; i >= 0; --i) {
        scl_high(io);
        if (sda_read(io)) byte |= (1 << i);
        scl_low(io);
    }
    // Send ACK (LOW) or NACK (HIGH)
    if (send_ack) sda_low(io);
    else sda_high(io);
    scl_high(io);
    scl_low(io);
    return byte;
}

// =============================================================================
//  3. DRIVER LAYER 
// =============================================================================
bool i2c_read_registers(trac_fw_io_t& io, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    i2c_start(io);
    if (!i2c_write_byte(io, (addr << 1) | 0)) { i2c_stop(io); return false; }
    if (!i2c_write_byte(io, reg)) { i2c_stop(io); return false; }
    
    // Repeated Start for the Read phase
    i2c_start(io);
    if (!i2c_write_byte(io, (addr << 1) | 1)) { i2c_stop(io); return false; }
    
    for (uint8_t i = 0; i < len; ++i) {
        bool is_last = (i == (len - 1));
        data[i] = i2c_read_byte(io, !is_last); // Master NACKs the last byte
    }
    i2c_stop(io);
    return true;
}

uint8_t read_who_am_i(trac_fw_io_t& io, uint8_t addr) {
    uint8_t id = 0;
    i2c_read_registers(io, addr, REG_WHO_AM_I, &id, 1);
    return id;
}

// =============================================================================
//  4. APPLICATION LAYER 
// =============================================================================
int main() {
    trac_fw_io_t io;

    // Enable internal pull-ups for the open-drain I2C lines
    io.set_pullup(SCL_PIN, true);
    io.set_pullup(SDA_PIN, true);

    // Initial bus idle state
    sda_high(io);
    scl_high(io);

    std::printf("--- TRACTIAN I2C INIT ---\n");

    // ── TMP64 Temperature Sensor ──
    uint8_t tmp64_id = read_who_am_i(io, TMP64_ADDR);
    if (tmp64_id == 0xA5) {
        std::printf("TMP64 found at 0x%02X (WHO_AM_I: 0x%02X)\n", TMP64_ADDR, tmp64_id);
    } else {
        std::printf("TMP64 NOT FOUND!\n");
    }

    // ── Bus Scan & HMD10 Discovery ──
    uint8_t hmd10_addr = 0x00;
    std::printf("Scanning bus (0x08 - 0x77)...\n");
    
    // The valid scan range specified in the datasheet
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        i2c_start(io);
        bool ack = i2c_write_byte(io, (addr << 1) | 0); // Write test
        i2c_stop(io);

        if (ack) {
            std::printf("Device found at address: 0x%02X\n", addr);
            if (addr != TMP64_ADDR) {
                hmd10_addr = addr;
                uint8_t hum_id = read_who_am_i(io, hmd10_addr);
                std::printf("  -> Unknown sensor identified! WHO_AM_I: 0x%02X\n", hum_id);
            }
        }
    }

    // ── The 1 Hz Non-Blocking Loop ──
    uint32_t last_read_time = io.millis();

    while (true) {
        uint32_t current_time = io.millis();

        // 1000 ms sampling interval without busy-waiting
        if (current_time - last_read_time >= 1000) {
            last_read_time = current_time;

            // Read TMP64 (Temperature)
            uint8_t t_data[4];
            if (i2c_read_registers(io, TMP64_ADDR, REG_TEMP_RAW, t_data, 4)) {
                // Reconstruct 32-bit signed integer (Big-Endian)
                int32_t temp_raw = (t_data[0] << 24) | (t_data[1] << 16) | (t_data[2] << 8) | t_data[3];
                float temp_c = temp_raw / 1000.0f;

                std::printf("TEMP: %.3f °C\n", temp_c);

                // Update Display (Registers 6-7)
                char buf[9] = {};
                std::snprintf(buf, sizeof(buf), "%8.3f", temp_c);
                uint32_t r6, r7;
                std::memcpy(&r6, buf + 0, 4);
                std::memcpy(&r7, buf + 4, 4);
                io.write_reg(6, r6);
                io.write_reg(7, r7);
            }

            // Read HMD10 (Humidity) if discovered
            if (hmd10_addr != 0x00) {
                uint8_t h_data[4];
                if (i2c_read_registers(io, hmd10_addr, REG_TEMP_RAW, h_data, 4)) {
                    // Reconstruct 32-bit signed integer (Big-Endian)
                    int32_t hum_raw = (h_data[0] << 24) | (h_data[1] << 16) | (h_data[2] << 8) | h_data[3];
                    float hum_pct = hum_raw / 1000.0f;

                    std::printf("HUM:  %.3f %%\n", hum_pct);

                    // Update Display (Registers 4-5)
                    char buf[9] = {};
                    std::snprintf(buf, sizeof(buf), "%7.3f%%", hum_pct);
                    uint32_t r4, r5;
                    std::memcpy(&r4, buf + 0, 4);
                    std::memcpy(&r5, buf + 4, 4);
                    io.write_reg(4, r4);
                    io.write_reg(5, r5);
                }
            }
        }
    }
}