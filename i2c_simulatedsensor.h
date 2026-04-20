#ifndef I2C_SIMULATEDSENSOR_H
#define I2C_SIMULATEDSENSOR_H

#include <stdint.h>

// 7-bit I2C address — matches real Keller sensor default
#define SENSOR_I2C_ADDR   0x40

// Simulated pressure raw values — mirrors real Keller encoding
// Formula: P_raw = (p_bar * 32768 / 100) + 16384  (for 0-100 bar sensor)
// 0 bar  → 16384 (0x4000)
// 2 bar  → 17039 (0x42AF)
#define COUNTER_MIN       16384
#define COUNTER_MAX       17039
#define COUNTER_STEP      1

void i2c_simulatedsensor_init(void);

#endif // I2C_SIMULATEDSENSOR_H
