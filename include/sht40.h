/*
Things I learned:
- Pico i2c hardware API within C/C++ SDK, details about how i2c works with one controller multiple slaves and scheduling
- Golden rule for includes: Every .cpp or .h file should include the headers it needs to compile independently.
- Using comment headers

- Restructured/refactored my original driver into object-oriented codebase/sensor suite
*/


/*
@file sht40.h
@author Wilson Liu, ChatGPT executive assistant
@brief Homegrown DARE driver for SHT40 sensor on Pico 2 using Pico C/C++ SDK
@date May 13, 2025
@version 2

----

Revision History:
Version            Update comments
2                  Integrated driver with existing sensor suite codebase (object-oriented) by Marwan Ali
1                  Wrote driver to use SHT40 sensor with Pico 2, development done using Pico C/C++ SDK
*/


#include "hardware_config.h"
#include "sensor.h"
#include <stdexcept>

#define MEASURE_CMD     0xFD   // Command for high-precision measurement

#pragma once
using namespace std;


// Creating sht40 class, subclass of sensor
class sht40 : public sensor
{
    public:
    sht40(const string& name, uint8_t i2c_addr) : 
        sensor(name, i2c_addr)  // Just passing parameters to sensor class constructor
    {}

    // No initialization needed for this sensor, just connect to I2C and start writing/reading
    // TODO: Test functions
    // No calibration function needed, SHT40 comes factory-calibrated


    void read() {
        uint8_t command = MEASURE_CMD;
        uint8_t buffer[6];

        // Send measurement command
        if (i2c_write_blocking(m_i2c_s, m_addr_s, &command, 1, false) < 0) {
            printf("SHT40: I2C Write failed\n");
            return -1;
        }

        sleep_ms(10);  // Wait for measurement (max 8.2ms for high precision)

        // Read 6 bytes of data into buffer
        if (i2c_read_blocking(m_i2c_s, m_addr_s, buffer, 6, false) < 0) {
            printf("SHT40: I2C Read failed\n");
            return -1;
        }

        // Convert raw temperature data
        uint16_t raw_temp = (buffer[0] << 8) | buffer[1];
        float temperature = -45 + (175.0 * raw_temp / 65535.0);

        // Convert raw humidity data
        uint16_t raw_humidity = (buffer[3] << 8) | buffer[4];
        float humidity = 100.0 * raw_humidity / 65535.0;

        printf("[%.3f s] Temperature: %.2f °C, Humidity: %.2f%%\n", get_time(), temperature, humidity);
    }
};