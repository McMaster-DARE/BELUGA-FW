#include "hardware_config.h"
#include <stdexcept>
#include "sensor.h"

#define TMP117_REG_TEMP         0x00
#define TMP117_REG_CFGR         0x01
#define TMP117_REG_HIGH_LIM     0x02
#define TMP117_REG_LOW_LIM      0x03
#define TMP117_REG_EEPROM_UL    0x04
#define TMP117_REG_EEPROM1      0x05
#define TMP117_REG_EEPROM2      0x06
#define TMP117_REG_TEMP_OFFSET  0x07
#define TMP117_REG_EEPROM3      0x08
#define TMP117_REG_DEVICE_ID    0x0F
#define TMP116_DEVICE_ID        0x1116
#define TMP117_DEVICE_ID        0x0117
#define TMP117_RESOLUTION_10UC  78125
#define MICRODEGREE_PER_10MILLIDEGREE 10000


#pragma once

using namespace std;

class tmp117 : public sensor
{
    public:
    tmp117(const string& name, uint8_t i2c_addr, int16_t calib = 0 ) : 
        sensor(name, i2c_addr),
        m_calibbias(calib)
    {}

    unsigned do_test() override
    {
        bool pass = true;
        pass &= who_am_i_test();

        if (!pass)
        {
            return 0;
        }
        
        return 1;
    }

    pair<double, float> read()
    {
        int16_t temp_raw = tmp117_read_reg(TMP117_REG_TEMP);
        float temp_C = (float)temp_raw * TMP117_RESOLUTION_10UC / MICRODEGREE_PER_10MILLIDEGREE / 1000.0f;  // Convert to Celsius
        
        printf("[%.3f s] Temperature: %.2f °C\n", get_time(), temp_C);
        return make_pair(get_time(), temp_C);
    }

    private:
    int16_t tmp117_read_reg(uint8_t reg) 
    {
        uint8_t buffer[2] = {0};
        i2c_write_blocking(m_i2c_s, m_addr_s, &reg, 1, true);  // Send register address
        i2c_read_blocking(m_i2c_s, m_addr_s, buffer, 2, false); // Read 2 bytes
        return (int16_t)((buffer[0] << 8) | buffer[1]);  // Combine bytes into 16-bit value
    }

    void tmp117_write_reg(uint8_t reg, int16_t value) 
    {
        uint8_t buffer[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
        i2c_write_blocking(m_i2c_s, m_addr_s, buffer, 3, false);  // Write register address and value
    }

    void tmp117_set_calibbias() 
    {
        tmp117_write_reg(TMP117_REG_TEMP_OFFSET, m_calibbias);
    }

    bool who_am_i_test()
    {
        int16_t who_am_i = tmp117_read_reg(TMP117_REG_DEVICE_ID);
        if (who_am_i == TMP117_DEVICE_ID)
            return true;
        else
        {
            printf("WHO_AM_I value does not match.\n");
            return false;
        }
    }

    int16_t m_calibbias;
    string  m_name;
};