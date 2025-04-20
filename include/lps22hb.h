#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lps22hb_reg.h"

#include <stdio.h>
#include <stdexcept>

using namespace std;

// Class for an LPS22HB sensor
// This class consolidates all LPS22HB related operations such as:
// Configurations (Output Data Rate, Block Data Update, etc.), testing, and w/r functions 
class lps22hb : public sensor
{
    public:
    lps22hb(uint8_t i2c_addr)
    {
        m_addr_s = i2c_addr;
    }
    
    void do_init() override
    {    
        printf("Initializing LPS22HB I2C...\n");
        // Initialize the LPS22HB sensor
        m_lps22hb.write_reg = platform_write;
        m_lps22hb.read_reg = platform_read;
        m_lps22hb.mdelay = platform_delay;
        m_lps22hb.handle = this; // allows w/r functions to access the class members

        lps22hb_data_rate_set(&m_lps22hb, LPS22HB_ODR_10_Hz); 
        lps22hb_block_data_update_set(&m_lps22hb, PROPERTY_ENABLE);

        printf("LPS22HB sensor initialized.\n");
    }

    void do_test() override
    {
        // Do all tests here
        bool pass = true;
        uint8_t test_value = 0xAA; 
        uint8_t test_reg;

        pass &= who_am_i_test();
        pass &= write_test(LPS22HB_CTRL_REG1, &test_value, 1);
        pass &= read_test(LPS22HB_WHO_AM_I, &test_reg, 1);
        pass &= write_and_read_test(LPS22HB_CTRL_REG1, &test_value, 1);

        if (!pass)
        {
            printf("LPS22HB tests failed.\n");
            throw runtime_error("LPS22HB tests failed.");
        }
        else
            printf("LPS22HB tests passed.\n");
    }

    pair<float, float> do_read()
    {
        uint8_t pressure_ready, temp_ready;
        lps22hb_data_ready_get(&m_lps22hb, &pressure_ready, &temp_ready);

        if(pressure_ready && temp_ready)
        {
            uint32_t raw_pressure;
            int16_t raw_temperature;

            lps22hb_pressure_raw_get(&m_lps22hb, &raw_pressure);
            lps22hb_temperature_raw_get(&m_lps22hb, &raw_temperature);

            float pressure_hPa = lps22hb_from_lsb_to_hpa(raw_pressure);
            float temperature_degC = lps22hb_from_lsb_to_degc(raw_temperature);

            return make_pair(pressure_hPa, temperature_degC);
        }
        else
            return make_pair(0.0, 0.0);
    }

    private:
    // Platform-specific Functions for LPS22HB
    // LPS22HB is a register-based device. Meaning we need to write the register address before writing the data
    // In the case of the LPS22HB, the register address is the first byte of the data buffer
    // in `platform_write', we write the register address and the data to the device in one go
    // in `platform_read', we write the register address and then read the data from the device
    static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) 
    {
        lps22hb *self = static_cast<lps22hb *>(handle);

        uint8_t buffer[len + 1];
        buffer[0] = reg;
        for (uint16_t i = 0; i < len; i++) 
            buffer[i + 1] = bufp[i];
        
        if (i2c_write_blocking(self->m_i2c_s, self->m_addr_s, buffer, len + 1, false) == PICO_ERROR_GENERIC) 
            return -1;
        
        return 0;
    }

    static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) 
    {
        lps22hb *self = static_cast<lps22hb *>(handle);

        if (i2c_write_blocking(self->m_i2c_s, self->m_addr_s, &reg, 1, true) == PICO_ERROR_GENERIC) 
            return -1;
        
        if (i2c_read_blocking(self->m_i2c_s, self->m_addr_s, bufp, len, false) == PICO_ERROR_GENERIC) 
            return -1;
        
        return 0;
    }

    static void platform_delay(uint32_t ms) 
    {
        sleep_ms(ms);
    }

    // Testing Functions
    bool write_test(uint8_t reg, uint8_t *data, uint16_t len)
    {
        if (platform_write(this, reg, data, len) == 0)
            return true;
    
        printf("Write failed to register 0x%02X with data 0x%02X.\n", reg, *data);
        return false;
    }
    
    bool read_test(uint8_t reg, uint8_t *data, uint16_t len)
    {
        if (platform_read(this, reg, data, len) == 0)
            return true;
    
        printf("Read failed from register 0x%02X.\n", reg);
        return false;
    }
    
    bool write_and_read_test(uint8_t reg, uint8_t *data, uint16_t len)
    {
        if (write_test(reg, data, len))
        {
            uint8_t read_data;
            if (read_test(reg, &read_data, 1))
            {
                if (read_data == *data)
                    return true;
                else
                {
                    printf("Write and read values do not match.\n");
                }
            }
        }
    
        return false;
    }

    bool who_am_i_test()
    {
        uint8_t who_am_i;
        if (read_test(LPS22HB_WHO_AM_I, &who_am_i, 1))
        {
            if (who_am_i == LPS22HB_ID)
                return true;
            else
            {
                printf("WHO_AM_I value does not match.\n");
            }
        }

        return false;
    }

    stmdev_ctx_t    m_lps22hb;
};