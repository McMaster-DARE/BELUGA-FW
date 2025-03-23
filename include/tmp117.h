#include "hardware_config.h"
#include <stdexcept>

// TMP117 Register Addresses
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

// TMP117 Device IDs
#define TMP116_DEVICE_ID        0x1116
#define TMP117_DEVICE_ID        0x0117

// TMP117 Resolution
#define TMP117_RESOLUTION_10UC  78125
#define MICRODEGREE_PER_10MILLIDEGREE 10000

// TMP117 I2C Address (default is 0x48, can be changed via ADD pins)
#define TMP117_I2C_ADDR         0x48

using namespace std;

class tmp117
{
    public:
    tmp117(i2c_inst_t *i2c, unsigned sda_pin, unsigned scl_pin, uint8_t i2c_addr, unsigned baud_rate) :
        m_i2c_port(i2c), 
        m_i2c_addr(i2c_addr),
        m_calibbias(0),
        m_sda_pin(sda_pin),
        m_scl_pin(scl_pin),
        m_baud_rate(baud_rate)
    {}

    void do_init()
    {
        // Initialize I2C
        i2c_init(m_i2c_port, m_baud_rate);
        gpio_set_function(m_sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(m_scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(m_sda_pin);
        gpio_pull_up(m_scl_pin);

        // Initialize the TMP117 sensor
        tmp117_init();
    }

    void do_test()
    {
        // Do all tests here
        bool pass = true;

        pass &= who_am_i_test();

        if (!pass)
        {
            printf("TMP117 tests failed.\n");
            throw runtime_error("TMP117 tests failed.");
        }
        else
            printf("TMP117 tests passed.\n");
    }

    float read_temp()
    {
        return tmp117_read_temp();
    }

    private:
    // Function to read a 16-bit register from TMP117
    int16_t tmp117_read_reg(uint8_t reg) 
    {
        uint8_t buffer[2] = {0};
        i2c_write_blocking(m_i2c_port, m_i2c_addr, &reg, 1, true);  // Send register address
        i2c_read_blocking(m_i2c_port, m_i2c_addr, buffer, 2, false); // Read 2 bytes
        return (int16_t)((buffer[0] << 8) | buffer[1]);  // Combine bytes into 16-bit value
    }

    // Function to write a 16-bit register to TMP117
    void tmp117_write_reg(uint8_t reg, int16_t value) 
    {
        uint8_t buffer[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
        i2c_write_blocking(m_i2c_port, m_i2c_addr, buffer, 3, false);  // Write register address and value
    }

    // Function to read the temperature from TMP117
    float tmp117_read_temp() 
    {
        int16_t raw_temp = tmp117_read_reg(TMP117_REG_TEMP);
        return (float)raw_temp * TMP117_RESOLUTION_10UC / MICRODEGREE_PER_10MILLIDEGREE / 1000.0f;  // Convert to Celsius
    }

    // Function to set the calibration bias
    void tmp117_set_calibbias() 
    {
        tmp117_write_reg(TMP117_REG_TEMP_OFFSET, m_calibbias);
    }

    // Function to initialize the TMP117 sensor
    void tmp117_init() 
    {

        // Check device ID
        int16_t dev_id = tmp117_read_reg(TMP117_REG_DEVICE_ID);
        if (dev_id != TMP117_DEVICE_ID && dev_id != TMP116_DEVICE_ID) 
        {
            printf("Error: Unsupported device ID (0x%04X)\n", dev_id);
            return;
        }

        printf("TMP117 initialized successfully.\n");
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

    i2c_inst_t* m_i2c_port;
    uint8_t     m_i2c_addr;
    int16_t     m_calibbias;
    unsigned    m_sda_pin;
    unsigned    m_scl_pin;
    unsigned    m_baud_rate;
};