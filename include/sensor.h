// Parent class for a sensor
// Includes phases for initialization and testing


#include "hardware_config.h"
#include <vector>

#pragma once 

using namespace std;

class sensor
{
    public:
    sensor() : m_i2c_s(nullptr), m_addr_s(0) {}

    virtual void do_init() {}
    virtual void do_test() {}
    virtual void do_calibrate() {}

    virtual void add_to_i2c(i2c_inst_t* i2c_bus)
    {
        m_i2c_s = i2c_bus;  // Common bus provided by sensor_suite
    }

    protected:
    i2c_inst_t* m_i2c_s;
    uint8_t m_addr_s; // Sensor-specific I2C address
};


class sensor_suite
{
    public:
    sensor_suite()
    {}

    void add(sensor* s)
    {
        m_sensors.push_back(s);
    }

    void connect_i2c(i2c_inst_t* p, unsigned sda_pin, unsigned scl_pin, unsigned baud_rate)
    {
        printf("Connecting sensors to I2C bus...\n");
        i2c_init(p, baud_rate);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);

        // Add sensors to I2C bus
        for (auto s : m_sensors)
            s->add_to_i2c(p);
    }

    void init_loop()
    {
        printf("==== INIT PHASE ====\n");
        for (auto s : m_sensors)
            s->do_init();
        printf("Initialization complete.\n");
    }

    void test_loop()
    {
        printf("==== TEST PHASE ====\n");
        for (auto s : m_sensors)
            s->do_test();
        printf("All tests passed.\n");
    }

    void calibrate_loop()
    {
        printf("==== CALIBRATION PHASE ====\n");
        for (auto s : m_sensors)
            s->do_calibrate();
        printf("Calibration complete.\n");
    }

    private:
    vector<sensor*> m_sensors;
};