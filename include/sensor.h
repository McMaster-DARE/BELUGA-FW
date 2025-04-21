// Parent class for a sensor
// Includes phases for initialization and testing


#include "hardware_config.h"
#include <vector>
#include <stdexcept>
#include <string>

#pragma once 

using namespace std;

class sensor
{
    public:
    sensor(const string& name, const uint8_t addr) : 
        m_i2c_s(nullptr), 
        m_name(name), 
        m_addr_s(addr)
    {}

    virtual void do_init() {}
    virtual unsigned do_test() { return 1; }
    virtual void do_calibrate() {}
    
    virtual const char* get_name() { return m_name.c_str(); }
    double get_time() { return to_ms_since_boot(get_absolute_time()) / 1000.0; } // in seconds

    virtual void add_to_i2c(i2c_inst_t* i2c_bus)
    {
        m_i2c_s = i2c_bus;  // Common bus provided by sensor_suite
    }

    protected:
    i2c_inst_t* m_i2c_s;
    string m_name;
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
        {
            printf("Initializing %s...\n", s->get_name());
            s->do_init();
        }
        printf("Initialization complete.\n");
    }

    void test_loop()
    {
        printf("==== TEST PHASE ====\n");

        unsigned test_accum = 0;
        for (auto s : m_sensors)
        {
            printf("Validating sensor %s (%d/%d)\n", s->get_name(), test_accum, m_sensors.size());
            
            unsigned res = s->do_test();
            if (res == 0)
                printf("Sensor %s failed test!\n", s->get_name());
            test_accum += res;
        }

        if (test_accum != m_sensors.size())
        {
            printf("Sensor Suite test phase FAILED! %d/%d\n", test_accum, m_sensors.size());
            throw runtime_error("Some tests failed.");
        }
    
        printf("Sensor Suite testphase PASSED! (%d/%d)\n", test_accum, m_sensors.size());
    }

    void calibrate_loop()
    {
        printf("==== CALIBRATION PHASE ====\n");
        for (auto s : m_sensors)
        {
            printf("Calibrating sensor %s...\n", s->get_name());
            s->do_calibrate(); 
        }
        printf("Calibration complete.\n");
    }

    private:
    vector<sensor*> m_sensors;
};