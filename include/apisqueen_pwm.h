#include "hardware_config.h"
#include "init.h"
#include <utility>
#include <stdexcept>

#pragma once


// PWM motor driver class for the APISQUEEN U2 MINI Thruster motor
class motor_driver
{
    private:
    
    public:
    static constexpr double k_freq = 50;
    motor_driver(unsigned pin, unsigned range = 255) :
        m_pwm_range(range)
    {
        // Initialize GPIO for PWM
        std::pair<unsigned, unsigned> x = init_gpio(pin);
        m_slice_num = x.first;
        m_channel_num = x.second;

        // Configure PWM
        init_pwm(m_slice_num, k_freq, range);
        // Set initial duty cycle
        pwm_set_chan_level(m_slice_num, m_channel_num, 0);
    }

    void drive(unsigned duty_cycle) 
    {
    /*
        This function controls the APISQUEEN U2 MINI Thruster motor using PWM.

        The motor operates on a 50Hz PWM signal that ranges from a 1 ms to 2 ms pulse width.

            1.5 ms: Stops the motor (neutral position).

            1.5 - 2 ms: Positive thrust (forward direction).

            1 - 1.5 ms: Negative thrust (reverse direction).

        Power Delivery:
        - The pulse width determines the power delivered to the motor.
        - A larger pulse width (closer to 2 ms) delivers more power, resulting in faster thrust.
        - A smaller pulse width (closer to 1 ms) delivers less power, resulting in slower thrust.
        - Values between 1 ms and 2 ms provide proportional control over thrust.

        1.5 ms pulse width refers to the duration of the high time within one period of the PWM signal.
        High (On) Time: |=====| (1 ms to 2 ms)
        Low (Off) Time: |=================| (18 ms to 19 ms)
        Total Period:   |===================| (20 ms)

        The pulse width is controlled by the duty cycle of the PWM signal.
        Duty Cycle = (Pulse width High Time / Total Period) * 100

        Positive Thrust Example:
        Duty Cycle = (2 ms / 20 ms) * 100 = 10%
        drive(10);

        Negative Thrust Example:
        Duty Cycle = (1 ms / 20 ms) * 100 = 5%
        drive(5);

        Stopping the motor:
        Duty Cycle = (1.5 ms / 20 ms) * 100 = 7.5%
        drive(7.5);
    */
    
        if (duty_cycle < 0 || duty_cycle > m_pwm_range) 
        {
            printf("ERROR: Duty cycle must be between 0 and %d\n", m_pwm_range);
            throw std::invalid_argument(0);
        }

        double pulse_wdth = (duty_cycle / 100.0) * (1/k_freq);
        if (pulse_wdth < 1e-3 || pulse_wdth > 2e-3) 
        {
            printf("ERROR: Pulse width must be between 1 ms and 2 ms for the APISQUEEN U2 MINI Thruster motor\n");
            throw std::invalid_argument(0);
        }

        pwm_set_chan_level(m_slice_num, m_channel_num, duty_cycle);
    }
    
    // Helper Function
    void sleep_s(unsigned sec) 
    {
        sleep_ms(sec * 1000);
    }
    
    private:
    unsigned m_slice_num; // PWM hardware slice number 
    unsigned m_channel_num; // PWM channel number
    unsigned m_pwm_range;
};

