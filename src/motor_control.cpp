// #include <wiringPi.h>
// #include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Replace std::this_thread::sleep_for with sleep_ms from the Pico SDK
void sleep_seconds(int seconds) {
    sleep_ms(seconds * 1000); // Convert seconds to milliseconds
}

int main() {
    const int LED_PIN = 1; // GPIO pin number
    const int PWM_FREQUENCY = 50; // PWM frequency in Hz
    const int PWM_RANGE = 1000; // PWM range
    const int INITIAL_DUTYCYCLE = 0; // Initial duty cycle

    // Initialize the Pico SDK
    stdio_init_all();

    // Initialize GPIO for PWM
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM); // Set GPIO to PWM function
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN); // Get PWM slice
    uint channel_num = pwm_gpio_to_channel(LED_PIN); // Get PWM channel

    // Configure PWM
    pwm_config config = pwm_get_default_config(); // Get default PWM config
    pwm_config_set_clkdiv(&config, 4.f); // Set clock divider (adjust for frequency)
    pwm_config_set_wrap(&config, PWM_RANGE); // Set PWM range
    pwm_init(slice_num, &config, true); // Initialize PWM

    // Set initial duty cycle
    pwm_set_chan_level(slice_num, channel_num, INITIAL_DUTYCYCLE);
    sleep_seconds(1);

    // Set duty cycle to 75 (7.5%)
    pwm_set_chan_level(slice_num, channel_num, 75);
    sleep_seconds(3);

    // Set duty cycle to 100 (10%) for positive rotation
    pwm_set_chan_level(slice_num, channel_num, 100);
    sleep_seconds(15);

    // Set duty cycle to 60 (6%) for reverse rotation
    pwm_set_chan_level(slice_num, channel_num, 60);
    sleep_seconds(5);

    // Set duty cycle back to 75 (7.5%)
    pwm_set_chan_level(slice_num, channel_num, 75);
    sleep_seconds(5);

    return 0;
}