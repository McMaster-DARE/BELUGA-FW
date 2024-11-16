#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <chrono>

void sleep_seconds(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

int main() {
    const int LED_PIN = 1; 
    const int PWM_FREQUENCY = 50;
    const int PWM_RANGE = 1000; 
    const int INITIAL_DUTYCYCLE = 0; 

    // So we initialize the WiringPi instance and set the pin mode.
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        return 1;
    }

    pinMode(LED_PIN, PWM_OUTPUT); 
    pwmSetMode(PWM_MODE_MS); // PWM mode mark-space
    pwmSetRange(PWM_RANGE); 
    pwmSetClock(192); // Clock divisor (we might need to adjust this)

    // Init duty cycle and freq
    pwmWrite(LED_PIN, INITIAL_DUTYCYCLE);
    sleep_seconds(1);

    // 75/1000 = 7.5% duty cycle
    pwmWrite(LED_PIN, 75);
    sleep_seconds(3);

    // +ve rotation: set to 10% duty cycle
    pwmWrite(LED_PIN, 100);
    sleep_seconds(15);

    // -ve rotation: set to 6% duty cycle
    pwmWrite(LED_PIN, 60);
    sleep_seconds(5);

    // Neutral duty cycle (7.5%)
    pwmWrite(LED_PIN, 75);
    sleep_seconds(5);

    return 0;
}
