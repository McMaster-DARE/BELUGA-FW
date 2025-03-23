// This file will initialize the following peripherals:
// - SPI -> init_spi()
// - I2C -> init_i2c()
// - DMA -> init_dma()
// - Interpolator -> init_interp()
// - Timer -> init_timer()
// - Watchdog -> init_watchdog()
// - UART -> init_uart()

#include <utility>

void init_spi();

void init_i2c();

void init_dma(const char* src, char* dst);

void init_interp();

void init_timer();

void init_watchdog();

void init_uart();

std::pair<unsigned, unsigned> init_gpio(unsigned pin);

void init_pwm(unsigned slice_num, unsigned freq, unsigned range);

void init(const char* src, char* dst);