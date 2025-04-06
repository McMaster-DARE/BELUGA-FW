#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define SHT40_I2C_ADDR  0x44  // Default I2C address for SHT40
#define MEASURE_CMD     0xFD   // Command for high-precision measurement

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define SDA_PIN 8
#define SCL_PIN 9

void sht40_init() {
    i2c_init(I2C_PORT, 400 * 1000); // 100 kHz I2C
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

bool sht40_read(float *temperature, float *humidity) {
    uint8_t command = MEASURE_CMD;
    uint8_t data[6];

    // Send measurement command
    if (i2c_write_blocking(I2C_PORT, SHT40_I2C_ADDR, &command, 1, false) < 0) {
        printf("I2C Write failed\n");
        return false;
    }

    sleep_ms(10);  // Wait for measurement (max 8.2ms for high precision)

    // Read 6 bytes of data
    if (i2c_read_blocking(I2C_PORT, SHT40_I2C_ADDR, data, 6, false) < 0) {
        printf("I2C Read failed\n");
        return false;
    }

    // Convert raw temperature data
    uint16_t raw_temp = (data[0] << 8) | data[1];
    *temperature = -45 + (175.0 * raw_temp / 65535.0);

    // Convert raw humidity data
    uint16_t raw_humidity = (data[3] << 8) | data[4];
    *humidity = 100.0 * raw_humidity / 65535.0;

    return true;
}

int main() {
    stdio_init_all();
    sht40_init();
    printf("SHT40 Sensor Initialized\n");

    while (true) {
        float temperature, humidity;

        if (sht40_read(&temperature, &humidity)) {
            printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
        } else {
            printf("Failed to read from SHT40\n");
        }

        sleep_ms(2000);  // Read every 2 seconds
    }

    return 0;
}


/*
// Use this main() to test serial communication w board, works!

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
*/