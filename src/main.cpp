#include "hardware_config.h"
#include "apisqueen_pwm.h"
#include "blink.pio.h"
#include "lps22hb_reg.h"
#include "lps22hb.h"
#include "tmp117.h"
#include "imu.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq)
{
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

void blink_test()
{
    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 1000000);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio
}

void startup_msg()
{
    // THis function will countdown from 5-1 to let the user know the program is starting
    for (int i = 5; i > 0; i--)
    {
        printf("Starting in %d seconds\n", i);
        sleep_ms(1000);
    }
    printf("Starting now\n");
    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks
}

int main()
{
    stdio_init_all();

    startup_msg();
    // blink_test();

    /* ----------------- Sensor Suite Setup -----------------
    Pinout:    
    - GPIO 22: APISQUEEN Motor Driver
    - GPIO 4/5 (I2C 0): LPS22HB Pressure Sensor
    - GPIO 6/7 (I2C 1): TMP117 Temperature Sensor
    */
    sensor_suite suite;

    lps22hb s_lps22hb("lps22hb", 0x5D);
    imu     s_bno055("bno055", 0x28);
    tmp117 s_tmp117("tmp117", 0x48);
    apisqueen_thruster s_motor_driver("motor1", 0, 22); // GPIO pin 22 for motor driver

    suite.add(&s_lps22hb);
    suite.add(&s_bno055);
    suite.add(&s_tmp117);

    // Connect all sensors to I2C bus
    suite.connect_i2c(i2c1, 6, 7, 100 * 1000);

    // Initialize all sensors
    suite.init_loop();

    // Test all sensors
    suite.test_loop();

    // Calibrate all sensors
    suite.calibrate_loop();

    while (true) 
    {
        sleep_ms(1000);
        s_lps22hb.read();
        s_bno055.read_gyro_xyz();
        s_tmp117.read();
    }
}