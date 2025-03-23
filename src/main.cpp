#include "hardware_config.h"
#include "apisqueen_pwm.h"
#include "blink.pio.h"
#include "lps22hb_reg.h"
#include "lps22hb.h"

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
    */

    // LPS22HB Setup
    lps22hb s_lps22hb = lps22hb(i2c0, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, 0x5D, 100 * 1000);
    s_lps22hb.do_init();
    s_lps22hb.do_test();

    // APISQUEEN Motor Setup
    apisqueen_thruster s_motor_driver = apisqueen_thruster(22); 
    s_motor_driver.do_init();
    s_motor_driver.do_test();


    pair<float, float> press_temp_data;
    while (true) 
    {
        // printf("Hello, world!\n");
        // sleep_ms(1000);
        // tight_loop_contents();
        
        press_temp_data = s_lps22hb.do_read();
        printf("Pressure: %.2f hPa, Temperature: %.2f °C\n", press_temp_data.first, press_temp_data.second);

    }
}