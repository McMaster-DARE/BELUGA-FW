#include "hardware_config.h"
#include "init.h"
#include "apisqueen_pwm.h"
#include "blink.pio.h"
#include "lps22hb_reg.h"


// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

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
}

void motor_test()
{
    // Motor init
    motor_driver motor(1); // Initialize motor on GPIO 1
    motor.drive(10); // Drive motor at 10% duty cycle
    motor.sleep_s(2); // Sleep for 2 seconds
    motor.drive(5); // Drive motor at 5% duty cycle
    motor.sleep_s(2); // Sleep for 2 seconds
    motor.drive(7.5); // Stop motor
    motor.sleep_s(2); // Sleep for 2 seconds
    motor.drive(15); // This will throw an exception
}

void lps22_test(stmdev_ctx_t dev_ctx)
{
    // Variables to store pressure and temperature
    float pressure_hPa;
    float temperature_degC;

    // Read pressure and temperature
    lps22hb_pressure_raw_get(&dev_ctx, (uint32_t*)&pressure_hPa);
    lps22hb_temperature_raw_get(&dev_ctx, (int16_t*)&temperature_degC);

    // Convert raw data to human-readable values
    pressure_hPa = pressure_hPa / 4096.0f;  // Convert to hPa
    temperature_degC = temperature_degC / 100.0f;  // Convert to degrees Celsius

    // Print the values
    printf("Pressure: %.2f hPa, Temperature: %.2f °C\n", pressure_hPa, temperature_degC);

    // Wait for 1 second before the next reading
    sleep_ms(1000);
}

int main()
{
    stdio_init_all();
    // init(src, dst); // This is not working?

    startup_msg();

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks
    
    // Initialize I2C
    i2c_init(i2c_default, 100 * 1000);  // 100 kHz
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Initialize the LPS22HB sensor
    stmdev_ctx_t dev_ctx;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = nullptr;  // No handle needed for this implementation

    // Check sensor ID
    uint8_t who_am_i;
    lps22hb_device_id_get(&dev_ctx, &who_am_i);
    if (who_am_i != LPS22HB_ID) {
        printf("LPS22HB not found!\n");
        return -1;
    }

    // Configure the sensor
    lps22hb_data_rate_set(&dev_ctx, LPS22HB_ODR_10_Hz);  // Set output data rate to 10 Hz

    // blink_test();

    while (true) {
        printf("Hello, world!\n");

        // sleep_ms(1000);
        
        lps22_test(dev_ctx);
    }
}
