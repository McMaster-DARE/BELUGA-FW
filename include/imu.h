#include "hardware_config.h"
#include "sensor.h"

#include "bno055.h"
#include <stdexcept>
#include <map>
#include <utility>
#include <array>


#pragma once

using namespace std;

class imu : public sensor
{
    public:
    imu(const string& name, uint8_t i2c_addr) :
        sensor(name, i2c_addr)
    {}
    
    void do_init() override
    {
        active_inst = this; 

        m_bno055.bus_write = platform_write;
        m_bno055.bus_read = platform_read;
        m_bno055.delay_msec = platform_delay;
        m_bno055.dev_addr = m_addr_s; // allows w/r functions to access the class members

        comres = bno055_init(&m_bno055);
        power_mode = BNO055_POWER_MODE_NORMAL;
        comres += bno055_set_power_mode(power_mode);
        comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    }

    unsigned do_test() override
    {
        // BNO055 has more than 256 bytes worth of registers, but I2C register
        // addressing is limited to 1 byte (0x00–0xFF). To work around that, the
        // sensor has it's register mapping divided into two "pages" — like a
        // register bank switcher. Each page maps 0x00–0xFF, but they mean
        // different things depending on the PAGE_ID register

        bool pass = true;
        uint8_t set_page = 0x01;
        uint8_t read_val = 0;
        uint8_t test_write = 0x5A;

        // Check PAGE_ID register (should be 0x00)
        pass &= write_and_read_test(BNO055_PAGE_ID_ADDR, &set_page, 1);
        set_page = 0x00;
        pass &= write_and_read_test(BNO055_PAGE_ID_ADDR, &set_page, 1);
    
        // Test r/w on MAG_RADIUS_MSB register (from Page 1)
        set_page = 0x01;
        pass &= write_test(BNO055_PAGE_ID_ADDR, &set_page, 1);
        pass &= read_test(BNO055_MAG_RADIUS_MSB_ADDR, &read_val, 1);
        pass &= write_and_read_test(BNO055_MAG_RADIUS_MSB_ADDR, &test_write, 1);
    
        // Switch back to Page 0
        set_page = 0x00;
        pass &= write_test(BNO055_PAGE_ID_ADDR, &set_page, 1);

        if (!pass)
        {
            return 0;
        }
        
        return 1;
    }

    pair<double, array<double, 3>> read_accel_xyz()
    {
        comres = bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
        double t = get_time();
    
        if (comres != BNO055_SUCCESS)
            printf("Error reading accelerometer data.\n");
    
        printf("[%.3f s] Accelerometer: x: %f, y: %f, z: %f\n", t, d_accel_xyz.x, d_accel_xyz.y, d_accel_xyz.z);
    
        return make_pair(t, array<double, 3>{d_accel_xyz.x, d_accel_xyz.y, d_accel_xyz.z});
    }
    
    pair<double, array<double, 3>> read_euler_hpr()
    {
        comres = bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
        double t = get_time();
    
        if (comres != BNO055_SUCCESS)
            printf("Error reading euler data.\n");
    
        printf("[%.3f s] Euler: h: %f, p: %f, r: %f\n", t, d_euler_hpr.h, d_euler_hpr.p, d_euler_hpr.r);
    
        return make_pair(t, array<double, 3>{d_euler_hpr.h, d_euler_hpr.p, d_euler_hpr.r});
    }
    
    pair<double, array<double, 3>> read_gyro_xyz()
    {
        comres = bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
        double t = get_time();
    
        if (comres != BNO055_SUCCESS)
            printf("Error reading gyroscope data.\n");
    
        printf("[%.3f s] Gyroscope: x: %f, y: %f, z: %f\n", t, d_gyro_xyz.x, d_gyro_xyz.y, d_gyro_xyz.z);
    
        return make_pair(t, array<double, 3>{d_gyro_xyz.x, d_gyro_xyz.y, d_gyro_xyz.z});
    }
    
    pair<double, array<double, 3>> read_mag_xyz()
    {
        comres = bno055_convert_double_mag_xyz_uT(&d_mag_xyz);
        double t = get_time();
    
        if (comres != BNO055_SUCCESS)
            printf("Error reading magnetometer data.\n");
    
        printf("[%.3f s] Magnetometer: x: %f, y: %f, z: %f\n", t, d_mag_xyz.x, d_mag_xyz.y, d_mag_xyz.z);
    
        return make_pair(t, array<double, 3>{d_mag_xyz.x, d_mag_xyz.y, d_mag_xyz.z});
    }
    
    pair<double, array<double, 3>> read_linear_accel_xyz()
    {
        comres = bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);
        double t = get_time();
    
        if (comres != BNO055_SUCCESS)
            printf("Error reading linear acceleration data.\n");
    
        printf("[%.3f s] Linear Acceleration: x: %f, y: %f, z: %f\n", t, d_linear_accel_xyz.x, d_linear_accel_xyz.y, d_linear_accel_xyz.z);
    
        return make_pair(t, array<double, 3>{d_linear_accel_xyz.x, d_linear_accel_xyz.y, d_linear_accel_xyz.z});
    }
    
    pair<double, array<double, 3>> read_gravity_xyz()
    {
        comres = bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
        double t = get_time();
    
        if (comres != BNO055_SUCCESS)
            printf("Error reading gravity data.\n");
    
        printf("[%.3f s] Gravity: x: %f, y: %f, z: %f\n", t, d_gravity_xyz.x, d_gravity_xyz.y, d_gravity_xyz.z);
    
        return make_pair(t, array<double, 3>{d_gravity_xyz.x, d_gravity_xyz.y, d_gravity_xyz.z});
    }
    
    pair<double, array<double, 4>> read_quaternion_wxyz()
    {
        comres = bno055_read_quaternion_wxyz(&quaternion_wxyz);
        double t = get_time();
    
        if (comres != BNO055_SUCCESS)
            printf("Error reading quaternion data.\n");
    
        printf("[%.3f s] Quaternion: w: %f, x: %f, y: %f, z: %f\n", t, quaternion_wxyz.w, quaternion_wxyz.x, quaternion_wxyz.y, quaternion_wxyz.z);
    
        return make_pair(t, array<double, 4>{
            static_cast<double>(quaternion_wxyz.w),
            static_cast<double>(quaternion_wxyz.x),
            static_cast<double>(quaternion_wxyz.y),
            static_cast<double>(quaternion_wxyz.z)
        });
    }    

    private:
    static s8 platform_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {      
        if (!active_inst || !active_inst->m_i2c_s) 
        {
            printf("Active instance or I2C bus not set.\n");
            return (s8)BNO055_ERROR;
        }

        unsigned I2C_BUFFER_LEN = 8;
        u8 array[I2C_BUFFER_LEN];
    
        array[0] = reg_addr;
        for (uint8_t i = 0; i < cnt; i++) {
            array[i + 1] = reg_data[i];
        }
    
        int ret = i2c_write_blocking(
            active_inst->m_i2c_s,
            active_inst->m_addr_s,
            array,
            cnt + 1,
            false
        );
    
        if (ret == PICO_ERROR_GENERIC) {
            printf("I2C write failed.\n");
            return (s8)BNO055_ERROR;
        }
    
        sleep_ms(1);
        return (s8)BNO055_SUCCESS;
    }
    
    static s8 platform_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        if (!active_inst || !active_inst->m_i2c_s) {
            printf("Active instance or I2C bus not set.\n");
            return (s8)BNO055_ERROR;
        }
    
        int ret = i2c_write_blocking(
            active_inst->m_i2c_s,
            active_inst->m_addr_s,
            &reg_addr,
            1,
            true
        );
    
        if (ret == PICO_ERROR_GENERIC) {
            printf("I2C address write failed.\n");
            return (s8)BNO055_ERROR;
        }
    
        sleep_ms(1);
    
        ret = i2c_read_blocking(
            active_inst->m_i2c_s,
            active_inst->m_addr_s,
            reg_data,
            cnt,
            false
        );
    
        if (ret == PICO_ERROR_GENERIC) {
            printf("I2C data read failed.\n");
            return (s8)BNO055_ERROR;
        }
    
        return (s8)BNO055_SUCCESS;
        
    }

    static void platform_delay(u32 ms) 
    {
        sleep_ms(ms);
    }

    bool write_test(uint8_t reg, uint8_t *data, uint16_t len)
    {
        // printf("Writing to register 0x%02X with data 0x%02X...\n", reg, *data);
        if (platform_write(m_addr_s, reg, data, len) == BNO055_SUCCESS)
        {
            // printf("Write successful to register 0x%02X with data 0x%02X.\n", reg, *data);
            return true;
        }
        
        printf("Write failed to register 0x%02X with data 0x%02X.\n", reg, *data);
        return false;
    }

    bool read_test(uint8_t reg, uint8_t *data, uint16_t len)
    {
        // printf("Reading from register 0x%02X...\n", reg);
        if (platform_read(m_addr_s, reg, data, len) == BNO055_SUCCESS)
        {
            // printf("Read successful from register 0x%02X with data 0x%02X.\n", reg, *data);
            return true;
        }

        printf("Read failed from register 0x%02X.\n", reg);
        return false;
    }

    bool write_and_read_test(uint8_t reg, uint8_t *data, uint16_t len)
    {
        // printf("Writing data 0x%02X to register 0x%02X and reading back...\n", *data, reg);
        if (platform_write(m_addr_s, reg, data, len) == BNO055_SUCCESS)
        {
            // printf("Write successful to register 0x%02X with data 0x%02X.\n", reg, *data);
            // printf("Reading back...\n");
            uint8_t read_data;
            if (platform_read(m_addr_s, reg, &read_data, len) == BNO055_SUCCESS)
            {
                // printf("Read successful from register 0x%02X with data 0x%02X.\n", reg, read_data);
                if (read_data == *data)
                {
                    // printf("Write and read values match.\n");
                    // printf("Register 0x%02X: 0x%02X\n", reg, *data);
                    // printf("Read value: 0x%02X\n", read_data);
                    // printf("Write and read values match.\n");
                    return true;
                }
                else
                {
                    printf("Write and read values do not match.\n");
                    printf("Register 0x%02X: 0x%02X\n", reg, *data);
                    printf("Read value: 0x%02X\n", read_data);
                    printf("Write and read values do not match.\n");
                    return false;
                }
            }
            else
                printf("Read failed from register 0x%02X.\n", reg);
        }

        printf("Write failed to register 0x%02X with data 0x%02X.\n", reg, *data);
        return false;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8 = BNO055_INIT_VALUE;
        u8 bno055_page_zero_u8 = BNO055_PAGE_ZERO;
    
        /* Array holding the Software revision id
         */
        u8 a_SW_ID_u8[BNO055_REV_ID_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    
        /* stuct parameters are assign to bno055*/
        p_bno055 = bno055;
    
        /* Write the default page as zero*/
        com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                   BNO055_PAGE_ID_REG,
                                                   &bno055_page_zero_u8,
                                                   BNO055_GEN_READ_WRITE_LENGTH);
    
        /* Read the chip id of the sensor from page
         * zero 0x00 register*/
        com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                   BNO055_CHIP_ID_REG,
                                                   &data_u8,
                                                   BNO055_GEN_READ_WRITE_LENGTH);
        p_bno055->chip_id = data_u8;
    
        /* Read the accel revision id from page
         * zero 0x01 register*/
        com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                   BNO055_ACCEL_REV_ID_REG,
                                                   &data_u8,
                                                   BNO055_GEN_READ_WRITE_LENGTH);
        p_bno055->accel_rev_id = data_u8;
    
        /* Read the mag revision id from page
         * zero 0x02 register*/
        com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                   BNO055_MAG_REV_ID_REG,
                                                   &data_u8,
                                                   BNO055_GEN_READ_WRITE_LENGTH);
        p_bno055->mag_rev_id = data_u8;
    
        /* Read the gyro revision id from page
         * zero 0x02 register*/
        com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                   BNO055_GYRO_REV_ID_REG,
                                                   &data_u8,
                                                   BNO055_GEN_READ_WRITE_LENGTH);
        p_bno055->gyro_rev_id = data_u8;
    
        /* Read the boot loader revision from page
         * zero 0x06 register*/
        com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                   BNO055_BL_REV_ID_REG,
                                                   &data_u8,
                                                   BNO055_GEN_READ_WRITE_LENGTH);
        p_bno055->bl_rev_id = data_u8;
    
        /* Read the software revision id from page
         * zero 0x04 and 0x05 register( 2 bytes of data)*/
        com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                   BNO055_SW_REV_ID_LSB_REG,
                                                   a_SW_ID_u8,
                                                   BNO055_LSB_MSB_READ_LENGTH);
        a_SW_ID_u8[BNO055_SW_ID_LSB] = BNO055_GET_BITSLICE(a_SW_ID_u8[BNO055_SW_ID_LSB], BNO055_SW_REV_ID_LSB);
        p_bno055->sw_rev_id =
            (u16)((((u32)((u8)a_SW_ID_u8[BNO055_SW_ID_MSB])) << BNO055_SHIFT_EIGHT_BITS) | (a_SW_ID_u8[BNO055_SW_ID_LSB]));
    
        /* Read the page id from the register 0x07*/
        com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                   BNO055_PAGE_ID_REG,
                                                   &data_u8,
                                                   BNO055_GEN_READ_WRITE_LENGTH);
        p_bno055->page_id = data_u8;
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(u8 power_mode_u8)
    {
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /* The write operation effective only if the operation
             * mode is in config mode, this part of code is checking the
             * current operation mode and set the config mode */
            stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
            if (stat_s8 == BNO055_SUCCESS)
            {
                if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
                {
                    stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
                }
                if (stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of power mode */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_POWER_MODE_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_POWER_MODE, power_mode_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_POWER_MODE_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
        {
            /* set the operation mode
             * of previous operation mode*/
            com_rslt += bno055_set_operation_mode(prev_opmode_u8);
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(u8 operation_mode_u8)
    {
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /* The write operation effective only if the operation
             * mode is in config mode, this part of code is checking the
             * current operation mode and set the config mode */
            stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* If the previous operation mode is config it is
                 * directly write the operation mode */
                if (prev_opmode_u8 == BNO055_OPERATION_MODE_CONFIG)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_OPERATION_MODE_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_OPERATION_MODE, operation_mode_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_OPERATION_MODE_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
    
                        /* Config mode to other
                         * operation mode switching
                         * required delay of 600ms*/
                        p_bno055->delay_msec(BNO055_MODE_SWITCHING_DELAY);
                    }
                }
                else
                {
                    /* If the previous operation
                     * mode is not config it is
                     * write the config mode */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_OPERATION_MODE_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_OPERATION_MODE, BNO055_OPERATION_MODE_CONFIG);
                        com_rslt +=
                            bno055_write_register(BNO055_OPERATION_MODE_REG, &data_u8r, BNO055_GEN_READ_WRITE_LENGTH);
    
                        /* other mode to config mode switching
                         * required delay of 20ms*/
                        p_bno055->delay_msec(BNO055_CONFIG_MODE_SWITCHING_DELAY);
                    }
    
                    /* Write the operation mode */
                    if (operation_mode_u8 != BNO055_OPERATION_MODE_CONFIG)
                    {
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_OPERATION_MODE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_OPERATION_MODE, operation_mode_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_OPERATION_MODE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
    
                            /* Config mode to other
                             * operation mode switching
                             * required delay of 600ms*/
                            p_bno055->delay_msec(BNO055_MODE_SWITCHING_DELAY);
                        }
                    }
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(u8 *operation_mode_u8)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, operation mode is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the value of operation mode*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_OPERATION_MODE_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                *operation_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_OPERATION_MODE);
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(u8 page_id_u8)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /* Read the current page*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_PAGE_ID_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
    
            /* Check condition for communication BNO055_SUCCESS*/
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_PAGE_ID, page_id_u8);
    
                /* Write the page id*/
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_PAGE_ID_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    p_bno055->page_id = page_id_u8;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_write_register(u8 addr_u8, u8 *data_u8, u8 len_u8)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /* Write the values of respective given register */
            com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr, addr_u8, data_u8, len_u8);
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_xyz(struct bno055_accel_t *accel)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Array holding the accel xyz value
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
         */
        u8 data_u8[BNO055_ACCEL_XYZ_DATA_SIZE] = {
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
        };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_ACCEL_DATA_X_LSB_VALUEX_REG,
                                                          data_u8,
                                                          BNO055_ACCEL_XYZ_DATA_SIZE);
    
                /* Data X*/
                data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                            BNO055_ACCEL_DATA_X_LSB_VALUEX);
                data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                            BNO055_ACCEL_DATA_X_MSB_VALUEX);
                accel->x =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));
    
                /* Data Y*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                            BNO055_ACCEL_DATA_Y_LSB_VALUEY);
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                            BNO055_ACCEL_DATA_Y_MSB_VALUEY);
                accel->y =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));
    
                /* Data Z*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                            BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                            BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
                accel->z =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(u8 accel_unit_u8)
    {
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /* The write operation effective only if the operation
             * mode is in config mode, this part of code is checking the
             * current operation mode and set the config mode */
            stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
            if (stat_s8 == BNO055_SUCCESS)
            {
                if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
                {
                    stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
                }
                if (stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the accel unit */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_UNIT_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_UNIT, accel_unit_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_UNIT_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
        {
            /* set the operation mode
             * of previous operation mode*/
            com_rslt += bno055_set_operation_mode(prev_opmode_u8);
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_msq(struct bno055_accel_double_t *accel_xyz)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        struct bno055_accel_t reg_accel_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
        u8 accel_unit_u8 = BNO055_INIT_VALUE;
    
        /* Read the current accel unit and set the
         * unit as m/s2 if the unit is in mg*/
        com_rslt = bno055_get_accel_unit(&accel_unit_u8);
        if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
        {
            com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
        }
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Read the accel raw xyz data*/
            com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Convert raw xyz to m/s2*/
                accel_xyz->x = (double)(reg_accel_xyz.x / BNO055_ACCEL_DIV_MSQ);
                accel_xyz->y = (double)(reg_accel_xyz.y / BNO055_ACCEL_DIV_MSQ);
                accel_xyz->z = (double)(reg_accel_xyz.z / BNO055_ACCEL_DIV_MSQ);
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_hrp(struct bno055_euler_t *euler)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Array holding the Euler hrp value
         * data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] - h->LSB
         * data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] - h->MSB
         * data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] - r->MSB
         * data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] - r->MSB
         * data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] - p->MSB
         * data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] - p->MSB
         */
        u8 data_u8[BNO055_EULER_HRP_DATA_SIZE] = {
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
        };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the six byte of Euler hrp data*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_EULER_H_LSB_VALUEH_REG,
                                                          data_u8,
                                                          BNO055_EULER_HRP_DATA_SIZE);
    
                /* Data h*/
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] = BNO055_GET_BITSLICE(
                    data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB],
                    BNO055_EULER_H_LSB_VALUEH);
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] = BNO055_GET_BITSLICE(
                    data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB],
                    BNO055_EULER_H_MSB_VALUEH);
                euler->h =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]));
    
                /* Data r*/
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] = BNO055_GET_BITSLICE(
                    data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB],
                    BNO055_EULER_R_LSB_VALUER);
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] = BNO055_GET_BITSLICE(
                    data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB],
                    BNO055_EULER_R_MSB_VALUER);
                euler->r =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]));
    
                /* Data p*/
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] = BNO055_GET_BITSLICE(
                    data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB],
                    BNO055_EULER_P_LSB_VALUEP);
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] = BNO055_GET_BITSLICE(
                    data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB],
                    BNO055_EULER_P_MSB_VALUEP);
                euler->p =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(u8 euler_unit_u8)
    {
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /* The write operation effective only if the operation
             * mode is in config mode, this part of code is checking the
             * current operation mode and set the config mode */
            stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
            if (stat_s8 == BNO055_SUCCESS)
            {
                if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
                {
                    stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
                }
                if (stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the Euler unit*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_EULER_UNIT_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_EULER_UNIT, euler_unit_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_EULER_UNIT_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
        {
            /* set the operation mode
             * of previous operation mode*/
            com_rslt += bno055_set_operation_mode(prev_opmode_u8);
        }
    
        return com_rslt;
    }
    
    BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_deg(struct bno055_euler_double_t *euler_hpr)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        struct bno055_euler_t reg_euler = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
        u8 euler_unit_u8 = BNO055_INIT_VALUE;
    
        /* Read the current Euler unit and set the
         * unit as degree if the unit is in radians */
        com_rslt = bno055_get_euler_unit(&euler_unit_u8);
        if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
        {
            com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
        }
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Read Euler raw h data*/
            com_rslt += bno055_read_euler_hrp(&reg_euler);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Convert raw Euler hrp to degree*/
                euler_hpr->h = (double)(reg_euler.h / BNO055_EULER_DIV_DEG);
                euler_hpr->p = (double)(reg_euler.p / BNO055_EULER_DIV_DEG);
                euler_hpr->r = (double)(reg_euler.r / BNO055_EULER_DIV_DEG);
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(u8 *gyro_unit_u8)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, gyro unit is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the gyro unit */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_GYRO_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                *gyro_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_UNIT);
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro_t *gyro)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Array holding the gyro xyz value
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
         */
        u8 data_u8[BNO055_GYRO_XYZ_DATA_SIZE] = {
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
        };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the six bytes data of gyro xyz*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_GYRO_DATA_X_LSB_VALUEX_REG,
                                                          data_u8,
                                                          BNO055_GYRO_XYZ_DATA_SIZE);
    
                /* Data x*/
                data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                            BNO055_GYRO_DATA_X_LSB_VALUEX);
                data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                            BNO055_GYRO_DATA_X_MSB_VALUEX);
                gyro->x =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));
    
                /* Data y*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                            BNO055_GYRO_DATA_Y_LSB_VALUEY);
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                            BNO055_GYRO_DATA_Y_MSB_VALUEY);
                gyro->y =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));
    
                /* Data z*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                            BNO055_GYRO_DATA_Z_LSB_VALUEZ);
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                            BNO055_GYRO_DATA_Z_MSB_VALUEZ);
                gyro->z =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(u8 gyro_unit_u8)
    {
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /* The write operation effective only if the operation
             * mode is in config mode, this part of code is checking the
             * current operation mode and set the config mode */
            stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
            if (stat_s8 == BNO055_SUCCESS)
            {
                if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
                {
                    stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
                }
                if (stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the gyro unit */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_UNIT_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_UNIT, gyro_unit_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_UNIT_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
        {
            /* set the operation mode
             * of previous operation mode*/
            com_rslt += bno055_set_operation_mode(prev_opmode_u8);
        }
    
        return com_rslt;
    }
    

    BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_dps(struct bno055_gyro_double_t *gyro_xyz)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        struct bno055_gyro_t reg_gyro_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
        u8 gyro_unit_u8 = BNO055_INIT_VALUE;
    
        /* Read the current gyro unit and set the
         * unit as dps if the unit is in rps */
        com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
        if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
        {
            com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
        }
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Read gyro raw xyz data */
            com_rslt += bno055_read_gyro_xyz(&reg_gyro_xyz);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Convert gyro raw xyz to dps*/
                gyro_xyz->x = (double)(reg_gyro_xyz.x / BNO055_GYRO_DIV_DPS);
                gyro_xyz->y = (double)(reg_gyro_xyz.y / BNO055_GYRO_DIV_DPS);
                gyro_xyz->z = (double)(reg_gyro_xyz.z / BNO055_GYRO_DIV_DPS);
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    
        return com_rslt;
    }
    BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag_t *mag)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Array holding the mag xyz value
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
         */
        u8 data_u8[BNO055_MAG_XYZ_DATA_SIZE] = {
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
        };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /*Read the six byte value of mag xyz*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_MAG_DATA_X_LSB_VALUEX_REG,
                                                          data_u8,
                                                          BNO055_MAG_XYZ_DATA_SIZE);
    
                /* Data X*/
                data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                            BNO055_MAG_DATA_X_LSB_VALUEX);
                data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                            BNO055_MAG_DATA_X_MSB_VALUEX);
                mag->x =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));
    
                /* Data Y*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                            BNO055_MAG_DATA_Y_LSB_VALUEY);
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                            BNO055_MAG_DATA_Y_MSB_VALUEY);
                mag->y =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));
    
                /* Data Z*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                            BNO055_MAG_DATA_Z_LSB_VALUEZ);
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                            BNO055_MAG_DATA_Z_MSB_VALUEZ);
                mag->z =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_xyz_uT(struct bno055_mag_double_t *mag_xyz)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        struct bno055_mag_t reg_mag_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    
        /* Read raw mag xyz data */
        com_rslt = bno055_read_mag_xyz(&reg_mag_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw mag xyz to microTesla*/
            mag_xyz->x = (double)(reg_mag_xyz.x / BNO055_MAG_DIV_UT);
            mag_xyz->y = (double)(reg_mag_xyz.y / BNO055_MAG_DIV_UT);
            mag_xyz->z = (double)(reg_mag_xyz.z / BNO055_MAG_DIV_UT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    
        return com_rslt;
    }
    BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_xyz(struct bno055_linear_accel_t *linear_accel)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Array holding the linear accel xyz value
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
         */
        u8 data_u8[BNO055_ACCEL_XYZ_DATA_SIZE] = {
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
        };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the six byte value
                 *  of linear accel xyz data*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_REG,
                                                          data_u8,
                                                          BNO055_ACCEL_XYZ_DATA_SIZE);
    
                /* Data x*/
                data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                            BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
                data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                            BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
                linear_accel->x =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));
    
                /* Data y*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                            BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                            BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
                linear_accel->y =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));
    
                /* Data z*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                            BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                            BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
                linear_accel->z =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_xyz_msq(
        struct bno055_linear_accel_double_t *linear_accel_xyz)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        struct bno055_linear_accel_t reg_linear_accel_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    
        /* Read the raw xyz of linear accel */
        com_rslt = bno055_read_linear_accel_xyz(&reg_linear_accel_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw xyz of linear accel to m/s2 */
            linear_accel_xyz->x = (double)(reg_linear_accel_xyz.x / BNO055_LINEAR_ACCEL_DIV_MSQ);
            linear_accel_xyz->y = (double)(reg_linear_accel_xyz.y / BNO055_LINEAR_ACCEL_DIV_MSQ);
            linear_accel_xyz->z = (double)(reg_linear_accel_xyz.z / BNO055_LINEAR_ACCEL_DIV_MSQ);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_xyz(struct bno055_gravity_t *gravity)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Array holding the gravity xyz value
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
         * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
         */
        u8 data_u8[BNO055_GRAVITY_XYZ_DATA_SIZE] = {
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
        };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the six byte value
                 * of gravity xyz data*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_GRAVITY_DATA_X_LSB_VALUEX_REG,
                                                          data_u8,
                                                          BNO055_GRAVITY_XYZ_DATA_SIZE);
    
                /* Data x*/
                data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                            BNO055_GRAVITY_DATA_X_LSB_VALUEX);
                data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                            BNO055_GRAVITY_DATA_X_MSB_VALUEX);
                gravity->x =
                    (s16)(((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB]) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));
    
                /* Data y*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                            BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
                data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                            BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
                gravity->y =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));
    
                /* Data z*/
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                            BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
                data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                            BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
                gravity->z =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gravity_xyz_msq(struct bno055_gravity_double_t *gravity_xyz)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        struct bno055_gravity_t reg_gravity_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    
        /* Read raw gravity of xyz */
        com_rslt = bno055_read_gravity_xyz(&reg_gravity_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw gravity of xyz to m/s2 */
            gravity_xyz->x = (double)(reg_gravity_xyz.x / BNO055_GRAVITY_DIV_MSQ);
            gravity_xyz->y = (double)(reg_gravity_xyz.y / BNO055_GRAVITY_DIV_MSQ);
            gravity_xyz->z = (double)(reg_gravity_xyz.z / BNO055_GRAVITY_DIV_MSQ);
        }
        else
        {
            com_rslt += BNO055_ERROR;
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_wxyz(struct bno055_quaternion_t *quaternion)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    
        /* Array holding the quaternion wxyz value
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB] - w->LSB
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB] - w->MSB
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB] - x->LSB
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB] - x->MSB
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB] - y->MSB
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB] - z->MSB
         * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB] - z->MSB
         */
        u8 data_u8[BNO055_QUATERNION_WXYZ_DATA_SIZE] = {
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
            BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
        };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the eight byte value
                 * of quaternion wxyz data*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_QUATERNION_DATA_W_LSB_VALUEW_REG,
                                                          data_u8,
                                                          BNO055_QUATERNION_WXYZ_DATA_SIZE);
    
                /* Data W*/
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB],
                                        BNO055_QUATERNION_DATA_W_LSB_VALUEW);
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB],
                                        BNO055_QUATERNION_DATA_W_MSB_VALUEW);
                quaternion->w =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB]));
    
                /* Data X*/
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB],
                                        BNO055_QUATERNION_DATA_X_LSB_VALUEX);
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB],
                                        BNO055_QUATERNION_DATA_X_MSB_VALUEX);
                quaternion->x =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB]));
    
                /* Data Y*/
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB],
                                        BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB],
                                        BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
                quaternion->y =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB]));
    
                /* Data Z*/
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB],
                                        BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
                data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB] =
                    BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB],
                                        BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
                quaternion->z =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_unit(u8 *accel_unit_u8)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, accel unit is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the accel unit */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_ACCEL_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                *accel_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_UNIT);
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(u8 *euler_unit_u8)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8r = BNO055_INIT_VALUE;
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, Euler unit is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the Euler unit */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_EULER_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                *euler_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_EULER_UNIT);
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(s16 *gyro_x_s16)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        u8 data_u8[BNO055_GYRO_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
        s8 stat_s8 = BNO055_ERROR;
    
        /* Check the struct p_bno055 is empty */
        if (p_bno055 == NULL)
        {
            return BNO055_E_NULL_PTR;
        }
        else
        {
            /*condition check for page, chip id is
             * available in the page zero*/
            if (p_bno055->page_id != BNO055_PAGE_ZERO)
            {
                /* Write the page zero*/
                stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
            }
            if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
            {
                /* Read the gyro 16 bit x value*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_GYRO_DATA_X_LSB_VALUEX_REG,
                                                          data_u8,
                                                          BNO055_LSB_MSB_READ_LENGTH);
                data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                      BNO055_GYRO_DATA_X_LSB_VALUEX);
                data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                      BNO055_GYRO_DATA_X_MSB_VALUEX);
                *gyro_x_s16 =
                    (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                          (data_u8[BNO055_SENSOR_DATA_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
    
        return com_rslt;
    }

    BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_dps(float *gyro_x_f)
    {
        /* Variable used to return value of
         * communication routine*/
        BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
        s16 reg_gyro_x_s16 = BNO055_INIT_VALUE;
        float data_f = BNO055_INIT_VALUE;
        u8 gyro_unit_u8 = BNO055_INIT_VALUE;
    
        /* Read the current gyro unit and set the
         * unit as dps if the unit is in rps */
        com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
        if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
        {
            com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
        }
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Read gyro raw x data */
            com_rslt += bno055_read_gyro_x(&reg_gyro_x_s16);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Convert the raw gyro x to dps*/
                data_f = (float)(reg_gyro_x_s16 / BNO055_GYRO_DIV_DPS);
                *gyro_x_f = data_f;
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    
        return com_rslt;
    }


    // Variable used to return value on communication routine
    s32 comres = BNO055_ERROR;
    // variable used to set the power mode of the sensor
    u8 power_mode = BNO055_INIT_VALUE;

    /*********read raw accel data***********/
    s16 accel_datax = BNO055_INIT_VALUE;
    s16 accel_datay = BNO055_INIT_VALUE;
    s16 accel_dataz = BNO055_INIT_VALUE;
    struct bno055_accel_t accel_xyz;

    /*********read raw mag data***********/
    s16 mag_datax = BNO055_INIT_VALUE;
    s16 mag_datay = BNO055_INIT_VALUE;
    s16 mag_dataz = BNO055_INIT_VALUE;
    /* structure used to read the mag xyz data */
    struct bno055_mag_t mag_xyz;

    /***********read raw gyro data***********/
    s16 gyro_datax = BNO055_INIT_VALUE;
    s16 gyro_datay = BNO055_INIT_VALUE;
    s16 gyro_dataz = BNO055_INIT_VALUE;
    /* structure used to read the gyro xyz data */
    struct bno055_gyro_t gyro_xyz;

    /*************read raw Euler data************/
    s16 euler_data_h = BNO055_INIT_VALUE;
    s16 euler_data_r = BNO055_INIT_VALUE;
    s16 euler_data_p = BNO055_INIT_VALUE;
    /* structure used to read the euler hrp data */
    struct bno055_euler_t euler_hrp;

    /************read raw quaternion data**************/
    s16 quaternion_data_w = BNO055_INIT_VALUE;
    s16 quaternion_data_x = BNO055_INIT_VALUE;
    s16 quaternion_data_y = BNO055_INIT_VALUE;
    s16 quaternion_data_z = BNO055_INIT_VALUE;
    /* structure used to read the quaternion wxyz data */
    struct bno055_quaternion_t quaternion_wxyz;

    /************read raw linear acceleration data***********/
    s16 linear_accel_data_x = BNO055_INIT_VALUE;
    s16 linear_accel_data_y = BNO055_INIT_VALUE;
    s16 linear_accel_data_z = BNO055_INIT_VALUE;
    /* structure used to read the linear accel xyz data */
    struct bno055_linear_accel_t linear_acce_xyz;

    /*****************read raw gravity sensor data****************/
    s16 gravity_data_x = BNO055_INIT_VALUE;
    s16 gravity_data_y = BNO055_INIT_VALUE;
    s16 gravity_data_z = BNO055_INIT_VALUE;
    /* structure used to read the gravity xyz data */
    struct bno055_gravity_t gravity_xyz;

    /*************read accel converted data***************/
    double d_accel_datax = BNO055_INIT_VALUE;
    double d_accel_datay = BNO055_INIT_VALUE;
    double d_accel_dataz = BNO055_INIT_VALUE;
    /* structure used to read the accel xyz data output as m/s2 or mg */
    struct bno055_accel_double_t d_accel_xyz;

    /******************read mag converted data********************/
    double d_mag_datax = BNO055_INIT_VALUE;
    double d_mag_datay = BNO055_INIT_VALUE;
    double d_mag_dataz = BNO055_INIT_VALUE;
    /* structure used to read the mag xyz data output as uT*/
    struct bno055_mag_double_t d_mag_xyz;

    /*****************read gyro converted data************************/
    double d_gyro_datax = BNO055_INIT_VALUE;
    double d_gyro_datay = BNO055_INIT_VALUE;
    double d_gyro_dataz = BNO055_INIT_VALUE;
    /* structure used to read the gyro xyz data output as dps or rps */
    struct bno055_gyro_double_t d_gyro_xyz;

    /*******************read euler converted data*******************/

    double d_euler_data_h = BNO055_INIT_VALUE;
    double d_euler_data_r = BNO055_INIT_VALUE;
    double d_euler_data_p = BNO055_INIT_VALUE;
    /* structure used to read the euler hrp data output
     * as as degree or radians */
    struct bno055_euler_double_t d_euler_hpr;

    /*********read linear acceleration converted data**********/
    double d_linear_accel_datax = BNO055_INIT_VALUE;
    double d_linear_accel_datay = BNO055_INIT_VALUE;
    double d_linear_accel_dataz = BNO055_INIT_VALUE;
    /* structure used to read the linear accel xyz data output as m/s2*/
    struct bno055_linear_accel_double_t d_linear_accel_xyz;

    /********************Gravity converted data**********************/
    double d_gravity_data_x = BNO055_INIT_VALUE;
    double d_gravity_data_y = BNO055_INIT_VALUE;
    double d_gravity_data_z = BNO055_INIT_VALUE;
    /* structure used to read the gravity xyz data output as m/s2*/
    struct bno055_gravity_double_t d_gravity_xyz;

    bno055_t*       p_bno055;
    bno055_t        m_bno055;
    static imu*     active_inst;
};