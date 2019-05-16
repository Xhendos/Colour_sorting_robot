#include "../i2c.h"
#include "rgb.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * initialize function for TCS34725.
 */
void init()
{
    //TODO requires read function.

    /* Check if correct device is found */
    if(!checkDeviceID())
       return ;

    /* Set intergration time */
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, TCS34725_ATIME);
    i2c_send_byte(TCS34725_ATIME_2_4MS);
    i2c_stop_transmission();

    /* Set gain */
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, TCS34725_CONTROL);
    i2c_send_byte(TCS34725_CONTROL_GAIN_1);
    i2c_stop_transmission();

    /* Note: by default, the device is in power down mode on bootup */
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, TCS34725_ENABLE);
    i2c_send_byte(TCS34725_ENABLE_PON);
    i2c_stop_transmission();
    vTaskDelay(pdMS_TO_TICKS(3));

    /* Enables the adc */
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, TCS34725_ENABLE);
    i2c_send_byte(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    i2c_stop_transmission();
    vTaskDelay(pdMS_TO_TICKS(3));
}
/*
 * Returns the value of of each colour including clear (0-255) of the given rgb sensor.
 */
struct RGB getRGB(uint8_t position)
{
    struct RGB tmp_RGB;
    //TODO requires read function
    return tmp_RGB;
}

/*
 * Returns the value of red(0-255) of the given rgb sensor.
 */
uint8_t getRed(uint8_t position)
{
    //TODO requires read function
    return 0;
}

/*
 * Returns the value of green(0-255) of the given rgb sensor.
 */
uint8_t getGreen(uint8_t position)
{
    //TODO requires read function
    return 0;
}

/*
 * Returns the value of blue(0-255) of the given rgb sensor.
 */
uint8_t getBlue(uint8_t position)
{
    //TODO requires read function
    return 0;
}

/*
 * Checks if ID is equal to 0x44 so the corresponding sensor is either TCS34721 or TCS34725.
 */
static _Bool checkDeviceID()
{
    //TODO requires read function
    uint8_t x = 0;
    if (x == 0x44)
    {
        return 1;
    }
    return 0;
}