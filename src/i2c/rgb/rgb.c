#include "rgb.h"
#include "../i2c.h"
#include "FreeRTOS.h"
#include "task.h"

static _Bool checkDeviceID();

/* Enable register */
union EnableRegister
{
    struct
    {
        uint8_t powerOn    : 1;
        uint8_t ADCEnable  : 1;
        uint8_t : 1;
        uint8_t waitEnable : 1;
        uint8_t RGBCIntEnb : 1;
        uint8_t : 3;
    } Bits;
    uint8_t value;
};

/* RGBC timing register */
/* The RGBC timing register controls the internal integration time of
 * the RGBC clear and IR channel ADCs in2.4-ms increments.
 * Max RGBC Count = (256 − ATIME) × 1024 up to a maximum of 65535.
 */
union RGBCTimingRegister
{
    struct
    {
        uint8_t ATIME: 8;
    } Bits;
    uint8_t value;
};

/* Wait Time Register */
/* Wait time is set 2.4 ms increments unless the WLONG bit is asserted,
 * in which case the wait times are 12×longer.
 * WTIME is programmed as a 2’s complement number.
 */
union WaitTimeRegister
{
    struct
    {
        uint8_t WTIME: 8;
    } Bits;
    uint8_t value;
};

/* Control Register */
union ControlRegister
{
    struct
    {
        uint8_t ATIME: 8;
    } Bits;
    uint8_t value;
};

/*
 * initialize function for TCS34725.
 */
void rgbInit()
{
    //TODO requires read function.

    /* Enables the adc */
    /* Select enable register(0x00) */
    /* Power ON, RGBC enable, wait time disable(0x03) */
    union EnableRegister enRegister;
    enRegister.Bits.ADCEnable = 1;
    enRegister.Bits.powerOn = 1;
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, (TCS34725_ENABLE | TCS34725_COMMAND_BIT));
    i2c_send_byte(enRegister.value);
    i2c_stop_transmission();
    vTaskDelay(pdMS_TO_TICKS(3));

    /* Set intergration time */
    /* Select ALS time register(0x81) */
    /* Atime = 700 ms(0x00) */
    union RGBCTimingRegister timeReg;
    timeReg.value = 0x00;
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, (TCS34725_RGBCTIME | TCS34725_COMMAND_BIT));
    i2c_send_byte(timeReg.value);
    i2c_stop_transmission();


    /* Set gain */
    /* Select Wait Time register(0x83) */
    /* WTIME : 2.4ms(0xFF) */
    union WaitTimeRegister waitReg;
    waitReg.value = 0xFF;
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, (TCS34725_WAITTIME | TCS34725_COMMAND_BIT));
    i2c_send_byte(waitReg.value);
    i2c_stop_transmission();

    /* Select control register(0x8F) */
    /* AGAIN = 1x(0x00) */
    union ControlRegister cntrlReg;
    cntrlReg.value = 0x00;
    i2c_begin_transmission(TCS34725_ADDRESS, I2C_WRITE, (TCS34725_CNTRLREG | TCS34725_COMMAND_BIT));
    i2c_send_byte(cntrlReg.value);
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