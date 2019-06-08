#include "rgb.h"
#include "../i2c.h"

#include <unistd.h>

/* addr */
#define TCS34725_ADDRESS (0x29)     /* < I2C address */
#define TCS34725_COMMAND_BIT (0x80) /* < Command bit */
#define TCS34725_ID (0x12)          /* < 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_ENABLE (0x00) /* Enable Register (0x00) */
#define TCS34725_RGBCTIME (0x01) /* RGBC Timing Register  (0x01) */
#define TCS34725_WAITTIME (0X03) /* Wait Time Register  (0x03) */
#define TCS34725_CNTRLREG (0X0F) /* Control Register  (0x0F) */

/* readable registers voor determining colours */
#define TCS34725_CDATA (0x14) /* < Clear channel data low byte */
#define TCS34725_RDATA (0x16) /* < Red channel data low byte */
#define TCS34725_GDATA (0x18) /* < Green channel data low byte */
#define TCS34725_BDATA (0x1A) /* < Blue channel data low byte */

/* rgb pins for controlling led and reading */
#define TCS34725_SENSOR1 () /* define location */
#define TCS34725_SENSOR2 () /* define location */
#define TCS34725_SENSOR3 () /* define location */
#define TCS34725_SENSOR4 () /* define location */
#define TCS34725_SENSOR5 () /* define location */
#define TCS34725_SENSOR6 () /* define location */
#define TCS34725_SENSOR7 () /* define location */
#define TCS34725_SENSOR8 () /* define location */
#define TCS34725_SENSOR9 () /* define location */
#define TCS34725_SENSOR10 () /* define location */
#define TCS34725_SENSOR11 () /* define location */
#define TCS34725_SENSOR12 () /* define location */

/* private functions */
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
        uint8_t AGAIN: 2;
        uint8_t : 6;
    } Bits;
    uint8_t value;
};

/*
 * initialize function for TCS34725.
 */
unsigned long time = 0;
void rgb_init()
{
    /* Check device id */
    if(checkDeviceID());

        /* Enables the adc */
        /* Select enable register(0x00) */
        /* Power ON, RGBC enable, wait time disable(0x03) */
        union EnableRegister enRegister;
        enRegister.Bits.ADCEnable = 1;
        enRegister.Bits.powerOn = 1;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_ENABLE | TCS34725_COMMAND_BIT));
        i2c_send_byte(enRegister.value);
        i2c_stop_transmission();
        while(time < 1000000)
            time++;
        time = 0;

        /* Set intergration time */
        /* Select ALS time register(0x81) */
        /* Atime = 700 ms(0x00) */
        union RGBCTimingRegister timeReg;
        timeReg.value = 0x00;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_RGBCTIME | TCS34725_COMMAND_BIT));
        i2c_send_byte(timeReg.value);
        i2c_stop_transmission();
        while(time < 1000000)
            time++;
        time = 0;

        /* Set gain */
        /* Select Wait Time register(0x83) */
        /* WTIME : 2.4ms(0xFF) */
        union WaitTimeRegister waitReg;
        waitReg.value = 0xFF;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_WAITTIME | TCS34725_COMMAND_BIT));
        i2c_send_byte(waitReg.value);
        i2c_stop_transmission();
        while(time < 1000000)
            time++;
        time = 0;

        /* Select control register(0x8F) */
        /* AGAIN = 1x(0x00) */
        /* The gain register determines the sensitivity of the diodes */
        /* 00=1x, 01=4x, 10=16x, 11=60x Gain*/
        union ControlRegister cntrlReg;
        cntrlReg.value = 0x00;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_CNTRLREG | TCS34725_COMMAND_BIT));
        i2c_send_byte(cntrlReg.value);
        i2c_stop_transmission();
}
/*
 * Returns the value of of each colour including clear (0-255) of the given rgb sensor.
 */
struct RGB getRGB(uint8_t position)
{
    struct RGB tmp_RGB;
    tmp_RGB.Red = getRed(position);
    tmp_RGB.Green = getGreen(position);
    tmp_RGB.Blue = getBlue(position);
    return tmp_RGB;
}

/*
 * Returns the value of red(0-255) of the given rgb sensor.
 */
uint8_t getRed(uint8_t position)
{
    volatile uint16_t tmpRed = 0, tmpClear = 0;
    float fRed = 0;
    uint8_t red = 0;

    /* read low Byte clear */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_CDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();

    tmpClear = i2c_read_2_bytes(TCS34725_ADDRESS);

    /* read low Byte red */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_RDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();

    tmpRed = i2c_read_2_bytes(TCS34725_ADDRESS);

    fRed = (float)((float)tmpRed / (float)tmpClear) * 255.0;
    red = fRed;

    return red;
}

/*
 * Returns the value of green(0-255) of the given rgb sensor.
 */
uint8_t getGreen(uint8_t position)
{
    volatile uint16_t tmpGreen = 0, tmpClear = 0;
    float fGreen = 0;
    uint8_t green = 0;

    /* read low Byte clear */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_CDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();

    tmpClear = i2c_read_2_bytes(TCS34725_ADDRESS);

    /* read low Byte Green */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_GDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();

    tmpGreen = i2c_read_2_bytes(TCS34725_ADDRESS);

    fGreen = (float)((float)tmpGreen / (float)tmpClear) * 255.0;
    green = fGreen;

    return green;
}

/*
 * Returns the value of blue(0-255) of the given rgb sensor.
 */
uint8_t getBlue(uint8_t position)
{
    volatile uint16_t tmpBlue = 0, tmpClear = 0;
    float fBlue = 0;
    uint8_t blue = 0;

    /* read low Byte clear */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_CDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();

    tmpClear = i2c_read_2_bytes(TCS34725_ADDRESS);

    /* read low Byte red */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_BDATA | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();

    tmpBlue = i2c_read_2_bytes(TCS34725_ADDRESS);

    fBlue = (float)((float)tmpBlue / (float)tmpClear) * 255.0;
    blue = fBlue;

    return blue;
}

/*
 * Checks if ID is equal to 0x44 so the corresponding sensor is either TCS34721 or TCS34725.
 */
static _Bool checkDeviceID()
{
    /* Read id and check if rgb sensor is a TCS34725 */
    i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_ID | TCS34725_COMMAND_BIT);
    i2c_stop_transmission();

    volatile uint8_t ret = i2c_read_byte(TCS34725_ADDRESS);
    if (ret == 0x44)
    {
        return 1;
    }
    return 0;
}
