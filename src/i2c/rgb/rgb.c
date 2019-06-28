#include <unistd.h>
#include "stm32f103xb.h"

#include "rgb.h"
#include "../i2c.h"

/* free rtos */
#include "FreeRTOS.h"
#include "task.h"
#include "octo.h"

/* addr */
#define TCS34725_ADDRESS (0x29)     /* < I2C address */
#define TCS34725_COMMAND_BIT (0x80) /* < Command bit */
#define TCS34725_ID (0x12)          /* < 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_ENABLE (0x00)      /* Enable Register (0x00) */
#define TCS34725_RGBCTIME (0x01)    /* RGBC Timing Register  (0x01) */
#define TCS34725_WAITTIME (0X03)    /* Wait Time Register  (0x03) */
#define TCS34725_CNTRLREG (0X0F)    /* Control Register  (0x0F) */

/* readable registers voor determining colours */
#define TCS34725_CDATA (0x14)       /* < Clear channel data low byte */
#define TCS34725_RDATA (0x16)       /* < Red channel data low byte */
#define TCS34725_GDATA (0x18)       /* < Green channel data low byte */
#define TCS34725_BDATA (0x1A)       /* < Blue channel data low byte */

/* rgb pins for controlling led and reading */
/* GPIOx_CRL  x= A..G */
#define TCS34725_SENSOR1_8_CONFIG  (*((volatile uint32_t *) 0x40010800)) /*  define location addr of sensor 1-8  low input gpioA(0 - 7)*/
#define TCS34725_SENSOR1_8_VALUES  (*((volatile uint32_t *) 0x40010810)) /* define value addrs of sensor 1-8 */
#define TCS34725_SENSOR9_12_CONFIG (*((volatile uint32_t *) 0x40010c04)) /* define location addr of sensor 9-12  high input gpioB(12 - 15)*/
#define TCS34725_SENSOR9_12_VALUES (*((volatile uint32_t *) 0x40010c10)) /* define value addrs of sensor 9-12 */



/* Local variables */
unsigned long time = 0;

/* private functions */
static _Bool checkDeviceID();
static _Bool  initPins();
static void rgb_setPin(uint8_t sensorPosition);
static void rgb_resetPins();
static void rgb_resetPin(uint8_t sensorPosition);

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
void rgb_init()
{
    /*initialize pins*/
    initPins();
    int i = 0;
    for(i = 0; i < SENSORCOUNT; i++) {
        rgb_setPin(i);
        if(checkDeviceID())
        {
        /* Enables the adc */
        /* Select enable register(0x00) */
        /* Power ON, RGBC enable, wait time disable(0x03) */
        union EnableRegister enRegister;
        enRegister.Bits.ADCEnable = 1;
        enRegister.Bits.powerOn = 1;
        i2c_begin_transmission(TCS34725_ADDRESS, (TCS34725_ENABLE | TCS34725_COMMAND_BIT));
        i2c_send_byte(enRegister.value);
        i2c_stop_transmission();
        while(time < 1000)
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
        while(time < 1000)
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
        while(time < 1000)
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
        while(time < 1000)
            time++;
        time = 0;
        }
        else
        {
            TCS34725_SENSOR1_8_VALUES = 0x00ff0000;
        }
    }
}

/*
 * Returns the amount of sensor connected to the stm or cks.
 */
uint8_t getSensorCount()
{
    return SENSORCOUNT;
}

/*
 * Returns the value of of each colour including clear (0-255) of the given rgb sensor.
 */
struct RGB getRGB(uint8_t position)
{
    rgb_setPin(position);
    while(time < 200000)
        time++;
    time = 0;
    volatile struct RGB tmp_RGB;
    tmp_RGB.Red = getRed(position);
    while(time < 10)
        time++;
    time = 0;
    tmp_RGB.Green = getGreen(position);
    while(time < 10)
        time++;
    time = 0;
    tmp_RGB.Blue = getBlue(position);
    while(time < 100000)
        time++;
    time = 0;
    rgb_resetPin(position);
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

/* Private functions */
extern QueueHandle_t i2c_to_isr;
extern QueueHandle_t i2c_from_isr;
/*
 *
 * Checks if ID is equal to 0x44 so the corresponding sensor is either TCS34721 or TCS34725.
 */
static _Bool checkDeviceID()
{
    /* Read id and check if rgb sensor is a TCS34725 */
    //i2c_begin_transmission(TCS34725_ADDRESS, TCS34725_ID | TCS34725_COMMAND_BIT);
    //i2c_stop_transmission();

    struct i2c_message m;
    m.address = TCS34725_ADDRESS;
    m.byte = TCS34725_ID | TCS34725_COMMAND_BIT;
    m.write_finished = 0;
    m.read = 1;

    xQueueSend(i2c_to_isr, &m, portMAX_DELAY);
    _I2C_CR1 |= (1 << 8);
    xQueueReceive(i2c_from_isr, &m, portMAX_DELAY);

    volatile int i = 0;
    i++;
    volatile uint8_t ret = i2c_read_byte(TCS34725_ADDRESS);
    if (ret == 0x44) {
        return 1;

    }

    return 0;
}

/*
 * Function for initializing all gpio for the rgb sensors.
 */
static _Bool initPins()
{
    /* set pins 0 and 1 of A as output */
    /* 0x1 == 0001 push pull 10mhz */
    /* Set A0 to A7 sensor 1 to 8 */
    TCS34725_SENSOR1_8_CONFIG = 0x11111111;
    /* Set B12 to B15*/
    TCS34725_SENSOR9_12_CONFIG = 0x11110000;
    return 1;
}

/*
 * Used for resetting all rgb sensor pins.
 */
static void rgb_resetPins()
{
    TCS34725_SENSOR1_8_VALUES = 0x00FF0000;
    TCS34725_SENSOR9_12_VALUES = 0xF0000000;
}

/*
 * Used for resetting rgb sensor pin.
 */
static void rgb_resetPin(uint8_t sensorPosition)
{
    if(sensorPosition == 0)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00010000);
    }
    if(sensorPosition > 0 &&  sensorPosition < 8)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00010000 << sensorPosition);
    }
    else
    {
        TCS34725_SENSOR9_12_VALUES = (0x10000000 << (sensorPosition - 8));
    }
}

/*
 * Used for setting rgb sensor pin.
 */
static void rgb_setPin(uint8_t sensorPosition)
{
    /* Reset pins */
    rgb_resetPins();
    /* Set pin */
    if(sensorPosition == 0)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00000001);
    }
    if(sensorPosition > 0 &&  sensorPosition < 8)
    {
        TCS34725_SENSOR1_8_VALUES = (0x00000001 << sensorPosition);
    }
    else
    {
        TCS34725_SENSOR9_12_VALUES = (0x00001000 << (sensorPosition - 8));
    }
}
