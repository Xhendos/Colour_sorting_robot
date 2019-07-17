#ifndef _RGB_H
#define _RGB_H

#include <stdint.h>

/* Sensor count */
#define SENSORCOUNT 0

/* addr */
#define TCS34725_ADDRESS    (0x29)      /* < I2C address */
#define TCS34725_COMMAND_BIT (0x80)     /* < Command bit */
#define TCS34725_ID         (0x12)      /* < 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_ENABLE     (0x00)      /* Enable Register (0x00) */
#define TCS34725_RGBCTIME   (0x01)      /* RGBC Timing Register  (0x01) */
#define TCS34725_WAITTIME   (0X03)      /* Wait Time Register  (0x03) */
#define TCS34725_CNTRLREG   (0X0F)      /* Control Register  (0x0F) */

/* readable registers voor determining colours */
#define TCS34725_CDATA (0x14) /* < Clear channel data low byte */
#define TCS34725_RDATA (0x16) /* < Red channel data low byte */
#define TCS34725_GDATA (0x18) /* < Green channel data low byte */
#define TCS34725_BDATA (0x1A) /* < Blue channel data low byte */

/* rgb pins for controlling led and reading */
/* GPIOx_CRL  x= A..G */
#define TCS34725_SENSOR1_8_CONFIG  (*((volatile uint32_t *) 0x40010800)) /*  define location addr of sensor 1-8  low input gpioA (0 - 7)*/
#define TCS34725_SENSOR1_8_VALUES  (*((volatile uint32_t *) 0x40010810)) /* define value addrs of sensor 1-8 */
#define TCS34725_SENSOR9_12_CONFIG (*((volatile uint32_t *) 0x40010c04)) /* define location addr of sensor 9-12  high input gpioB (12 - 15)*/
#define TCS34725_SENSOR9_12_VALUES (*((volatile uint32_t *) 0x40010c10)) /* define value addrs of sensor 9-12 */

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

#endif	/* _RGB_H */
