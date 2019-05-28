#ifndef _RGB_H
#define _RGB_H

#include <stdint.h>

/* addr */
#define TCS34725_ADDRESS (0x29)     /* < I2C address */
#define TCS34725_COMMAND_BIT (0x80) /* < Command bit */
#define TCS34725_ID (0x12)          /* < 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_ENABLE (0x00) /* Enable Register (0x00) */
#define TCS34725_RGBCTIME (0x01) /* RGBC Timing Register  (0x01) */
#define TCS34725_WAITTIME (0X03) /* Wait Time Register  (0x03) */
#define TCS34725_CNTRLREG (0X0F) /* Control Register  (0x0F) */
/* struct */
struct RGB
{
    uint8_t Red;
    uint8_t Green;
    uint8_t Blue;
};

/* functions test */
void rgbInit();
struct RGB getRGB(uint8_t position);
uint8_t getRed(uint8_t position);
uint8_t getGreen(uint8_t position);
uint8_t getBlue(uint8_t position);

#endif	/* _RGB_H */
