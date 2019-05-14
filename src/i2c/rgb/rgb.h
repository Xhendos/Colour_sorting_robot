#ifndef _RGB_H
#define _RGB_H

/* TCS34725 address */
#define TCS34725_ADDRESS (0x29) /* TCS34725 address */
#define TCS34725_ID      (0x12) /* 0x44 = TCS34721/TCS34725 */
/* ENABLE register (0x00) */
#define TCS34725_ENABLE (0x00)  /* ENABLE register */
#define TCS34725_ENABLE_PON (0x01) /* power on bit 1 is on 0 is off*/
#define TCS34725_ENABLE_AEN (0x02) /* adc enable bit 1 is on 0 is off*/
/* RGBC Timing Register  (0x01) */
#define TCS34725_ATIME (0x01)  /* ATIME register  integration time */
#define TCS34725_ATIME_2_4MS (0xFF) /*< WLONG0 = 2.4ms   WLONG1 = 0.029s */
/* Control Register  (0x0F) */
#define TCS34725_CONTROL (0x0F) /* Address of control register */
#define TCS34725_CONTROL_GAIN_1 (0x00) /* Sets gain to 1x */
/* struct */
struct RGB
{
    uint8_t Red;
    uint8_t Green;
    uint8_t Blue;
};

/* functions test */
void init();
struct RGB getRGB(uint8_t position);
uint8_t getRed(uint8_t position);
uint8_t getGreen(uint8_t position);
uint8_t getBlue(uint8_t position);

/* Private functions */
static _Bool checkDeviceID();
#endif	/* _RGB_H */
