#ifndef _RGB_H
#define _RGB_H

#include <stdint.h>

/* struct */
struct RGB
{
    uint8_t Red;
    uint8_t Green;
    uint8_t Blue;
};

/* functions test */
void rgb_init();
struct RGB getRGB(uint8_t position);
uint8_t getRed(uint8_t position);
uint8_t getGreen(uint8_t position);
uint8_t getBlue(uint8_t position);

#endif	/* _RGB_H */
