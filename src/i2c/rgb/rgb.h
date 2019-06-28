#ifndef _RGB_H
#define _RGB_H

#include <stdint.h>

/* Amount of sensors */
#define SENSORCOUNT 1

/* struct for rgb */
struct RGB
{
    uint16_t Red;
    uint16_t Green;
    uint16_t Blue;
};

/* functions */
void rgb_init();
uint8_t getSensorCount();
struct RGB getRGB(uint8_t position);
uint8_t getRed(uint8_t position);
uint8_t getGreen(uint8_t position);
uint8_t getBlue(uint8_t position);

#endif	/* _RGB_H */
