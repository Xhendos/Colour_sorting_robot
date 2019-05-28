#ifndef _I2C_H
#define _I2C_H

#include <stdint.h>

/* Absolute register addresses */
#define _I2C_CR1	(*((volatile unsigned long *) 0x40005400)) /* Control register 1*/
#define _I2C_CR2	(*((volatile unsigned long *) 0x40005404)) /* Control register 2 */
#define _I2C_OAR1	(*((volatile unsigned long *) 0x40005408)) /* Own address register 1 */
#define _I2C_OAR2	(*((volatile unsigned long *) 0x4000540C)) /* Own address register 2 */
#define _I2C_DR		(*((volatile unsigned long *) 0x40005410)) /* Data register */
#define _I2C_SR1	(*((volatile unsigned long *) 0x40005414)) /* Status register 1 */
#define _I2C_SR2	(*((volatile unsigned long *) 0x40005418)) /* Status register 2 */
#define _I2C_CCR	(*((volatile unsigned long *) 0x4000541C)) /* Clock control register */
#define _I2C_TRISE	(*((volatile unsigned long *) 0x40005420)) /* TRISE register */

/* Error codes returned from library routines */
#define I2C_OK		0	/* Everything is fine. No error occured */


void i2c_init();

/*
 * Begins a transmission by sending a start bit
 * This function may only be called if the I2C1 module is not busy (e.g no communication is still going on) 
 * If this function has been called previously, this function may only be called again after the i2c_stop_transmission() has been called
 * _________    ___________________________         _______     _____________________________             _
 *         |    |                         |         |     |     |                           |             |
 *         |    |                         |         |     |     |                           |             |
 *         |____|                         |_________|     |_____|                           |_____________|
 *         Start bit     Slave address (7 bits)       R/W    A                   byte (8 bits)             
 */
uint8_t i2c_begin_transmission(uint8_t address, uint8_t byte);

/*
 * Send a byte on the I2C bus.
 *
 */
uint8_t i2c_send_byte(uint8_t byte);

/* 
 * Send a STOP bit to the I2C slave
 * This function may only be called if the i2c_begin_transmission() function has been called.
 *
 *              _________
 *              |
 *              |
 * _____________|
 *                Stop bit
 */
uint8_t i2c_stop_transmission();

uint8_t i2c_read_byte(uint8_t address);

uint16_t i2c_read_2_bytes();

#endif	/* _I2C_H */
