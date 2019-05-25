#include "i2c.h"
#include <stdint.h>

/* i2c_init() assumes that PB6 and PB7 are configured to be an alternate function pin */
void i2c_init()
{
	_I2C_CR2 = 0x8;		/* The peripheral clock frequency is 8 MHz */		
	_I2C_CR1 = 0x400;	/* Send acknowledgement after a byte is received */
	
	/*
	 * Thigh = CCR * TPCLK1
	 * CCR = Thigh / TPCLK1
	 *
 	 * To get TPCLK1 we use f = 1 / t
	 * 1 / 8000000 = 125 ns
	 * This means that TPCLK1 is 125 ns.
	 *
	 * Thigh = Tscl / 2
	 * To get Tscl we use Tscl = 1 / Fscl
	 * If we want to generate 100 KHz SCL (Fscl = 1000000 Hz) then Tscl is 10 microseconds
	 * Thigh = 10 us / 2 = 5000 ns
	 * 
	 * CCR = Thigh / TPCLK1
	 *     = 5000 (ns) / 125 ns = 40
	 * CCR should be 40 or 0x28 */

	_I2C_CCR &= ~(0xC000);		
	_I2C_CCR |= 0x28;			/* Generate 100 KHz serial clock speed */
	
	/* 
	 * TRISE = (Trise / TPCLK1) + 1
	 *       = 1000 (ns) / 125 (ns) + 1
	 *       = 9 */
	
	_I2C_TRISE = 0x9;			/* Maximum rise time */

	_I2C_OAR1 &= ~(0x80FF);		/* Use 7 bit slave addresses */
	_I2C_OAR1 |= (0x29 << 1);	/* TCS3472 uses I2C slave adderss 0x29 */

	_I2C_CR1 |= 1;				/* Turn on the peripheral */		
}


uint8_t i2c_begin_transmission(uint8_t address, I2C_dir dir, uint8_t byte)
{
	_I2C_CR1 |= (1 << 8);		/* Generate a START condition by pulling the I2C data bus logic LOW */
	while(!(_I2C_SR1 & 0x1));	/* Wait untill the START condition has been generated */
	
								/* Transmit slave address (7 bits) and read (1) or write (0) bit */
	_I2C_DR |= (address << 1) | (dir) ? 1 : 0;	
	while(!(_I2C_SR1 & 0x2));	/* Wait untill the slave address has been send */
	_I2C_SR1;					
	_I2C_SR2;					/* Dummy read to clear the _I2C_SR1 ADDR status bit */

	_I2C_DR |= byte;			/* Transmit the first byte */
	while(!(_I2C_SR1 & 0x04));	/* Wait untill the byte has been transfered */	
	
	return I2C_OK;
}

uint8_t i2c_send_byte(uint8_t byte)
{
	_I2C_DR |= byte;
	while(!(_I2C_SR1 & 0x04));	/* Wait untill the byte has been transfered */

	return I2C_OK;
}

uint8_t i2c_stop_transmission()
{
	_I2C_CR1 |= (1 << 9);		/* Generate a STOP condition by pulling the I2C data bus logic HIGH */
	while(_I2C_CR1 & 0x200);	/* Wait untill the STOP condition has been send */

	return I2C_OK;
}
