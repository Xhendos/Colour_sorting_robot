#include "i2c.h"
#include <stdint.h>

/* i2c_init() assumes that PB6 and PB7 are configured to be an alternate function pin */
void i2c_init()
{
	_I2C_CR2 = 0x8;		/* The peripheral clock frequency is 8 MHz */		
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

	_I2C_CR1 |= 1;				/* Turn on the peripheral */		
}


uint8_t i2c_begin_transmission(uint8_t address, uint8_t byte)
{
	_I2C_CR1 |= (1 << 8);		/* Generate a START condition by pulling the I2C data bus logic LOW */
	while(!(_I2C_SR1 & 0x1));	/* Wait untill the START condition has been generated */
	
								/* Transmit slave address (7 bits) and read (1) or write (0) bit */
	_I2C_DR = (address << 1) | 0;	
	while(!(_I2C_SR1 & 0x2));	/* Wait untill the slave address has been send */
	_I2C_SR1;					
	_I2C_SR2;					/* Dummy read to clear the _I2C_SR1 ADDR status bit */

	_I2C_DR = byte;				/* Transmit the first byte */
	while(!(_I2C_SR1 & 0x04));	/* Wait untill the byte has been transfered */	
	
	return I2C_OK;
}

uint8_t i2c_send_byte(uint8_t byte)
{
	_I2C_DR = byte;
	while(!(_I2C_SR1 & 0x04));	/* Wait untill the byte has been transfered */

	return I2C_OK;
}

uint8_t i2c_stop_transmission()
{
	_I2C_CR1 |= (1 << 9);		/* Generate a STOP condition by pulling the I2C data bus logic HIGH */
	while(_I2C_CR1 & 0x200);	/* Wait untill the STOP condition has been send */

	return I2C_OK;
}

uint8_t i2c_read_byte(uint8_t address)
{
	uint8_t ret;

	_I2C_CR1 |= (1 << 8);		/* Generate a START condition by pulling the I2C data bus logic LOW */
	while(!(_I2C_SR1 & 0x1));	/* Wait untill the START condition has been generated */
	
								/* Transmit slave address (7 bits) and read (1) or write (0) bit */
	_I2C_DR = (address << 1) | 1;	
	while(!(_I2C_SR1 & 0x2));	/* Wait untill the slave address has been send */
	_I2C_CR1 &= ~(1 << 10);		/* Do NOT send an acknowledge bit after receiving a byte */

	_I2C_SR1;					
	_I2C_SR2;					/* Dummy read to clear the _I2C_SR1 ADDR status bit */

	while(!(_I2C_SR1 & 0x40));	/* Wait untill the data receive register is not empty */
	ret = _I2C_DR;
	
	while(_I2C_CR1 & 0x200);	/* Wait untill the STOP bit has been transmitted */
	_I2C_CR1 |= (1 << 10);		/* Set acknowledgement returned after a byte is received on */	

	return ret;	
}

uint16_t i2c_read_2_bytes(uint8_t address)
{
	uint16_t ret;
	uint8_t tmp;
	_I2C_CR1 |= (1 << 8);			/* Generate a start bit */
	while(!(_I2C_SR1 & 0x01));		/* Wait untill start bit has been generated succesfully */

	_I2C_DR = (address << 1) | 1;	/* Sent the address bit and a read bit*/
	while(!(_I2C_SR1 & 0x02));		/* Wait untill an acknowledgement has been received */
	
	_I2C_CR1 |= (1 << 11);			/* Set the POS bit */
	_I2C_SR1;
	_I2C_SR2;					/* Dummy read to clear the ADDR flag */

	_I2C_CR1 &= ~(1 << 10);		/* Do not return an acknowledgement */

	while(!(_I2C_SR1 & 0x04));	/* Wait untill byte sent succesfully */

	_I2C_CR1 |= (1 << 9);		/* Send a stop bit */
	
	tmp = _I2C_DR;
	ret = (tmp << 8);
	tmp = _I2C_DR;
	ret |= tmp;
	
	while(_I2C_CR1 & 0x200);	/* Wait untill stop bit has been sent */

	_I2C_CR1 &= ~(1 << 11);		/* Clear the POS bit */
	_I2C_CR1 |= (1 << 10);		/* Set the acknowledgement bit */

	return ret;
}
