#include "i2c.h"

void i2c_init()
{
	_I2C_CR1 &= ~(1 << 1);	/* Set to i2c mode */
	_I2C_CR1 &= ~(1 << 6);	/* Generall call disabled address 00h is ACKed */
	/* bit 8 (START) and bit 9 (STOP) not sure yet. */
	_I2C_CR1 |= (1 << 10);	/* Acknowledgement after a byte is received */
	_I2C_CR1 &= ~(1 << 11);	/* POS acknowledge (N)ack controls the current byte in the shift register */
	_I2C_CR1 &= ~(1 << 12);	/* Packet error checking off */
	_I2C_CR1 &= ~(1 << 13); 	

	_I2C_CR2 = 0;			/* Disable all interrupts */
	_I2C_CR2 |= (1 << 3);	/* ABP clock frequency is 8 MHz */

	_I2C_OAR1 |= (1 << 14);	/* Should ALWAYS be one according to the datasheet */
	_I2C_OAR1 &= ~(1 << 15);	

	_I2C_OAR2 = 0;			/* Only OAR1 is recognized in the 7 bit addressing mode */

	
	_I2C_CCR |= (1 << 14);	/* Fast mode duty cycle */
	_I2C_CCR |= (1 << 15);	/* Fast mode I2C */
		
}
