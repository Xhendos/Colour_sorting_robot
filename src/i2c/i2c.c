#include "i2c.h"

/* i2c_init() assumes that PB6 and PB7 are configured to be an alternate function pin */
void i2c_init()
{
	_I2C1_CR2 = 0x8;	/* The peripheral clock frequency is 8 MHz */		
	_I2C1_CR1 = 0x400;	/* Send acknowledgement after a byte is received */
	
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
	
	_I2C1_CCR |= 0x28;	/* Generate 100 KHz serial clock speed */
	
	/* 
	 * TRISE = (Trise / TPCLK1) + 1
	 *       = 1000 (ns) / 125 (ns) + 1
	 *       = 9 */
	
	_I2C1_TRISE = 0x9;	/* Maximum rise time */

	_I2C1_OAR1 &= ~(0x80FF);	/* Use 7 bit slave addresses */
	_I2C1_OAR1 |= (0x29 << 1);	/* TCS3472 uses I2C slave adderss 0x29 */

		
}


void i2c_write(char byte)
{
	_I2C1_D
}
