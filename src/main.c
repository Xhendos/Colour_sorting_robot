#include "stm32f103xb.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#define _RCC_CR			(*((volatile unsigned long *) 0x40021000))		/* Clock control register */
#define _RCC_CFGR		(*((volatile unsigned long *) 0x40021004))		/* Clock configuration register */

#define	_RCC_APB2ENR	(*((volatile unsigned long *) 0x40021018))		/* Peripheral clock enable register */
#define _RCC_APB1ENR	(*((volatile unsigned long *) 0x4002101C))		/* Peripheral clock enable register */

#define	_GPIOB_CRL		(*((volatile unsigned long *) 0x40010C00))		/* Port configuration register low */


int main(void)
{
	_RCC_CR |= 1;				/* Turn on the internal 8 MHz oscillator */
	_RCC_CFGR &= ~(0x482);		/* Do NOT divide the HCLK (which is the ABP clock) and use the internal 8 MHz oscillator as clock source */

	_RCC_APB2ENR |= 0x1D;		/* Enable alternative function and GPIO port A, B, and C */
	_RCC_APB2ENR |= (1 << 14);	/* Enable the USART1 module */
	_RCC_APB1ENR |= (1 << 21);	/* Enable the I2C1 module */

	/************************************************************
	*  Pin number  *	Pin name   	*	Alternative function	*
	*************************************************************
	*      42      *     PB6		*			I2C1_SCL		*
	*************************************************************
	*      43	   *	 PB7		*			I2C_SDA			*
	*************************************************************
	*	   29	   *	 PA8		*			USART1_CK		*
	*************************************************************
	*	   30	   *	 PA9		*			USART1_TX		*
	*************************************************************
	*	   31	   *	 PA10		*			USART1_RX		*
	*************************************************************
	*	   32	   *	 PA11		*			USART1_CTS		*
	*************************************************************
	*	   33	   *	 PA12		*			USART1_RTS		*
	*************************************************************/

	_GPIOB_CRL |= 0x33000000;	/* PB6 and PB7 are ouput (this MUST be the case BEFORE setting the alternative function */
	_GPIOB_CRL |= 0x88000000;	/* PB6 and PB7 are alternative function */		


	while(1) {}
	
	return 0;					/* We should never reach this point */
}
