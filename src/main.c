#include "stm32f103xb.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "i2c/i2c.h"
#include "uart/uart.h"

#define _RCC_CR			(*((volatile unsigned long *) 0x40021000))		/* Clock control register */
#define _RCC_CFGR		(*((volatile unsigned long *) 0x40021004))		/* Clock configuration register */

#define	_RCC_APB2ENR	(*((volatile unsigned long *) 0x40021018))		/* Peripheral clock enable register */
#define _RCC_APB1ENR	(*((volatile unsigned long *) 0x4002101C))		/* Peripheral clock enable register */
#define _RCC_APB1RSTR	(*((volatile unsigned long *) 0x40021010))		/* Peripheral reset register */
#define _RCC_APB2RSTR	(*((volatile unsigned long *) 0x4002100C))		/* APB2 peripheral reset register */


/* GPIOA */
#define _GPIOA_CRH		(*((volatile unsigned long *) 0x40010804))		/* Port configuration register high */
#define	_GPIOA_BSRR		(*((volatile unsigned long *) 0x40010810))		/* set/reset register */

/* GPIOB */
#define	_GPIOB_CRL		(*((volatile unsigned long *) 0x40010C00))		/* Port configuration register low */
#define _GPIOB_BSRR		(*((volatile unsigned long *) 0x40010C10))		/* set/reset register */
#define _GPIOB_ODR		(*((volatile unsigned long *) 0x40010C0C))		/* Output data register */
#define _GPIOB_IDR		(*((volatile unsigned long *) 0x40010C08))		/* Input data register */

/* Altnerative function I/O pins */
#define _AFIO_EXTICR1	(*((volatile unsigned long *) 0x40010008))		/* External interrupt configuration 1 */

/* External interrupt/event controller */
#define _EXTI_IMR		(*((volatile unsigned long *) 0x40010400))		/* Interrupt mask register */
#define _EXTI_EMR		(*((volatile unsigned long *) 0x40010404))		/* Event mask register */
#define _EXTI_RTSR		(*((volatile unsigned long *) 0x40010408))		/* Rising trigger selection register */
#define _EXTI_FTSR		(*((volatile unsigned long *) 0x4001040C))		/* Falling trigger selection register */
#define _EXTI_SWIER		(*((volatile unsigned long *) 0x40010410))		/* Software interrupt event register */
#define _EXTI_PR		(*((volatile unsigned long *) 0x40010414))		/* Pending register */

void EXTI0_IRQ_handler(void)
{
	volatile static int i = 0;
	i++;
	if(_GPIOB_ODR & 0x02)
	{
		_GPIOB_BSRR |= (1 << 17);
		_GPIOB_BSRR |= (1 << 5);
		goto exit;
	}
	if(_GPIOB_ODR & 0x20)
	{
		_GPIOB_BSRR |= (1 << 1);
		_GPIOB_BSRR |= (1 << 21);
		goto exit;
	}

	_GPIOB_BSRR |= ~(1 << 5);
	

exit:
	_EXTI_PR = 0x01;		
}

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

	_GPIOB_CRL = 0;				
	_GPIOB_CRL |= 0x33000000;	/* PB6 and PB7 are ouput (this MUST be the case BEFORE setting the alternative function */
	_GPIOB_CRL |= 0xCC000000;	/* PB6 and PB7 are alternative function */		

	_GPIOA_CRH = 0;
	_GPIOA_CRH |= 0x30;			/* PA9 is an output pin */
	_GPIOA_CRH &= ~(0x300);		/* PA10 is an input pin */
	_GPIOA_CRH |= 0x80;			/* PA9 is an alternative function push pull pin */
	_GPIOA_CRH |= 0x400;		/* PA10 is an floating pin */

	_RCC_APB1RSTR |= ( 1 << 21);	/* Reset the I2C1 module */
	_RCC_APB1RSTR &= ~(1 << 21);	/* Stop resetting the I2C1 module */	

	_RCC_APB2RSTR |= (1 << 14);	/* Reset the USART1 module */
	_RCC_APB2RSTR &= ~(1 << 14);/* Stop resetting the USART1 module */
	
	
	//i2c_init();					/* Initialise the I2C1 module */
	//uart_init();				/* Initialise the USART1 module */

	_GPIOB_CRL |= 0x200024;			/* PB0 is an input pin. PB1 and PB3 are output push/pull */			
	_AFIO_EXTICR1 |= 0x01;	

	_EXTI_IMR |= 0x01;
	_EXTI_FTSR |= 0x01;
	
/*	
	NVIC_SetPriority(6, 0x03);
	NVIC_EnableIRQ(6);		
*/
	while(1)
	{
		volatile uint8_t data = _GPIOB_IDR & 0x01;
				
		if(data)
		{
			_GPIOB_BSRR |= (1 << 1);
			_GPIOB_BSRR |= (1 << 21);
		} else
		{
			_GPIOB_BSRR |= (1 << 17);
			_GPIOB_BSRR |= (1 << 5);
		}		
	}
	
	return 0;					/* We should never reach this point */
}
