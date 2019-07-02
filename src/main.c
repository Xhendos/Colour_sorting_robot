#include "stm32f103xb.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

void task_manager();

int main(void)
{
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

	/************************************************************
	*  Pin number  *	Pin name   	*	    General purpose     *
	*************************************************************
	*      ?       *     PA0--7		*			RGB0--7		    *
	*************************************************************
	*      ?	   *	 PB12--15	*			RGB12--15		*
	*************************************************************/

    RCC->CR |= 1;				/* Turn on the internal 8 MHz oscillator */
	RCC->CFGR &= ~(0x482);		/* Do NOT divide the HCLK (which is the ABP clock) and use the internal 8 MHz oscillator as clock source */

	RCC->APB2ENR |= 0x1D;		/* Enable alternative function and GPIO port A, B, and C */
	RCC->APB2ENR |= (1 << 14);	/* Enable the USART1 module */
	RCC->APB1ENR |= (1 << 21);	/* Enable the I2C1 module */

	GPIOB->CRL = 0;
	GPIOB->CRL |= 0x33000000;	/* PB6 and PB7 are ouput (this MUST be the case BEFORE setting the alternative function */
	GPIOB->CRL |= 0xCC000000;	/* PB6 and PB7 are alternative function */

	GPIOB->CRL |= 0x01;			/* PB0 is an output pin */
	GPIOB->CRL |= 0x04;			/* PB0 is an general purpose open drain pin */
    GPIOB->CRH = 0x11110000;    /* PB12--PB15 push-pull outputs */

    GPIOA->CRL = 0x11111111;    /* PA0--PA7 push-pull outputs */
	GPIOA->CRH = 0;
	GPIOA->CRH |= 0x30;			/* PA9 is an output pin */
	GPIOA->CRH &= ~(0x300);		/* PA10 is an input pin */
	GPIOA->CRH |= 0x80;			/* PA9 is an alternative function push pull pin */
	GPIOA->CRH |= 0x400;		/* PA10 is an floating pin */

	RCC->APB1RSTR |= ( 1 << 21);/* Reset the I2C1 module */
	RCC->APB1RSTR &= ~(1 << 21);/* Stop resetting the I2C1 module */

	RCC->APB2RSTR |= (1 << 14);	/* Reset the USART1 module */
	RCC->APB2RSTR &= ~(1 << 14);/* Stop resetting the USART1 module */

    USART1->CR1 |= 0x200C;       /* Enable the transmitter, receiver and the USART */
    USART1->BRR = 0x10;          /* TODO: validate the baud rate register. */

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

    /*
     * TRISE = (Trise / TPCLK1) + 1
     *       = 1000 (ns) / 125 (ns) + 1
     *       = 9 */

    I2C1->CR2 = 0x8;             /* The peripheral clock frequency is 8 MHz */
	I2C1->CCR &= ~(0xC000);
	I2C1->CCR |= 0x28;			/* Generate 100 KHz serial clock speed */
	I2C1->TRISE = 0x9;			/* Maximum rise time */
    I2C1->CR1 |= 1;              /* Turn on the peripheral */

    xTaskCreate(task_manager, "manager", 128, NULL, configMAX_PRIORITIES, NULL);
    vTaskStartScheduler();

    return -1;
}

void task_manager()
{
    static volatile BaseType_t count;

    for (;;)
    {
        ++count;
    }
}
