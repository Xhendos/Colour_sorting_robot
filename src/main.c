#include "stm32f103xb.h"
#include <stdint.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "i2c/i2c.h"
#include "uart/uart.h"

#include "ax12.h"

#define ARM_ID (41)

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

	_GPIOB_CRL |= 0x01;			/* PB0 is an output pin */
	_GPIOB_CRL |= 0x04;			/* PB0 is an general purpose open drain pin */

	_GPIOA_CRH = 0;
	_GPIOA_CRH |= 0x30;			/* PA9 is an output pin */
	_GPIOA_CRH &= ~(0x300);		/* PA10 is an input pin */
	_GPIOA_CRH |= 0x80;			/* PA9 is an alternative function push pull pin */
	_GPIOA_CRH |= 0x400;		/* PA10 is an floating pin */

	_RCC_APB1RSTR |= ( 1 << 21);	/* Reset the I2C1 module */
	_RCC_APB1RSTR &= ~(1 << 21);	/* Stop resetting the I2C1 module */	

	_RCC_APB2RSTR |= (1 << 14);	/* Reset the USART1 module */
	_RCC_APB2RSTR &= ~(1 << 14);/* Stop resetting the USART1 module */

	i2c_init();					/* Initialise the I2C1 module */
	uart_init();				/* Initialise the USART1 module */

	volatile uint16_t modelNumber;
	volatile uint16_t firmwareVersion;
	volatile uint16_t id;
	volatile uint16_t baudRate;
	volatile uint16_t returnDelayTime;
	volatile uint16_t cwAngleLimit;
	volatile uint16_t ccwAngleLimit;
	volatile uint16_t temperatureLimit;
	volatile uint16_t minVoltageLimit;
	volatile uint16_t maxVoltageLimit;
	volatile uint16_t maxTorque;
	volatile uint16_t statusReturnLevel;
	volatile uint16_t alarmLed;
	volatile uint16_t shutdown;
	volatile uint16_t torqueEnable;
	volatile uint16_t led;
	volatile uint16_t cwComplianceMargin;
	volatile uint16_t ccwComplianceMarge;
	volatile uint16_t cwComplianceSlope;
	volatile uint16_t ccwComplianceSlope;
	volatile uint16_t goalPosition;
	volatile uint16_t movingSpeed;
	volatile uint16_t torqueLimit;
	volatile uint16_t presentPosition;
	volatile uint16_t presentSpeed;
	volatile uint16_t presentLoad;
	volatile uint16_t presentVoltage;
	volatile uint16_t presentTemperature;
	volatile uint16_t registered;
	volatile uint16_t moving; 
	volatile uint16_t lock;
	volatile uint16_t punch;

	uint8_t n = 1;

	ax_write(ARM_ID, TORQUE_ENABLE, 0);
	ax_write(ARM_ID, MOVING_SPEED, 50);
	ax_write(ARM_ID, GOAL_POSITION, 1023);
	ax_write(ARM_ID, TORQUE_ENABLE, 1);

	while(1)
	{
		modelNumber = ax_read(ARM_ID, MODEL_NUMBER);
		firmwareVersion = ax_read(ARM_ID, FIRMWARE_VERSION);
		id = ax_read(ARM_ID, ID);
		baudRate = ax_read(ARM_ID, BAUD_RATE);
		returnDelayTime = ax_read(ARM_ID, RETURN_DELAY_TIME);
		cwAngleLimit = ax_read(ARM_ID, CW_ANGLE_LIMIT);
		ccwAngleLimit = ax_read(ARM_ID, CCW_ANGLE_LIMIT);
		temperatureLimit = ax_read(ARM_ID, TEMPERATURE_LIMIT);
		minVoltageLimit = ax_read(ARM_ID, MIN_VOLTAGE_LIMIT);
		maxVoltageLimit = ax_read(ARM_ID, MAX_VOLTAGE_LIMIT);
		maxTorque = ax_read(ARM_ID, MAX_TORQUE);
		statusReturnLevel = ax_read(ARM_ID, STATUS_RETURN_LEVEL);
		alarmLed = ax_read(ARM_ID, ALARM_LED);
		shutdown = ax_read(ARM_ID, SHUTDOWN);
		torqueEnable = ax_read(ARM_ID, TORQUE_ENABLE);
		led = ax_read(ARM_ID, LED);
		cwComplianceMargin = ax_read(ARM_ID, CW_COMPLIANCE_MARGIN);
		ccwComplianceMarge = ax_read(ARM_ID, CCW_COMPLIANCE_MARGIN);
		cwComplianceSlope = ax_read(ARM_ID, CW_COMPLIANCE_SLOPE);
		ccwComplianceSlope = ax_read(ARM_ID, CCW_COMPLIANCE_SLOPE);
		goalPosition = ax_read(ARM_ID, GOAL_POSITION);
		movingSpeed = ax_read(ARM_ID, MOVING_SPEED);
		torqueLimit = ax_read(ARM_ID, TORQUE_LIMIT);
		presentPosition = ax_read(ARM_ID, PRESENT_POSITION);
		presentSpeed = ax_read(ARM_ID, PRESENT_SPEED);
		presentLoad = ax_read(ARM_ID, PRESENT_LOAD);
		presentVoltage = ax_read(ARM_ID, PRESENT_VOLTAGE);
		presentTemperature = ax_read(ARM_ID, PRESENT_TEMPERATURE);
		registered = ax_read(ARM_ID, REGISTERED);
		moving = ax_read(ARM_ID, MOVING);
		lock = ax_read(ARM_ID, LOCK);
		punch = ax_read(ARM_ID, PUNCH);

		if (!moving && abs(presentPosition - goalPosition) <= 2)
		{
			if (n)
			{
				ax_write(ARM_ID, GOAL_POSITION, 0);
				n = 0;
			}
			else
			{
				ax_write(ARM_ID, GOAL_POSITION, 1023);
				n = 1;
			}
		}
	}
	
	return 0;					/* We should never reach this point */
}
