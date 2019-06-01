#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ax12.h"
#include "octo.h"
#include "uart.h"
#include "i2c.h"
#include "stm32f103xb.h"

position_t presentPositions[48];
position_t goalPositions[48];
uint8_t pings[48];
uint8_t dummy;
uint8_t inProgress;

QueueHandle_t uartPacketQueue;
QueueHandle_t uartSignalQueue;
QueueHandle_t uartResultQueue;

void USART1_IRQ_handler(void)
{
	static ax_packet_t packet;
	volatile static uint8_t tx[16];
	volatile static uint8_t rx[16];
	volatile static uint8_t txn = 0;
	volatile static uint8_t rxn = 0;
	volatile static uint8_t txbytes = 0;
	volatile static uint8_t rxbytes = 0;
	volatile static uint8_t index = 0;

	if (!inProgress)
	{
		if (xQueueReceiveFromISR(uartPacketQueue, &packet, NULL) == pdFALSE)
		{
			return;
		}

		tx[0] = 0xFF;
		tx[1] = 0xFF;
		tx[2] = packet.id;
		tx[3] = packet.params_length + 2;
		tx[4] = (uint8_t)packet.type;
		for (int i = 0; i < packet.params_length; ++i)
		{
			tx[5 + i] = packet.params[i];
		}
		txbytes = 6 + packet.params_length;
		tx[txbytes - 1] = ax_crc(tx[2], tx[3], tx[4], packet.params, packet.params_length);

		switch (packet.type)
		{
			case AX_PING:
				rxbytes = 6;
				break;
			case AX_READ:
				rxbytes = 6 + packet.params[1];
				break;
		}

		txn = 0;
		rxn = 0;
		index = idToIndex(packet.id);
		inProgress = 1;
	}

	volatile uint32_t sr = _USART_SR;
	volatile uint32_t dr = _USART_DR;
	volatile uint32_t cr1 = _USART_CR1;
	volatile uint8_t txe = (sr >> 7) & 1;
	volatile uint8_t tc = (sr >> 6) & 1;
	volatile uint8_t rxne = (sr >> 5) & 1;
	volatile uint8_t txeie = (cr1 >> 7) & 1;
	volatile uint8_t tcie = (cr1 >> 6) & 1;
	volatile uint8_t rxneie = (cr1 >> 5) & 1;
	volatile uint8_t data = (uint8_t)dr;

	if (tcie && tc)
	{
		_CR1_TCIE_CLEAR;
		_CR1_RXNEIE_SET;
		_GPIOB_BSRR |= (1 << 16);
	}

	if (txeie && txe)
	{
		if (txn < txbytes)
		{
			_GPIOB_BSRR |= 1;
			_USART_DR = tx[txn];
			++txn;
		}
		else
		{
			_CR1_TXEIE_CLEAR;
			_CR1_TCIE_SET;
		}
	}

	if (rxneie && rxne)
	{
		rx[rxn] = data;
		++rxn;

		if (rxn == rxbytes)
		{
			switch (packet.type)
			{
				case AX_PING:
					pings[index] = rx[4];
					break;
				case AX_READ:
					switch (packet.params[0])
					{
						case AX_PRESENT_POSITION:
							presentPositions[index].xa[0] = rx[4];
							presentPositions[index].xa[1] = rx[5];
							break;
					}
					break;
			}

			_CR1_RXNEIE_CLEAR;
			inProgress = 0;
		}
	}
}

void init_task()
{
	_RCC_CR |= 1;				/* Turn on the internal 8 MHz oscillator */
	_RCC_CFGR &= ~(0x482);		/* Do NOT divide the HCLK (which is the ABP clock) and use the internal 8 MHz oscillator as clock source */

	_RCC_APB2ENR |= 0x1D;		/* Enable alternative function and GPIO port A, B, and C */
	_RCC_APB2ENR |= (1 << 14);	/* Enable the USART1 module */
	_RCC_APB1ENR |= (1 << 21);	/* Enable the I2C1 module */

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

	_RCC_APB1RSTR |= ( 1 << 21);/* Reset the I2C1 module */
	_RCC_APB1RSTR &= ~(1 << 21);/* Stop resetting the I2C1 module */	

	_RCC_APB2RSTR |= (1 << 14);	/* Reset the USART1 module */
	_RCC_APB2RSTR &= ~(1 << 14);/* Stop resetting the USART1 module */

	i2c_init();					/* Initialise the I2C1 module */
	uart_init();				/* Initialise the USART1 module */

	//Good pings are 0x0 and could otherwise not be distinguished.
	memset(pings, ~0, sizeof(pings));

	//Create queues.
	uartPacketQueue = xQueueCreate(64, sizeof(ax_packet_t));
	uartSignalQueue = xQueueCreate(64, sizeof(uint8_t));
	uartResultQueue = xQueueCreate(64, sizeof(uint8_t));

	//Start uart and i2c tasks.
	//xTaskCreate(i2c_task, "i2c", 128, NULL, 11, NULL);

	//Start user, arm, ping, position, rgb tasks.
	//xTaskCreate(user_task, "user", 128, NULL, 10, NULL);
	//xTaskCreate(arm_task, "arm", 128, NULL, 9, NULL);
	xTaskCreate(ping_task, "ping", 128, NULL, 8, NULL);
	//xTaskCreate(position_task, "position", 128, NULL, 7, NULL);
	//xTaskCreate(rgb_task, "rgb", 128, NULL, 6, NULL);

	_USART_SR &= ~(1 << 6); 	/* Clear TC (transmission complete) bit */

	/* Set priorities and interrupts */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS); //https://www.freertos.org/RTOS-Cortex-M3-M4.html
	NVIC_SetPriority(37, 0);
	NVIC_ClearPendingIRQ(37);
	NVIC_EnableIRQ(37);

	//Init task suicide.
	vTaskDelete(NULL);
}

void i2c_task()
{

}

void user_task()
{

}

void arm_task()
{
	//These are the wrong numbers.
	//They should be in units instead of degrees.
	//150 / 0.29 = 517.
	goalPositions[0].x = 150;
	goalPositions[1].x = 195;
	goalPositions[2].x = 60;
	goalPositions[3].x = 60;
	goalPositions[4].x = 150;
	goalPositions[5].x = 150;

	goalPositions[6].x = 150;
	goalPositions[7].x = 195;
	goalPositions[8].x = 60;
	goalPositions[9].x = 60;
	goalPositions[10].x = 150;
	goalPositions[11].x = 150;

	goalPositions[42].x = 150;
	goalPositions[43].x = 195;
	goalPositions[44].x = 60;
	goalPositions[45].x = 60;
	goalPositions[46].x = 150;
	goalPositions[47].x = 150;

	instruction_t ins1 = {0, 1, 240, 60, 0, "t1", "t2"};
	instruction_t ins2 = {0, 2, 240, 150, 0, "t2", "f2"};
	instruction_t ins3 = {0, 8, 240, 60, 0, "t8", "t1"};
	instruction_t *inss[] = {&ins1, &ins2, &ins3};
	uint8_t inss_length = sizeof(inss) / sizeof(instruction_t *);

	while (1)
	{
		uint8_t flags = 0;
		for (int i = 0; i < inss_length; ++i)
		{
			instruction_t *ins = inss[i];
			if (ins->flag)
			{
				++flags;
				continue;
			}
			else
			{
				uint8_t allowed = 1;
				for (int j = 0; j < i; ++j)
				{
					instruction_t *prev_ins = inss[j];
					if (prev_ins->flag)
					{
						continue;
					}
					else
					{
						if (strcmp(ins->from, prev_ins->from) == 0
							|| strcmp(ins->from, prev_ins->to) == 0
							|| strcmp(ins->to, prev_ins->from) == 0
							|| strcmp(ins->to, prev_ins->to) == 0)
						{
							allowed = 0;
							break;
						}
					}
				}

				if (!allowed)
				{
					continue;
				}

				uint8_t stateChangeComplete = 1;
				for (int j = 0; j < 6; ++j)
				{
					uint8_t index = (ins->arm - 1) * 6 + j;
					if (abs(presentPositions[index].x - goalPositions[index].x) > 5)
					{
						//stateChangeComplete = 0;
					}
				}

				if (!stateChangeComplete)
				{
					continue;
				}

				switch (ins->state)
				{
					case 0: //rotate
						rotate(ins->arm, ins->r1);
						break;
					case 1: //extend
						stretch(ins->arm, 105, 105, 105);
						break;
					case 2: //close
						claw(ins->arm, 130, 170);
						break;
					case 3: //lift
						wrist(ins->arm, 60);
						break;
					case 4: //retract
						stretch(ins->arm, 195, 60, 60);
						break;
					case 5: //rotate
						rotate(ins->arm, ins->r2);
						break;
					case 6: //extend
						stretch(ins->arm, 105, 105, 60);
						break;
					case 7: //put
						wrist(ins->arm, 105);
						break;
					case 8: //open
						claw(ins->arm, 150, 150);
						break;
					case 9: //retract
						stretch(ins->arm, 195, 60, 60);
						break;
					case 10: //rotate
						rotate(ins->arm, 150);
						break;
					case 11:
						ins->flag = 1;
						continue;
					default: //Something went wrong.
						continue;
				}

				++ins->state;
			}
		}

		if (flags >= inss_length)
		{
			break;
		}
	}
}

void ping_task()
{
	while (1)
	{
		for (uint8_t arm = 10; arm <= 80; arm += 10)
		{
			for (uint8_t motor = 1; motor <= 6; ++motor)
			{
				ax_packet_t packet;
				//packet.id = arm + motor;
				packet.id = 61;
				packet.type = AX_PING;
				packet.params_length = 0;

				xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10));

				packet.type = AX_READ;
				packet.params[0] = AX_PRESENT_POSITION;
				packet.params[1] = 2;
				packet.params_length = 2;

				xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10));

				if (!inProgress)
				{
					_CR1_TXEIE_SET;
				}
			}
		}
	}
}

void position_task()
{

}

void rgb_task()
{

}

uint8_t idToIndex(uint8_t id)
{
	uint8_t motor = id % 10;
	uint8_t arm = (id - motor) / 10;
	return (arm - 1) * 6 + (motor - 1);
}

uint8_t indexToId(uint8_t index)
{
	uint8_t motor = (index % 6) + 1;
	uint8_t arm = (index / 6) + 1;
	return (arm * 10 + motor);
}

void rotate(uint8_t arm, uint16_t aDegrees)
{
	uint8_t index = (arm - 1) * 6;
	uint8_t aMotor = arm * 10 + 1;
	setSpeed(aMotor, 15);
	setGoalPosition(aMotor, aDegrees);
	goalPositions[index].x = aDegrees / 0.29;
}

void stretch(uint8_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees)
{
	uint8_t index = (arm - 1) * 6;
	uint8_t bMotor = arm * 10 + 2;
	uint8_t cMotor = bMotor + 1;
	uint8_t dMotor = cMotor + 1;
	uint16_t bDegreesDelta = abs(presentPositions[index + 1].x - bDegrees);
	uint16_t cDegreesDelta = abs(presentPositions[index + 2].x - cDegrees);
	uint16_t dDegreesDelta = abs(presentPositions[index + 3].x - dDegrees);
	uint16_t bRpm = (bDegreesDelta / 90.0) * 15;
	uint16_t cRpm = (cDegreesDelta / 90.0) * 15;
	uint16_t dRpm = (dDegreesDelta / 90.0) * 15;
	setSpeed(bMotor, bRpm);
	setSpeed(cMotor, cRpm);
	setSpeed(dMotor, dRpm);
	setGoalPosition(bMotor, bDegrees);
	setGoalPosition(cMotor, cDegrees);
	setGoalPosition(dMotor, dDegrees);
	goalPositions[index+1].x = bDegrees / 0.29;
	goalPositions[index+2].x = cDegrees / 0.29;
	goalPositions[index+3].x = dDegrees / 0.29;
}

void wrist(uint8_t arm, uint16_t dDegrees)
{
	uint8_t index = (arm - 1) * 6;
	uint8_t aMotor = arm * 10 + 4;
	setSpeed(aMotor, 15);
	setGoalPosition(aMotor, dDegrees);
	goalPositions[index+3].x = dDegrees / 0.29;
}

void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees)
{
	uint8_t index = (arm - 1) * 6;
	uint8_t eMotor = arm * 10 + 5;
	uint8_t fMotor = arm * 10 + 6;
	setSpeed(eMotor, 15);
	setSpeed(fMotor, 15);
	setGoalPosition(eMotor, eDegrees);
	setGoalPosition(fMotor, fDegrees);
	goalPositions[index+4].x = eDegrees / 0.29;
	goalPositions[index+5].x = fDegrees / 0.29;
}

//0--1023, 0.111 rpm per unit
void setSpeed(uint8_t motor, uint16_t rpm)
{
	uint16_t units = rpm / 0.111;
	units = units > 1023 ? 1023 : units;
	//TODO: send instruction packet.
}

//0--1023, 0.29 degree per unit
void setGoalPosition(uint8_t motor, uint16_t degrees)
{
	uint16_t units = degrees / 0.29;
	units = units > 1023 ? 1023 : units;
	//TODO: send instruction packet.
}
