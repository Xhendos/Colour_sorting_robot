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
QueueHandle_t usartPacketQueue;
QueueHandle_t uartSignalQueue;

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
    static uint8_t signal = 0x55;

	if (!inProgress)
	{
		if (xQueueReceiveFromISR(usartPacketQueue, &packet, NULL) == pdFALSE)
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

        //TODO: broadcast and rxbytes.

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
							presentPositions[index].xa[0] = rx[5];
							presentPositions[index].xa[1] = rx[6];
							break;
					}
					break;
			}

			_CR1_RXNEIE_CLEAR;
			inProgress = 0;
            xQueueSendFromISR(uartSignalQueue, &signal, NULL);
		}
	}
}

void uart_task()
{
	static ax_packet_t packet;
	static uint8_t received = 0;
	static uint8_t send = 0;
    uint8_t signal;

	while (1)
	{
		if (!received)
		{
			if (xQueueReceive(uartPacketQueue, &packet, portMAX_DELAY) == pdTRUE)
			{
				received = 1;
			}
		}
		else
		{
			if (!send)
			{
				if (xQueueSend(usartPacketQueue, &packet, portMAX_DELAY) == pdTRUE)
				{
					send = 1;
				}
			}
			else
			{
                    _CR1_TXEIE_SET;
                    xQueueReceive(uartSignalQueue, &signal, portMAX_DELAY);
                    received = send = 0;
			}
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
	memset(presentPositions, 0, sizeof(presentPositions));
	memset(goalPositions, 0, sizeof(presentPositions));

	//Queues.
	uartPacketQueue = xQueueCreate(1, sizeof(ax_packet_t));
	usartPacketQueue = xQueueCreate(1, sizeof(ax_packet_t));
	uartSignalQueue = xQueueCreate(1, sizeof(uint8_t));

	//Tasks.
	//xTaskCreate(i2c_task, "i2c", 128, NULL, 11, NULL);
    //xTaskCreate(arm_task, "arm", 128, NULL, 2, NULL);
	xTaskCreate(uart_task, "uart", 128, NULL, 3, NULL);
	xTaskCreate(ping_task, "ping", 128, NULL, 4, NULL);
	xTaskCreate(presentPosition_task, "presentPosition", 128, NULL, 4, NULL);
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
    uint8_t index = idToIndex(ARM_4_BASE + MOTOR_A);
	goalPositions[index].x = DEGREES_TO_UNITS(150);
	goalPositions[++index].x = DEGREES_TO_UNITS(195);
	goalPositions[++index].x = DEGREES_TO_UNITS(60);
	goalPositions[++index].x = DEGREES_TO_UNITS(60);
	goalPositions[++index].x = DEGREES_TO_UNITS(150);
	goalPositions[++index].x = DEGREES_TO_UNITS(150);

	instruction_t ins1 = {0, ARM_4, 240, 60, 0, "t5", "t6"};
	instruction_t *inss[] = {&ins1};
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
					volatile uint8_t index = ins->arm * 6 + j;
					volatile uint16_t difference = abs(presentPositions[index].x - goalPositions[index].x);
					if (difference > 2)
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
						//rotate(ins->arm, ins->r1);
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
						//rotate(ins->arm, ins->r2);
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
						//rotate(ins->arm, 150);
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
	static ax_packet_t packet;
    packet.type = AX_PING;
    packet.params_length = 0;

    while (1)
	{
		for (uint8_t arm = ARM_4_BASE; arm <= ARM_4_BASE; arm += 10)
		{
			for (uint8_t motor = MOTOR_B; motor <= MOTOR_F; ++motor)
			{
				packet.id = arm + motor;
				if (xQueueSend(uartPacketQueue, &packet, portMAX_DELAY) == pdFALSE)
                {
                    --motor;
                }
			}
		}
	}
}

void presentPosition_task()
{
    static ax_packet_t packet;
    packet.type = AX_READ;
    packet.params_length = 2;
    packet.params[0] = AX_PRESENT_POSITION;
    packet.params[1] = 2;

    while (1)
    {
		for (uint8_t arm = ARM_4_BASE; arm <= ARM_4_BASE; arm += 10)
		{
			for (uint8_t motor = MOTOR_B; motor <= MOTOR_F; ++motor)
            {
                packet.id = arm + motor;
				if (xQueueSend(uartPacketQueue, &packet, portMAX_DELAY) == pdFALSE)
                {
                    --motor;
                }
            }
        }
    }
}

void rgb_task()
{

}

uint8_t idToIndex(uint8_t id)
{
	uint8_t motor = id % 10;
	uint8_t arm = (id - motor) / 10;
	return arm * 6 + (motor - 1);
}

uint8_t indexToId(uint8_t index)
{
	uint8_t motor = (index % 6) + 1;
	uint8_t arm = (index / 6) * 10;
	return arm + motor;
}

void rotate(arm_t arm, uint16_t aDegrees)
{
	uint8_t aMotorId = arm * 10 + MOTOR_A;
	setSpeed(aMotorId, RPM);
	setGoalPosition(aMotorId, aDegrees);
}

void stretch(arm_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees)
{
	volatile uint8_t index = arm * 6;
	volatile uint8_t bMotorId = arm * 10 + MOTOR_B;
	volatile uint8_t cMotorId = bMotorId + 1;
	volatile uint8_t dMotorId = cMotorId + 1;
	volatile uint16_t bDegreesDelta = abs(presentPositions[index + 1].x - DEGREES_TO_UNITS(bDegrees));
	volatile uint16_t cDegreesDelta = abs(presentPositions[index + 2].x - DEGREES_TO_UNITS(cDegrees));
	volatile uint16_t dDegreesDelta = abs(presentPositions[index + 3].x - DEGREES_TO_UNITS(dDegrees));
	volatile uint16_t bRpm = (bDegreesDelta / 90.0) * RPM;
	volatile uint16_t cRpm = (cDegreesDelta / 90.0) * RPM;
	volatile uint16_t dRpm = (dDegreesDelta / 90.0) * RPM;
	setSpeed(bMotorId, bRpm);
	setSpeed(cMotorId, cRpm);
	setSpeed(dMotorId, dRpm);
	setGoalPosition(bMotorId, bDegrees);
	setGoalPosition(cMotorId, cDegrees);
	setGoalPosition(dMotorId, dDegrees);
}

void wrist(arm_t arm, uint16_t dDegrees)
{
	uint8_t dMotorId = arm * 10 + MOTOR_D;
	setSpeed(dMotorId, RPM);
	setGoalPosition(dMotorId, dDegrees);
}

void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees)
{
	uint8_t eMotorId = arm * 10 + MOTOR_E;
	uint8_t fMotorId = arm * 10 + MOTOR_F;
	setSpeed(eMotorId, RPM);
	setSpeed(fMotorId, RPM);
	setGoalPosition(eMotorId, eDegrees);
	setGoalPosition(fMotorId, fDegrees);
}

void setSpeed(uint8_t motorId, uint16_t rpm)
{
	uint16_t units = rpm / RPM_UNIT;
	ax_packet_t packet;
	packet.id = motorId;
	packet.type = AX_WRITE;
	packet.params[0] = AX_MOVING_SPEED;
	packet.params[1] = units & 0xFF;
	packet.params[2] = (units >> 8) & 0x03;
	packet.params_length = 3;
	while (xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10) == pdFALSE));
}

void setGoalPosition(uint8_t motorId, uint16_t degrees)
{
	uint16_t units = degrees / DEGREES_UNIT;
	uint8_t index = idToIndex(motorId);
	goalPositions[index].x = units;
	ax_packet_t packet;
	packet.id = motorId;
	packet.type = AX_WRITE;
	packet.params[0] = AX_GOAL_POSITION;
	packet.params[1] = units & 0xFF;
	packet.params[2] = (units >> 8) & 0x03;
	packet.params_length = 3;
	while (xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10) == pdFALSE));
}
