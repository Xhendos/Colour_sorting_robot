#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "octo.h"
#include "uart.h"
#include "i2c.h"
#include "stm32f103xb.h"

position_t presentPositions[48];
position_t goalPositions[48];
uint8_t pings[48];
volatile uint8_t isr_uart_flag;
volatile uint8_t isr_i2c_flag;

QueueHandle_t taskManagerQueue;
QueueHandle_t packetQueue;
QueueHandle_t uartSignalQueue;
QueueHandle_t uartResultQueue;

void USART1_IRQ_handler(void)
{
	//This is used to ignore interrupts during startup.
	//This flag will is set manually before putting data in the uart data register.
	//Setting the flag and putting the data is done in a critical section.
	if (!isr_uart_flag)
	{
		_USART_SR &= ~(1 << 6);
	}

	unsigned long tc = _USART_SR & (1 << 6);
	unsigned long rxne = _USART_SR & (1 << 5);
	unsigned long ore = _USART_SR & (1 << 3);

	//TC
	if (tc)
	{
	}

	//RXNE or ORE
	if (rxne || ore)
	{
	}

	//Clear TC by writing 0 to it.
	_USART_SR &= ~(1 << 6);

	//Clear RXNE by reading USART_SR.
	_USART_SR;

	//Clear ORE by reading USART_SR and USART_DR.
	_USART_DR;
	
	isr_uart_flag = 0;
}

//The task manager will delete a task based on task handles send to its queue.
void manager_task()
{
	TaskHandle_t handle;

	//Create init task and wait for it to finish.
	xTaskCreate(init_task, "init", 128, NULL, configMAX_PRIORITIES - 1, NULL);
	xQueueReceive(taskManagerQueue, &handle, portMAX_DELAY);
	vTaskDelete(handle);

	//Start uart and i2c tasks.
	xTaskCreate(uart_task, "uart", 128, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(i2c_task, "i2c", 128, NULL, configMAX_PRIORITIES - 1, NULL);

	//Start user, arm, ping, position, rgb tasks.
	xTaskCreate(user_task, "user", 128, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(arm_task, "arm", 128, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(ping_task, "ping", 128, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(position_task, "position", 128, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(rgb_task, "rgb", 128, NULL, configMAX_PRIORITIES - 1, NULL);

	while (1)
	{
		xQueueReceive(taskManagerQueue, &handle, portMAX_DELAY);
		vTaskDelete(handle);
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

	_USART_SR &= ~(1 << 6); 	/* Clear TC (transmission complete) bit */

	/* Set priorities and interrupts */
	NVIC_SetPriority(37, 0x02);
	NVIC_ClearPendingIRQ(37);
	NVIC_EnableIRQ(37);

	TaskHandle_t handle = xTaskGetCurrentTaskHandle();

	xQueueSend(taskManagerQueue, &handle, portMAX_DELAY);
}

void uart_task()
{
	ax_packet_t packet;
	uint8_t header = 0xFF;
	uint8_t length;
	uint8_t type;
	uint8_t crc;
	uint8_t dummy;
	uint8_t bytes;
	uint8_t byte;

	while (1)
	{
		xQueueReceive(packetQueue, &packet, portMAX_DELAY);

		length = packet.params_length + 2;
		type = (uint8_t)packet.type;
		crc = ax_crc(packet);
		bytes = 6 + packet.params_length;
		byte = 0;

		for (int i = 0; i < bytes; ++i)
		{
			if (i <= 1)
			{
				byte = header;
			}
			else if (i == 2)
			{
				byte = packet.id;
			}
			else if (i == 3)
			{
				byte = length;
			}
			else if (i == 4)
			{
				byte = type;
			}
			else if (i >= 5 && i <= bytes - 2)
			{
				byte = packet.params[i - 5];
			}
			else if (i == bytes - 1)
			{
				byte = crc;
			}
			
			taskENTER_CRITICAL();
			_USART_DR = byte;
			isr_uart_flag = 1;
			taskEXIT_CRITICAL();
			//Get signal from isr when byte has been transmitted.
			xQueueReceive(uartSignalQueue, &dummy, portMAX_DELAY);
		}

		uint8_t result;
		uint8_t index = idToIndex(packet.id);

		switch (packet.type)
		{
			case PING:
				bytes = 6;
				for (int i = 1; i <= bytes; ++i)
				{
					xQueueReceive(uartResultQueue, &result, portMAX_DELAY);
					if (i == 5)
					{
						pings[index] = result;
					}
				}
				break;
			//Only supports two byte reads.
			//And places result in presentPositions only.
			case READ:
				bytes = 8;
				for (int i = 1; i <= bytes; ++i)
				{
					xQueueReceive(uartResultQueue, &result, portMAX_DELAY);
					if (i == 6)
					{
						presentPositions[index].xa[0] = result;
					}
					else if (i == 7)
					{
						presentPositions[index].xa[1] = result;
					}
				}
				break;
			default:
				//Invalid type.
				break;
		}
	}
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

}

void position_task()
{

}

void rgb_task()
{

}

uint8_t ax_crc(ax_packet_t packet)
{
	uint8_t crc = packet.id + (packet.params_length + 2) + (uint8_t)packet.type;
	for (int i = 0; i < packet.params_length; ++i) {
		crc += packet.params[i];
	}
	return ~crc;
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
