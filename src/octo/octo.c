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
QueueHandle_t armInstructionQueue;

TaskHandle_t armHandle;
TaskHandle_t pingHandle;
TaskHandle_t goalPositionHandle;
TaskHandle_t presentPositionHandle;
TaskHandle_t movingHandle;

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
            case AX_WRITE:
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
                case AX_WRITE:
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
						case AX_GOAL_POSITION:
							goalPositions[index].xa[0] = rx[5];
							goalPositions[index].xa[1] = rx[6];
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
	armInstructionQueue = xQueueCreate(64, sizeof(instruction_t));

	//Tasks.
	//xTaskCreate(i2c_task, "i2c", 128, NULL, 11, NULL);
    xTaskCreate(arm_task, "arm", 128, NULL, 3, &armHandle);
	xTaskCreate(uart_task, "uart", 128, NULL, 2, NULL);
	xTaskCreate(ping_task, "ping", 128, NULL, 3, &pingHandle);
	xTaskCreate(goalPosition_task, "goalPosition", 128, NULL, 3, &goalPositionHandle);
	xTaskCreate(presentPosition_task, "presentPosition", 128, NULL, 3, &presentPositionHandle);
	xTaskCreate(moving_task, "moving", 128, NULL, 3, &movingHandle);
	xTaskCreate(prepareArms_task, "prepareArms", 128, NULL, 4, NULL);
	//xTaskCreate(rgb_task, "rgb", 128, NULL, 6, NULL);

	_USART_SR &= ~(1 << 6); 	/* Clear TC (transmission complete) bit */

	/* Set priorities and interrupts */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS); //https://www.freertos.org/RTOS-Cortex-M3-M4.html
	NVIC_SetPriority(37, 0);
	NVIC_ClearPendingIRQ(37);
	NVIC_EnableIRQ(37);

    instruction_t instruction = {0, ARM_4, 240, 60, 0, "t5", "t6"};
    xQueueSend(armInstructionQueue, &instruction, portMAX_DELAY);

    xTaskNotifyGive(pingHandle);

	//Init task suicide.
	vTaskDelete(NULL);
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
			if (xQueueReceive(uartPacketQueue, &packet, pdMS_TO_TICKS(10)) == pdTRUE)
			{
				received = 1;
			}
		}
		else
		{
			if (!send)
			{
				if (xQueueSend(usartPacketQueue, &packet, pdMS_TO_TICKS(10)) == pdTRUE)
				{
					send = 1;
                    _CR1_TXEIE_SET;
				}
			}
			else
			{
                if (xQueueReceive(uartSignalQueue, &signal, portMAX_DELAY) == pdTRUE)
                {
                    received = send = 0;
                }
			}
		}
	}
}

void arm_task()
{
    static instruction_t instructions[64];
    volatile static uint8_t count = 0;
    volatile static instruction_t *instruction;
    volatile static instruction_t *previous_instruction;
    volatile static uint8_t instruction_allowed;
    volatile static uint8_t instruction_stateChangeComplete;
    volatile static uint8_t index;

    while(uxQueueMessagesWaiting(armInstructionQueue))
    {
        if (xQueueReceive(armInstructionQueue, &instructions[count], pdMS_TO_TICKS(10)) == pdTRUE)
        {
            ++count;
        }
    }

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (uint8_t n = 0; n < count; ++n)
        {
            instruction = &instructions[n];

            if (instruction->flag)
            {
                continue;
            }

            instruction_allowed = 1;

            for (uint8_t nn = 0; nn < n; ++nn)
            {
                previous_instruction = &instructions[nn];

                if (previous_instruction->flag)
                {
                    continue;
                }

                if (strcmp(instruction->from, previous_instruction->from) == 0
                    || strcmp(instruction->from, previous_instruction->to) == 0
                    || strcmp(instruction->to, previous_instruction->from) == 0
                    || strcmp(instruction->to, previous_instruction->to) == 0)
                {
                    instruction_allowed = 0;
                    break;
                }
            }

            if (!instruction_allowed)
            {
                continue;
            }

            instruction_stateChangeComplete = 1;

            for (int motor = MOTOR_B; motor <= MOTOR_F; ++motor)
            {
                index = instruction->arm * 6 + motor - 1;

                if (abs(goalPositions[index].x - presentPositions[index].x) > 5)
                {
                    instruction_stateChangeComplete = 0;
                    break;
                }
            }

            if (!instruction_stateChangeComplete)
            {
                continue;
            }

            switch (instruction->state)
            {
                case 0: //rotate
                    //rotate(instruction->arm, instruction->r1);
                    break;
                case 1: //extend
                    stretch(instruction->arm, 105, 105, 105);
                    break;
                case 2: //close
                    claw(instruction->arm, 130, 170);
                    break;
                case 3: //lift
                    wrist(instruction->arm, 60);
                    break;
                case 4: //retract
                    stretch(instruction->arm, 195, 60, 60);
                    break;
                case 5: //rotate
                    //rotate(instruction->arm, instruction->r2);
                    break;
                case 6: //extend
                    stretch(instruction->arm, 105, 105, 60);
                    break;
                case 7: //put
                    wrist(instruction->arm, 105);
                    break;
                case 8: //open
                    claw(instruction->arm, 150, 150);
                    break;
                case 9: //retract
                    stretch(instruction->arm, 195, 60, 60);
                    break;
                case 10: //rotate
                    //rotate(instruction->arm, 150);
                    break;
                case 11: //all state changes complete for this instruction.
                    instruction->flag = 1;
                    continue;
                default: //Something went wrong.
                    continue;
            }

            ++instruction->state;
        }

        xTaskNotifyGive(pingHandle);
    }
}

void prepareArms_task()
{
    volatile static uint8_t id;
    static ax_packet_t packet;
    volatile static uint16_t cwAngleLimit = DEGREES_TO_UNITS(60);
    volatile static uint16_t ccwAngleLimit = DEGREES_TO_UNITS(240);
    volatile static uint16_t maxTorque = 0x03FF;
    volatile static uint16_t movingSpeed = RPM_TO_UNITS(RPM);
    volatile static uint16_t goalPosition = DEGREES_TO_UNITS(150);

    for (uint8_t arm = ARM_4_BASE; arm <= ARM_4_BASE; arm += 10)
    {
        for (uint8_t motor = MOTOR_B; motor <= MOTOR_F; ++motor)
        {
            id = arm + motor;
            //Torque Enable.
            packet = generateWritePacket(id, AX_TORQUE_ENABLE, 0);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //Return Delay Time.
            packet = generateWritePacket(id, AX_RETURN_DELAY_TIME, 50);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //CW Angle Limit.
            packet = generateWritePacket(id, AX_CW_ANGLE_LIMIT, cwAngleLimit);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //CCW Angle Limit.
            packet = generateWritePacket(id, AX_CCW_ANGLE_LIMIT, ccwAngleLimit);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //Max Torque.
            packet = generateWritePacket(id, AX_MAX_TORQUE, maxTorque);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //Status Return Level.
            //Return status packet for all instruction packets.
            packet = generateWritePacket(id, AX_STATUS_RETURN_LEVEL, 2);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //Alarm LED.
            packet = generateWritePacket(id, AX_ALARM_LED, 0);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //Moving Speed.
            packet = generateWritePacket(id, AX_MOVING_SPEED, movingSpeed);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //Goal Position.
            packet = generateWritePacket(id, AX_SHUTDOWN, 0);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            switch (motor)
            {
                case MOTOR_A:
                    goalPosition = DEGREES_TO_UNITS(150);
                    break;
                case MOTOR_B:
                    goalPosition = DEGREES_TO_UNITS(195);
                    break;
                case MOTOR_C:
                    goalPosition = DEGREES_TO_UNITS(60);
                    break;
                case MOTOR_D:
                    goalPosition = DEGREES_TO_UNITS(60);
                    break;
                case MOTOR_E:
                    goalPosition = DEGREES_TO_UNITS(150);
                    break;
                case MOTOR_F:
                    goalPosition = DEGREES_TO_UNITS(150);
                    break;
                default:
                    goalPosition = DEGREES_TO_UNITS(150);
                    break;
            }
            packet = generateWritePacket(id, AX_GOAL_POSITION, goalPosition);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            //Torque Enable.
            packet = generateWritePacket(id, AX_TORQUE_ENABLE, 1);
            xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
        }
    }

    vTaskDelete(NULL);
}

void ping_task()
{
	static ax_packet_t packet;
    packet.type = AX_PING;
    packet.params_length = 0;

    while (1)
	{
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		for (uint8_t arm = ARM_4_BASE; arm <= ARM_4_BASE; arm += 10)
		{
			for (uint8_t motor = MOTOR_B; motor <= MOTOR_F; ++motor)
			{
				packet.id = arm + motor;
				if (xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10)) == pdFALSE)
                {
                    --motor;
                }
			}
		}

        xTaskNotifyGive(goalPositionHandle);
	}
}

void goalPosition_task()
{
    static ax_packet_t packet;
    packet.type = AX_READ;
    packet.params_length = 2;
    packet.params[0] = AX_GOAL_POSITION;
    packet.params[1] = 2;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		for (uint8_t arm = ARM_4_BASE; arm <= ARM_4_BASE; arm += 10)
		{
			for (uint8_t motor = MOTOR_B; motor <= MOTOR_F; ++motor)
            {
                packet.id = arm + motor;
				if (xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10)) == pdFALSE)
                {
                    --motor;
                }
            }
        }

        xTaskNotifyGive(presentPositionHandle);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		for (uint8_t arm = ARM_4_BASE; arm <= ARM_4_BASE; arm += 10)
		{
			for (uint8_t motor = MOTOR_B; motor <= MOTOR_F; ++motor)
            {
                packet.id = arm + motor;
				if (xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10)) == pdFALSE)
                {
                    --motor;
                }
            }
        }

        xTaskNotifyGive(movingHandle);
    }
}

void moving_task()
{
    static ax_packet_t packet;
    packet.type = AX_READ;
    packet.params_length = 2;
    packet.params[0] = AX_MOVING;
    packet.params[1] = 1;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		for (uint8_t arm = ARM_4_BASE; arm <= ARM_4_BASE; arm += 10)
		{
			for (uint8_t motor = MOTOR_B; motor <= MOTOR_F; ++motor)
            {
                packet.id = arm + motor;
				if (xQueueSend(uartPacketQueue, &packet, pdMS_TO_TICKS(10)) == pdFALSE)
                {
                    --motor;
                }
            }
        }

        xTaskNotifyGive(armHandle);
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
	setGoalPosition(aMotorId, aDegrees);
}

void stretch(arm_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees)
{
	uint8_t bMotorId = arm * 10 + MOTOR_B;
	uint8_t cMotorId = arm * 10 + MOTOR_C;
	uint8_t dMotorId = arm * 10 + MOTOR_D;
	setGoalPosition(bMotorId, bDegrees);
	setGoalPosition(cMotorId, cDegrees);
	setGoalPosition(dMotorId, dDegrees);
}

void wrist(arm_t arm, uint16_t dDegrees)
{
	uint8_t dMotorId = arm * 10 + MOTOR_D;
	setGoalPosition(dMotorId, dDegrees);
}

void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees)
{
	uint8_t eMotorId = arm * 10 + MOTOR_E;
	uint8_t fMotorId = arm * 10 + MOTOR_F;
	setGoalPosition(eMotorId, eDegrees);
	setGoalPosition(fMotorId, fDegrees);
}

void setGoalPosition(uint8_t motorId, uint16_t degrees)
{
	uint16_t units = DEGREES_TO_UNITS(degrees);
	uint8_t index = idToIndex(motorId);
	ax_packet_t packet = generateWritePacket(motorId, AX_GOAL_POSITION, units);
	xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
}
