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
#include "rgb.h"
#include "algo.h"

uint8_t pings[48];
uint8_t movings[48];
struct RGB rgbsensors[12];
uint8_t dummy;
uint8_t inProgress;

QueueHandle_t uartPacketQueue;
QueueHandle_t usartPacketQueue;
QueueHandle_t uartSignalQueue;
QueueHandle_t armInstructionQueue;

TaskHandle_t armHandle;
TaskHandle_t movingHandle;

struct RGB test;

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
                        case AX_MOVING:
                            movings[index] = rx[5];
					}
					pings[index] = rx[4];
					break;
			}

			_CR1_RXNEIE_CLEAR;
			inProgress = 0;
            xQueueSendFromISR(uartSignalQueue, &signal, NULL);
		}
	}
}

QueueHandle_t i2c_to_isr;
QueueHandle_t i2c_from_isr;

volatile uint8_t i2c_busy = 0;

void I2C1_EV_IRQ_handler(void)
{
	static struct i2c_message m;

	if(_I2C1_SR & 0x01)	/* SB */
	{
		if(!(i2c_busy))
		{
			if(xQueueReceiveFromISR(i2c_to_isr, &m, NULL) == pdTRUE)
				i2c_busy = 1;
		}
		if(i2c_busy)
		{
			if(!(m.write_finished))
				_I2C_DR = (m.address << 1) | 0;
			if(m.write_finished)
				_I2C_DR = (m.address << 1) | 1;
		}
		_I2C1_SR &= ~(0x01);
	}

	if(_I2C1_SR & 0x02)	/* ADDR */
	{
		if(m.write_finished)
		{
			if(m.read == 2)
			{	
				_GPIOB_CRL &= ~(1 << 25);
				_I2C_CR1 |= (1 << 11);

				_I2C_SR1;
				_I2C_SR2;

				_I2C_CR1 &= ~(1 << 10);
				_GPIOB_CRL |= (1 << 25);
			}

			if(m.read == 1)
			{
				_I2C_CR1 &= ~(1 << 10);		
					
				_I2C_SR1;
				_I2C_SR2;

			}
		}

		if(!m.write_finished)
		{
			_I2C_SR1;
			_I2C_SR2;
			_I2C_DR = m.byte;
		}

		_I2C1_SR &= ~(0x02);
	}

	if(_I2C1_SR & 0x08)	/* ADD10*/
	{

		_I2C1_SR &= ~(0x08);
	}

	if(_I2C1_SR & 0x10)	/* STOPF */
	{
		_I2C1_SR &= ~(0x10);
	}

	if(_I2C1_SR & 0x04)	/* BTF */
	{
		if(m.write_finished)
		{
			_I2C_CR1 |= (1 << 9); 		/* Send a stop bit */
			m.read_bytes[0] = _I2C_DR;
			m.read_bytes[1] = _I2C_DR;

			while(_I2C_CR1 & 0x200);	/* Wait untill stop bit has been sent */
			_I2C_CR1 &= ~(1 << 11);		/* Clear the POS bit */
			_I2C_CR1 |= (1 << 10);		/* Set the acknowledgement bit */
		}

		if(!(m.write_finished))
		{
			_I2C_CR1 |= (1 << 9);		/* Send a stop bit */
			while(_I2C_CR1 & 0x200);	/* Wait untill stop bit has been sent */

			m.write_finished = 1;
			if(m.read)
			{
				_I2C_CR1 |= (1 << 8);	/* Generate a START condition by pulling the I2C data bus low */
			}

			if(!(m.read))
			{
            	xQueueSendFromISR(i2c_from_isr, &m, NULL);
			}
		}

		_I2C1_SR &= ~(0x04);
	}

	if(_I2C1_SR & 0x80)	/* TxE */
	{
		_I2C1_SR &= ~(0x80);
	}

	if(_I2C1_SR & 0x40)	/* RxNE */
	{
		if(m.read == 1)
		{
			m.read_bytes[0] = _I2C_DR;

			while(_I2C_CR1 & 0x200);		/* Wait untill STOP bit has been transmitted */
			_I2C_CR1 |= (1 << 10);			/* Set acknowledgement returned after byte is received on */

			m.read_bytes[0] = _I2C_DR;
			xQueueSendFromISR(i2c_from_isr, &m, NULL);
		}

		if(m.read == 2)
		{

		}

		_I2C1_SR &= ~(0x40);
	}
}

void init_task()
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

	_I2C1_CR2 |= 0x200;

	//i2c_init();					/* Initialise the I2C1 module */
	uart_init();				/* Initialise the USART1 module */
	//rgb_init();

	//Good pings are 0x0 and could otherwise not be distinguished.
	memset(pings, ~0, sizeof(pings));

	//Queues.
	uartPacketQueue = xQueueCreate(1, sizeof(ax_packet_t));
	usartPacketQueue = xQueueCreate(1, sizeof(ax_packet_t));
	uartSignalQueue = xQueueCreate(1, sizeof(uint8_t));
	armInstructionQueue = xQueueCreate(64, sizeof(instruction_t));

	i2c_to_isr = xQueueCreate(1, sizeof(struct i2c_message));
  i2c_from_isr = xQueueCreate(1, sizeof(struct i2c_message));

	//Tasks.
	//xTaskCreate(i2c_task, "i2c", 128, NULL, 11, NULL);
  xTaskCreate(arm_task, "arm", 128, NULL, 3, &armHandle);
	xTaskCreate(uart_task, "uart", 128, NULL, 2, NULL);
	xTaskCreate(moving_task, "moving", 128, NULL, 3, &movingHandle);
	xTaskCreate(prepareArms_task, "prepareArms", 128, NULL, 4, NULL);
	//xTaskCreate(rgb_task, "rgb", 128, NULL, 1, NULL);
  xTaskCreate(algo_task, "algo", 500, NULL, 10, NULL);

  _USART_SR &= ~(1 << 6); 	/* Clear TC (transmission complete) bit */
  _I2C_CR2 |= (1 << 9);

	/* Set priorities and interrupts */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS); //https://www.freertos.org/RTOS-Cortex-M3-M4.html
	NVIC_SetPriority(37, 0);
	/*NVIC_SetPriority(I2C1_EV_IRQn, 1);*/
	/*NVIC_ClearPendingIRQ(I2C1_EV_IRQn); */
	/*NVIC_EnableIRQ(I2C1_EV_IRQn); */
	NVIC_ClearPendingIRQ(37);
	NVIC_EnableIRQ(37);

  xTaskNotifyGive(movingHandle);
	//Init task suicide.
	vTaskDelete(NULL);
}

void algo_task()
{
    int aa[4] = {T0, T2, T4, T6};
    int bb[4] = {F0, F1, F2, F3};

    mmaaiinn(aa, bb);

    vTaskDelete(NULL);
}

void uart_task()
{
    static ax_packet_t packet;
    uint8_t signal;

    while (1)
    {
        xQueueReceive(uartPacketQueue, &packet, portMAX_DELAY);
        xQueueSend(usartPacketQueue, &packet, portMAX_DELAY);
        _CR1_TXEIE_SET;
        xQueueReceive(uartSignalQueue, &signal, portMAX_DELAY);
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
        xQueueReceive(armInstructionQueue, &instructions[count], portMAX_DELAY);
        ++count;
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

                if (instruction->from == previous_instruction->from
                    || instruction->from == previous_instruction->to
                    || instruction->to == previous_instruction->from
                    || instruction->to == previous_instruction->to)
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

            for (int motor = MOTOR_A; motor <= MOTOR_F; ++motor)
            {
                index = instruction->arm * 6 + motor - 1;

                if (movings[index])
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
                case 0:
                    setGoalPositions(instruction->arm, instruction->r1, 195, 60, 105, 140, 160);
                    break;
                case 1:
                    setGoalPositions(instruction->arm, instruction->r1, 60, 60, 240, 140, 160);
                    break;
                case 2:
                    setGoalPositions(instruction->arm, instruction->r1, 60, 80, 220, 140, 160);
                    break;
                case 3:
                    setGoalPositions(instruction->arm, instruction->r1, 60, 80, 220, 160, 140);
                    break;
                case 4:
                    setGoalPositions(instruction->arm, instruction->r1, 105, 80, 220, 160, 140);
                    break;
                case 5:
                    setGoalPositions(instruction->arm, instruction->r1, 195, 60, 60, 160, 140);
                    break;
                case 6:
                    setGoalPositions(instruction->arm, instruction->r2, 195, 60, 60, 160, 140);
                    break;
                case 7:
                    setGoalPositions(instruction->arm, instruction->r2, 195, 60, 105, 160, 140);
                    break;
                case 8:
                    setGoalPositions(instruction->arm, instruction->r2, 60, 60, 240, 160, 140);
                    break;
                case 9:
                    setGoalPositions(instruction->arm, instruction->r2, 60, 80, 220, 160, 140);
                    break;
                case 10:
                    setGoalPositions(instruction->arm, instruction->r2, 60, 80, 220, 140, 160);
                    break;
                case 11:
                    setGoalPositions(instruction->arm, instruction->r2, 60, 60, 240, 140, 160);
                    break;
                case 12:
                    setGoalPositions(instruction->arm, instruction->r2, 105, 60, 240, 140, 160);
                    break;
                case 13:
                    setGoalPositions(instruction->arm, instruction->r2, 195, 60, 60, 140, 160);
                    break;
                case 14:
                    setGoalPositions(instruction->arm, 150, 195, 60, 60, 140, 160);
                    break;
                case 15:
                    instruction->flag = 1;
                default: //Something went wrong.
                    continue;
            }

            ++instruction->state;
        }

        xTaskNotifyGive(movingHandle);
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

    for (uint8_t arm = ARM_0_BASE; arm <= ARM_7_BASE; arm += 10)
    {
        for (uint8_t motor = MOTOR_A; motor <= MOTOR_F; ++motor)
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
            /*Status Return Level.
            Return status packet for all instruction packets./**/
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
                    goalPosition = DEGREES_TO_UNITS(140);
                    break;
                case MOTOR_F:
                    goalPosition = DEGREES_TO_UNITS(160);
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

		for (uint8_t arm = ARM_0_BASE; arm <= ARM_7_BASE; arm += 10)
		{
			for (uint8_t motor = MOTOR_A; motor <= MOTOR_F; ++motor)
            {
                packet.id = arm + motor;
				xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
            }
        }

        xTaskNotifyGive(armHandle);
    }
}

void rgb_task()
{
    uint8_t currentRGB = 0;
    while (1)
    {
        for (currentRGB = 0; currentRGB < SENSORCOUNT; currentRGB++)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            rgbsensors[currentRGB] = getRGB(currentRGB);
        }
    }
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

void setGoalPositions(arm_t arm, uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e, uint16_t f)
{
    uint8_t armBase = arm * 10;
    uint16_t degrees[6] = {a, b, c, d, e, f};

    for (int motor = MOTOR_A; motor <= MOTOR_F; ++motor)
    {
        uint8_t id = armBase + motor;
        setGoalPosition(id, degrees[motor - 1]);
    }
}

void setGoalPosition(uint8_t motorId, uint16_t degrees)
{
	uint16_t units = DEGREES_TO_UNITS(degrees);
	ax_packet_t packet = generateWritePacket(motorId, AX_GOAL_POSITION, units);
	xQueueSend(uartPacketQueue, &packet, portMAX_DELAY);
}
