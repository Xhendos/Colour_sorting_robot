#include <string.h>
#include "ax12.h"
#include "uart.h"

uint8_t ax_crc(uint8_t id, uint8_t length, ax_instruction_type_t type, uint8_t params[], uint8_t params_length)
{
	uint8_t crc = id + length + (uint8_t)type;
	for (int i = 0; i < params_length; ++i)
	{
		crc += params[i];
	}
	return ~crc;
}

uint16_t ax_read(uint8_t id, ax_register_t r)
{
	uint8_t buffer[8];
	uint8_t length;
	ax_instruction_type_t type;
	uint8_t params[2];
	volatile uint8_t crc;

	memset(buffer, 0, sizeof(buffer));

	switch (r)
	{
		case FIRMWARE_VERSION:
		case ID:
		case BAUD_RATE:
		case RETURN_DELAY_TIME:
		case TEMPERATURE_LIMIT:
		case MIN_VOLTAGE_LIMIT:
		case MAX_VOLTAGE_LIMIT:
		case STATUS_RETURN_LEVEL:
		case ALARM_LED:
		case SHUTDOWN:
		case TORQUE_ENABLE:
		case LED:
		case CW_COMPLIANCE_MARGIN:
		case CCW_COMPLIANCE_MARGIN:
		case CW_COMPLIANCE_SLOPE:
		case CCW_COMPLIANCE_SLOPE:
		case PRESENT_VOLTAGE:
		case PRESENT_TEMPERATURE:
		case REGISTERED:
		case MOVING:
		case LOCK:
			params[1] = 1;
			break;

		case MODEL_NUMBER:
		case CW_ANGLE_LIMIT:
		case CCW_ANGLE_LIMIT:
		case MAX_TORQUE:
		case GOAL_POSITION:
		case MOVING_SPEED:
		case TORQUE_LIMIT:
		case PRESENT_POSITION:
		case PRESENT_SPEED:
		case PRESENT_LOAD:
		case PUNCH:
			params[1] = 2;
				break;
	}

	params[0] = (uint8_t)r;
	length = 4; //Instruction, Param1, Param2 and Checksum.
	type = READ;
	crc = ax_crc(id, length, type, params, 2);

	_GPIOB_BSRR |= 1;

	uart_send_byte(0xff);
	uart_send_byte(0xff);
	uart_send_byte(id);
	uart_send_byte(length);
	uart_send_byte((uint8_t)type);
	uart_send_byte(params[0]);
	uart_send_byte(params[1]);
	uart_send_byte(crc);

	_GPIOB_BSRR |= (1 << 16);

	uint8_t n = 6 + params[1];
	
	for (int i = 0; i < n; ++i)
	{
		buffer[i] = uart_receive_byte();
	}

	if (params[1] == 1)
	{
		return buffer[5];
	}
	else
	{
		return (uint16_t)(buffer[5] | buffer[6] << 8);
	}
}

uint8_t ax_write(uint8_t id, ax_register_t r, uint16_t d)
{
	uint8_t buffer[6];
	uint8_t length;
	ax_instruction_type_t type;
	uint8_t params[3];
	uint8_t params_length;
	volatile uint8_t crc;

	memset(buffer, 0, sizeof(buffer));

	params[0] = (uint8_t)r;

	switch (r)
	{
		case FIRMWARE_VERSION:
		case ID:
		case BAUD_RATE:
		case RETURN_DELAY_TIME:
		case TEMPERATURE_LIMIT:
		case MIN_VOLTAGE_LIMIT:
		case MAX_VOLTAGE_LIMIT:
		case STATUS_RETURN_LEVEL:
		case ALARM_LED:
		case SHUTDOWN:
		case TORQUE_ENABLE:
		case LED:
		case CW_COMPLIANCE_MARGIN:
		case CCW_COMPLIANCE_MARGIN:
		case CW_COMPLIANCE_SLOPE:
		case CCW_COMPLIANCE_SLOPE:
		case PRESENT_VOLTAGE:
		case PRESENT_TEMPERATURE:
		case REGISTERED:
		case MOVING:
		case LOCK:
			params[1] = (uint8_t)d;
			params_length = 2;
			break;

		case MODEL_NUMBER:
		case CW_ANGLE_LIMIT:
		case CCW_ANGLE_LIMIT:
		case MAX_TORQUE:
		case GOAL_POSITION:
		case MOVING_SPEED:
		case TORQUE_LIMIT:
		case PRESENT_POSITION:
		case PRESENT_SPEED:
		case PRESENT_LOAD:
		case PUNCH:
			params[1] = (uint8_t)d;
			params[2] = (uint8_t)(d >> 8);
			params_length = 3;
			break;
	}

	length = 2 + params_length;
	type = WRITE;
	crc = ax_crc(id, length, type, params, params_length);

	_GPIOB_BSRR |= 1;

	uart_send_byte(0xff);
	uart_send_byte(0xff);
	uart_send_byte(id);
	uart_send_byte(length);
	uart_send_byte((uint8_t)type);
	for (int i = 0; i < params_length; ++i)
	{
		uart_send_byte(params[i]);
	}
	uart_send_byte(crc);

	_GPIOB_BSRR |= (1 << 16);

	uint8_t n = 6;

	for (int i = 0; i < n; ++i)
	{
		buffer[i] = uart_receive_byte();
	}

	return buffer[5];
}

void ax_debug()
{
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
    volatile uint8_t armId = 254;
	uint8_t n = 1;

	//ax_write(armId, TORQUE_ENABLE, 0);
	//ax_write(armId, MOVING_SPEED, 50);
	//ax_write(armId, GOAL_POSITION, 1023);
	//ax_write(armId, TORQUE_ENABLE, 1);

	while(1)
	{
		modelNumber = ax_read(armId, MODEL_NUMBER);
		firmwareVersion = ax_read(armId, FIRMWARE_VERSION);
		id = ax_read(armId, ID);
		baudRate = ax_read(armId, BAUD_RATE);
		returnDelayTime = ax_read(armId, RETURN_DELAY_TIME);
		cwAngleLimit = ax_read(armId, CW_ANGLE_LIMIT);
		ccwAngleLimit = ax_read(armId, CCW_ANGLE_LIMIT);
		temperatureLimit = ax_read(armId, TEMPERATURE_LIMIT);
		minVoltageLimit = ax_read(armId, MIN_VOLTAGE_LIMIT);
		maxVoltageLimit = ax_read(armId, MAX_VOLTAGE_LIMIT);
		maxTorque = ax_read(armId, MAX_TORQUE);
		statusReturnLevel = ax_read(armId, STATUS_RETURN_LEVEL);
		alarmLed = ax_read(armId, ALARM_LED);
		shutdown = ax_read(armId, SHUTDOWN);
		torqueEnable = ax_read(armId, TORQUE_ENABLE);
		led = ax_read(armId, LED);
		cwComplianceMargin = ax_read(armId, CW_COMPLIANCE_MARGIN);
		ccwComplianceMarge = ax_read(armId, CCW_COMPLIANCE_MARGIN);
		cwComplianceSlope = ax_read(armId, CW_COMPLIANCE_SLOPE);
		ccwComplianceSlope = ax_read(armId, CCW_COMPLIANCE_SLOPE);
		goalPosition = ax_read(armId, GOAL_POSITION);
		movingSpeed = ax_read(armId, MOVING_SPEED);
		torqueLimit = ax_read(armId, TORQUE_LIMIT);
		presentPosition = ax_read(armId, PRESENT_POSITION);
		presentSpeed = ax_read(armId, PRESENT_SPEED);
		presentLoad = ax_read(armId, PRESENT_LOAD);
		presentVoltage = ax_read(armId, PRESENT_VOLTAGE);
		presentTemperature = ax_read(armId, PRESENT_TEMPERATURE);
		registered = ax_read(armId, REGISTERED);
		moving = ax_read(armId, MOVING);
		lock = ax_read(armId, LOCK);
		punch = ax_read(armId, PUNCH);

		//if (!moving && abs(presentPosition - goalPosition) <= 2)
		//{
		//	if (n)
		//	{
		//		ax_write(armId, GOAL_POSITION, 0);
		//		n = 0;
		//	}
		//	else
		//	{
		//		ax_write(armId, GOAL_POSITION, 1023);
		//		n = 1;
		//	}
		//}
	}
}
