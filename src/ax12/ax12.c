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

