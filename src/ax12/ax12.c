#include "ax12.h"
#include "uart.h"
#include "octo.h"

uint8_t ax_crc(uint8_t id, uint8_t length, ax_instruction_type_t type, uint8_t params[], uint8_t params_length)
{
	uint8_t crc = id + length + (uint8_t)type;
	for (int i = 0; i < params_length; ++i)
	{
		crc += params[i];
	}
	return ~crc;
}

ax_packet_t generateWritePacket(uint8_t id, ax_register_t r, uint16_t data)
{
    ax_packet_t packet;
    packet.id = id;
    packet.type = AX_WRITE;
    packet.params[0] = r;
	switch (r)
	{
		case AX_FIRMWARE_VERSION:
		case AX_ID:
		case AX_BAUD_RATE:
		case AX_RETURN_DELAY_TIME:
		case AX_TEMPERATURE_LIMIT:
		case AX_MIN_VOLTAGE_LIMIT:
		case AX_MAX_VOLTAGE_LIMIT:
		case AX_STATUS_RETURN_LEVEL:
		case AX_ALARM_LED:
		case AX_SHUTDOWN:
		case AX_TORQUE_ENABLE:
		case AX_LED:
		case AX_CW_COMPLIANCE_MARGIN:
		case AX_CCW_COMPLIANCE_MARGIN:
		case AX_CW_COMPLIANCE_SLOPE:
		case AX_CCW_COMPLIANCE_SLOPE:
		case AX_PRESENT_VOLTAGE:
		case AX_PRESENT_TEMPERATURE:
		case AX_REGISTERED:
		case AX_MOVING:
		case AX_LOCK:
            packet.params[1] = LOW(data);
			packet.params_length = 2;
			break;

		case AX_MODEL_NUMBER:
		case AX_CW_ANGLE_LIMIT:
		case AX_CCW_ANGLE_LIMIT:
		case AX_MAX_TORQUE:
		case AX_GOAL_POSITION:
		case AX_MOVING_SPEED:
		case AX_TORQUE_LIMIT:
		case AX_PRESENT_POSITION:
		case AX_PRESENT_SPEED:
		case AX_PRESENT_LOAD:
		case AX_PUNCH:
            packet.params[1] = LOW(data);
            packet.params[2] = HIGH(data);
			packet.params_length = 3;
				break;
	}
    return packet;
}

uint16_t ax_read(uint8_t id, ax_register_t r)
{
	uint8_t buffer[8];
	uint8_t length;
	ax_instruction_type_t type;
	uint8_t params[2];
	volatile uint8_t crc;

	switch (r)
	{
		case AX_FIRMWARE_VERSION:
		case AX_ID:
		case AX_BAUD_RATE:
		case AX_RETURN_DELAY_TIME:
		case AX_TEMPERATURE_LIMIT:
		case AX_MIN_VOLTAGE_LIMIT:
		case AX_MAX_VOLTAGE_LIMIT:
		case AX_STATUS_RETURN_LEVEL:
		case AX_ALARM_LED:
		case AX_SHUTDOWN:
		case AX_TORQUE_ENABLE:
		case AX_LED:
		case AX_CW_COMPLIANCE_MARGIN:
		case AX_CCW_COMPLIANCE_MARGIN:
		case AX_CW_COMPLIANCE_SLOPE:
		case AX_CCW_COMPLIANCE_SLOPE:
		case AX_PRESENT_VOLTAGE:
		case AX_PRESENT_TEMPERATURE:
		case AX_REGISTERED:
		case AX_MOVING:
		case AX_LOCK:
			params[1] = 1;
			break;

		case AX_MODEL_NUMBER:
		case AX_CW_ANGLE_LIMIT:
		case AX_CCW_ANGLE_LIMIT:
		case AX_MAX_TORQUE:
		case AX_GOAL_POSITION:
		case AX_MOVING_SPEED:
		case AX_TORQUE_LIMIT:
		case AX_PRESENT_POSITION:
		case AX_PRESENT_SPEED:
		case AX_PRESENT_LOAD:
		case AX_PUNCH:
			params[1] = 2;
				break;
	}

	params[0] = (uint8_t)r;
	length = 4; //Instruction, Param1, Param2 and Checksum.
	type = AX_READ;
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
	uint8_t params_length = 0; //Initialized to remove -Wmaybe-uninitialized.
	volatile uint8_t crc;

	params[0] = (uint8_t)r;

	switch (r)
	{
		case AX_FIRMWARE_VERSION:
		case AX_ID:
		case AX_BAUD_RATE:
		case AX_RETURN_DELAY_TIME:
		case AX_TEMPERATURE_LIMIT:
		case AX_MIN_VOLTAGE_LIMIT:
		case AX_MAX_VOLTAGE_LIMIT:
		case AX_STATUS_RETURN_LEVEL:
		case AX_ALARM_LED:
		case AX_SHUTDOWN:
		case AX_TORQUE_ENABLE:
		case AX_LED:
		case AX_CW_COMPLIANCE_MARGIN:
		case AX_CCW_COMPLIANCE_MARGIN:
		case AX_CW_COMPLIANCE_SLOPE:
		case AX_CCW_COMPLIANCE_SLOPE:
		case AX_PRESENT_VOLTAGE:
		case AX_PRESENT_TEMPERATURE:
		case AX_REGISTERED:
		case AX_MOVING:
		case AX_LOCK:
			params[1] = (uint8_t)d;
			params_length = 2;
			break;

		case AX_MODEL_NUMBER:
		case AX_CW_ANGLE_LIMIT:
		case AX_CCW_ANGLE_LIMIT:
		case AX_MAX_TORQUE:
		case AX_GOAL_POSITION:
		case AX_MOVING_SPEED:
		case AX_TORQUE_LIMIT:
		case AX_PRESENT_POSITION:
		case AX_PRESENT_SPEED:
		case AX_PRESENT_LOAD:
		case AX_PUNCH:
			params[1] = (uint8_t)d;
			params[2] = (uint8_t)(d >> 8);
			params_length = 3;
			break;
	}

	length = 2 + params_length;
	type = AX_WRITE;
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
