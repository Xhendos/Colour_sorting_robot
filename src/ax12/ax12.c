#include "ax12.h"
#include "uart.h"

uint8_t ax_crc(uint8_t id, uint8_t length, ax_instruction_type_t type, uint8_t params[], uint8_t params_length)
{
	uint8_t crc = id + params_length + 2 + (uint8_t)type;
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
	uint8_t param1;
	uint8_t param2;
	uint8_t params[2];
	uint8_t crc;

	switch (r)
	{
		case PRESENT_POSITION:
			param1 = (uint8_t)r;
			param2 = 2;
				break;
	}

	length = 2 + params[1];
	type = READ;
	params[0] = param1;
	params[1] = param2;
	crc = ax_crc(id, length, type, params, params[1]);

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
		return (buffer[5] << 8) | buffer[6];
	}
}

