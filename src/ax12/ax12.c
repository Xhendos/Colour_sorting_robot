#include <stdint.h>
#include "ax12.h"
#include "../uart/uart.h"
#include "../octo/octo.h"

uint8_t ax_ping(uint8_t id) {
	uint8_t crc = ax_crc(id, 2, AX_PING, NULL);

	_GPIOB_BSRR |= 1;

	uart_send_byte(0xFF);
	uart_send_byte(0xFF);
	uart_send_byte(id);
	uart_send_byte(0x02);
	uart_send_byte(0x01);
	uart_send_byte(crc);

	_GPIOB_BSRR |= (1 << 16);

	//volatile uint8_t result1 = uart_receive_byte();
	//volatile uint8_t result2 = uart_receive_byte();
	//volatile uint8_t result3 = uart_receive_byte();
	//volatile uint8_t result4 = uart_receive_byte();	
	//volatile uint8_t result5 = uart_receive_byte();
	//volatile uint8_t result6 = uart_receive_byte();

	return crc;
}

uint8_t ax_crc(uint8_t id, uint8_t length, uint8_t instruction, uint8_t parameters[]) {
	uint8_t crc = id + length + instruction;
	for (int i = 0; i < length - 2; ++i) {
		crc += parameters[i];
	}
	return ~crc;
}
