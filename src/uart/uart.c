#include "uart.h"

/* TODO: Fix the baud rate register because an 8 Mhz crystal is not quick enough */
void uart_init()
{
	_USART_CR1 |= 0x200C;		/* Enable the transmitter, receiver and the USART */
	_USART_BRR |= 0x01;			/* Divide the clock by 1 */
}

uint8_t uart_send_byte(uint8_t byte)
{
	while(!(_USART_SR & 0x80));	/* Wait untill the buffer is available */

	_USART_DR |= byte;
	while(!(_USART_SR & 0x40));	/* Wait untill the transmission is done */

	return 0;
}
