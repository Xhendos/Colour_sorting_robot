#include "uart.h"
#include "stm32f103xb.h"

/* TODO: Fix the baud rate register. */
void uart_init()
{
	_USART_CR1 |= 0x200C;		/* Enable the transmitter, receiver and the USART */
	
	/*
     * Baudrate = Fck / (16 * USARTDIV)
	 * where USARTDIV = DIV_Mantissa + (DIV_Fraction / 16)
	 * 
	 * We want USARTDIV to be 1 which means we divide the Fck (8 Mhz) by 16
	 * This means we have a baud rate of 500 000 bits per second
	 */
	
	_USART_BRR = 0x10;		/* TODO: validate the baud rate register. */

	_USART_CR1 |= 0x60;		/* Generate an interrupt when we received a byte
							 * or when we succesfully transmitted a byte */
	NVIC_SetPriority(37, 0x02);
	NVIC_EnableIRQ(37);
}

uint8_t uart_send_byte(uint8_t byte)
{
	while(!(_USART_SR & 0x80));	/* Wait untill the buffer is available */

	_USART_DR = byte;
	while(!(_USART_SR & 0x40));	/* Wait untill the transmission is done */

	uint32_t dummy = _USART_SR & 0x40;
	//Software sequence to clear Transmission complete (TC).
	dummy = _USART_SR;
	_USART_DR = 0x55; //Garbage
	dummy = _USART_SR;

	return 0;
}

uint8_t uart_receive_byte()
{
	while(!(_USART_SR & 0x20));	/* Wait untill there is something to read */

	return (_USART_DR & 0xFF);	/* Return the received byte */
}
