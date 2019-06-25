#ifndef _UART_H
#define _UART_H

#include <stdio.h>
#include <stdint.h>

#define	_USART_SR		(*((volatile unsigned long *) 0x40013800))	/* Status register */
#define _USART_DR		(*((volatile unsigned long *) 0x40013804))	/* Data register */
#define _USART_BRR		(*((volatile unsigned long *) 0x40013808))	/* Baud rate register */
#define _USART_CR1		(*((volatile unsigned long *) 0x4001380C))	/* Control register 1 */
#define _USART_CR2		(*((volatile unsigned long *) 0x40013810))	/* Control register 2 */
#define _USART_CR3		(*((volatile unsigned long *) 0x40013814))	/* Control register 3 */
#define _USART_GTPR		(*((volatile unsigned long *) 0x40013818))	/* Guard time and prescaler register */

void uart_init();

/*
 * Send a byte through the Tx pin.
 */
uint8_t uart_send_byte(uint8_t byte);

/*
 * Receive a byte through the Rx pin.
 */
uint8_t uart_receive_byte();

#endif	/* _UART_H */
