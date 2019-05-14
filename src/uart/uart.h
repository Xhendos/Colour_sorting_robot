#ifndef _UART_H
#define _UART_H

#include <stdio.h>

#define	_USART_SR		(*((volatile unsigned long *) 0x40013800))	/* Status register */
#define _USART_DR		(*((volatile unsigned long *) 0x40013804))	/* Data register */
#define _USART_BRR		(*((volatile unsigned long *) 0x40013808))	/* Baud rate register */
#define _USART_CR1		(*((volatile unsigned long *) 0x4001380C))	/* Control register 1 */
#define _USART_CR2		(*((volatile unsigned long *) 0x40013810))	/* Control register 2 */
#define _USART_CR3		(*((volatile unsigned long *) 0x40013814))	/* Control register 3 */
#define _USART_GTPR		(*((volatile unsigned long *) 0x40013818))	/* Guard time and prescaler register */



#endif	/* _UART_H */
