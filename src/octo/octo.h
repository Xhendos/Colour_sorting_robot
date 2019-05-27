#ifndef _OCTO_H
#define _OCTO_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#define _RCC_CR			(*((volatile unsigned long *) 0x40021000))		/* Clock control register */
#define _RCC_CFGR		(*((volatile unsigned long *) 0x40021004))		/* Clock configuration register */

#define	_RCC_APB2ENR	(*((volatile unsigned long *) 0x40021018))		/* Peripheral clock enable register */
#define _RCC_APB1ENR	(*((volatile unsigned long *) 0x4002101C))		/* Peripheral clock enable register */
#define _RCC_APB1RSTR	(*((volatile unsigned long *) 0x40021010))		/* Peripheral reset register */
#define _RCC_APB2RSTR	(*((volatile unsigned long *) 0x4002100C))		/* APB2 peripheral reset register */

/* GPIOA */
#define _GPIOA_CRH		(*((volatile unsigned long *) 0x40010804))		/* Port configuration register high */
#define	_GPIOA_BSRR		(*((volatile unsigned long *) 0x40010810))		/* set/reset register */

/* GPIOB */
#define	_GPIOB_CRL		(*((volatile unsigned long *) 0x40010C00))		/* Port configuration register low */
#define _GPIOB_BSRR		(*((volatile unsigned long *) 0x40010C10))		/* set/reset register */

#define GPIOC_CLK       (*((volatile uint32_t *) 0x40021018))
#define GPIOC_HIGH      (*((volatile uint32_t *) 0x40011004))
#define GPIOC_SR		(*((volatile uint32_t *) 0x40011010))

typedef enum {
	PING = 1,
	READ = 2,
	WRITE = 3,
	REG_WRITE = 4,
	ACTION = 5,
	FACTORY_RESET = 6,
	REBOOT = 8,
	SYNC_WRITE = 131,
	BULK_READ = 156,
} ax_instruction_type_t;

typedef struct {
	uint8_t flag;
	uint8_t arm;
	uint16_t r1;
	uint16_t r2;
	uint8_t state;
	char *from;
	char *to;
} instruction_t;

typedef struct {
	uint8_t id;
	ax_instruction_type_t type;
	uint8_t params_length;
	uint8_t params[];
} ax_packet_t;

typedef union {
	struct strct {
		uint8_t l;
		uint8_t h;
	} lh;
	uint16_t x;
	uint8_t xa[2];
} position_t;

extern position_t presentPositions[48];
extern position_t goalPositions[48];
extern uint8_t pings[48];

extern QueueHandle_t xQueue;
extern QueueHandle_t packetQueue;
extern QueueHandle_t uartSignalQueue;
extern QueueHandle_t uartResultQueue;

extern void led_task();
extern void arm_controller_task();
extern void uart_controller_task();
extern void test_task();
extern void uart_test_task();

extern uint8_t ax_crc(ax_packet_t packet);

extern uint8_t idToIndex(uint8_t id);
extern uint8_t indexToId(uint8_t index);

extern void USART1_IRQ_handler(void);

extern void rotate(uint8_t arm, uint16_t aDegrees);
extern void stretch(uint8_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees);
extern void wrist(uint8_t arm, uint16_t degrees);
extern void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees);
extern void setSpeed(uint8_t motor, uint16_t rpm);
extern void setGoalPosition(uint8_t motor, uint16_t degrees);

#endif

