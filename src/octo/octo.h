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

typedef struct {
	uint8_t flag;
	uint8_t arm;
	uint16_t r1;
	uint16_t r2;
	uint8_t state;
	char *from;
	char *to;
} instruction_t;

extern uint16_t currentPositions[48];
extern uint16_t goalPositions[48];
extern QueueHandle_t xQueue;

extern void led_task();
extern void arm_controller_task();
extern void uart_controller_task();
extern void test_task();

extern void rotate(uint8_t arm, uint16_t aDegrees);
extern void stretch(uint8_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees);
extern void wrist(uint8_t arm, uint16_t degrees);
extern void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees);
extern void setSpeed(uint8_t motor, uint16_t rpm);
extern void setGoalPosition(uint8_t motor, uint16_t degrees);

#endif

