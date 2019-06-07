#ifndef _OCTO_H
#define _OCTO_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "ax12.h"

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

extern position_t presentPositions[48];
extern position_t goalPositions[48];
extern uint8_t pings[48];
extern uint8_t dummy;
extern uint8_t inProgress;

extern QueueHandle_t uartPacketQueue;
extern QueueHandle_t usartPacketQueue;

extern void manager_task();
extern void init_task();
extern void i2c_task();
extern void user_task();
extern void arm_task();
extern void ping_task();
extern void position_task();
extern void rgb_task();

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

