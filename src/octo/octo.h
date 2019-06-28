#ifndef _OCTO_H
#define _OCTO_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "ax12.h"
#include "rgb.h" /* Included for rgb struct */

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

/* I2C */
#define _I2C1_CR2		(*((volatile unsigned long *) 0x40005404))		/* Control register 2 */
#define _I2C1_SR		(*((volatile unsigned long *) 0x40005414))		/* Status register 1 */

#define GPIOC_CLK       (*((volatile uint32_t *) 0x40021018))
#define GPIOC_HIGH      (*((volatile uint32_t *) 0x40011004))
#define GPIOC_SR		(*((volatile uint32_t *) 0x40011010))

struct i2c_message
{
    uint8_t address;
    uint8_t byte;
    uint8_t write_finished;
    uint8_t read;
    uint8_t read_bytes[2];
};

typedef enum {
	ARM_0 = 0,
	ARM_1,
	ARM_2,
	ARM_3,
	ARM_4,
	ARM_5,
	ARM_6,
	ARM_7,
} arm_t;

typedef enum {
	ARM_0_BASE = 0,
	ARM_1_BASE = 10,
	ARM_2_BASE = 20,
	ARM_3_BASE = 30,
	ARM_4_BASE = 40,
	ARM_5_BASE = 50,
	ARM_6_BASE = 60,
	ARM_7_BASE = 70,
} arm_base_t;

typedef enum {
	MOTOR_A = 1,
	MOTOR_B,
	MOTOR_C,
	MOTOR_D,
	MOTOR_E,
	MOTOR_F,
} motor_t;

typedef enum {
	MOTOR_A_INDEX = 0,
	MOTOR_B_INDEX,
	MOTOR_C_INDEX,
	MOTOR_D_INDEX,
	MOTOR_E_INDEX,
	MOTOR_F_INDEX,
} motor_index_t;

typedef enum {
	T0 = 0,
	T1,
	T2,
	T3,
	T4,
	T5,
	T6,
	T7,
	F0,
	F1,
	F2,
	F3,
} placeholders_t;

extern uint8_t pings[48];
extern uint8_t movings[48];
extern struct RGB rgbsensors[12];
extern uint8_t dummy;
extern uint8_t inProgress;

extern QueueHandle_t uartPacketQueue;
extern QueueHandle_t usartPacketQueue;
extern QueueHandle_t uartSignalQueue;
extern QueueHandle_t armInstructionQueue;

extern TaskHandle_t armHandle;
extern TaskHandle_t movingHandle;

extern void manager_task();
extern void init_task();
extern void i2c_task();
extern void user_task();
extern void uart_task();
extern void arm_task();
extern void ping_task();
extern void goalPosition_task();
extern void presentPosition_task();
extern void moving_task();
extern void position_task();
extern void rgb_task();
extern void prepareArms_task();
extern void algo_task();

extern uint8_t idToIndex(uint8_t id);
extern uint8_t indexToId(uint8_t index);

extern void USART1_IRQ_handler(void);

extern void rotate(uint8_t arm, uint16_t aDegrees);
extern void stretch(uint8_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees);
extern void wrist(uint8_t arm, uint16_t degrees);
extern void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees);
extern void setSpeed(uint8_t motor, uint16_t rpm);
extern void setGoalPosition(uint8_t motor, uint16_t degrees);
extern void setGoalPositions(arm_t arm, uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e, uint16_t f);

#endif

