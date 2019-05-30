#ifndef _AX12_H
#define _AX12_H

#include <stdint.h>

#define _GPIOB_BSRR		(*((volatile unsigned long *) 0x40010C10))

typedef enum
{
	PING = 1,
	READ = 2,
	WRITE = 3,
	REG_WRITE = 4,
	ACTION = 5,
	FACTORY_RESET = 6,
	REBOOT = 8,
	SYNC_WRITE = 131,
	BULK_READ = 146,
} ax_instruction_type_t;

typedef enum
{
	ID = 3,
	GOAL_POSITION = 30,
	MOVING_SPEED = 32,
	PRESENT_POSITION = 36,
	PRESENT_SPEED = 38,
	PRESENT_TEMPERATURE = 43,
} ax_register_t;

uint8_t ax_crc(uint8_t id, uint8_t length, ax_instruction_type_t type, uint8_t params[], uint8_t params_length);
uint16_t ax_read(uint8_t id, ax_register_t r);

#endif
