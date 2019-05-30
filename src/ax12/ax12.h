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
	MODEL_NUMBER = 0,
	FIRMWARE_VERSION = 2,
	ID = 3,
	BAUD_RATE = 4,
	RETURN_DELAY_TIME = 5,
	CW_ANGLE_LIMIT = 6,
	CCW_ANGLE_LIMIT = 8,
	TEMPERATURE_LIMIT = 11,
	MIN_VOLTAGE_LIMIT = 12,
	MAX_VOLTAGE_LIMIT = 13,
	MAX_TORQUE = 14,
	STATUS_RETURN_LEVEL = 16,
	ALARM_LED = 17,
	SHUTDOWN = 18,
	TORQUE_ENABLE = 24,
	LED = 25,
	CW_COMPLIANCE_MARGIN = 26,
	CCW_COMPLIANCE_MARGIN = 27,
	CW_COMPLIANCE_SLOPE = 28,
	CCW_COMPLIANCE_SLOPE = 29,
	GOAL_POSITION = 30,
	MOVING_SPEED = 32,
	TORQUE_LIMIT = 34,
	PRESENT_POSITION = 36,
	PRESENT_SPEED = 38,
	PRESENT_LOAD = 40,
	PRESENT_VOLTAGE = 42,
	PRESENT_TEMPERATURE = 43,
	REGISTERED = 44,
	MOVING = 46,
	LOCK = 47,
	PUNCH = 48,
} ax_register_t;

uint8_t ax_crc(uint8_t id, uint8_t length, ax_instruction_type_t type, uint8_t params[], uint8_t params_length);
uint16_t ax_read(uint8_t id, ax_register_t r);
uint8_t ax_write(uint8_t id, ax_register_t r, uint16_t d);

#endif
