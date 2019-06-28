#ifndef _AX12_H
#define _AX12_H

#include <stdint.h>

#define DEGREES_UNIT        (0.29)
#define RPM_UNIT            (0.111)
#define DEGREES_TO_UNITS(x) ((uint16_t)(x / DEGREES_UNIT))
#define RPM_TO_UNITS(x)     ((uint16_t)(x / RPM_UNIT))
#define RPM                 (15)
#define LOW(x)              ((x & 0xFF))
#define HIGH(x)             (((x >> 8) & 0xFF))

typedef enum
{
	AX_PING = 1,
	AX_READ = 2,
	AX_WRITE = 3,
	//AX_REG_WRITE = 4,
	//AX_ACTION = 5,
	//AX_FACTORY_RESET = 6,
	//AX_REBOOT = 8,
	//AX_SYNC_WRITE = 131,
	//AX_BULK_READ = 146,
} ax_instruction_type_t;

typedef enum
{
	AX_MODEL_NUMBER = 0,
	AX_FIRMWARE_VERSION = 2,
	AX_ID = 3,
	AX_BAUD_RATE = 4,
	AX_RETURN_DELAY_TIME = 5,
	AX_CW_ANGLE_LIMIT = 6,
	AX_CCW_ANGLE_LIMIT = 8,
	AX_TEMPERATURE_LIMIT = 11,
	AX_MIN_VOLTAGE_LIMIT = 12,
	AX_MAX_VOLTAGE_LIMIT = 13,
	AX_MAX_TORQUE = 14,
	AX_STATUS_RETURN_LEVEL = 16,
	AX_ALARM_LED = 17,
	AX_SHUTDOWN = 18,
	AX_TORQUE_ENABLE = 24,
	AX_LED = 25,
	AX_CW_COMPLIANCE_MARGIN = 26,
	AX_CCW_COMPLIANCE_MARGIN = 27,
	AX_CW_COMPLIANCE_SLOPE = 28,
	AX_CCW_COMPLIANCE_SLOPE = 29,
	AX_GOAL_POSITION = 30,
	AX_MOVING_SPEED = 32,
	AX_TORQUE_LIMIT = 34,
	AX_PRESENT_POSITION = 36,
	AX_PRESENT_SPEED = 38,
	AX_PRESENT_LOAD = 40,
	AX_PRESENT_VOLTAGE = 42,
	AX_PRESENT_TEMPERATURE = 43,
	AX_REGISTERED = 44,
	AX_MOVING = 46,
	AX_LOCK = 47,
	AX_PUNCH = 48,
} ax_register_t;

typedef struct {
	uint8_t flag;
	uint8_t arm;
	uint16_t r1;
	uint16_t r2;
	uint8_t state;
    uint8_t from;
    uint8_t to;
} instruction_t;

typedef struct {
	uint8_t id;
	ax_instruction_type_t type;
	uint8_t params_length;
	uint8_t params[3];
} ax_packet_t;

typedef union {
	struct strct {
		uint8_t l;
		uint8_t h;
	} lh;
	uint16_t x;
	uint8_t xa[2];
} position_t;

uint8_t ax_crc(uint8_t id, uint8_t length, ax_instruction_type_t type, uint8_t params[], uint8_t params_length);
ax_packet_t generateWritePacket(uint8_t id, ax_register_t r, uint16_t data);
uint16_t ax_read(uint8_t id, ax_register_t r);
uint8_t ax_write(uint8_t id, ax_register_t r, uint16_t d);
void ax_debug();

#endif
