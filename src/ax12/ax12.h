#ifndef _AX12_H
#define _AX12_H

#include <stdint.h>

#define AX_PING ((uint8_t)0x01)
#define AX_READ ((uint8_t)0x02)
#define AX_WRITE ((uint8_t)0x03)

extern uint8_t ax_ping(uint8_t id);
extern void ax_write(uint8_t id, uint8_t address, uint8_t data);
extern uint8_t ax_read(uint8_t id, uint8_t address);
extern uint8_t ax_crc(uint8_t id, uint8_t length, uint8_t instruction, uint8_t parameters[]);

#endif

