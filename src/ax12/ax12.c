#include "ax12.h"
#include "uart.h"
#include "octo.h"

uint8_t ax_crc(uint8_t id, uint8_t length, ax_instruction_type_t type, uint8_t params[], uint8_t params_length)
{
	uint8_t crc = id + length + (uint8_t)type;
	for (int i = 0; i < params_length; ++i)
	{
		crc += params[i];
	}
	return ~crc;
}

ax_packet_t generateWritePacket(uint8_t id, ax_register_t r, uint16_t data)
{
    ax_packet_t packet;
    packet.id = id;
    packet.type = AX_WRITE;
    packet.params[0] = r;
	switch (r)
	{
		case AX_FIRMWARE_VERSION:
		case AX_ID:
		case AX_BAUD_RATE:
		case AX_RETURN_DELAY_TIME:
		case AX_TEMPERATURE_LIMIT:
		case AX_MIN_VOLTAGE_LIMIT:
		case AX_MAX_VOLTAGE_LIMIT:
		case AX_STATUS_RETURN_LEVEL:
		case AX_ALARM_LED:
		case AX_SHUTDOWN:
		case AX_TORQUE_ENABLE:
		case AX_LED:
		case AX_CW_COMPLIANCE_MARGIN:
		case AX_CCW_COMPLIANCE_MARGIN:
		case AX_CW_COMPLIANCE_SLOPE:
		case AX_CCW_COMPLIANCE_SLOPE:
		case AX_PRESENT_VOLTAGE:
		case AX_PRESENT_TEMPERATURE:
		case AX_REGISTERED:
		case AX_MOVING:
		case AX_LOCK:
            packet.params[1] = LOW(data);
			packet.params_length = 2;
			break;

		case AX_MODEL_NUMBER:
		case AX_CW_ANGLE_LIMIT:
		case AX_CCW_ANGLE_LIMIT:
		case AX_MAX_TORQUE:
		case AX_GOAL_POSITION:
		case AX_MOVING_SPEED:
		case AX_TORQUE_LIMIT:
		case AX_PRESENT_POSITION:
		case AX_PRESENT_SPEED:
		case AX_PRESENT_LOAD:
		case AX_PUNCH:
            packet.params[1] = LOW(data);
            packet.params[2] = HIGH(data);
			packet.params_length = 3;
				break;
	}
    return packet;
}

uint16_t ax_read(uint8_t id, ax_register_t r)
{
	uint8_t buffer[8];
	uint8_t length;
	ax_instruction_type_t type;
	uint8_t params[2];
	volatile uint8_t crc;

	switch (r)
	{
		case AX_FIRMWARE_VERSION:
		case AX_ID:
		case AX_BAUD_RATE:
		case AX_RETURN_DELAY_TIME:
		case AX_TEMPERATURE_LIMIT:
		case AX_MIN_VOLTAGE_LIMIT:
		case AX_MAX_VOLTAGE_LIMIT:
		case AX_STATUS_RETURN_LEVEL:
		case AX_ALARM_LED:
		case AX_SHUTDOWN:
		case AX_TORQUE_ENABLE:
		case AX_LED:
		case AX_CW_COMPLIANCE_MARGIN:
		case AX_CCW_COMPLIANCE_MARGIN:
		case AX_CW_COMPLIANCE_SLOPE:
		case AX_CCW_COMPLIANCE_SLOPE:
		case AX_PRESENT_VOLTAGE:
		case AX_PRESENT_TEMPERATURE:
		case AX_REGISTERED:
		case AX_MOVING:
		case AX_LOCK:
			params[1] = 1;
			break;

		case AX_MODEL_NUMBER:
		case AX_CW_ANGLE_LIMIT:
		case AX_CCW_ANGLE_LIMIT:
		case AX_MAX_TORQUE:
		case AX_GOAL_POSITION:
		case AX_MOVING_SPEED:
		case AX_TORQUE_LIMIT:
		case AX_PRESENT_POSITION:
		case AX_PRESENT_SPEED:
		case AX_PRESENT_LOAD:
		case AX_PUNCH:
			params[1] = 2;
				break;
	}

	params[0] = (uint8_t)r;
	length = 4; //Instruction, Param1, Param2 and Checksum.
	type = AX_READ;
	crc = ax_crc(id, length, type, params, 2);

	_GPIOB_BSRR |= 1;

	uart_send_byte(0xff);
	uart_send_byte(0xff);
	uart_send_byte(id);
	uart_send_byte(length);
	uart_send_byte((uint8_t)type);
	uart_send_byte(params[0]);
	uart_send_byte(params[1]);
	uart_send_byte(crc);

	_GPIOB_BSRR |= (1 << 16);

	uint8_t n = 6 + params[1];
	for (int i = 0; i < n; ++i)
	{
		buffer[i] = uart_receive_byte();
	}

	if (params[1] == 1)
	{
		return buffer[5];
	}
	else
	{
		return (uint16_t)(buffer[5] | buffer[6] << 8);
	}
}

uint8_t ax_write(uint8_t id, ax_register_t r, uint16_t d)
{
	uint8_t buffer[6];
	uint8_t length;
	ax_instruction_type_t type;
	uint8_t params[3];
	uint8_t params_length = 0; //Initialized to remove -Wmaybe-uninitialized.
	volatile uint8_t crc;

	params[0] = (uint8_t)r;

	switch (r)
	{
		case AX_FIRMWARE_VERSION:
		case AX_ID:
		case AX_BAUD_RATE:
		case AX_RETURN_DELAY_TIME:
		case AX_TEMPERATURE_LIMIT:
		case AX_MIN_VOLTAGE_LIMIT:
		case AX_MAX_VOLTAGE_LIMIT:
		case AX_STATUS_RETURN_LEVEL:
		case AX_ALARM_LED:
		case AX_SHUTDOWN:
		case AX_TORQUE_ENABLE:
		case AX_LED:
		case AX_CW_COMPLIANCE_MARGIN:
		case AX_CCW_COMPLIANCE_MARGIN:
		case AX_CW_COMPLIANCE_SLOPE:
		case AX_CCW_COMPLIANCE_SLOPE:
		case AX_PRESENT_VOLTAGE:
		case AX_PRESENT_TEMPERATURE:
		case AX_REGISTERED:
		case AX_MOVING:
		case AX_LOCK:
			params[1] = LOW(d);
			params_length = 2;
			break;

		case AX_MODEL_NUMBER:
		case AX_CW_ANGLE_LIMIT:
		case AX_CCW_ANGLE_LIMIT:
		case AX_MAX_TORQUE:
		case AX_GOAL_POSITION:
		case AX_MOVING_SPEED:
		case AX_TORQUE_LIMIT:
		case AX_PRESENT_POSITION:
		case AX_PRESENT_SPEED:
		case AX_PRESENT_LOAD:
		case AX_PUNCH:
			params[1] = LOW(d);
			params[2] = HIGH(d);
			params_length = 3;
			break;
	}

	length = 2 + params_length;
	type = AX_WRITE;
	crc = ax_crc(id, length, type, params, params_length);

	_GPIOB_BSRR |= 1;

	uart_send_byte(0xff);
	uart_send_byte(0xff);
	uart_send_byte(id);
	uart_send_byte(length);
	uart_send_byte((uint8_t)type);
	for (int i = 0; i < params_length; ++i)
	{
		uart_send_byte(params[i]);
	}
	uart_send_byte(crc);

	_GPIOB_BSRR |= (1 << 16);

	uint8_t n = 6;

	for (int i = 0; i < n; ++i)
	{
		buffer[i] = uart_receive_byte();
	}

	return buffer[5];
}

void ax_debug()
{
    volatile uint16_t modelNumber;
	volatile uint16_t firmwareVersion;
	volatile uint16_t id;
	volatile uint16_t baudRate;
	volatile uint16_t returnDelayTime;
	volatile uint16_t cwAngleLimit;
	volatile uint16_t ccwAngleLimit;
	volatile uint16_t temperatureLimit;
	volatile uint16_t minVoltageLimit;
	volatile uint16_t maxVoltageLimit;
	volatile uint16_t maxTorque;
	volatile uint16_t statusReturnLevel;
	volatile uint16_t alarmLed;
	volatile uint16_t shutdown;
	volatile uint16_t torqueEnable;
	volatile uint16_t led;
	volatile uint16_t cwComplianceMargin;
	volatile uint16_t ccwComplianceMarge;
	volatile uint16_t cwComplianceSlope;
	volatile uint16_t ccwComplianceSlope;
	volatile uint16_t goalPosition;
	volatile uint16_t movingSpeed;
	volatile uint16_t torqueLimit;
	volatile uint16_t presentPosition;
	volatile uint16_t presentSpeed;
	volatile uint16_t presentLoad;
	volatile uint16_t presentVoltage;
	volatile uint16_t presentTemperature;
	volatile uint16_t registered;
	volatile uint16_t moving;
	volatile uint16_t lock;
	volatile uint16_t punch;
    volatile uint8_t armId = 254;
	uint8_t n = 1;

	//ax_write(armId, AX_TORQUE_ENABLE, 0);
	//ax_write(armId, AX_MOVING_SPEED, 50);
	//ax_write(armId, AX_GOAL_POSITION, 1023);
	//ax_write(armId, AX_TORQUE_ENABLE, 1);

	while(1)
	{
		modelNumber = ax_read(armId, AX_MODEL_NUMBER);
		firmwareVersion = ax_read(armId, AX_FIRMWARE_VERSION);
		id = ax_read(armId, AX_ID);
		baudRate = ax_read(armId, AX_BAUD_RATE);
		returnDelayTime = ax_read(armId, AX_RETURN_DELAY_TIME);
		cwAngleLimit = ax_read(armId, AX_CW_ANGLE_LIMIT);
		ccwAngleLimit = ax_read(armId, AX_CCW_ANGLE_LIMIT);
		temperatureLimit = ax_read(armId, AX_TEMPERATURE_LIMIT);
		minVoltageLimit = ax_read(armId, AX_MIN_VOLTAGE_LIMIT);
		maxVoltageLimit = ax_read(armId, AX_MAX_VOLTAGE_LIMIT);
		maxTorque = ax_read(armId, AX_MAX_TORQUE);
		statusReturnLevel = ax_read(armId, AX_STATUS_RETURN_LEVEL);
		alarmLed = ax_read(armId, AX_ALARM_LED);
		shutdown = ax_read(armId, AX_SHUTDOWN);
		torqueEnable = ax_read(armId, AX_TORQUE_ENABLE);
		led = ax_read(armId, AX_LED);
		cwComplianceMargin = ax_read(armId, AX_CW_COMPLIANCE_MARGIN);
		ccwComplianceMarge = ax_read(armId, AX_CCW_COMPLIANCE_MARGIN);
		cwComplianceSlope = ax_read(armId, AX_CW_COMPLIANCE_SLOPE);
		ccwComplianceSlope = ax_read(armId, AX_CCW_COMPLIANCE_SLOPE);
		goalPosition = ax_read(armId, AX_GOAL_POSITION);
		movingSpeed = ax_read(armId, AX_MOVING_SPEED);
		torqueLimit = ax_read(armId, AX_TORQUE_LIMIT);
		presentPosition = ax_read(armId, AX_PRESENT_POSITION);
		presentSpeed = ax_read(armId, AX_PRESENT_SPEED);
		presentLoad = ax_read(armId, AX_PRESENT_LOAD);
		presentVoltage = ax_read(armId, AX_PRESENT_VOLTAGE);
		presentTemperature = ax_read(armId, AX_PRESENT_TEMPERATURE);
		registered = ax_read(armId, AX_REGISTERED);
		moving = ax_read(armId, AX_MOVING);
		lock = ax_read(armId, AX_LOCK);
		punch = ax_read(armId, AX_PUNCH);

		//if (!moving && abs(presentPosition - goalPosition) <= 2)
		//{
		//	if (n)
		//	{
		//		ax_write(armId, AX_GOAL_POSITION, 0);
		//		n = 0;
		//	}
		//	else
		//	{
		//		ax_write(armId, AX_GOAL_POSITION, 1023);
		//		n = 1;
		//	}
		//}
	}
}
