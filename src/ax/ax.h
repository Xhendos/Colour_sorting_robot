#ifndef _AX_H
#define _AX_H

#define axDEGREES_UNIT        (0.29)
#define axRPM_UNIT            (0.111)
#define axDEGREES_TO_UNITS(x) ((unsigned short int)(x / axDEGREES_UNIT))
#define axRPM_TO_UNITS(x)     ((unsigned short int)(x / axRPM_UNIT))

typedef enum {
	ePing = 1,
	eRead = 2,
	eWrite = 3,
	eRegWrite = 4,
	eAction = 5,
	eFactoryReset = 6,
	eReboot = 8,
	eSyncWrite = 131,
	eBulkRead = 146,
} eInstructionType;

typedef enum {
	eModelNumber = 0,
	eFirmwareVersion = 2,
	eId = 3,
	eBaudRate = 4,
	eReturnDelayTime = 5,
	eCwAngleLimit = 6,
	eCcwAngleLimit = 8,
	eTemperatureLimit = 11,
	eMinVoltageLimit = 12,
	eMaxVoltageLimit = 13,
	eMaxTorque = 14,
	eStatusReturnLevel = 16,
	eAlarmLed = 17,
	eShutdown = 18,
	eTorqueEnable = 24,
	eLed = 25,
	eCwComplianceMargin = 26,
	eCcwComplianceMargin = 27,
	eCwComplianceSlope = 28,
	eCcwComplianceSlope = 29,
	eGoalPosition = 30,
	eMovingSpeed = 32,
	eTorqueLimit = 34,
	ePresentPosition = 36,
	ePresentSpeed = 38,
	ePresentLoad = 40,
	ePresentVoltage = 42,
	ePresentTemperature = 43,
	eRegistered = 44,
	eMoving = 46,
	eLock = 47,
	ePunch = 48,
} eRegister;

#endif /* _AX_H */

