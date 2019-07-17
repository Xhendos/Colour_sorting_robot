#include "ax.h"

unsigned char ucByteSize(eRegister eRegister)
{
    switch (eRegister)
    {
        case eFirmwareVersion:
        case eId:
        case eBaudRate:
        case eReturnDelayTime:
        case eTemperatureLimit:
        case eMinVoltageLimit:
        case eMaxVoltageLimit:
        case eStatusReturnLevel:
        case eAlarmLed:
        case eShutdown:
        case eTorqueEnable:
        case eLed:
        case eCwComplianceMargin:
        case eCcwComplianceMargin:
        case eCwComplianceSlope:
        case eCcwComplianceSlope:
        case ePresentVoltage:
        case ePresentTemperature:
        case eRegistered:
        case eMoving:
        case eLock:
            return 1;
        case eModelNumber:
        case eCwAngleLimit:
        case eCcwAngleLimit:
        case eMaxTorque:
        case eGoalPosition:
        case eMovingSpeed:
        case eTorqueLimit:
        case ePresentPosition:
        case ePresentSpeed:
        case ePresentLoad:
        case ePunch:
            return 2;
        default:
            return 0;
    }
}

