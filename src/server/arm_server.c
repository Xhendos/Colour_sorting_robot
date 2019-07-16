#include "arm_server.h"
#include "ax.h"
#include "octo.h"
#include "uart.h"

typedef struct xMOTOR {
    const unsigned char ucId;
    unsigned short int usPresentPositionInUnits;
    unsigned short int usGoalPositionInUnits;
    unsigned short int usDifferenceInUnits;
    unsigned short int usMovingSpeedInUnits;
    unsigned char ucParticipating;
} Motor_t;

typedef struct xARM {
    Motor_t * pxMotorA;
    Motor_t * pxMotorB;
    Motor_t * pxMotorC;
    Motor_t * pxMotorD;
    Motor_t * pxMotorE;
    Motor_t * pxMotorF;
} Arm_t;

static void prvClearMotorParticipations( void );
static void prvSetGoalPositionAndParticipation( Motor_t * pxMotor, unsigned short int usDegrees );
static unsigned short int prvRead( unsigned char ucId, eRegister eRegister );
static void prvWrite( unsigned char ucId, eRegister eRegister, unsigned short int usValue );
static void prvRegWrite( unsigned char ucId, eRegister eRegister, unsigned short int usValue );
static void prvAction();

static Motor_t xMotors[8][6] = {
    { { 1}, { 2}, { 3}, { 4}, { 5}, { 6} },
    { {11}, {12}, {13}, {14}, {15}, {16} },
    { {21}, {22}, {23}, {24}, {25}, {26} },
    { {31}, {32}, {33}, {34}, {35}, {36} },
    { {41}, {42}, {43}, {44}, {45}, {46} },
    { {51}, {52}, {53}, {54}, {55}, {56} },
    { {61}, {62}, {63}, {64}, {65}, {66} },
    { {71}, {72}, {73}, {74}, {75}, {76} },
};

TaskHandle_t xArmServerTask;
QueueHandle_t xArmServerMessageQueue;
static QueueHandle_t xQueueForUartResponse;

void vTaskArmServer( void * pvParameters )
{
ArmServerMessage_t xMessage;
eArmMovement eBufferedMovement[8];
unsigned short int usBufferedFirstRotationInDegrees[8];
unsigned short int usBufferedSecondRotationInDegrees[8];

    xArmServerMessageQueue = xQueueCreate(ARM_SERVER_MESSAGE_QUEUE_SIZE, sizeof(ArmServerMessage_t));

    if ( xArmServerMessageQueue == NULL )
    {
        /* Message queue did not get created. */
    }

    xQueueForUartResponse = xQueueCreate(1, sizeof(unsigned short int));

    if ( xQueueForUartResponse == NULL )
    {
        /* Queue for uart response did not get created. */
    }

    /* Configure servo motors. This does not put any arm in a position. */
    prvWrite(axBROADCAST_ID, eStatusReturnLevel, 2);
    prvWrite(axBROADCAST_ID, eReturnDelayTime, 50);
    for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
    {
        for (UBaseType_t uxMotorIndex = octoMOTOR_A_INDEX; uxMotorIndex <= octoMOTOR_F_INDEX; uxMotorIndex += octoMOTOR_INDEX_INCREMENT)
        {
            Motor_t * pxMotor = &xMotors[uxArmIndex][uxMotorIndex];

            prvWrite(pxMotor->ucId, eTorqueEnable, 0);
            prvWrite(pxMotor->ucId, eCwAngleLimit, axDEGREES_TO_UNITS(60));
            prvWrite(pxMotor->ucId, eCcwAngleLimit, axDEGREES_TO_UNITS(240));
            prvWrite(pxMotor->ucId, eMaxTorque, 0x03FF);
            prvWrite(pxMotor->ucId, eAlarmLed, 0);
            prvWrite(pxMotor->ucId, eMovingSpeed, axRPM_TO_UNITS(octoRPM));
            prvWrite(pxMotor->ucId, eShutdown, 0);
            prvWrite(pxMotor->ucId, eTorqueEnable, 1);
        }
    }

    while (1)
    {
        if (xQueueReceive(xArmServerMessageQueue, &xMessage, portMAX_DELAY) == pdFALSE)
        {
            /* Failed retrieving item from queue. */
        }

        for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
        {
            /* Decipher arm encoding. */
            if (!((xMessage.eArms >> uxArmIndex) & 1))
            {
                continue;
            }

            eBufferedMovement[uxArmIndex] = xMessage.eMovement;

            if (xMessage.eMovement == eDisplace)
            {
                usBufferedFirstRotationInDegrees[uxArmIndex] = xMessage.usFirstRotationInDegrees;
                usBufferedSecondRotationInDegrees[uxArmIndex] = xMessage.usSecondRotationInDegrees;
            }
        }

        if (!xMessage.eExecute)
        {
            continue;
        }

        UBaseType_t uxAmountOfPositions = 0;

        switch (xMessage.eMovement)
        {
            case eDisplace:
                uxAmountOfPositions = 15;
                break;
            case eRest:
                uxAmountOfPositions = 1;
                break;
            case eNoMovement:
                uxAmountOfPositions = 0;
                break;
        }

        /* Loop through each position of a movement. */
        for (UBaseType_t uxPosition = 0; uxPosition < uxAmountOfPositions; ++uxPosition)
        {
            prvClearMotorParticipations();

            /* Loop through each arm that should perform a movement. */
            for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
            {
                if (eBufferedMovement[uxArmIndex] != xMessage.eMovement)
                {
                    continue;
                }

                /* Servo motors of an arm are tagged as participating and their goal positions are set. */
                switch (xMessage.eMovement)
                {
                    case eDisplace:
                        switch (uxPosition)
                        {
                            case 0:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_A_INDEX], axDEGREES_TO_UNITS(usBufferedFirstRotationInDegrees[uxArmIndex]));
                                break;
                            case 1:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_B_INDEX], axDEGREES_TO_UNITS(60));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(240));
                                break;
                            case 2:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_C_INDEX], axDEGREES_TO_UNITS(80));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(220));
                                break;
                            case 3:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_E_INDEX], axDEGREES_TO_UNITS(160));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_F_INDEX], axDEGREES_TO_UNITS(140));
                                break;
                            case 4:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_B_INDEX], axDEGREES_TO_UNITS(105));
                                break;
                            case 5:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_B_INDEX], axDEGREES_TO_UNITS(195));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_C_INDEX], axDEGREES_TO_UNITS(60));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(60));
                                break;
                            case 6:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_A_INDEX], axDEGREES_TO_UNITS(usBufferedSecondRotationInDegrees[uxArmIndex]));
                                break;
                            case 7:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(105));
                                break;
                            case 8:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_B_INDEX], axDEGREES_TO_UNITS(60));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(240));
                                break;
                            case 9:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_C_INDEX], axDEGREES_TO_UNITS(80));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(220));
                                break;
                            case 10:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_E_INDEX], axDEGREES_TO_UNITS(140));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_F_INDEX], axDEGREES_TO_UNITS(160));
                                break;
                            case 11:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_C_INDEX], axDEGREES_TO_UNITS(60));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(240));
                                break;
                            case 12:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_B_INDEX], axDEGREES_TO_UNITS(105));
                                break;
                            case 13:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_B_INDEX], axDEGREES_TO_UNITS(195));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(60));
                                break;
                            case 14:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_A_INDEX], axDEGREES_TO_UNITS(150));
                                break;
                        }
                        break;
                    case eRest:
                        switch (uxPosition)
                        {
                            case 0:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_B_INDEX], axDEGREES_TO_UNITS(195));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_C_INDEX], axDEGREES_TO_UNITS(60));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_D_INDEX], axDEGREES_TO_UNITS(60));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_E_INDEX], axDEGREES_TO_UNITS(140));
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_F_INDEX], axDEGREES_TO_UNITS(160));
                                break;
                            case 1:
                                prvSetGoalPositionAndParticipation(&xMotors[uxArmIndex][octoMOTOR_A_INDEX], axDEGREES_TO_UNITS(150));
                                break;
                        }
                        break;
                }
            }

            /* To make the participating servo motors reach their position simultaneously, the difference between their goal position and present position are calculated. */
            for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
            {
                if (eBufferedMovement[uxArmIndex] != xMessage.eMovement)
                {
                    continue;
                }

                for (UBaseType_t uxMotorIndex = octoMOTOR_A_INDEX; uxMotorIndex <= octoMOTOR_F_INDEX; uxMotorIndex += octoMOTOR_INDEX_INCREMENT)
                {
                    Motor_t * pxMotor = &xMotors[uxArmIndex][uxMotorIndex];

                    if (!pxMotor->ucParticipating)
                    {
                        continue;
                    }

                    pxMotor->usPresentPositionInUnits = prvRead(pxMotor->ucId, ePresentPosition);

                    /* abs(). */
                    if (pxMotor->usPresentPositionInUnits > pxMotor->usGoalPositionInUnits)
                    {
                        pxMotor->usDifferenceInUnits = pxMotor->usPresentPositionInUnits - pxMotor->usGoalPositionInUnits;
                    }
                    else if (pxMotor->usPresentPositionInUnits < pxMotor->usGoalPositionInUnits)
                    {
                        pxMotor->usDifferenceInUnits = pxMotor->usGoalPositionInUnits - pxMotor->usPresentPositionInUnits;
                    }
                    else
                    {
                        pxMotor->usDifferenceInUnits = 0;
                    }
                }
            }

            UBaseType_t uxLargestDifference = 0;

            /* Find the largest difference. */
            for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
            {
                if (eBufferedMovement[uxArmIndex] != xMessage.eMovement)
                {
                    continue;
                }

                for (UBaseType_t uxMotorIndex = octoMOTOR_A_INDEX; uxMotorIndex <= octoMOTOR_F_INDEX; uxMotorIndex += octoMOTOR_INDEX_INCREMENT)
                {
                    Motor_t * pxMotor = &xMotors[uxArmIndex][uxMotorIndex];

                    if (!pxMotor->ucParticipating)
                    {
                        continue;
                    }

                    if (pxMotor->usDifferenceInUnits > uxLargestDifference)
                    {
                        uxLargestDifference = pxMotor->usDifferenceInUnits;
                    }
                }
            }

            /* The servo motor that has to turn the most degrees turns with a speed of a specified rpm like 15. The servo motors that have to turn a smaller degrees will have their movement speed compensated to a lower value. The moving speed is set via a write instruction packet. Afterwards the goal position is set via a regwrite instruction packet. The regwrite instructions are all executed at the same time via a broadcasted action instruction. */
            for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
            {
                if (eBufferedMovement[uxArmIndex] != xMessage.eMovement)
                {
                    continue;
                }

                for (UBaseType_t uxMotorIndex = octoMOTOR_A_INDEX; uxMotorIndex <= octoMOTOR_F_INDEX; uxMotorIndex += octoMOTOR_INDEX_INCREMENT)
                {
                    Motor_t * pxMotor = &xMotors[uxArmIndex][uxMotorIndex];

                    if (!pxMotor->ucParticipating)
                    {
                        continue;
                    }

                    float fMovingSpeedRatio = (float) pxMotor->usDifferenceInUnits / uxLargestDifference;

                    pxMotor->usMovingSpeedInUnits = axRPM_TO_UNITS(octoRPM) * fMovingSpeedRatio;

                    prvWrite(pxMotor->ucId, eMovingSpeed, pxMotor->usMovingSpeedInUnits);
                    prvRegWrite(pxMotor->ucId, eGoalPosition, pxMotor->usGoalPositionInUnits);
                }
            }

            prvAction();

            /* Wait for each servo motor to be done with moving. */
            unsigned char ucAllServoMotorsDoneMoving;
            do
            {
                ucAllServoMotorsDoneMoving = 1;

                for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
                {
                    if (eBufferedMovement[uxArmIndex] != xMessage.eMovement)
                    {
                        continue;
                    }

                    for (UBaseType_t uxMotorIndex = octoMOTOR_A_INDEX; uxMotorIndex <= octoMOTOR_F_INDEX; uxMotorIndex += octoMOTOR_INDEX_INCREMENT)
                    {
                        Motor_t * pxMotor = &xMotors[uxArmIndex][uxMotorIndex];

                        if (!pxMotor->ucParticipating)
                        {
                            continue;
                        }

                        unsigned char ucIsMoving = prvRead(pxMotor->ucId, eMoving);

                        if (ucIsMoving)
                        {
                            ucAllServoMotorsDoneMoving = 0;
                        }
                    }
                }
            } while (!ucAllServoMotorsDoneMoving);
        }

        /* Clear buffered movements. */
        for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
        {
            eBufferedMovement[uxArmIndex] = 0;
        }

        xTaskNotifyGive(xMessage.xSenderOfMessage);
    }
}

static void prvClearMotorParticipations()
{
    for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
    {
        for (UBaseType_t uxMotorIndex = octoMOTOR_A_INDEX; uxMotorIndex <= octoMOTOR_F_INDEX; uxMotorIndex += octoMOTOR_INDEX_INCREMENT)
        {
            xMotors[uxArmIndex][uxMotorIndex].ucParticipating = 0;
        }
    }
}

static void prvSetGoalPositionAndParticipation( Motor_t * pxMotor, unsigned short int usDegreesInUnits )
{
    pxMotor->usGoalPositionInUnits = usDegreesInUnits;
    pxMotor->ucParticipating = 1;
}

static unsigned short int prvRead( unsigned char ucId, eRegister eRegister )
{
    unsigned short int usResponse;
    UartMessage_t xUartMessage;

    xUartMessage.xInstructionPacket.eInstructionType = eRead;
    xUartMessage.xInstructionPacket.ucId = ucId;
    xUartMessage.xInstructionPacket.eRegister = eRegister;
    xUartMessage.xInstructionPacket.usParam = ucByteSize(eRegister);
    xUartMessage.xQueueToSendResponseTo = xQueueForUartResponse;

    xQueueSend(xUartMessageQueue, &xUartMessage, portMAX_DELAY);
    xQueueReceive(xQueueForUartResponse, &usResponse, portMAX_DELAY);

    return usResponse;
}

static void prvWrite( unsigned char ucId, eRegister eRegister, unsigned short int usValue )
{
    unsigned short int usResponse;
    UartMessage_t xUartMessage;

    xUartMessage.xInstructionPacket.eInstructionType = eWrite;
    xUartMessage.xInstructionPacket.ucId = ucId;
    xUartMessage.xInstructionPacket.eRegister = eRegister;
    xUartMessage.xInstructionPacket.usParam = usValue;
    xUartMessage.xQueueToSendResponseTo = xQueueForUartResponse;

    xQueueSend(xUartMessageQueue, &xUartMessage, portMAX_DELAY);
    xQueueReceive(xQueueForUartResponse, &usResponse, portMAX_DELAY);
}

static void prvRegWrite( unsigned char ucId, eRegister eRegister, unsigned short int usValue )
{
    unsigned short int usResponse;
    UartMessage_t xUartMessage;

    xUartMessage.xInstructionPacket.eInstructionType = eRegWrite;
    xUartMessage.xInstructionPacket.ucId = ucId;
    xUartMessage.xInstructionPacket.eRegister = eRegister;
    xUartMessage.xInstructionPacket.usParam = usValue;
    xUartMessage.xQueueToSendResponseTo = xQueueForUartResponse;

    xQueueSend(xUartMessageQueue, &xUartMessage, portMAX_DELAY);
    xQueueReceive(xQueueForUartResponse, &usResponse, portMAX_DELAY);
}

static void prvAction()
{
    unsigned short int usResponse;
    UartMessage_t xUartMessage;

    xUartMessage.xInstructionPacket.eInstructionType = eAction;
    xUartMessage.xInstructionPacket.ucId = axBROADCAST_ID;
    xUartMessage.xQueueToSendResponseTo = xQueueForUartResponse;

    xQueueSend(xUartMessageQueue, &xUartMessage, portMAX_DELAY);
    xQueueReceive(xQueueForUartResponse, &usResponse, portMAX_DELAY);
}

