#include "arm_server.h"
#include "ax.h"
#include "octo.h"

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
static unsigned short int prvRead( Motor_t * pxMotor, eRegister eRegister );
static void prvWrite( Motor_t * pxMotor, eRegister eRegister, unsigned short int usValue );
static void prvRegWrite( Motor_t * pxMotor, eRegister eRegister, unsigned short int usValue );
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

QueueHandle_t xArmServerMessageQueue;

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

        /* Loop through each position of a movement. This movement is for displacing a ball and consists of 15 positions. */
        for (UBaseType_t uxPosition = 0; uxPosition < 15; ++uxPosition)
        {
            prvClearMotorParticipations();

            /* Loop through each arm that should perform the movement. */
            for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
            {
                if (eBufferedMovement[uxArmIndex] != eDisplace)
                {
                    continue;
                }

                /* Servo motors of an arm are tagged as participating and their goal positions are set. */
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
            }

            /* To make the participating servo motors reach their position simultaneously, the difference between their goal position and present position are calculated. */
            for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
            {
                if (eBufferedMovement[uxArmIndex] != eDisplace)
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

                    pxMotor->usPresentPositionInUnits = prvRead(pxMotor, ePresentPosition);

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
                if (eBufferedMovement[uxArmIndex] != eDisplace)
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
                if (eBufferedMovement[uxArmIndex] != eDisplace)
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

                    prvWrite(pxMotor, eMovingSpeed, pxMotor->usMovingSpeedInUnits);
                    prvRegWrite(pxMotor, eGoalPosition, pxMotor->usGoalPositionInUnits);
                }
            }

            prvAction();

            /* Wait for each servo motor to be done with moving. */
            unsigned char ucAllServoMotorsDoneMoving = 1;
            do
            {
                for (UBaseType_t uxArmIndex = octoARM_A_INDEX; uxArmIndex <= octoARM_H_INDEX; uxArmIndex += octoARM_INDEX_INCREMENT)
                {
                    if (eBufferedMovement[uxArmIndex] != eDisplace)
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

                        unsigned char ucIsMoving = prvRead(pxMotor, eMoving);

                        if (ucIsMoving)
                        {
                            ucAllServoMotorsDoneMoving = 0;
                        }
                    }
                }
            } while (!ucAllServoMotorsDoneMoving);
        }
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

static unsigned short int prvRead( Motor_t * pxMotor, eRegister eRegister )
{
    return 0;
}

static void prvWrite( Motor_t * pxMotor, eRegister eRegister, unsigned short int usValue )
{

}

static void prvRegWrite( Motor_t * pxMotor, eRegister eRegister, unsigned short int usValue )
{

}

static void prvAction()
{

}

