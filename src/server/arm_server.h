#ifndef _ARM_SERVER_H
#define _ARM_SERVER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ARM_SERVER_MESSAGE_QUEUE_SIZE  ( 1 )

#define ARM0        ( 1 << 0 )
#define ARM1        ( 1 << 1 )
#define ARM2        ( 1 << 2 )
#define ARM3        ( 1 << 3 )
#define ARM4        ( 1 << 4 )
#define ARM5        ( 1 << 5 )
#define ARM6        ( 1 << 6 )
#define ARM7        ( 1 << 7 )
#define ALL_ARMS    ( ARM0 | ARM1 | ARM2 | ARM3 | ARM4 | ARM5 | ARM6 | ARM7 )
#define ODD_ARMS    ( ARM1 | ARM3 | ARM5 | ARM7 )
#define EVEN_ARMS   ( ARM0 | ARM2 | ARM4 | ARM6 )

typedef enum {
    eSet0 = ARM0,
    eSet1 = ARM1,
    eSet2 = ARM2,
    eSet3 = ARM3,
    eSet4 = ARM4,
    eSet5 = ARM5,
    eSet6 = ARM6,
    eSet7 = ARM7,
    eSet8 = ALL_ARMS,
    eSet9 = ODD_ARMS,
    eSet10 = EVEN_ARMS,
} eSetOfArms;

typedef enum {
    eNoMovement = 0,
    eDisplace,
    eRest,
} eArmMovement;

typedef enum {
    eDontExecute = 0,
    eDoExecute,
} eExecuteMovement;

typedef struct xARM_SEVER_MESSAGE {
    eSetOfArms eArms;
    eArmMovement eMovement;
    eExecuteMovement eExecute;
    unsigned short int usFirstRotationInDegrees;
    unsigned short int usSecondRotationInDegrees;
    TaskHandle_t xSenderOfMessage;
} ArmServerMessage_t;

extern TaskHandle_t xArmServerTask;
extern QueueHandle_t xArmServerMessageQueue;
extern UBaseType_t uxArmServerDoneConfiguring;

void vTaskArmServer( void * pvParameters );

#endif /* _ARM_SERVER_H */

