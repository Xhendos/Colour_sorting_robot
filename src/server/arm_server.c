#include "arm_server.h"


void vTaskArmServer( void * pvParameters )
{
ArmServerMessage_t *pxMessage;

    xArmServerMessageQueue = xQueueCreate(ARM_SERVER_MESSAGE_QUEUE_SIZE, sizeof(ArmServerMessage_t));

    if ( xArmServerMessageQueue == NULL )
    {
        /* Message queue did not get created. */
    }

    while (1)
    {

    }
}

