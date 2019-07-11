#include "manager.h"

#include "FreeRTOS.h"
#include "task.h"
#include "client.h"

void vTaskManager( void * pvParameters )
{
    xTaskCreate(vTaskClient, "client", 1000, NULL, 1, NULL);

    vTaskDelete(NULL);
}

