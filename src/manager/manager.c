#include "manager.h"

#include "FreeRTOS.h"
#include "task.h"

#include "client.h"
#include "rgb_server.h"

void vTaskManager( void * pvParameters )
{
    xTaskCreate(vTaskClient, "client", 1000, NULL, 1, NULL);

    xTaskCreate(vTaskRgbServer, "rgb_server", 128, NULL, 3, NULL);
    
    vTaskDelete(NULL);
}

