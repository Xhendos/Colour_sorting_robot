#include "manager.h"

#include "FreeRTOS.h"
#include "task.h"
#include "client.h"
#include "arm_server.h"
#include "uart.h"

void vTaskManager( void * pvParameters )
{
    xTaskCreate(vTaskClient, "client", 1000, NULL, 1, &xClientTask);
    xTaskCreate(vTaskArmServer, "arm_server", 128, NULL, 2, &xArmServerTask);
    xTaskCreate(vTaskUart, "uart", 128, NULL, 3, &xUartTask);

    vTaskDelete(NULL);
}

