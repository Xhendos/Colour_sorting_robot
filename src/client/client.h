#ifndef _CLIENT_H
#define _CLIENT_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern TaskHandle_t xClientTask;

void vTaskClient( void * pvParameters );

#endif /* _CLIENT_H */

