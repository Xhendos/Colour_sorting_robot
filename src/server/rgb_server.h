#ifndef RGB_SERVER_H
#define RGB_SERVER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "octo.h"

#define RGB_SERVER_MESSAGE_QUEUE_SIZE  (1)

/* struct */
typedef struct xRGBSERVERMESSAGE
{
    ePlaceholder ePlaceholder;
    QueueHandle_t xQueueDestination;
} RgbServerMessage_t;

typedef struct xRGB
{
    uint32_t ulRed;
    uint32_t ulGreen;
    uint32_t ulBlue;
} RgbColours_t;

extern QueueHandle_t xFromRgbServer;
extern QueueHandle_t xToRgbServer;
extern TaskHandle_t xRgbServerTask;

void vTaskRgbServer( void * pvParameters );

#endif /* RGB_SERVER_H */
