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
    uint8_t ucRed;
    uint8_t ucGreen;
    uint8_t ucBlue;
} RgbColours_t;

extern QueueHandle_t xFromRgbServer;
extern QueueHandle_t xToRgbServer;
extern TaskHandle_t xRgbServerTask;
extern UBaseType_t uxRgbServerDoneConfiguring;

void vTaskRgbServer( void * pvParameters );

#endif /* RGB_SERVER_H */
