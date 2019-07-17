#ifndef _UARTH_H
#define _UART_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ax.h"

#define uartSERVER_MESSAGE_QUEUE_SIZE   ( 1 )

typedef struct xINSTRUCTION_PACKET {
    eInstructionType eInstructionType;
    unsigned char ucId;
    eRegister eRegister;
    unsigned short int usParam;
} InstructionPacket_t;

typedef struct xUART_MESSAGE {
    InstructionPacket_t xInstructionPacket;
    QueueHandle_t xQueueToSendResponseTo;
} UartMessage_t;

extern TaskHandle_t xUartTask;
extern QueueHandle_t xUartMessageQueue;

void vTaskUart( void * pvParameters );

#endif /* _UART_H */

