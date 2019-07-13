#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "stm32f103xb.h"
#include "octo.h"
#include "ax.h"

TaskHandle_t xUartTask;
QueueHandle_t xUartMessageQueue;

static unsigned char ucTx[16];
static unsigned char ucRx[16];
static unsigned char ucTxBytes = 0;
static unsigned char ucRxBytes = 0;

void vTaskUart( void * pvParameters )
{
UartMessage_t xMessage;
InstructionPacket_t xInstructionPacket;

    xUartMessageQueue = xQueueCreate(uartSERVER_MESSAGE_QUEUE_SIZE, sizeof(UartMessage_t));

    if( xUartMessageQueue == NULL )
    {
        /* Message queue did not get created. */
    }

    ucTx[0] = 0xFF;
    ucTx[1] = 0xFF;

    while (1)
    {
        if (xQueueReceive(xUartMessageQueue, &xMessage, portMAX_DELAY) == pdFALSE)
        {
            /* Failed retrieving item from queue. */
        }

        xInstructionPacket = xMessage.xInstructionPacket;

        ucTx[2] = xInstructionPacket.ucId;

        switch (xInstructionPacket.eInstructionType)
        {
            case eRead:
                ucTx[3] = axINSTRUCTION_PACKET_LENGTH_CONSTANT + 2;
                break;
            case eWrite:
            case eRegWrite:
                ucTx[3] = axINSTRUCTION_PACKET_LENGTH_CONSTANT + 1 + ucByteSize(xInstructionPacket.eRegister);
                break;
            case eAction:
                ucTx[3] = axINSTRUCTION_PACKET_LENGTH_CONSTANT;
                break;
        }

        ucTx[4] = xInstructionPacket.eInstructionType;

        switch (xInstructionPacket.eInstructionType)
        {
            case eRead:
                ucTx[5] = xInstructionPacket.eRegister;
                ucTx[6] = xInstructionPacket.usParam & 0xFF;
                ucTx[7] = ~(ucTx[2] + ucTx[3] + ucTx[4] + ucTx[5] + ucTx[6]);
                break;
            case eWrite:
            case eRegWrite:
                ucTx[5] = xInstructionPacket.eRegister;
                ucTx[6] = xInstructionPacket.usParam & 0xFF;
                if (ucByteSize(xInstructionPacket.eRegister) == 1)
                {
                    ucTx[7] = ~(ucTx[2] + ucTx[3] + ucTx[4] + ucTx[5] + ucTx[6]);
                }
                else
                {
                    ucTx[7] = (xInstructionPacket.usParam >> 8) & 0xFF;
                    ucTx[8] = ~(ucTx[2] + ucTx[3] + ucTx[4] + ucTx[5] + ucTx[6] + ucTx[7]);
                }
                break;
            case eAction:
                ucTx[5] = ~(ucTx[2] + ucTx[3] + ucTx[4]);
                break;
        }

        ucTxBytes = 4 + ucTx[3];

        if (xInstructionPacket.eInstructionType == eRead)
        {
            ucRxBytes = 6 + ucTx[6];
        }
        else
        {
            ucRxBytes = 6;
        }

        /* Set direction to TX. Enable TXE interrupts to get into the interrupt handler. Wait on a notify from the interrupt handler when it is done receiving the status packet. */
        GPIOB->BSRR = GPIO_BSRR_BS0;
        USART1->CR1 |= USART_CR1_TXEIE;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        unsigned short usResponse;
        if (xInstructionPacket.eInstructionType == eRead)
        {
            if (ucByteSize(xInstructionPacket.eRegister) == 2)
            {
                usResponse = (ucRx[6] << 8) + ucRx[5];
            }
            else
            {
                usResponse = ucRx[5];
            }
        }
        xQueueSend(xMessage.xQueueToSendResponseTo, &usResponse, portMAX_DELAY);
    }
}

void USART1_IRQ_handler()
{
    static unsigned char index = 0;
    UBaseType_t uxSr = USART1->SR;
    UBaseType_t uxDr = USART1->DR;
    UBaseType_t uxCr1 = USART1->CR1;
    UBaseType_t uxTxeie = uxCr1 & USART_CR1_TXEIE;
    UBaseType_t uxTcie = uxCr1 & USART_CR1_TCIE;
    UBaseType_t uxRxneie = uxCr1 & USART_CR1_RXNEIE;
    UBaseType_t uxTxe = uxSr & USART_SR_TXE;
    UBaseType_t uxTc = uxSr & USART_SR_TC;
    UBaseType_t uxRxne = uxSr & USART_SR_RXNE;
    UBaseType_t uxOre = uxSr & USART_SR_ORE;

    if (uxTcie && uxTc)
    {
        GPIOB->BSRR = GPIO_BSRR_BR0;
        USART1->CR1 |= USART_CR1_RXNEIE;
        USART1->CR1 &= ~(USART_CR1_TCIE);
        index = 0;
    }

    if (uxTxeie && uxTxe)
    {
        if (index < ucTxBytes)
        {
            USART1->DR = ucTx[index];
            ++index;
        }
        else
        {
            USART1->CR1 |= USART_CR1_TCIE;
            USART1->CR1 &= ~(USART_CR1_TXEIE);
        }
    }

    if (uxRxneie && uxRxne)
    {
        ucRx[index] = uxDr;
        ++index;

        if (index == ucRxBytes)
        {
            USART1->CR1 &= ~(USART_CR1_RXNEIE);
            index = 0;
            vTaskNotifyGiveFromISR(xUartTask, NULL);
        }
    }

    /* RXNEIE enables interrupts for both RXNE and ORE. This program does not act on ORE interrupts but it could be useful during debugging. */
    if (uxRxneie && uxOre)
    {
        USART1->SR;
        USART1->DR;
    }
}

