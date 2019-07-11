#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "stm32f103xb.h"
#include "octo.h"

void vTaskUart( void * pvParameters )
{
UartMessage_t xMessage;
InstructionPacket_t xInstructionPacket;
unsigned char tx[16];
unsigned char rx[16];

    tx[0] = 0xFF;
    tx[1] = 0xFF;

    while (1)
    {
        //Read instruction packet from queue.
        if (xQueueReceive(xUartMessageQueue, &xMessage, portMAX_DELAY) == pdFALSE)
        {
            /* Failed retrieving item from queue. */
        }

        xInstructionPacket = xMessage.xInstructionPacket;

        //Dissect to bytes.
        tx[2] = xInstructionPacket.ucId;

        switch (xInstructionPacket.eInstructionType)
        {
            case eRead:
                tx[3] = axINSTRUCTION_PACKET_LENGTH_CONSTANT + 2;
                break;
            case eWrite:
            case eRegWrite:
                tx[3] = axINSTRUCTION_PACKET_LENGTH_CONSTANT + 1 + ucByteSize(xInstructionPacket.eRegister);
                break;
            case eAction:
                tx[3] = axINSTRUCTION_PACKET_LENGTH_CONSTANT;
                break;
            default:
                configASSERT( xInstructionPacket.eInstructionType );
        }

        tx[4] = xInstructionPacket.eInstructionType;

        switch (xInstructionPacket.eInstructionType)
        {
            case eRead:
                tx[5] = xInstructionPacket.eRegister;
                tx[6] = xInstructionPacket.usParam & 0xFF;
                tx[7] = ~(tx[2] + tx[3] + tx[4] + tx[5] + tx[6]);
                break;
            case eWrite:
            case eRegWrite:
                tx[5] = xInstructionPacket.eRegister;
                tx[6] = xInstructionPacket.usParam & 0xFF;
                if (ucByteSize(xInstructionPacket.eRegister) == 1)
                {
                    tx[7] = ~(tx[2] + tx[3] + tx[4] + tx[5] + tx[6]);
                }
                else
                {
                    tx[7] = (xInstructionPacket.usParam >> 8) & 0xFF;
                    tx[8] = ~(tx[2] + tx[3] + tx[4] + tx[5] + tx[6] + tx[7]);
                }
            default:
                configASSERT( xInstructionPacket.eInstructionType );
        }

        ////Send bytes.
        //data direction as output.
        GPIOB->BSRR = GPIO_BSRR_BS0;
        for (unsigned char ucI = 0; ucI < (4 + tx[3]); ++ucI)
        {
            //enable txeie.
            USART1->CR1 |= USART_CR1_TXEIE;
            //put byte in dr.
            USART1->DR = tx[ucI];
            //wait for notify from interrupt handler.
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        //data direction as input.
        GPIOB->BSRR = GPIO_BSRR_BR0;
        USART1->CR1 |= USART_CR1_RXNEIE;
        //Wait on bytes from status packet.
        //if read type handle like 6 + usByteSize(xInstructionPacket.eRegister) else handle like 6 byte status packet.
        if (xInstructionPacket.eInstructionType == eRead)
        {
            for (unsigned char ucI = 0; ucI < 6 + tx[6]; ++ucI)
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                rx[ucI] = USART1->DR;
            }
        }
        else
        {
            for (unsigned char ucI = 0; ucI < 6; ++ucI)
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                rx[ucI] = USART1->DR;
            }
        }
        /* This isn't needed to disable since a motor will only respond to an instruction. This could even be enabled during init and kept enabled during program execution. */
        USART1->CR1 &= ~USART_CR1_RXNEIE;

        unsigned short usResponse;

        if (xInstructionPacket.eInstructionType == eRead)
        {
            if (ucByteSize(xInstructionPacket.eRegister) == 2)
            {
                usResponse = (rx[6] << 8) + rx[5];
            }
            else
            {
                usResponse = rx[5];
            }
        }

        //Return value if instruction packet was of type Read.
        xQueueSend(xMessage.xQueueToSendResponseTo, &usResponse, portMAX_DELAY);
    }
}

