#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"

#include "stm32f103xb.h"

void USART1_IRQ_handler()
{
    UBaseType_t uxCr1 = USART1->CR1;
    UBaseType_t uxSr = USART1->SR;
    UBaseType_t uxDr = USART1->DR;
    UBaseType_t uxTxeie = uxCr1 & USART_CR1_TXEIE;
    UBaseType_t uxRxneie = uxCr1 & USART_CR1_RXNEIE;
    UBaseType_t uxTxe = uxSr & USART_SR_TXE;
    UBaseType_t uxRxne = uxSr & USART_SR_RXNE;
    UBaseType_t uxOre = uxSr & USART_SR_ORE;

    /* Data has been put in data register by uart task. Uart task then enabled TXEIE. Uart task is waiting on notify so it knows when to send the next byte. */
    if (uxTxeie & uxTxe)
    {
        USART1->CR1 &= ~(USART_CR1_TXEIE);
        vTaskNotifyGiveFromISR(xUartTask, NULL);
        return;
    }

    if (uxRxneie & uxRxne)
    {
        //USART1->DR;
        /* Apperantly reading data register does not clear RXNE flag? That is why the flag gets cleared explicitly. */
        //USART1->SR &= ~(USART_SR_RXNE);
        volatile UBaseType_t sr = USART1->SR;
        vTaskNotifyGiveFromISR(xUartTask, NULL);
        return;
    }

    if (uxRxneie & uxOre)
    {
        USART1->SR;
        USART1->DR;
        return;
    }
}
