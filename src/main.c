#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f103xb.h"
#include "i2c.h"

typedef struct xMESSAGE_I2C
{
    uint8_t ucAddress;              /* I2C slave address */
    uint8_t ucByte;                 /* Byte that should be send to the slave */
    uint8_t ucWriteFinished;        /* ALWAYS set this to 0 */
    uint8_t ucRead;                 /* 0 = do not read, 1 = read 1 byte from slave, 2 = read 2 bytes from slave */
    uint8_t *pucReadBytes;          /* Bytes received from the slave */
} MessageI2c;

static _Bool xI2cPeripheralBusy = 0;
static MessageI2c xI2cIsrMessage;
static MessageI2c xI2cDummyMessage;
static QueueHandle_t xI2cToIsr;
static QueueHandle_t xI2cFromIsr;

void I2C1_EV_IRQ_handler(void)
{
	if(I2C1->SR1 & I2C_SR1_SB)		/* (0x01) Start bit has been send */
	{
        if(!xI2cPeripheralBusy)
        {
            if(xQueuePeekFromISR(xI2cToIsr, &xI2cIsrMessage))
                xI2cPeripheralBusy = 1; 
        }
        
        if(xI2cPeripheralBusy)
        {
            if(!xI2cIsrMessage.ucWriteFinished)
            {
                I2C1->SR1;
                I2C1->DR = (xI2cIsrMessage.ucAddress << 1) | 0;                
            } else if(xI2cIsrMessage.ucWriteFinished)
            {
                I2C1->SR1;
                I2C1->DR = (xI2cIsrMessage.ucAddress << 1) | 1;
            }
        }           
 
        I2C1->SR1 &= ~(I2C_SR1_SB); /* Acknowledge the interrupt */				
        return;
	}

	if(I2C1->SR1 & I2C_SR1_ADDR)	/* (0x02) I2C slave address has been send */
	{
        if(xI2cIsrMessage.ucWriteFinished)
        {
            if(xI2cIsrMessage.ucRead == 1)
            {
                I2C1->CR1 &= ~(I2C_CR1_ACK);
            } else if(xI2cIsrMessage.ucRead == 2)
            {
                
            }
        } else if(!xI2cIsrMessage.ucWriteFinished)
        {           
            I2C1->SR1;                  /* FIRST acknowledge the interrupt before write to data register */
            I2C1->SR2;                  /* See Figure 271 in the reference manual, transfer sequence diagram for master transmitter */

            I2C1->DR = xI2cIsrMessage.ucByte;
        }    
         
        I2C1->SR1;                  /* Acknowledge the interrupt */
        I2C1->SR2;                  
        I2C1->SR1 &= ~(I2C_SR1_ADDR);
        return;
	}

    if(I2C1->SR1 & I2C_SR1_RXNE)	/* Receive data register not empty */
	{
        I2C1->SR1;                  /* Dummy read to clear the BTF (byte transfer finished) flag */
        xI2cIsrMessage.pucReadBytes[0] = I2C1->DR;  /* Must be the next action after the dummy SR1 read */

               
        while(I2C1->CR1 & I2C_CR1_STOP);
        I2C1->CR1 |= I2C_CR1_ACK;       /* Set acknowledgement after a byte is received on again */ 

        xI2cPeripheralBusy = 0;         /* We finished this request and are free to accept a new one */
        xQueueReceiveFromISR(xI2cToIsr, &xI2cDummyMessage, NULL);   /* Delete the peeked request, basically acknowledge it */
        xQueueSendFromISR(xI2cFromIsr, &xI2cIsrMessage, NULL);      /* Send the response to the caller */

        I2C1->SR1 &= ~(I2C_SR1_RXNE);   /* Acknowledge the interrupt */
	    //I2C1->DR;
        return;
    }

	if(I2C1->SR1 & I2C_SR1_BTF)		/* (0x04) Byte transfer is finished */
	{
        if(xI2cIsrMessage.ucWriteFinished)
        {
            volatile int i = 0;
            i += 1;
        } else if(!xI2cIsrMessage.ucWriteFinished)
        {
            I2C1->CR1 |= (I2C_CR1_STOP);        /* Send a stop bit since we only want to write 1 byte */
            xI2cIsrMessage.ucWriteFinished = 1; /* We succesfully wrote our byte to the slave */  

            while(I2C1->CR1 & I2C_CR1_STOP);
            if(xI2cIsrMessage.ucRead)           
            {
                I2C1->CR1 |= (I2C_CR1_START);   /* Generate a new START bit (for the read action) */ 
            } else if(!xI2cIsrMessage.ucRead)
            {
                xI2cPeripheralBusy = 0;         /* We finished this request and are free to accept a new one */
                xQueueReceiveFromISR(xI2cToIsr, &xI2cDummyMessage, NULL);/* Delete the peeked request, basically acknowledge it */
                xQueueSendFromISR(xI2cFromIsr, &xI2cIsrMessage, NULL);  /* Send the response to the caller */
            }
        }         

        I2C1->SR1 &= ~(I2C_SR1_BTF);/* Acknowledge the interrupt */
	    return;
    }

	if(I2C1->SR1 & I2C_SR1_TXE)		/* Transmit data register empty */
	{

        I2C1->SR1 &= ~(I2C_SR1_TXE);/* Acknowledge the interrupt */ 
	    return;
    }

	if(I2C1->SR1 & I2C_SR1_RXNE)	/* Receive data register not empty */
	{
        xI2cIsrMessage.pucReadBytes[0] = I2C1->DR;
        
        while(I2C1->CR1 & I2C_CR1_STOP);
        I2C1->CR1 |= I2C_CR1_ACK;       /* Set acknowledgement after a byte is received on again */ 

        xI2cPeripheralBusy = 0;         /* We finished this request and are free to accept a new one */        
        xQueueSendFromISR(xI2cFromIsr, &xI2cIsrMessage, NULL);      /* Send the response to the caller */
        xQueueReceiveFromISR(xI2cToIsr, &xI2cDummyMessage, NULL);   /* Delete the peeked request, basically acknowledge it */

        I2C1->SR1 &= ~(I2C_SR1_RXNE);   /* Acknowledge the interrupt */
	    return;
    }
}
/*-----------------------------------------------------------*/ 

void rgb_task(void)
{
uint8_t ucI2cResult[2];
MessageI2c xWhoAmI;
MessageI2c xI2cResponse;
volatile uint16_t usVar;

    xWhoAmI.ucAddress = 0x29;
    xWhoAmI.ucByte = 0x80 | 0x12;
    xWhoAmI.ucWriteFinished = 0;
    xWhoAmI.ucRead = 1;
    xWhoAmI.pucReadBytes = ucI2cResult;
loop:
    while(1)
    {
        xWhoAmI.ucWriteFinished = 0; 
       if(xQueueSend(xI2cToIsr, &xWhoAmI, portMAX_DELAY) != pdTRUE)
            goto loop;               
        I2C1->CR1 |= (1 << I2C_CR1_START_Pos);

loop2:        
        if(xQueueReceive(xI2cFromIsr, &xI2cResponse, portMAX_DELAY) != pdTRUE)
            goto loop2;
        
        usVar += 1;         
        
    }
}
/*-----------------------------------------------------------*/ 

int main(void)
{
    /************************************************************
    *   Pin number   *    Pin name  *   Alternative function    *
    *************************************************************
    *       42       *      PB6     *         I2C1_SCL          *
    *************************************************************
    *       43       *      PB7     *         I2C_SDA           *
    *************************************************************
    *       29       *      PA8     *         USART1_CK         *
    *************************************************************
    *       30       *      PA9     *         USART1_TX         *
    *************************************************************
    *       31       *      PA10    *         USART1_RX         *
    *************************************************************
    *       32       *      PA11    *         USART1_CTS        *
    *************************************************************
    *       33       *      PA12    *         USART1_RTS        *
    *************************************************************/

    /************************************************************
    *   Pin number   *   Pin name   *      General purpose      *
    *************************************************************
    *     10--17     *   PA0--7     *         RGB0--7           *
    *************************************************************
    *     25--28     *   PB12--15   *         RGB8--11          *
    *************************************************************/

    RCC->CR = 0;
    RCC->CR |= (RCC_CR_HSION | 0x10 << RCC_CR_HSITRIM_Pos);

    RCC->CFGR = 0;
    RCC->CFGR |= (RCC_CFGR_SW_HSI | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1);

    RCC->APB2RSTR = ~0;
    RCC->APB2RSTR = 0;

    RCC->APB1RSTR = ~0;
    RCC->APB1RSTR = 0;

    RCC->APB2ENR = 0;
    RCC->APB2ENR = (RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_USART1EN);

    RCC->APB1ENR = 0;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOA->CRL = 0;
    GPIOA->CRL |= GPIO_CRL_MODE0_0;
    GPIOA->CRL |= GPIO_CRL_MODE1_0;
    GPIOA->CRL |= GPIO_CRL_MODE2_0;
    GPIOA->CRL |= GPIO_CRL_MODE3_0;
    GPIOA->CRL |= GPIO_CRL_MODE4_0;
    GPIOA->CRL |= GPIO_CRL_MODE5_0;
    GPIOA->CRL |= GPIO_CRL_MODE6_0;
    GPIOA->CRL |= GPIO_CRL_MODE7_0;

    GPIOA->CRH = 0;
    GPIOA->CRH |= GPIO_CRL_MODE1;
    GPIOA->CRH &= ~GPIO_CRL_MODE2;
    GPIOA->CRH |= GPIO_CRL_CNF1_1;
    GPIOA->CRH |= GPIO_CRL_CNF2_0;

    GPIOB->CRL = 0;
    GPIOB->CRL |= (GPIO_CRL_MODE0_0 | GPIO_CRL_CNF0_0);
    GPIOB->CRL |= (GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOB->CRL |= (GPIO_CRL_MODE7 | GPIO_CRL_CNF7);

    GPIOB->CRH = 0;
    GPIOB->CRH |= GPIO_CRL_MODE4_0;
    GPIOB->CRH |= GPIO_CRL_MODE5_0;
    GPIOB->CRH |= GPIO_CRL_MODE6_0;
    GPIOB->CRH |= GPIO_CRL_MODE7_0;

    USART1->CR1 = 0;
    USART1->CR1 |= (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);

    USART1->BRR = 0;
    USART1->BRR |= 1 << USART_BRR_DIV_Mantissa_Pos;

    /*
     * Thigh = CCR * TPCLK1
     * CCR = Thigh / TPCLK1
     *
     * To get TPCLK1 we use f = 1 / t
     * 1 / 8000000 = 125 ns
     * This means that TPCLK1 is 125 ns.
     *
     * Thigh = Tscl / 2
     * To get Tscl we use Tscl = 1 / Fscl
     * If we want to generate 100 KHz SCL (Fscl = 1000000 Hz) then Tscl is 10 microseconds
     * Thigh = 10 us / 2 = 5000 ns
     *
     * CCR = Thigh / TPCLK1
     *     = 5000 (ns) / 125 ns = 40
     * CCR should be 40 or 0x28 */

    /*
     * TRISE = (Trise / TPCLK1) + 1
     *       = 1000 (ns) / 125 (ns) + 1
     *       = 9 */

    I2C1->CR2 = 0;
    I2C1->CR2 |= I2C_CR2_FREQ_3;                /* Clock frequency. FREQ_3 is 8 MHz */
    I2C1->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN); /* Embrace I2C1 event interrupts (including RxNE and TxE interrupt) */

    I2C1->CCR = 0;
    I2C1->CCR |= 0x28 << I2C_CCR_CCR_Pos;       /* Generate 100 KHz serial clock speed */

    I2C1->TRISE = 0;
    I2C1->TRISE |= 0x9 << I2C_TRISE_TRISE_Pos;  /* Maximum rise time */

    I2C1->CR1 = 0;
    I2C1->CR1 |= I2C_CR1_PE;                    /* Turn on the peripheral */ 

    /* Create and initialise queues */
    xI2cFromIsr = xQueueCreate(1, sizeof(MessageI2c));   
    xI2cToIsr = xQueueCreate(1, sizeof(MessageI2c));

    if(xI2cFromIsr == NULL || xI2cToIsr == NULL)
        return -2;                              /* No sufficient memory to create the queue */

    NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS); /* https://www.freertos.org/RTOS-Cortex-M3-M4.html */
    NVIC_SetPriority(I2C1_EV_IRQn, 2);
    NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_EV_IRQn);   
 

    xTaskCreate(rgb_task, "rgb_task", 500, NULL, 3, NULL);
    vTaskStartScheduler();                      /* Start the FreeRTOS scheduler */

    return -1;                                  /* We should never hit this line */
}
/*-----------------------------------------------------------*/
