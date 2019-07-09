#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f103xb.h"

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
    I2C1->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_BUFEN); /* Embrace I2C1 event interrupts (including RxNE and TxE interrupt) */

    I2C1->CCR = 0;
    I2C1->CCR |= 0x28 << I2C_CCR_CCR_Pos;       /* Generate 100 KHz serial clock speed */

    I2C1->TRISE = 0;
    I2C1->TRISE |= 0x9 << I2C_TRISE_TRISE_Pos;  /* Maximum rise time */

    I2C1->CR1 = 0;
    I2C1->CR1 |= I2C_CR1_PE;                    /* Turn on the peripheral */

    return -1;
}

