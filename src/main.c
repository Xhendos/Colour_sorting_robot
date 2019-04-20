#include "stm32f103xb.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#define GPIOC_CLK       (*((volatile uint32_t *) 0x40021018))
#define GPIOC_HIGH      (*((volatile uint32_t *) 0x40011004))
#define GPIOC_SR        (*((volatile uint32_t *) 0x40011010))

int i = 0;

static void led_task(void *args)
{
    while(1)
    {
        i += 1;
        GPIOC_SR = (1 << 13);
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIOC_SR = (1 << 29);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{

    GPIOC_CLK |= (1 << 4);

    GPIOC_HIGH &= ~(3 << 20);
    GPIOC_HIGH |= (1 << 20);
    GPIOC_HIGH &= ~(3 << 22);
    GPIOC_HIGH |= (0 << 22);

    xTaskCreate(led_task, "LED_blink_1", 128, NULL, configMAX_PRIORITIES-1, NULL);

    vTaskStartScheduler();    
    
    return 0;
}
