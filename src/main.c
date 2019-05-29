#include "FreeRTOS.h"
#include "task.h"
#include "octo.h"

int main(void)
{
	taskManagerQueue = xQueueCreate(16, sizeof(TaskHandle_t));
	uartSignalQueue = xQueueCreate(16, sizeof(instruction_t));
	uartResultQueue = xQueueCreate(16, sizeof(uint8_t));

	xTaskCreate(manager_task, "manager", 128, NULL, configMAX_PRIORITIES - 1, NULL);
	vTaskStartScheduler();
	return 0;
}

