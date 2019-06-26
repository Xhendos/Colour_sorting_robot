#include "FreeRTOS.h"
#include "task.h"
#include "octo.h"

int main(void)
{
	xTaskCreate(init_task, "init", 128, NULL, configMAX_PRIORITIES, NULL);
	vTaskStartScheduler();
	return 0;
}

