#include "octo.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

uint16_t currentPositions[48];
uint16_t goalPositions[48];
QueueHandle_t xQueue;

void led_task()
{
	while(1)
	{
		uint8_t speed;
		xQueueReceive(xQueue, &speed, portMAX_DELAY);
		GPIOC_SR = (1 << 13);
		vTaskDelay(pdMS_TO_TICKS(250 * speed));
		GPIOC_SR = (1 << 29);
		vTaskDelay(pdMS_TO_TICKS(250 * speed));
	}
}

void test_task()
{
	uint8_t n = 0;
	uint8_t fast = 1;
	uint8_t slow = 4;

	while (1)
	{
		++n;
		if (n <= 3)
		{
			xQueueSend(xQueue, &slow, portMAX_DELAY);
		}
		else if (n <= 6)
		{
			xQueueSend(xQueue, &fast, portMAX_DELAY);
		}
		else
		{
			n = 0;
		}
	}
}

void arm_controller_task()
{
	//These are the wrong numbers.
	//They should be in units instead of degrees.
	//150 / 0.29 = 517.
	goalPositions[0] = 150;
	goalPositions[1] = 195;
	goalPositions[2] = 60;
	goalPositions[3] = 60;
	goalPositions[4] = 150;
	goalPositions[5] = 150;

	goalPositions[6] = 150;
	goalPositions[7] = 195;
	goalPositions[8] = 60;
	goalPositions[9] = 60;
	goalPositions[10] = 150;
	goalPositions[11] = 150;

	goalPositions[42] = 150;
	goalPositions[43] = 195;
	goalPositions[44] = 60;
	goalPositions[45] = 60;
	goalPositions[46] = 150;
	goalPositions[47] = 150;

	instruction_t ins1 = {0, 1, 240, 60, 0, "t1", "t2"};
	instruction_t ins2 = {0, 2, 240, 150, 0, "t2", "f2"};
	instruction_t ins3 = {0, 8, 240, 60, 0, "t8", "t1"};
	instruction_t *inss[] = {&ins1, &ins2, &ins3};
	uint8_t inss_length = sizeof(inss) / sizeof(instruction_t *);

	while (1)
	{
		uint8_t flags = 0;
		for (int i = 0; i < inss_length; ++i)
		{
			instruction_t *ins = inss[i];
			if (ins->flag)
			{
				++flags;
				continue;
			}
			else
			{
				uint8_t allowed = 1;
				for (int j = 0; j < i; ++j)
				{
					instruction_t *prev_ins = inss[j];
					if (prev_ins->flag)
					{
						continue;
					}
					else
					{
						if (strcmp(ins->from, prev_ins->from) == 0
							|| strcmp(ins->from, prev_ins->to) == 0
							|| strcmp(ins->to, prev_ins->from) == 0
							|| strcmp(ins->to, prev_ins->to) == 0)
						{
							allowed = 0;
							break;
						}
					}
				}

				if (!allowed)
				{
					continue;
				}

				uint8_t stateChangeComplete = 1;
				for (int j = 0; j < 6; ++j)
				{
					uint8_t index = (ins->arm - 1) * 6 + j;
					if (abs(currentPositions[index] - goalPositions[index]) > 5)
					{
						//stateChangeComplete = 0;
					}
				}

				if (!stateChangeComplete)
				{
					continue;
				}

				switch (ins->state)
				{
					case 0: //rotate
						rotate(ins->arm, ins->r1);
						break;
					case 1: //extend
						stretch(ins->arm, 105, 105, 105);
						break;
					case 2: //close
						claw(ins->arm, 130, 170);
						break;
					case 3: //lift
						wrist(ins->arm, 60);
						break;
					case 4: //retract
						stretch(ins->arm, 195, 60, 60);
						break;
					case 5: //rotate
						rotate(ins->arm, ins->r2);
						break;
					case 6: //extend
						stretch(ins->arm, 105, 105, 60);
						break;
					case 7: //put
						wrist(ins->arm, 105);
						break;
					case 8: //open
						claw(ins->arm, 150, 150);
						break;
					case 9: //retract
						stretch(ins->arm, 195, 60, 60);
						break;
					case 10: //rotate
						rotate(ins->arm, 150);
						break;
					case 11:
						ins->flag = 1;
						continue;
					default: //Something went wrong.
						continue;
				}

				++ins->state;
			}
		}

		if (flags >= inss_length)
		{
			break;
		}
	}
}

void uart_controller_task()
{
	uint8_t test;
	xQueueReceive(xQueue, &test, portMAX_DELAY);
}

void rotate(uint8_t arm, uint16_t aDegrees) {
	uint8_t index = (arm - 1) * 6;
	uint8_t aMotor = arm * 10 + 1;
	setSpeed(aMotor, 15);
	setGoalPosition(aMotor, aDegrees);
	goalPositions[index] = aDegrees / 0.29;
}

void stretch(uint8_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees) {
	uint8_t index = (arm - 1) * 6;
	uint8_t bMotor = arm * 10 + 2;
	uint8_t cMotor = bMotor + 1;
	uint8_t dMotor = cMotor + 1;
	uint16_t bDegreesDelta = abs(currentPositions[index + 1] - bDegrees);
	uint16_t cDegreesDelta = abs(currentPositions[index + 2] - cDegrees);
	uint16_t dDegreesDelta = abs(currentPositions[index + 3] - dDegrees);
	uint16_t bRpm = (bDegreesDelta / 90.0) * 15;
	uint16_t cRpm = (cDegreesDelta / 90.0) * 15;
	uint16_t dRpm = (dDegreesDelta / 90.0) * 15;
	setSpeed(bMotor, bRpm);
	setSpeed(cMotor, cRpm);
	setSpeed(dMotor, dRpm);
	setGoalPosition(bMotor, bDegrees);
	setGoalPosition(cMotor, cDegrees);
	setGoalPosition(dMotor, dDegrees);
	goalPositions[index+1] = bDegrees / 0.29;
	goalPositions[index+2] = cDegrees / 0.29;
	goalPositions[index+3] = dDegrees / 0.29;
}

void wrist(uint8_t arm, uint16_t dDegrees) {
	uint8_t index = (arm - 1) * 6;
	uint8_t aMotor = arm * 10 + 4;
	setSpeed(aMotor, 15);
	setGoalPosition(aMotor, dDegrees);
	goalPositions[index+3] = dDegrees / 0.29;
}

void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees) {
	uint8_t index = (arm - 1) * 6;
	uint8_t eMotor = arm * 10 + 5;
	uint8_t fMotor = arm * 10 + 6;
	setSpeed(eMotor, 15);
	setSpeed(fMotor, 15);
	setGoalPosition(eMotor, eDegrees);
	setGoalPosition(fMotor, fDegrees);
	goalPositions[index+4] = eDegrees / 0.29;
	goalPositions[index+5] = fDegrees / 0.29;
}


//0--1023, 0.111 rpm per unit
void setSpeed(uint8_t motor, uint16_t rpm) {
	uint16_t units = rpm / 0.111;
	units = units > 1023 ? 1023 : units;
	//TODO: send instruction packet.
}

//0--1023, 0.29 degree per unit
void setGoalPosition(uint8_t motor, uint16_t degrees) {
	uint16_t units = degrees / 0.29;
	units = units > 1023 ? 1023 : units;
	//TODO: send instruction packet.
}
