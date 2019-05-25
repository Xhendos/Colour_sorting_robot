#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
	uint8_t flag;
	uint8_t arm;
	uint16_t r1;
	uint16_t r2;
	uint8_t state;
	char *from;
	char *to;
} instruction_t;

static uint16_t currentPositions[48];
static uint16_t goalPositions[48];

void rotate(uint8_t arm, uint16_t aDegrees);
void stretch(uint8_t arm, uint16_t bDegrees, uint16_t cDegrees, uint16_t dDegrees);
void wrist(uint8_t arm, uint16_t degrees);
void claw(uint8_t arm, uint16_t eDegrees, uint16_t fDegrees);
void setSpeed(uint8_t motor, uint16_t rpm);
void setGoalPosition(uint8_t motor, uint16_t degrees);

int main() {
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

	//The loop breaks when all arms have done their movements.
	while (1) {
		//Increments when the flag of an instruction is set.
		//Later the loop will break if this is equal to the amount of instructions.
		uint8_t flags = 0;
		//Go through all instructions.
		for (int i = 0; i < inss_length; ++i) {
			instruction_t *ins = inss[i];
			//Has instruction been dealt with?
			if (ins->flag) {
				++flags;
				continue;
			} else {
				uint8_t allowed = 1;
				//If instruction has not been dealt with, compare to all previous instructions to see if it is allowed to start its movement.
				for (int j = 0; j < i; ++j) {
					instruction_t *prev_ins = inss[j];
					if (prev_ins->flag) {
						continue;
					} else {
						if (strcmp(ins->from, prev_ins->from) == 0
							|| strcmp(ins->from, prev_ins->to) == 0
							|| strcmp(ins->to, prev_ins->from) == 0
							|| strcmp(ins->to, prev_ins->to) == 0) {
							allowed = 0;
							break;
						}
					}
				}

				printf("Arm %d in state %d is%s allowed.\n", ins->arm, ins->state, allowed ? "" : " not");

				if (!allowed) {
					continue;
				}

				uint8_t stateChangeComplete = 1;
				//Have all servo motors in the arm reached their goal position?
				for (int j = 0; j < 6; ++j) {
					uint8_t index = (ins->arm - 1) * 6 + j;
					if (abs(currentPositions[index] - goalPositions[index]) > 5) {
						//stateChangeComplete = 0;
					}
				}

				if (!stateChangeComplete) {
					continue;
				}

				//Based on state, tell uart process what kind of instruction packet to send.
				//For each stage give enough information so the uart process can construct the instruction packet.
				switch (ins->state) {
					case 0: //rotate
						//B, C, D, E, F: -
						//A: r1
						rotate(ins->arm, ins->r1);
					    break;
					case 1: //extend
						//A, E, F: -
						//B, C, D: 105, 105, 105
						stretch(ins->arm, 105, 105, 105);
					    break;
					case 2: //close
						//A, B, C, D: -
						//E, F: 130, 170
						claw(ins->arm, 130, 170);
					    break;
					case 3: //lift
						//A, B, C, E, F: -
						//D: 60
						wrist(ins->arm, 60);
					    break;
					case 4: //retract
						//A,E,F:-
						//b,c,d:195,60,60
						stretch(ins->arm, 195, 60, 60);
					    break;
					case 5: //rotate
						//b,c,d,e,f:-
						//a:r2
						rotate(ins->arm, ins->r2);
					    break;
					case 6: //extend
						//a,e,f:-
						//b,c,d:105,105,60
						stretch(ins->arm, 105, 105, 60);
					    break;
					case 7: //put
						//a,b,c,e,f:-
						//d:105
						wrist(ins->arm, 105);
					    break;
					case 8: //open
						//a,b,c,d:-
						//e,f:150
						claw(ins->arm, 150, 150);
					    break;
					case 9: //retract
						//a,e,f:-
						//b,c,d:195,60,60
						stretch(ins->arm, 195, 60, 60);
					    break;
					case 10: //rotate
						//b,c,d,e,f:-
						//a:150
						rotate(ins->arm, 150);
					    break;
					case 11:
						ins->flag = 1;
						continue;
					default: //Something went wrong.
						//Notify user.
						continue;
				}

				++ins->state;
			}
		}

		if (flags >= inss_length) {
			break;
		}
	}

	return 0;
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
