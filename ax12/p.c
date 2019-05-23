#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef struct {
	uint8_t flag;
	uint8_t arm;
	uint16_t r1;
	uint16_t r2;
	uint8_t state;
	char *from;
	char *to;
} instruction_t;

int main() {
	instruction_t ins1 = {0, 1, 240, 60, 0, "t1", "t2"};
	instruction_t ins2 = {0, 2, 240, 150, 0, "t2", "f2"};
	instruction_t *inss[] = {&ins1, &ins2};
	uint8_t inss_length = sizeof(inss) / sizeof(instruction_t *);
	uint8_t beun_variable = 0;

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
						if (strcmp(prev_ins->to, ins->from) == 0 || strcmp(prev_ins->from, ins->from)) {
							allowed = 0;
							break;
						}
					}
				}
				printf("[%d] is%s allowed.\n", i, allowed ? "" : " not");
				if (!allowed) {
					continue;
				}
				//Based on state, tell uart process what kind of instruction packet to send.
				//For each stage give enough information so the uart process can construct the instruction packet.
				switch (ins->state) {
					case 0: //rotate
						//B, C, D, E, F: -
						//A: r1
					    break;
					case 1: //extend
						//A, E, F: -
						//B, C, D: 105, 105, 105
					    break;
					case 2: //close
						//A, B, C, D: -
						//E, F: 130, 170
					    break;
					case 3: //lift
						//A, B, C, E, F: -
						//D: 60
					    break;
					case 4: //retract
						//A,E,F:-
						//b,c,d:195,60,60
					    break;
					case 5: //rotate
						//b,c,d,e,f:-
						//a:r2
					    break;
					case 6: //extend
						//a,e,f:-
						//b,c,d:105,105,60
					    break;
					case 7: //put
						//a,b,c,e,f:-
						//d:105
					    break;
					case 8: //open
						//a,b,c,d:-
						//e,f:150
					    break;
					case 9: //retract
						//a,e,f:-
						//b,c,d:195,60,60
					    break;
					case 10: //rotate
						//b,c,d,e,f:-
						//a:150
					    break;
					default: //Something went wrong.
						//Notify user.
					    break;
				}
				//Go blocked and wait for uart to give back result. (wait until status packet has been received so the uart line is not busy anymore and a new instruction packet can be send.)
				//If last state has been reached, set its flag.
				printf("Arm %d rotated to %d and %d.\n", ins->arm, ins->r1, ins->r2);
				printf("Ball moved from %s to %s.\n", ins->from, ins->to);

				//DEBUG
				if (beun_variable >= 1) {
					ins->flag = 1;
				} else {
					++beun_variable;
				}
			}
		}

		if (flags >= inss_length) {
			break;
		}
	}

	printf("Sorting is done. Jolly good!\n");

	return 0;
}

