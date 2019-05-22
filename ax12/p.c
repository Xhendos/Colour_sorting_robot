#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct {
	uint8_t flag;
	uint8_t arm;
	uint16_t r1;
	uint16_t r2;
	uint8_t state;
} instruction_t;

int main() {
	instruction_t ins1 = {0, 1, 240, 60, 0};
	instruction_t ins2 = {0, 2, 240, 150, 0};
	
	instruction_t *inss[] = {&ins1, &ins2};
	uint8_t inss_length = sizeof(inss) / sizeof(instruction_t *);

	while (1) {
		uint8_t flags = 0;
		for (int i = 0; i < inss_length; ++i) {
			instruction_t *ins = inss[i];
			if (ins->flag) {
			    	++flags;
				continue;
			} else {
			    printf("Arm %d rotated to %d and %d.\n", ins->arm, ins->r1, ins->r2);
			    ins->flag = 1;
			}
		}

		if (flags >= inss_length) {
			break;
		}
	}

	printf("Sorting is done. Jolly good!\n");

	return 0;
}
