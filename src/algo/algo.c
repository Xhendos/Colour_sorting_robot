#include <stdlib.h>
#include <assert.h>
#include "algo.h"
#include "octo.h"

#define INFINITY 9999
#define MAX 12
#define MAX_BALL 4

#define RED 4
#define GREEN 6
#define BLUE 7
#define WHITE 1

#define RED_END 8
#define GREEN_END 9
#define BLUE_END 10
#define WHITE_END 11

typedef enum {
    HORIZONTAL,
    VERTICAL,
    DIAGONAL,
} direction_t;

typedef enum {
    NORTH,
    EAST,
    SOUTH,
    WEST,
} cardinal_t;

typedef struct {
    uint8_t x;
    uint8_t y;
} point_t;

typedef struct {
    cardinal_t cardinal;
    arm_t n;
} Arm_t;

void mmaaiinn(int begin[4], int end[4]);
static int dijkstra(int begin[], int index, int* next_move, size_t size, int end[]);
static int checkpos(int position[], int* round_position, size_t size);
static int check_finished(int check_array[]);

static int skip, MAXROUNDS = 0;

void mmaaiinn(int begin[4], int end[4])
{
	int temp[MAX_BALL];
    int next[MAX_BALL];
    int i;
    int check;
    int round = 0;
    int pos_check = 0;
    int check1;
    int loop;

	for(loop = 0; loop < MAX_BALL; loop++)
    {
	    next[loop] = begin[loop];
    }

    do
    {
	    check = check_finished(begin);
	    pos_check = checkpos(begin, temp, sizeof(temp) / sizeof(temp[0]));
        if(pos_check == 1)
        {
            for (i = 0; i < MAX_BALL; i++)
            {
                check1 = dijkstra(begin, i, next, sizeof(next) / sizeof(next[0]), end);
                if(check1 != 100)
                {
                    begin[i] = check1;
                }
            }
        }
        else if(pos_check== 0)
        {
            for(i = 0; i < MAX_BALL; i++)
            {
                check1 = dijkstra(temp, i, next, sizeof(next) / sizeof(next[0]), end);
                if(check1 == 99)
                {
                    begin[i] = check1;
                }
            }
        }

        round++;

        if(skip!=0)
        {
            MAXROUNDS++;
        }
    } while(check != 1 && MAXROUNDS <= 5);
}

int check_finished(int check_array[])
{
    int checked = 0;
    int i;

    for(i = 0; i < MAX_BALL; i++)
    {
        if (check_array[i] == 99)
        {
            checked = 1;
        }
        else
        {
            return 0;
        }
    }

    return checked;
}

int checkpos(int position[], int* round_position, size_t size)
{
    int i;
    int k;
    int temp;
    int check = 1;

    for(i = 0; i < MAX_BALL; i++)
    {
        temp = position[i];
        for(k = 0; k < MAX_BALL; k++)
        {
            if(temp == position[k] && (!k) == i && temp != 99)
            {
                round_position[k] = position[k];
                check = 0;
            }
        }
    }

    return check;
}

int dijkstra(int begin[], int index, int* next_move, size_t size, int end[])
{
    int check_position;
    assert(size >= 4);
    if(begin[index] == 100)
    {
        return 0;
    }

    int G[MAX][MAX] =  {{0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},   //t1
                        {2, 0, 1, 0, 0, 0, 0, 0, 2, 1, 0, 0},   //t2
                        {0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0},   //t3
                        {0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0},   //t4
                        {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0},   //t5
                        {0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1},   //t6
                        {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1},   //t7
                        {1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1},   //t8
                        {1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},   //f1
                        {0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},   //f2
                        {0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},   //f3
                        {0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0}};  //f4

	int cost[MAX][MAX];
    int distance[MAX];
    int pred[MAX];
    int n = MAX;
	int visited[MAX];
    int count;
    int mindistance;
    int nextnode;
    int i;
    int j;
    int k;
    int previous;
    int begin_value;
    int end_value;
    int temp;

    end_value = end[index];
    begin_value = begin[index];

	//pred[] stores the predecessor of each node
	//count gives the number of nodes seen so far
	//create the cost matrix
	for(i = 0; i < n; i++)
    {
		for(j = 0; j < n; j++)
        {
			if(G[i][j] == 0)
            {
				cost[i][j] = INFINITY;
            }
			else
            {
				cost[i][j] = G[i][j];
            }
        }
    }

    //initialize pred[],distance[] and visited[]
	for(i = 0; i < n; i++)
	{
	    distance[i] = cost[end_value][i];
	    pred[i] = end_value;
	    visited[i] = 0;
	}

	distance[end_value] = 0;
	visited[end_value] = 1;
	count = 1;

	while(count < n - 1)
	{
	    mindistance = INFINITY;

	    //nextnode gives the node at minimum distance
		for(i = 0; i < n; i++)
        {
	        if(distance[i] < mindistance && !visited[i])
		    {
			    mindistance = distance[i];
			    nextnode = i;
		    }
        }

		//check if a better path exists through nextnode
		visited[nextnode] = 1;
		for(i = 0; i < n; i++)
        {
		    if(!visited[i])
            {
			    if(mindistance + cost[nextnode][i] < distance[i])
			    {
				    distance[i] = mindistance + cost[nextnode][i];
				    pred[i] = nextnode;
			    }
            }
        }
        count++;
	}

	//the path and distance of each node
    for(i = 0; i < n; i++)
    {
	    if(i != end_value)
	    {
	        if(i == begin_value)
	        {
	            previous = i;

	            j = i;
		        j = pred[j];

                temp = next_move[index];
                next_move[index] = j;

                for(k = 0; k < MAX_BALL; k++)
                {
                    if (j == next_move[k] && k != index)
                    {
                        skip++;
                        next_move[index] = temp;
                        return 100;
                    }
                }

                check_position = checkpos(begin, next_move, 0);
                if(check_position == 0)
                {
                    return 100;
                }

                instruction_t instruction = edge_to_instruction((uint8_t)previous, (uint8_t)j);
                xQueueSend(armInstructionQueue, &instruction, portMAX_DELAY);

	            if(j != end_value)
	            {
	                return j;
	            }
	            else
                {
                    return 99;
                }
            }
	    }
	}

	return 99;
}

instruction_t edge_to_instruction(placeholders_t ta, placeholders_t tb)
{
    Arm_t *arms[8][8];

    Arm_t arm0 = {NORTH, ARM_0};
    Arm_t arm1 = {EAST, ARM_1};
    Arm_t arm2 = {EAST, ARM_2};
    Arm_t arm3 = {SOUTH, ARM_3};
    Arm_t arm4 = {SOUTH, ARM_4};
    Arm_t arm5 = {WEST, ARM_5};
    Arm_t arm6 = {WEST, ARM_6};
    Arm_t arm7 = {NORTH, ARM_7};

    arms[1][0] = &arm0;
    arms[0][1] = &arm1;
    arms[0][3] = &arm2;
    arms[1][4] = &arm3;
    arms[3][4] = &arm4;
    arms[4][3] = &arm5;
    arms[4][1] = &arm6;
    arms[3][0] = &arm7;

    point_t placeholders[12] = {
        {0, 0},
        {2, 0},
        {4, 0},
        {4, 2},
        {4, 4},
        {2, 4},
        {0, 4},
        {0, 2},
        {1, 1},
        {3, 1},
        {3, 3},
        {1, 3},
    };

    int8_t dx = placeholders[ta].x - placeholders[tb].x;
    int8_t dy = placeholders[ta].y - placeholders[tb].y;
    direction_t direction;

    if (abs(dx) == 2)
    {
        direction = HORIZONTAL;
    }
    else if (abs(dy) == 2)
    {
        direction = VERTICAL;
    }
    else if (abs(dx) + abs(dy) == 2)
    {
        direction = DIAGONAL;
    }
    else
    {
        instruction_t instruction = {1, 0, 0, 0, 255, 0, 0};
        return instruction;
    }

    uint8_t tax = placeholders[ta].x;
    uint8_t tay = placeholders[ta].y;
    uint8_t tbx = placeholders[tb].x;
    uint8_t tby = placeholders[tb].y;

    Arm_t *arm;
    uint8_t x;
    uint8_t y;
    uint16_t r1;
    uint16_t r2;

    if (direction == HORIZONTAL)
    {
        if (tax > tbx)
        {
            x = tax - 1;
            y = tay;
            arm = arms[x][y];
            if (arm->cardinal == NORTH)
            {
                r1 = 60;
                r2 = 240;
            }
            else if (arm->cardinal == SOUTH)
            {
                r1 = 240;
                r2 = 60;
            }
        }
        else if (tax < tbx)
        {
            x = tax + 1;
            y = tay;
            arm = arms[x][y];
            if (arm->cardinal == NORTH)
            {
                r1 = 240;
                r2 = 60;
            }
            else if (arm->cardinal == SOUTH)
            {
                r1 = 60;
                r2 = 240;
            }
        }
    }
    else if (direction == VERTICAL)
    {
        if (tay > tby)
        {
            x = tax;
            y = tay - 1;
            arm = arms[x][y];
            if (arm->cardinal == EAST)
            {
                r1 = 240;
                r2 = 60;
            }
            else if (arm->cardinal == WEST)
            {
                r1 = 60;
                r2 = 240;
            }
        }
        else if (tay < tby)
        {
            x = tax;
            y = tay + 1;
            arm = arms[x][y];
            if (arm->cardinal == EAST)
            {
                r1 = 60;
                r2 = 240;
            }
            else if (arm->cardinal == WEST)
            {
                r1 = 240;
                r2 = 60;
            }
        }
    }
    else if (direction == DIAGONAL)
    {
        point_t p1 = {tbx + dx, tby};
        point_t p2 = {tbx, tby + dy};

        uint8_t armx;
        uint8_t army;

        armx = p1.x;
        army = p1.y;

        arm = arms[armx][army];

        if (arm == NULL)
        {
            armx = p2.x;
            army = p2.y;
            arm = arms[armx][army];
        }

        if (arm->cardinal == NORTH)
        {
            if (armx > tax)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (army < tay)
            {
                r1 = 150;
                if (armx < tbx)
                {
                    r2 = 60;
                }
                else
                {
                    r2 = 240;
                }
            }
            else if (armx < tax)
            {
                r1 = 60;
                r2 = 150;
            }
        }
        else if (arm->cardinal == EAST)
        {
            if (army < tay)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (armx < tax)
            {
                r1 = 150;
                if (army < tby)
                {
                    r2 = 240;
                }
                else
                {
                    r2 = 60;
                }
            }
            else if (army > tay)
            {
                r1 = 60;
                r2 = 150;
            }
        }
        else if (arm->cardinal == SOUTH)
        {
            if (armx < tax)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (army > tay)
            {
                r1 = 150;
                if (armx < tbx)
                {
                    r2 = 240;
                }
                else
                {
                    r2 = 60;
                }
            }
            else if (armx > tax)
            {
                r1 = 60;
                r2 = 150;
            }
        }
        else if (arm->cardinal == WEST)
        {
            if (army > tay)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (armx > tax)
            {
                r1 = 150;
                if (army < tby)
                {
                    r2 = 60;
                }
                else
                {
                    r2 = 240;
                }
            }
            else if (army < tay)
            {
                r1 = 60;
                r2 = 150;
            }
        }
    }

    instruction_t instruction = {0, arm->n, r1, r2, 0, ta, tb};

    return instruction;
}

