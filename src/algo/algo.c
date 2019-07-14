#include <stdlib.h>
#include <stdint.h>
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
    unsigned char n;
} Arm_t;

static int lDijkstra( ePlaceholder * peBegin, int lIndex, int * plNextMove, size_t xSize, ePlaceholder * peEnd );
static int lCheckPos( ePlaceholder * pePosition, int * plRoundPosition, size_t xSize );
static int lCheckFinished( ePlaceholder * peCheckArray );
static void vEdgeToDisplaceInformation( ePlaceholder ePlaceholderFrom, ePlaceholder ePlaceholderTo, DisplaceInformation_t * pxDisplaceInformation );

static int skip, MAXROUNDS = 0;
static unsigned short ucDisplaceInformationIndex;
static DisplaceInformation_t *  pxDisplaceInformation;

unsigned short usAlgorithmEntryPoint( ePlaceholder ePlaceholdersFrom[4], ePlaceholder ePlaceholdersTo[4], DisplaceInformation_t pxDisplaceInformations[64] )
{
    int temp[MAX_BALL];
    int next[MAX_BALL];
    int i;
    int check;
    int round = 0;
    int pos_check = 0;
    int check1;
    int loop;

    ucDisplaceInformationIndex = 0;
    pxDisplaceInformation = pxDisplaceInformations;

    for(loop = 0; loop < MAX_BALL; loop++)
    {
        next[loop] = ePlaceholdersFrom[loop];
    }

    do
    {
        check = lCheckFinished(ePlaceholdersFrom);
        pos_check = lCheckPos(ePlaceholdersFrom, temp, sizeof(temp) / sizeof(temp[0]));
        if(pos_check == 1)
        {
            for (i = 0; i < MAX_BALL; i++)
            {
                check1 = lDijkstra(ePlaceholdersFrom, i, next, sizeof(next) / sizeof(next[0]), ePlaceholdersTo);
                if(check1 != 100)
                {
                    ePlaceholdersFrom[i] = check1;
                }
            }
        }
        else if(pos_check== 0)
        {
            for(i = 0; i < MAX_BALL; i++)
            {
                check1 = lDijkstra((ePlaceholder *)temp, i, next, sizeof(next) / sizeof(next[0]), ePlaceholdersTo);
                if(check1 == 99)
                {
                    ePlaceholdersFrom[i] = check1;
                }
            }
        }

        round++;

        if(skip!=0)
        {
            MAXROUNDS++;
        }
    } while(check != 1 && MAXROUNDS <= 5);

    return ucDisplaceInformationIndex;
}

static int lCheckFinished(ePlaceholder * eCheckArray)
{
    int checked = 0;
    int i;

    for(i = 0; i < MAX_BALL; i++)
    {
        if (eCheckArray[i] == 99)
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

int lCheckPos( ePlaceholder * pePosition, int * plRoundPosition, size_t xSize)
{
    int i;
    int k;
    int temp;
    int check = 1;

    for(i = 0; i < MAX_BALL; i++)
    {
        temp = pePosition[i];
        for(k = 0; k < MAX_BALL; k++)
        {
            if(temp == pePosition[k] && (!k) == i && temp != 99)
            {
                plRoundPosition[k] = pePosition[k];
                check = 0;
            }
        }
    }

    return check;
}

int lDijkstra( ePlaceholder * peBegin, int lIndex, int * plNextMove, size_t xSize, ePlaceholder * peEnd)
{
    int check_position;
    if(peBegin[lIndex] == 100)
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

    end_value = peEnd[lIndex];
    begin_value = peBegin[lIndex];

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

        //Remove unused warning.
        nextnode = -1;

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

                temp = plNextMove[lIndex];
                plNextMove[lIndex] = j;

                for(k = 0; k < MAX_BALL; k++)
                {
                    if (j == plNextMove[k] && k != lIndex)
                    {
                        skip++;
                        plNextMove[lIndex] = temp;
                        return 100;
                    }
                }

                check_position = lCheckPos(peBegin, plNextMove, 0);
                if(check_position == 0)
                {
                    return 100;
                }

                vEdgeToDisplaceInformation(previous, j, &pxDisplaceInformation[ucDisplaceInformationIndex++]);

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

void vEdgeToDisplaceInformation( ePlaceholder ePlaceholderFrom, ePlaceholder ePlaceholderTo, DisplaceInformation_t * pxDisplaceInformation )
{
    Arm_t *arms[8][8];

    Arm_t arm0 = {NORTH, octoA0};
    Arm_t arm1 = {EAST, octoA1};
    Arm_t arm2 = {EAST, octoA2};
    Arm_t arm3 = {SOUTH, octoA3};
    Arm_t arm4 = {SOUTH, octoA4};
    Arm_t arm5 = {WEST, octoA5};
    Arm_t arm6 = {WEST, octoA6};
    Arm_t arm7 = {NORTH, octoA7};

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

    uint8_t ta = ePlaceholderFrom;
    uint8_t tb = ePlaceholderTo;
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
        return;
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

    pxDisplaceInformation->ePlaceholderFrom = ePlaceholderFrom;
    pxDisplaceInformation->ePlaceholderTo = ePlaceholderTo;
    pxDisplaceInformation->ucArm = arm->n;
    pxDisplaceInformation->usFirstRotationInDegrees = r1;
    pxDisplaceInformation->usSecondRotationInDegrees = r2;
}

