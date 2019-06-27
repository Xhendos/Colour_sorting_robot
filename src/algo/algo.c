#include "octo.h"
#include <stdlib.h>

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

#include <stdio.h>
#include <assert.h>
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

int dijkstra(int begin[], int index, int* next_move, size_t size);
int checkpos(int position[], int* round_position, size_t size);
int check_finished(int check_array[]);
int array_to_zero(int* array, size_t size);

int skip, MAXROUNDS = 0;

int main()
{
	int temp[MAX_BALL],next[MAX_BALL],i, check, round=0, pos_check =0, check1, loop, loop1;

	int begin[MAX_BALL] = {RED,GREEN,BLUE,WHITE};
	for(loop=0;loop<MAX_BALL;loop++)
	    next[loop] = begin[loop];
    /*
	printf("\nEnter the begin 1 position:");
	scanf("%d",&begin[0]);

	printf("\nEnter the begin 2 position:");
	scanf("%d",&begin[1]);

	printf("\nEnter the begin 3 position:");
	scanf("%d",&begin[2]);

	printf("\nEnter the begin 4 position:");
	scanf("%d",&begin[3]);
     */
	do
    {
        printf("\n-----------------------------------------This is round: %d----------------------------------------------\n", round);
        for(loop1=0;loop<MAX_BALL;loop1++)
            printf("%d ",next[loop1]);
	    check = check_finished(begin);
	    pos_check = checkpos(begin, temp, sizeof(temp) / sizeof(temp[0]));
        if(pos_check == 1) {
            for (i = 0; i < MAX_BALL; i++) {
                check1 = dijkstra(begin, i, next, sizeof(next) / sizeof(next[0]));
                if(check1 != 100)
                    begin[i] = check1;
            }
        }
        else if(pos_check== 0)
        {
            for (i = 0; i < MAX_BALL; i++) {
                //printf("temp[%d] is: %d\n", i, temp[i]);
                check1 = dijkstra(temp, i, next, sizeof(next) / sizeof(next[0]));
                if(check1 == 99)
                    begin[i] = check1;
            }
        }
        round++;
        //for(loop=0;loop<MAX_BALL;loop++)
        //    next[loop] = begin[loop];
       //array_to_zero(next, sizeof(next) / sizeof(next[0]));

       if(skip!=0)
        MAXROUNDS++;
    
    }while(check != 1 && MAXROUNDS<=5);

	return 0;
}

int array_to_zero(int* array, size_t size)
{
    int i;
    assert(size >= 4);
    for(i=0;i<MAX_BALL;i++)
    {
        array[i] = 0;
    }
    return 0;
}

int check_finished(int check_array[])
{
    int checked=0,i;
    for(i=0;i<MAX_BALL; i++)
    {
        if(check_array[i] == 99)
            checked = 1;
        else
            return 0;
    }
    return checked;
}

int checkpos(int position[], int* round_position, size_t size)
{
    int i,k, temp, check=1;
    //assert(size >= 4);
    for(i=0; i<MAX_BALL; i++)
    {
        temp = position[i];
        //printf("temp =%d\n",temp);
        for(k=0;k<MAX_BALL;k++)
        {
            if(temp==position[k] && (!k) == i && temp != 99 )
            {
                round_position[k] = position[k];
               // printf("temp is %d, position = %d, k = %d, i = %d\n", temp, position[k], k, i);
                check = 0;
            }
            //else if(temp != position[k] &&! k == i && temp != 99)
                //round_position[k] = 100;
        }
    }
    return check;
}

int dijkstra(int begin[], int index, int* next_move, size_t size)
{
    int check_position;
    assert(size >= 4);
    if(begin[index] == 100)
        return 0;


    int G[MAX][MAX] =  {{0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},   //t1
                        {1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},   //t2
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

	int cost[MAX][MAX],distance[MAX],pred[MAX], end[MAX_BALL]={RED_END,GREEN_END,BLUE_END,WHITE_END},n = MAX;
	int visited[MAX],count,mindistance,nextnode,i,j,k, previous, begin_value, end_value, temp;
    /*for(k=0;k<MAX_BALL;k++)
	{
	    end_value=end[k];
	    begin_value=begin[k];

    */
    end_value = end[index];
    begin_value = begin[index];
	//pred[] stores the predecessor of each node
	//count gives the number of nodes seen so far
	//create the cost matrix
	for(i=0;i<n;i++)
		for(j=0;j<n;j++)
			if(G[i][j]==0)
				cost[i][j]=INFINITY;
			else
				cost[i][j]=G[i][j];


	    //initialize pred[],distance[] and visited[]
	    for(i=0;i<n;i++)
	    {
		    distance[i]=cost[end_value][i];
		    pred[i]= end_value;
		    visited[i]=0;
	    }

	    distance[end_value]=0;
	    visited[end_value]=1;
	    count=1;

	    while(count<n-1)
	    {
		    mindistance=INFINITY;

		    //nextnode gives the node at minimum distance
	    	for(i=0;i<n;i++)
		    	if(distance[i]<mindistance&&!visited[i])
			    {
				    mindistance=distance[i];
				    nextnode=i;
			    }

			    //check if a better path exists through nextnode
			    visited[nextnode]=1;
			    for(i=0;i<n;i++)
				    if(!visited[i])
					    if(mindistance+cost[nextnode][i]<distance[i])
					    {
						    distance[i]=mindistance+cost[nextnode][i];
						    pred[i]=nextnode;
					    }
    		count++;
	    }

	    //print the path and distance of each node

        for(i=0;i<n;i++)
	        if(i!=end_value)
	        {
	            if(i == begin_value)
	            {
	                previous = i;
	                if(i>=8)
	                    printf("\nDistance of node f%d to f%d = %d\n",i-7, end_value-7,distance[i]);
		            else
		                printf("\nDistance of node t%d to f%d = %d\n",i+1, end_value-7,distance[i]);

	                 j=i;
			            j=pred[j];

                    temp = next_move[index];
                    next_move[index] = j;
                    for(k=0; k<MAX_BALL; k++)
                    {
                        //printf("\nnextmove [%d] is: %d, nextmove k: %d is: %d",index, next_move[index], k, next_move[k]);
                        if (j == next_move[k] && k != index) {
                            printf("Next round ");
                            skip++;
                            next_move[index] = temp;
                            return 100;
                        }
                    }


                    check_position = checkpos(begin, next_move,sizeof(next_move) / sizeof(next_move[0]));
                    if(check_position == 0)
                        return 100;


                   printf("Path =");
			            if(j >= 8 && j < 12)
			            {
			                if(previous >= 8 && previous < 12)
			                {
			                    printf("f%df%d \n",previous-7, j-7);

			                }
			                else
			                {
			                    printf("t%df%d \n",previous+1, j-7);
			                }
			            }
			            else if(previous >= 8 && previous < 12)
			            {
			                printf("f%dt%d \n",previous-7, j+1);
			            }
			            else
			            {
			                printf("t%dt%d \n", previous+1, j+1);
			            }
			        //begin[k]=j;
			        //dijkstra(G,n,end, begin);
		           //}
	               //while(j!=end_value);
	              if(j!=end_value)
	               {
	                    return j;
	                    //dijkstra(begin, index);
	               }
	                else return 99;
	        }

	}
	return 99;
}
