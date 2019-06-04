#include <stdio.h>
#define INFINITY 9999
#define MAX 12
 
void dijkstra(int G[MAX][MAX],int n,int startnode, int end);
 
int main()
{
	int G[MAX][MAX] =  { {0, 2, 0, 0, 0, 0, 0, 2, 3, 0, 0, 0},
                        {2, 0, 2, 0, 0, 0, 0, 0, 1, 1, 0, 0},
                        {0, 2, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0},
                        {0, 0, 2, 0, 2, 0, 0, 0, 0, 1, 1, 0},
                        {0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 3, 0},
                        {0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 1, 1},
                        {0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 3},
                        {2, 0, 0, 0, 0, 0, 2, 0, 1, 0, 0, 1},
                        {3, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
                        {0, 1, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 1, 3, 1, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 1, 3, 1, 0, 0, 0, 0}};
	                    
	int i,j,n = 12,u=0, end_pos;

	printf("\nEnter the begin position:");
	scanf("%d",&u);
	
	printf("\nEnter the end position:");
	scanf("%d",&end_pos);
	
	dijkstra(G,n,end_pos, u);
	
	return 0;
}
 
void dijkstra(int G[MAX][MAX],int n,int startnode, int end)
{
 
	int cost[MAX][MAX],distance[MAX],pred[MAX];
	int visited[MAX],count,mindistance,nextnode,i,j, previous;
	
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
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}
	
	distance[startnode]=0;
	visited[startnode]=1;
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
		if(i!=startnode)
		{
		    if(i == end)
		    {
		        previous = i;
		        printf("\nDistance of node %d= %d",i,distance[i]);
			    //printf("\nPath= %d",i);
			    printf("\nPath =");
			    
			    j=i;
			    do
			    {
				    j=pred[j];
				    //printf("<-%d",j);
				    if(j >= 8 && j < 12)
				    {
				        if(previous >= 8 && previous < 12)
				        {
				            printf(" f%df%d",previous-7, j-7);
				        
				        }
				        else
				        {
				            printf(" t%df%d",previous+1, j-7);
				        }
				    }
				    else if(previous >= 8 && previous < 12)
				    {
				            printf(" f%dt%d",previous-7, j+1);
				        
				    }
				    else
			        {
			            printf(" t%dt%d",previous+1, j+1);
			        }
				    previous = j;
			    }while(j!=startnode);
		    }
		    

	}
}