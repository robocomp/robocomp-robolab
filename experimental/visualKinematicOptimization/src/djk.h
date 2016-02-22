#include <vector>
#include <iostream>

#define DJ_INFINITY 9999999.

using namespace std;

class Dijkstra
{
public:
	Dijkstra(std::vector< std::vector< float > > *adjMatrix_)
	{
		adjMatrix = adjMatrix_;
		numOfVertices = adjMatrix->size();
		distance.resize(numOfVertices);
		predecessor.resize(numOfVertices);
		mark.resize(numOfVertices);
	}

	std::vector< std::vector< float > > *adjMatrix;
	std::vector< float > predecessor, distance;
	std::vector< bool > mark;
	int source;
	int numOfVertices;

	void initialize()
	{
		for(int i=0;i<numOfVertices;i++)
		{
			mark[i] = false;
			predecessor[i] = -1;
			distance[i] = DJ_INFINITY;
		}
		distance[source]= 0;
	}

	float go(int node, std::vector <int> &path)
	{
		if (node == source)
		{
			path.push_back(node);
		}
		else if(predecessor[node] == -1)
		{
			return -1;
		}
		else
		{
			go(predecessor[node], path);
			path.push_back(node);
		}
		return distance[node];
	}

	void printPath(int node)
	{
		if(node == source)
		{
			cout<< node << " ";
		}
		else if(predecessor[node] == -1)
			cout<<"No path from “<<source<<”to "<< node <<endl;
		else
		{
			printPath(predecessor[node]);
			cout << node <<" ";
		}
	}



	int getClosestUnmarkedNode()
	{
		int minDistance = DJ_INFINITY;
		int closestUnmarkedNode;
		for(int i=0;i<numOfVertices;i++)
		{
			if((!mark[i]) && ( minDistance >= distance[i]))
			{
				minDistance = distance[i];
				closestUnmarkedNode = i;
			}
		}
		return closestUnmarkedNode;
	}


	void calculateDistance(int source_)
	{
		source = source_;
		initialize();
		int closestUnmarkedNode;
		int count = 0;
		while(count < numOfVertices)
		{
			closestUnmarkedNode = getClosestUnmarkedNode();
			mark[closestUnmarkedNode] = true;
			for(int i=0;i<numOfVertices;i++)
			{
				if((!mark[i]) && ((*adjMatrix)[closestUnmarkedNode][i]>0) )
				{
					if(distance[i] > distance[closestUnmarkedNode]+(*adjMatrix)[closestUnmarkedNode][i])
					{
						distance[i] = distance[closestUnmarkedNode]+(*adjMatrix)[closestUnmarkedNode][i];
						predecessor[i] = closestUnmarkedNode;
					}
				}
			}
			count++;
		}
	}



	void output()
	{
		for(int i=0;i<numOfVertices;i++)
		{
			if(i == source)
				cout<< source <<" .. "<<source;
			else
				printPath(i);
			cout<<" -> "<<distance[i]<<endl;
		}
	}


};
