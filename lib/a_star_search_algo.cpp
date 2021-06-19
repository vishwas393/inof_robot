#include <bits/stdc++.h>
#include "../include/inof_robot/a_star_search_algo.h"

#define R 30
#define C 52


/***********************************  UTILITIES  *************************************/
bool operator== (const node &T1, const node &T2)
{
	if(T1.coords.first == T2.coords.first && T1.coords.second == T2.coords.second )
		return true;
	else
		return false;
}


static double heuristic_cost(node c, node n)
{
	double dist = sqrt(pow((c.coords.first - n.coords.first), 2) + pow((c.coords.second - n.coords.second), 2));
	return dist;
}

/*static double edge_cost(node c, node n)
{
	return ( std::abs(c.coords.first-n.coords.first) + std::abs(c.coords.second-n.coords.second) );
}*/


bool graph::is_not_obstacle(node a)
{
	if(map[a.coords.first][a.coords.second] == 1 && a.coords.first>=0 && a.coords.second>=0 && a.coords.first<R && a.coords.second<C)
		return true;
	else
		return false;
}


static std::vector<node> return_all_neighbours(node c)
{
	std::vector<node> n;
	node n1(c.coords.first-1, c.coords.second);	//Upper
	n.push_back(n1);
	
	node n2(c.coords.first+1, c.coords.second);	//Lower
	n.push_back(n2);
	
	node n3(c.coords.first,   c.coords.second+1);	//Right
	n.push_back(n3);
	
	node n4(c.coords.first,   c.coords.second-1);	//Left
	n.push_back(n4);
	
	node n5(c.coords.first-1, c.coords.second+1);	//Upper-Right
	n.push_back(n5);
	
	node n6(c.coords.first-1, c.coords.second-1);	//Upper-Left
	n.push_back(n6);
	
	node n7(c.coords.first+1, c.coords.second+1);	//Upper-Right
	n.push_back(n7);
	
	node n8(c.coords.first+1, c.coords.second-1);	//Lower-Left
	n.push_back(n8);

	return n;
}


bool least_cost_comp(node a, node b)
{
	return (a.est_t_cost < b.est_t_cost);
}

/***********************************************************************************/
void graph::prepare_path(void)
{
	std::cout << "Source" << std::setw(7) << "(" << src.coords.first << "," << src.coords.second << ")" << std::endl;
	std::cout << "Destination" << std::setw(2) << "(" << dest.coords.first << "," << dest.coords.second << ")" << std::endl;
	
	node* temp = &nodes[CLOSED.at(CLOSED.size()-1).coords.first][CLOSED.at(CLOSED.size()-1).coords.second];
	
	while(temp->parent != NULL)
	{
		//std::cout << "(" << temp->coords.first << "," << temp->coords.second << ")" << std::endl;
		final_path.push_back(*temp);
		temp = temp->parent; 
	}
	//std::cout << "(" << temp->coords.first << "," << temp->coords.second << ")" << std::endl;
	final_path.push_back(*temp);

	std::reverse(final_path.begin(), final_path.end());
}


bool graph::initialize_search_params(void)
{
	if(is_not_obstacle(src) && is_not_obstacle(dest))
	{
		for(int i=0; i<R; i++)
		{
			for(int j=0; j<C; j++)
			{
				node a(i,j);
				nodes[i][j] = a;
			}
		}

		nodes[src.coords.first][src.coords.second].parent = NULL;
		OPEN.push_back(nodes[src.coords.first][src.coords.second]);
		
		return true;
	}
	else
		return false;
}


bool graph::start_search(void)
{
	while(OPEN.size() != 0)
	{
		node curr = OPEN.at(0);

		OPEN.erase(OPEN.begin());
		CLOSED.push_back(curr);
		
		//std::cout << curr.coords.first << "," << curr.coords.second << "," << std::endl; 

		if(curr == dest) { 
			return true;
		}

		std::vector<node> nbr_nodes = return_all_neighbours(curr);

		for(std::vector<node>::iterator itr=nbr_nodes.begin(); itr!=nbr_nodes.end(); itr++)
		{
			
			if( std::find(CLOSED.begin(), CLOSED.end(), *itr) == CLOSED.end() && is_not_obstacle(*itr) )
			{
				double tentative_past_cost = past_cost[curr.coords.first][curr.coords.second] + heuristic_cost(curr, *itr);
				if(tentative_past_cost < past_cost[itr->coords.first][itr->coords.second])
				{
					past_cost[itr->coords.first][itr->coords.second] = tentative_past_cost;
					nodes[itr->coords.first][itr->coords.second].est_t_cost = past_cost[itr->coords.first][itr->coords.second] + heuristic_cost(dest, *itr);
					nodes[itr->coords.first][itr->coords.second].p_coords.first = curr.coords.first;
					nodes[itr->coords.first][itr->coords.second].p_coords.second = curr.coords.second;

					nodes[itr->coords.first][itr->coords.second].parent = &nodes[curr.coords.first][curr.coords.second];
			
					OPEN.push_back(nodes[itr->coords.first][itr->coords.second]);
					std::sort(OPEN.begin(), OPEN.end(), least_cost_comp);

				}
			}
		}
	}


	return false;
}



std::vector<node> path_planning_fn(std::pair<int, int> si, std::pair<int, int> di)
{
	int map_ext[R][C] = 
	{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, }};
 
 	node d(di.first, di.second);
	node s(si.first, si.second);

	graph astar_graph(map_ext, s, d); 
	if(astar_graph.initialize_search_params())
	{
		if(astar_graph.start_search()) {
			astar_graph.prepare_path();
		}
		else {
			std::cout << "No Path possible to reach to the destination!" << std::endl;
		}
	}
	else
	{
			std::cout << "Invalid source or destination!" << std::endl;
	}

	return astar_graph.final_path;
}
