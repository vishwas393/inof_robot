#include <bits/stdc++.h>
#include "../include/inof_robot/a_star_search_algo.h"

#define R 36
#define C 33


/***********************************  CLASS DEFINITION  *******************************
class node {
	public:
	node(void){}
	node(int a, int b) { coords.first = a; coords.second = b; }
	std::pair<int, int> coords;
	std::pair<int, int> p_coords;	//Parent's coords
	void operator= (const node& T) { 
		coords.first = T.coords.first;
		coords.second = T.coords.second;
	}

	friend bool operator== (const node& T1, const node& T2); 
};

class graph {
	public:
	graph(int m[][C], node& a, node& b) { 
		src = a; 
		dest = b;
		for (int i=0; i<R; i++) {
			for(int j=0; j<C; j++) {
				map[i][j] = m[i][j];
				}
		}
	}
	
	int map[R][C];
	node src, dest;
	std::vector<node> OPEN;
	std::vector<node> CLOSED;
	std::vector<node> PATH;
	int past_cost[R][C];

	
	bool initialize_search_params(void);
	bool start_search(void);
	bool is_not_obstacle(node a);
	void print_path(void);

};*/


/***********************************  UTILITIES  *************************************/
bool operator== (const node &T1, const node &T2)
{
	if(T1.coords.first == T2.coords.first && T1.coords.second == T2.coords.second )
		return true;
	else
		return false;
}


static float euclidean_cost(node c, node n)
{
	float dist = sqrt(pow((c.coords.first - n.coords.first), 2) + pow((c.coords.second - n.coords.second), 2));
	return dist;
}


bool graph::is_not_obstacle(node a)
{
	if(map[a.coords.first][a.coords.second] == 1 && a.coords.first>=0 && a.coords.second>=0 && a.coords.first<R && a.coords.second<C)
		return true;
	else;
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

/***********************************************************************************/
void graph::print_path(void)
{
	std::cout << "Source" << std::setw(7) << "(" << src.coords.first << "," << src.coords.second << ")" << std::endl;
	std::cout << "Destination" << std::setw(2) << "(" << dest.coords.first << "," << dest.coords.second << ")" << std::endl;
	std::cout << "Total nodes" << std::setw(2) <<PATH.size() << std::endl;
	for(int i=0; i<PATH.size(); i++)
	{
		std::cout << "(" << PATH.at(i).coords.first << "," << PATH.at(i).coords.second << ")" << std::endl;
	}
}


bool graph::initialize_search_params(void)
{
	if(is_not_obstacle(src) && is_not_obstacle(dest))
	{
		OPEN.push_back(src);
		PATH.push_back(src);
		//std::fill(past_cost.begin(), past_cost.end(), 1000);
		memset(past_cost, 1, sizeof(int)*R*C);
		past_cost[src.coords.first][src.coords.second] = 0;
		return true;
	}
	std::cout << "Wrong Destination or Source Provided!" << std::endl;;
	return false;
}


bool graph::start_search(void)
{
	int cnt=0;
	while(!OPEN.empty())
	{
		float tentative_past_cost;
		node curr;

		curr = OPEN.front();
		OPEN.erase(OPEN.end());
		CLOSED.push_back(curr);
		
		

		if(curr == dest)
		{
			return true;
		}
		
		
		std::vector<node> nbr_lst = return_all_neighbours(curr);
		std::vector<float> est_total_cost;
		//std::fill(est_total_cost.begin(), est_total_cost.end(), 1000);
		std::vector<node>::iterator it;
		for(std::vector<node>::const_iterator itr = nbr_lst.begin(); itr != nbr_lst.end(); itr++)
		{
			node nbr = *itr;
			float etc = 100;
			//std::cout << std::endl << nbr.coords.first << " " << nbr.coords.second;
			it = std::find(CLOSED.begin(), CLOSED.end(), nbr); 
			if(it == CLOSED.end() && is_not_obstacle(nbr))
			{
				tentative_past_cost = past_cost[curr.coords.first][curr.coords.second] + euclidean_cost(curr,nbr);
				//std::cout << " TPC: " <<tentative_past_cost << " PC: " << past_cost[nbr.coords.first][nbr.coords.second];
				if(tentative_past_cost < past_cost[nbr.coords.first][nbr.coords.second])
				{
					past_cost[nbr.coords.first][nbr.coords.second] = tentative_past_cost;
					nbr.p_coords = curr.coords;
					etc = past_cost[nbr.coords.first][nbr.coords.second] + euclidean_cost(dest, nbr);
					//std::cout << " etc: " << etc;
				}
			}
			est_total_cost.push_back(etc);
		}
		
		
		float tmp = *std::min_element(est_total_cost.begin(), est_total_cost.end());
		if(tmp != 100){
			int idx = std::find(est_total_cost.begin(), est_total_cost.end(), tmp) - est_total_cost.begin();
			//std::cout << nbr_lst.at(idx).coords.first << " " <<nbr_lst.at(idx).coords.second << std::endl;
			OPEN.push_back(nbr_lst.at(idx));
			PATH.push_back(nbr_lst.at(idx));
			//std::cout << std::endl << nbr_lst.at(idx).coords.first << ", " << nbr_lst.at(idx).coords.second << std::endl;
		}
	}
	return false;
}



std::vector<node> path_planning_fn(std::pair<int, int> si, std::pair<int, int> di)
{
	std::vector<node> final_path;
	int map_ext[R][C] = 
{{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, }};

	node d(di.first, di.second);
	node s(si.first, si.second);

	graph astar_graph(map_ext, s, d); 
	if(astar_graph.initialize_search_params())
	{
		if(astar_graph.start_search()) {
			final_path = astar_graph.PATH;
		}
		else {
			std::cout << "No Path possible to reach to the destination!" << std::endl;
		}
	}
	return final_path;
}
