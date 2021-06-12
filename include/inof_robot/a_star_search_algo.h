#include <iostream>
#include <cstring>
#include <algorithm>
#include <list>
#include <utility>

#define R 30
#define C 52

class node {
	public:
	node(void){}
	node(int a, int b) { coords.first = a; coords.second = b; est_t_cost = 0;}
	std::pair<int, int> coords;
	std::pair<int, int> p_coords;	//Parent's coords
	void operator= (const node& T) { 
		coords.first = T.coords.first;
		coords.second = T.coords.second;
		p_coords.first = T.p_coords.first;
		p_coords.second = T.p_coords.second;
		est_t_cost = T.est_t_cost;
		parent = T.parent;
	}

	double est_t_cost;
	node *parent;
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
				past_cost[i][j] = 1000;
				}
		}
		past_cost[a.coords.first][a.coords.second] = 0;
	}
	
	int map[R][C];
	node nodes[R][C];
	node src, dest;
	std::vector<node> OPEN;
	std::vector<node> CLOSED;
	std::vector<node> final_path;
	double past_cost[R][C];

	
	bool initialize_search_params(void);
	bool start_search(void);
	bool is_not_obstacle(node a);
	void prepare_path(void);
};

std::vector<node> path_planning_fn(std::pair<int, int> si, std::pair<int, int> di);
