#ifndef MAPF_HPP
#define MAPF_HPP

#include <iostream>
#include <vector>
#include "graph.hpp"

namespace mapf_adapters{
	class mapf{
	public:
		mapf(int x_dim, int y_dim, std::vector<std::pair<int, int> > obstacles, std::vector<std::pair<int, int> > goals):
			x_dim(x_dim),
			y_dim(y_dim),
			obstacles(obstacles),
			goals(goals)	{
				g = mapf_adapters::Graph(x_dim, y_dim, obstacles);
			}

		int get_x(){	return x_dim;	}
		int get_y(){	return y_dim;	}
		std::vector<std::pair<int, int> > get_obstacles(){	return obstacles;	}
		std::vector<std::pair<int, int> > get_goals(){	return goals;	}
		mapf_adapters::Graph get_graph(){	return g;	}

	private:
		int x_dim, y_dim;
		std::vector<std::pair<int, int> > obstacles, goals;
		mapf_adapters::Graph g;
	};
}

#endif