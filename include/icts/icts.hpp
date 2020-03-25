#ifndef ICTS_HPP
#define ICTS_HPP

#include <vector>
#include <queue>
#include <unordered_set> 

typedef std::pair<int, std::pair<int, std::pair<int, int> > > p_pair;

class HighLevelNode{
public:
	HighLevelNode(std::vector<int> cost):	cost(cost)	{}

	int sum(int from, int to){
		int ans = 0;
		for(int i=from; i<to; i++){
			ans += cost[i];
		}
		return ans;
	}

	int sum(){
		int ans = 0;
		for(int i=0; i<cost.size(); i++){
			ans += cost[i];
		}
		return ans;	
	}

	bool equals(std::vector<int> cost2){
		return cost == cost2;
	}

	std::vector<int> get_added_cost(int i, int val){
		std::vector<int> new_cost(cost);
		new_cost[i] += val;
		return new_cost;
	}

	int hash_code() const{
		int ans = 0;
		for(int i = 0; i<cost.size(); i++){
			ans += cost[i] * PRIMES_FOR_HASHING[i%21];
		}
		return ans;
	}

	bool operator ==(const HighLevelNode & obj) const {
		if (this->hash_code() == obj.hash_code())
			return true;
		else
			return false;
	}

private:
	std::vector<int> cost;
	int PRIMES_FOR_HASHING[21] = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 71, 73, 79 };
};

namespace std {
	template<>
	struct hash<HighLevelNode>{
		size_t operator()(const HighLevelNode & obj) const {
			return hash<int>()(obj.hash_code());
		}
	};
}

namespace ICT{
	template<typename Mapf>
	class ICTS{
	public:
		ICTS()	{}
		bool search(Mapf mapf, std::vector<std::pair<int, int> > starts, std::pair<int, std::vector<std::pair<int, int> > > *solution){
			std::vector<int> path_lengths = find_shortest_path(mapf, starts);
			int num_of_agents = starts.size();
			HighLevelNode n(path_lengths);
			
			std::queue<HighLevelNode> high_level_search;
			std::unordered_set<HighLevelNode> visited;

			high_level_search.push(n);
			visited.insert(n);

			while(!high_level_search.empty()){
				HighLevelNode node = high_level_search.front();
				high_level_search.pop();

				//Perform the low level search on the node, If it succeeds, then return the solution
				
				//Dummy test
				if(node.sum() >1000){	
					std::cout<<"Do the Low Level Search "<<node.sum()<<std::endl;
					break;
				}

				for(int i = 0; i < num_of_agents; i++){
					std::vector<int> new_cost = node.get_added_cost(i, 1);
					HighLevelNode new_hL(new_cost);
					if(visited.find(new_hL) == visited.end()){
						high_level_search.push(new_hL);
						visited.insert(new_hL);
					}
				}
			}
			
			return true;
		}

		int heuristic(std::pair<int, int> a, std::pair<int, int> b){
			return std::abs(a.first-b.first) + std::abs(a.second-b.second);
		}

		std::vector<int> find_shortest_path(Mapf mapf, std::vector<std::pair<int, int> > starts){
			std::vector<std::pair<int, int> > goals = mapf.get_goals();
			std::vector<int> path_lengths(starts.size());

			for(int i=0; i<starts.size(); i++){
				std::vector< std::vector<int> > visited(mapf.get_x(), std::vector<int> (mapf.get_y(), -1));
				std::pair<int, int> goal= goals[i];

				std::priority_queue<p_pair, std::vector<p_pair>, std::greater<p_pair> > pq;
				pq.push(std::make_pair(heuristic(starts[i], goals[i]), std::make_pair(0, starts[i]))); 

				while(!pq.empty()){
					p_pair node = pq.top();
					pq.pop();

					int g = node.second.first;
					int node_x = node.second.second.first;
					int node_y = node.second.second.second;

					if(visited[node_x][node_y] != -1)	continue;

					if(node.second.second == goal){
						path_lengths[i] = g;
						break;
					}

					visited[node_x][node_y] = g;
					std::vector<std::pair<int, int> > neighbors = mapf.get_graph().get_neighbor(node_x, node_y);
					for(auto it = neighbors.begin(); it != neighbors.end(); ++it){
						int neigh_x = it->first;
						int neigh_y = it->second;
						if(visited[neigh_x][neigh_y] == -1){
							std::pair<int, int> neigh = std::make_pair(neigh_x, neigh_y);
							int f = heuristic(neigh, goal) + g + 1;
							pq.push(std::make_pair(f, std::make_pair(g+1, neigh)));
						}
					}

				}
			}

			return path_lengths;
		}

	};
}

#endif