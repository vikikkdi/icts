#ifndef ICTS_HPP
#define ICTS_HPP

#include <stack>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <chrono>

#include <algorithm>

typedef std::pair<int, int> pair_1;
typedef std::pair<int, pair_1 > pair_2;
typedef std::pair<int, pair_2 > pair_3;
typedef std::tuple<int, int, int> tup;
typedef std::vector<tup> v_tup;

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

	int get_cost(int i){
		return cost[i];
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
		return this->cost == obj.cost;
		
		/* Another method to check for equal objects 
		if (this->hash_code() == obj.hash_code())
			return true;
		else
			return false;
		*/
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

template<typename Mapf>
class MDD{
public:
	MDD(int x, int y):	x_dim(x), y_dim(y)	{}

	void construct_MDD_first_time(Mapf mapf, int depth){
		cur_depth = depth;
		for(int c=0; c<depth; c++){
			for(int i=0; i<x_dim; i++){
				for(int j=0; j<y_dim; j++){
					adjacency_list[std::make_tuple(i, j, c)] = std::vector<tup>();
				}
			}
		}

		for(int i=0; i<x_dim; i++){
			for(int j=0; j<y_dim; j++){
				std::vector<pair_1 > neighbors = mapf.get_graph().get_neighbor(i, j);
				neighbors.emplace_back(std::make_pair(i, j));
				for(int c=0; c<depth-1; c++){
					//Create the adjacency list for tuple (i, j, c)
					for(auto it=neighbors.begin(); it!=neighbors.end(); ++it){
						adjacency_list[std::make_tuple(i, j, c)].push_back(std::make_tuple(it->first, it->second, c+1));
						adjacency_list[std::make_tuple(it->first, it->second, c+1)].push_back(std::make_tuple(i, j, c));
					}
				}
			}
		}
		//print_MDD();
	}

	void increment_depth(Mapf mapf){
		int depth = cur_depth;
		cur_depth++;
		for(int i=0; i<x_dim; i++){
			for(int j=0; j<y_dim; j++){
				adjacency_list[std::make_tuple(i, j, depth)] = std::vector<tup>();
			}
		}
		for(int i=0; i<x_dim; i++){
			for(int j=0; j<y_dim; j++){
				std::vector<pair_1 > neighbors = mapf.get_graph().get_neighbor(i, j);
				neighbors.emplace_back(std::make_pair(i, j));
					//Create the adjacency list for tuple (i, j, c)
				for(auto it=neighbors.begin(); it!=neighbors.end(); ++it){
					adjacency_list[std::make_tuple(i, j, depth-1)].push_back(std::make_tuple(it->first, it->second, depth));
					adjacency_list[std::make_tuple(it->first, it->second, depth)].push_back(std::make_tuple(i, j, depth-1));
				}
			}
		}
	}

	int get_cur_depth(){	return cur_depth;	}

	std::map<tup, std::vector<tup> > get_DAG(pair_1 start, pair_1 goal, int depth){
		std::map<tup, std::vector<tup> > DAG_formed;
		std::map<tup, std::vector<tup> > bfs_start;
		std::map<tup, std::vector<tup> > bfs_goal;

		std::map<tup, int> visited;

		std::queue<tup> bfs_queue;

		bfs_queue.push(std::make_tuple(start.first, start.second, 0));
		visited[std::make_tuple(start.first, start.second, 0)] = 1;

		while(!bfs_queue.empty()){
			tup n = bfs_queue.front();
			bfs_queue.pop();

			if(bfs_start.find(n) == bfs_start.end()){
				bfs_start[n] = std::vector<tup>();
			}

			if(std::get<2>(n) == depth){
				continue;
			}

			for(auto it=adjacency_list[n].begin(); it!=adjacency_list[n].end(); ++it){
				int x = std::get<0>(*it);
				int y = std::get<1>(*it);
				int c = std::get<2>(*it);

				if(c == std::get<2>(n)+1){
					if(visited.find(std::make_tuple(x, y, c)) == visited.end())
						bfs_queue.push(*it);
					visited[std::make_tuple(x, y, c)] = 1;
					bfs_start[n].push_back(*it);
				}
			}
		}
		
		visited.clear();
		bfs_queue = std::queue<tup>();

		bfs_queue.push(std::make_tuple(goal.first, goal.second, depth));
		visited[std::make_tuple(goal.first, goal.second, depth)] = 1;

		while(!bfs_queue.empty()){
			tup n = bfs_queue.front();
			bfs_queue.pop();

			if(bfs_goal.find(n) == bfs_goal.end()){
				bfs_goal[n] = std::vector<tup>();
			}


			if(std::get<2>(n) == 0){
				continue;
			}

			for(auto it=adjacency_list[n].begin(); it!=adjacency_list[n].end(); ++it){
				int x = std::get<0>(*it);
				int y = std::get<1>(*it);
				int c = std::get<2>(*it);
				
				if(c+1 == std::get<2>(n)){
					if(visited.find(std::make_tuple(x, y, c)) == visited.end())
						bfs_queue.push(*it);
					visited[std::make_tuple(x, y, c)] = 1;
					bfs_goal[n].push_back(*it);
				}
			}
		}

		for(auto it1 = bfs_start.begin(); it1 != bfs_start.end(); ++it1){
			auto it2 = bfs_goal.find(it1->first);
			if(it2 != bfs_goal.end()){
				DAG_formed[it1->first] = std::vector<tup>();
				if(std::get<0>(it1->first) == goal.first && std::get<1>(it1->first) == goal.second && std::get<2>(it1->first) == depth){
					continue;
				}
				else if(it1->second.size()>0 && it2->second.size()>0){
					for(auto m1=it1->second.begin(); m1!=it1->second.end(); ++m1){					
						int x1 = std::get<0>(*m1);
						int y1 = std::get<1>(*m1);
						int c1 = std::get<2>(*m1);
						for(auto m2=it2->second.begin(); m2!=it2->second.end(); ++m2){					
							int x2 = std::get<0>(*m2);
							int y2 = std::get<1>(*m2);
							int c2 = std::get<2>(*m2);
							if(x1==x2 && y1==y2){
								DAG_formed[it1->first].push_back(*m1);
							}
						}
					}
				}
				else if(it1->second.size() == 0){
					DAG_formed[it1->first] = it2->second;
				}
				else if(it2->second.size() == 0){
					DAG_formed[it1->first] = it1->second;
				}
			}
		}
		//print_map(DAG_formed);

		return DAG_formed;
	}

	void print_MDD(){
		for(auto it1 = adjacency_list.begin(); it1!=adjacency_list.end(); ++it1){
			std::cout<<std::get<0>(it1->first)<<" "<<std::get<1>(it1->first)<<" "<<std::get<2>(it1->first)<<" :: ";
			for(auto it2 = it1->second.begin(); it2!=it1->second.end(); ++it2){
				std::cout<<"("<<std::get<0>(*it2)<<","<<std::get<1>(*it2)<<","<<std::get<2>(*it2)<<") ";
			}
			std::cout<<std::endl;
		}
	}

	void print_map(std::map<tup, std::vector<tup> > mp){
		for(auto it1 = mp.begin(); it1!=mp.end(); ++it1){
			std::cout<<std::get<0>(it1->first)<<" "<<std::get<1>(it1->first)<<" "<<std::get<2>(it1->first)<<" :: ";
			for(auto it2 = it1->second.begin(); it2!=it1->second.end(); ++it2){
				std::cout<<"("<<std::get<0>(*it2)<<","<<std::get<1>(*it2)<<","<<std::get<2>(*it2)<<") ";
			}
			std::cout<<std::endl;
		}
	}

private:
	std::map<tup, std::vector<tup> > adjacency_list;
	int x_dim;
	int y_dim;
	int cur_depth;
};

namespace ICT{
	template<typename Mapf>
	class ICTS{
	public:
		ICTS()	{}
		bool search(Mapf mapf, std::vector<pair_1 > starts, std::pair<int, std::vector<std::vector<pair_1 > > > *solution){
			auto astar = std::chrono::system_clock::now();
			std::vector<int> path_lengths = find_shortest_path(mapf, starts);
			auto astar_end = std::chrono::system_clock::now();
			std::cout<<"ASTAR END" <<std::chrono::duration<double>(astar_end - astar).count()<<std::endl;
			int num_of_agents = starts.size();
			HighLevelNode n(path_lengths);
			
			std::queue<HighLevelNode> high_level_search;
			std::unordered_set<HighLevelNode> visited;

			MDD<Mapf> mdd(mapf.get_x(), mapf.get_y());
			auto mdd_time= std::chrono::system_clock::now();
			mdd.construct_MDD_first_time(mapf, *(std::max_element(path_lengths.begin(), path_lengths.end())));
			auto mdd_end = std::chrono::system_clock::now();
			std::cout<<"MDD CONS" <<std::chrono::duration<double>(mdd_end - mdd_time).count()<<std::endl;
			high_level_search.push(n);
			visited.insert(n);

			while(!high_level_search.empty()){
				HighLevelNode node = high_level_search.front();
				high_level_search.pop();

				//Perform the low level search on the node, If it succeeds, then return the solution
				if(low_level_search(mdd, node, mapf, starts, solution)){
					//Solution Found
					return true;
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
			
			return false;
		}

		bool node_check(v_tup node, std::vector<pair_1 > check){
			for(int i=0; i<check.size(); i++){
				if(std::get<0>(node[i])!=check[i].first || std::get<1>(node[i])!=check[i].second)	return false;
			}
			return true;
		}

		std::vector<v_tup> cart_product (const std::vector<v_tup>& v) {
			std::vector<v_tup> s = {{}};
			for (const auto& u : v) {
				std::vector<v_tup> r;
				for (const auto& x : s) {
					for (const auto y : u) {
						r.push_back(x);
						r.back().push_back(y);
					}
				}
				s = std::move(r);
			}
			return s;
		}

		bool check_compatible_node(v_tup parent, v_tup child){
			std::map<tup, int> cur_nodes;
			for(auto it=child.begin(); it!=child.end(); ++it){
				cur_nodes[*(it)]++;
			}
			
			if(cur_nodes.size() != child.size()){
				return false;
			}

			for(int i=0; i<parent.size(); i++){
				int x1_p = std::get<0>(parent[i]);
				int y1_p = std::get<1>(parent[i]);
				int x1_c = std::get<0>(child[i]);
				int y1_c = std::get<1>(child[i]);
				for(int j=0; j<parent.size(); j++){
					if(i == j)	continue;
					int x2_p = std::get<0>(parent[j]);
					int y2_p = std::get<1>(parent[j]);
					int x2_c = std::get<0>(child[j]);
					int y2_c = std::get<1>(child[j]);

					if(x1_p == x2_c && y1_p == y2_c && x1_c == x2_p && y1_c == y2_p){
						return false;
					}
				}
			}
			
			return true;
		}

		bool check_compatible_paths(MDD<Mapf> mdd, HighLevelNode nodeHL, Mapf mapf, std::vector<pair_1 > starts, std::vector<pair_1 > goals, std::vector< std::map<tup, std::vector<tup> > > mdds, std::pair<int, std::vector<std::vector<pair_1 > > > *solution){
			//Do search for compatible paths in the MDD search space 
			std::stack< v_tup > dfs_stack;
			v_tup node;
			for(int i=0; i<starts.size(); i++){
				int x = starts[i].first;
				int y = starts[i].second;
				node.push_back(std::make_tuple(x, y, 0));
			}
			dfs_stack.push(node);

			std::map<v_tup, v_tup> parent;

			while(!dfs_stack.empty()){
				v_tup node = dfs_stack.top();
				dfs_stack.pop();

				if(node_check(node, goals)){
					//Solution Found
					std::vector<v_tup> sol;
					sol.push_back(node);
					while(!node_check(sol[sol.size()-1], starts)){
						sol.push_back(parent[sol[sol.size()-1]]);
					}

					solution->first = nodeHL.sum();
					solution->second.resize(starts.size());
					
					for(int i=0;i<starts.size(); i++){
						solution->second[i] = {};
					}

					for(int i=sol.size()-1; i>=0; i--){
						for(int j=0; j<sol[i].size(); j++){
							solution->second[j].push_back(std::make_pair(std::get<0>(sol[i][j]), std::get<1>(sol[i][j])));
						}
					}

					return true;
				}

				std::vector<v_tup> temp;
				for(int i = 0; i<node.size(); i++){
					v_tup temp_mdd = mdds[i][node[i]];
					if(temp_mdd.size() == 0){
						temp_mdd.push_back(node[i]);
					}
					temp.push_back(temp_mdd);
				}
				std::vector<v_tup> s = cart_product(temp);
				
				for(auto it=s.begin(); it!=s.end(); ++it){
					v_tup j = *(it);
					if(check_compatible_node(node, j) && parent.find(j) == parent.end()){
						parent[j] = node;
						dfs_stack.push(j);
					}
				}
			}

			return true;
		}

		bool low_level_search(MDD<Mapf> mdd, HighLevelNode nodeHL, Mapf mapf, std::vector<pair_1 > starts, std::pair<int, std::vector<std::vector<pair_1 > > > *solution){
			std::vector<pair_1 > goals = mapf.get_goals();
			std::vector< std::map<tup, std::vector<tup> > > mdds;
			//Build the MDD for each agent
			auto mdd_ss = std::chrono::system_clock::now();
			
			for(int i=0; i<starts.size(); i++){
				pair_1 start = starts[i];
				pair_1 goal = goals[i];
				if(mdd.get_cur_depth()<=nodeHL.get_cost(i)){
					mdd.increment_depth(mapf);
				}
				mdds.push_back(mdd.get_DAG(start, goal, nodeHL.get_cost(i)));
			}
			auto mdd_s = std::chrono::system_clock::now();
			std::cout<<"ASTAR END" <<std::chrono::duration<double>(mdd_s - mdd_ss).count()<<std::endl;
			return check_compatible_paths(mdd, nodeHL, mapf, starts, goals, mdds, solution);
		}

		int heuristic(pair_1 a, pair_1 b){
			return std::abs(a.first-b.first) + std::abs(a.second-b.second);
		}

		std::vector<int> find_shortest_path(Mapf mapf, std::vector<pair_1 > starts){
			std::vector<pair_1 > goals = mapf.get_goals();
			std::vector<int> path_lengths(starts.size());

			for(int i=0; i<starts.size(); i++){
				std::vector< std::vector<int> > visited(mapf.get_x(), std::vector<int> (mapf.get_y(), -1));
				pair_1 goal= goals[i];

				std::priority_queue<pair_3, std::vector<pair_3>, std::greater<pair_3> > pq;
				pq.push(std::make_pair(heuristic(starts[i], goals[i]), std::make_pair(0, starts[i]))); 

				while(!pq.empty()){
					pair_3 node = pq.top();
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
					std::vector<pair_1 > neighbors = mapf.get_graph().get_neighbor(node_x, node_y);
					for(auto it = neighbors.begin(); it != neighbors.end(); ++it){
						int neigh_x = it->first;
						int neigh_y = it->second;
						if(visited[neigh_x][neigh_y] == -1){
							pair_1 neigh = std::make_pair(neigh_x, neigh_y);
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
