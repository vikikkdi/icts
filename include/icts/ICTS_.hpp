#ifndef ICTS_NEW_HPP
#define ICTS_NEW_HPP

#include <stack>
#include <vector>
#include <queue>
#include <deque>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <chrono>

#include <boost/heap/fibonacci_heap.hpp>

#include <algorithm>

typedef std::pair<int, int> pair_1;
typedef std::pair<int, pair_1 > pair_2;
typedef std::pair<int, pair_2 > pair_3;
typedef std::tuple<int, int, int> tup;
typedef std::vector<tup> v_tup;
typedef std::set<tup> s_tup;

class TreeNode{
public:
	TreeNode(std::vector<int> path_costs){
		agent_path_costs = path_costs;
		childs.resize(0);
	}

	std::vector<int> get_costs(){	return agent_path_costs;	}

	TreeNode* get_ith_child(int i){	
		if(i > childs.size())	return NULL;
		return childs[i];	
	}

	std::vector<TreeNode*> get_childs(){	return childs;	}

	void add_child(std::vector<int> new_path_costs){
		TreeNode* new_node = new TreeNode(new_path_costs);
		childs.push_back(new_node);
	}

	void expand_node(){
		for(int i=0; i<agent_path_costs.size(); i++){
			std::vector<int> new_costs = agent_path_costs;
			new_costs[i] += 1;
			TreeNode* new_child = new TreeNode(new_costs);
			childs.push_back(new_child);
		}
	}

private:
	std::vector<int> agent_path_costs;
	std::vector<TreeNode*> childs;
};

class IncreasingCostTree{
public:
	IncreasingCostTree(std::vector<std::vector<bool> > mp, std::vector<pair_1> g, std::vector<pair_1> s, std::vector<int> initial_estimate){
		my_map = mp;
		goals = g;
		starts = s;
		initial_cost = initial_estimate;

		root = new TreeNode(initial_estimate);
		open_list.push_back(root);
		closed_list.insert(initial_estimate);
	}

	std::deque<TreeNode*> get_open_list(){	return open_list;	}
	
	TreeNode* get_next_node_to_expand(){	return open_list.front();	}
	
	void pop_next_node_to_expand(){	open_list.pop_front();	}

	void add_node_to_open_list(TreeNode* node){	open_list.push_back(node);	}

	void add_unexplored_node_to_open_list(TreeNode* node){
		std::vector<int> node_cost = node->get_costs();

		if(closed_list.find(node_cost) == closed_list.end()){
			closed_list.insert(node_cost);
			open_list.push_back(node);
		}
	}

	void expand_next_node(){
		TreeNode* next_node = get_next_node_to_expand();
		next_node->expand_node();
		std::vector<TreeNode*> ch = next_node->get_childs();
		for(int i=0; i<ch.size(); i++){
			add_unexplored_node_to_open_list(ch[i]);
		}
	}

private:
	std::vector<std::vector<bool> > my_map;
	std::vector<pair_1> goals;
	std::vector<pair_1> starts;
	std::vector<int> initial_cost;
	TreeNode* root;
	std::deque<TreeNode*> open_list;
	std::set<std::vector<int> > closed_list;
};

namespace ICT_NEW{

	class MDD{
	public:
		MDD(){
			agent = -1;
		}
		MDD(int ag, pair_1 s, pair_1 g, std::vector<std::vector<bool> > my_map, int d, MDD last_mdd = MDD(), bool generate = true){
			agent = ag;
			start = s;
			goal = g;
			depth = d;

			std::cout<<last_mdd.agent<<std::endl;

			mdd = std::map<tup, s_tup>();
			level = std::map<int, std::set<pair_1> >();
			bfs_tree = std::map<tup, s_tup>();
			visited = std::set<tup>();
			fringe = std::set<tup>();
			fringe_prevs = std::map<tup, s_tup>();

			if(generate){
				if(last_mdd.agent != -1 && last_mdd.depth < depth && last_mdd.agent == agent){
					generate_mdd(my_map, last_mdd);
				}else{
					generate_mdd(my_map);
				}
			}
		}

		void generate_mdd(std::vector<std::vector<bool> > my_map, MDD last_mdd = MDD()){
			if(last_mdd.agent != -1){
				bootstrap_depth_d_bfs_tree(my_map, last_mdd);
			} else {
				get_depth_d_bfs_tree(my_map);
			}
			bfs_to_mdd();
			if(mdd.size()){
				populate_levels();
			}
		}

		void populate_levels(){
			level[0] = std::set<pair_1>();
			level[0].insert(start);

			for(auto it=mdd.begin(); it!=mdd.end(); it++){
				for(auto it2 = it->second.begin(); it2!=it->second.end(); it2++){
					if(level.find(std::get<2>(*(it2))) == level.end()){
						level[std::get<2>(*(it2))] = std::set<pair_1>();
					}
					level[std::get<2>(*(it2))].insert({std::get<0>(*(it2)), std::get<1>(*(it2))});
				}
			}
		}

		void bootstrap_depth_d_bfs_tree(std::vector<std::vector<bool> > my_map, MDD last_mdd){
			std::deque<tup> fringe_bfs;
			std::set<tup> old_fringe = last_mdd.fringe;
			for(auto i:old_fringe){
				fringe_bfs.push_back(i);
			}

			std::map<tup, s_tup> prev_dict = last_mdd.bfs_tree;
			for(auto i:old_fringe){
				for(auto j:last_mdd.fringe_prevs[i])
					prev_dict[i].insert(j);
			}

			std::set<tup> vis = last_mdd.visited;
			main_bfs_loop(my_map, fringe_bfs, prev_dict, vis);

		}

		void get_depth_d_bfs_tree(std::vector<std::vector<bool> > my_map){
			std::deque<tup> fringe_bfs;
			fringe_bfs.push_back({start.first, start.second, 0});
			std::map<tup, s_tup> prev_dict = std::map<tup, s_tup>();
			std::set<tup> vis = std::set<tup>();
			main_bfs_loop(my_map, fringe_bfs, prev_dict, vis);
		}

		void main_bfs_loop(std::vector<std::vector<bool> > my_map, std::deque<tup> fringe_bfs, std::map<tup, s_tup> prev_dict, std::set<tup> vis){
			std::set<tup> depth_d_plus_one_fringe;
			std::map<tup, s_tup> fringe_prevs_bfs;

			while(!fringe_bfs.empty()){
				tup cur = fringe_bfs.front();
				fringe_bfs.pop_front();

				int agent_loc_x = std::get<0>(cur);
				int agent_loc_y = std::get<1>(cur);
				int agent_loc_d = std::get<2>(cur);

				for(int i=0; i<op.size(); i++){
					int new_agent_x = agent_loc_x + op[i].first;
					int new_agent_y = agent_loc_y + op[i].second;
					int new_agent_d = agent_loc_d + 1;
					if(new_agent_x < my_map.size() && new_agent_x > 0 && my_map.size() > 0 && new_agent_y > 0 && new_agent_y < my_map[0].size() && my_map[new_agent_x][new_agent_y] == false){
						if(new_agent_d <= depth){
							if(prev_dict.find({new_agent_x, new_agent_y, new_agent_d}) != prev_dict.end())
								prev_dict[{new_agent_x, new_agent_y, new_agent_d}].insert(cur);
							else{
								prev_dict[{new_agent_x, new_agent_y, new_agent_d}] = std::set<tup>();
								prev_dict[{new_agent_x, new_agent_y, new_agent_d}].insert(cur);
							}
							if(vis.find({new_agent_x, new_agent_y, new_agent_d}) == vis.end()){
								fringe_bfs.push_back({new_agent_x, new_agent_y, new_agent_d});
								vis.insert({new_agent_x, new_agent_y, new_agent_d});
							}
						}
						if(new_agent_d == depth + 1){
							depth_d_plus_one_fringe.insert({new_agent_x, new_agent_y, new_agent_d});
							if(fringe_prevs_bfs.find({new_agent_x, new_agent_y, new_agent_d}) == fringe_prevs_bfs.end())
								fringe_prevs_bfs[{new_agent_x, new_agent_y, new_agent_d}] = std::set<tup>();
							fringe_prevs_bfs[{new_agent_x, new_agent_y, new_agent_d}].insert(cur);
						}
					}
				}
			}

			bfs_tree = prev_dict;
			visited = vis;
			fringe = depth_d_plus_one_fringe;
			fringe_prevs = fringe_prevs_bfs;
		}

		void bfs_to_mdd(){
			tup goal_time = {goal.first, goal.second, depth};
			std::set<std::pair<tup, tup> > vis;

			if(bfs_tree.find(goal_time) == bfs_tree.end())	return;

			std::deque<std::pair<tup, tup> > trace_list;
			
			for(auto parent:bfs_tree[goal_time]){
				trace_list.push_back({parent, goal_time});
				vis.insert({parent, goal_time});
			}

			while(!trace_list.empty()){
				std::pair<tup, tup> t = trace_list.front();
				trace_list.pop_front();
				tup cur = t.first;
				tup child = t.second;

				if(mdd.find(cur) == mdd.end()){
					mdd[cur] = std::set<tup>();
				}

				mdd[cur].insert(child);
				for(auto p:bfs_tree[cur]){
					if(vis.find({p, cur}) == vis.end()){
						vis.insert({p, cur});
						trace_list.push_back({p, cur});
					}
				}
			}
		}

	private:
		int agent;
		pair_1 start;
		pair_1 goal;
		int depth;
		std::map<tup, s_tup> mdd;
		std::map<int, std::set<pair_1> > level;
		std::map<tup, s_tup> bfs_tree;
		std::set<tup> visited;
		std::set<tup> fringe;
		std::map<tup, s_tup> fringe_prevs;
		std::vector<pair_1> op = {{0,0}, {-1,0}, {0,1}, {1,0}, {0,-1}};
	};


	template<typename Mapf>
	class ICTS{
	public:
		ICTS()	{}
		bool search(Mapf mapf, std::vector<pair_1 > starts, std::pair<int, std::vector<std::vector<pair_1 > > > *solution){
			obstacles = mapf.get_obstacles();
			goals = mapf.get_goals();
			x = mapf.get_x();
			y = mapf.get_y();
			int upper_bound = (goals.size() * goals.size()) * generate_map();
			compute_heuristics();
			std::vector<int> optimal_cost = find_shortest_path(starts);

			MDD mdd(0, starts[0], goals[0], temp_map, optimal_cost[0]);
			MDD mdd1(0, starts[0], goals[0], temp_map, optimal_cost[0]+1, mdd);

			return true;
		}

		int generate_map(){
			int num_of_open_spaces = 0;
			temp_map.resize(x);
			for(int i=0; i<x; i++){
				temp_map[i] = std::vector<bool>(y, false);
			}
			for(int i=0; i<obstacles.size(); i++){
				temp_map[obstacles[i].first][obstacles[i].second] = true;
				num_of_open_spaces--;
			}
			return num_of_open_spaces;
		}

		void compute_heuristics(){
			heuristics.resize(goals.size());
			for(int i=0; i<x; i++){
				for(int j=0; j<y; j++){
					if(temp_map[i][j] == false){
						//not an obstacle
						for(int k=0; k<goals.size(); k++){
							heuristics[k][{i, j}] = manhattan_distance({i, j}, goals[i]);
						}
					}
				}
			}
		}

		int manhattan_distance(pair_1 a, pair_1 b){
			return abs(a.first - b.first) + abs(a.second - b.second);
		}

		struct compare_node{
			bool operator()(const pair_3 & n1, const pair_3 & n2) const{
				return n1.first > n2.first;
			}
		};

		std::vector<int> find_shortest_path(std::vector<pair_1 > starts){
			std::vector<int> path_lengths(starts.size());

			for(int i=0; i<starts.size(); i++){
				std::vector< std::vector<int> > visited(x, std::vector<int> (y, -1));
				pair_1 goal= goals[i];

				std::priority_queue<pair_3, std::vector<pair_3>, std::greater<pair_3> > pq;
				//boost::heap::fibonacci_heap<pair_3, boost::heap::compare<compare_node> > heap;	//Use anything in the future
				pq.push(std::make_pair(manhattan_distance(starts[i], goals[i]), std::make_pair(0, starts[i]))); 

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

					for(int i=0; i<op.size(); i++){
						int neigh_x = node_x + op[i].first;
						int neigh_y = node_y + op[i].second;
						if(neigh_x >= 0 && neigh_x < x && neigh_y >= 0 && neigh_y < y && temp_map[neigh_x][neigh_y] == false && visited[neigh_x][neigh_y] == -1){
							pair_1 neigh = std::make_pair(neigh_x, neigh_y);
							int f = manhattan_distance(neigh, goal) + g + 1;
							pq.push(std::make_pair(f, std::make_pair(g+1, neigh)));
						}
					}
				}
			}

			return path_lengths;
		}
	private:
		int x, y;
		std::vector<pair_1> obstacles;
		std::vector<pair_1> goals;
		std::vector<std::map<pair_1, int> > heuristics;
		std::vector<std::vector<bool> > temp_map;
		std::vector<pair_1> op = {{0,0}, {-1,0}, {0,1}, {1,0}, {0,-1}};
	};
}

#endif
