#include "search-strategies.h"
#include <queue>
#include <stack>
#include <set>
#include "memusage.h"

typedef struct {
	std::shared_ptr<SearchState> parent;
	SearchAction parent_act;
	int depth; // Just for DFS, no need in BFS
} Node;
std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::set<SearchState> closed;
	std::queue<std::shared_ptr<SearchState>> open;
	std::map<std::shared_ptr<SearchState>, Node> tree;

	if(init_state.isFinal()){
		return {};
	}
	bool reached_final = false;
	std::shared_ptr<SearchState> parent_state = std::make_shared<SearchState>(init_state);
	open.push(parent_state);

	auto old_memory = getCurrentRSS(); 
	while(!open.empty()){
		auto current_parent = open.front();
		SearchState working_state(*current_parent);
		open.pop();
		auto actions = working_state.actions();
		auto taken_memory = getCurrentRSS();
		if((taken_memory - old_memory)*4 + taken_memory > mem_limit_){
			return {};
		}
		old_memory = taken_memory;
		for(auto act : actions){
			auto new_state = act.execute(working_state);
			if(closed.count(new_state) == 0){
				closed.insert(new_state);
				auto new_shared = std::make_shared<SearchState>(new_state);
				open.push(new_shared);
				Node parent_node = {current_parent, act, 0};		
				tree.insert({new_shared,parent_node});
				if(new_state.isFinal()){
					reached_final = true;
					parent_state = new_shared;
					break;
				}
			}
		}
		if(reached_final){
			break;
		}
	}
	if(reached_final){
		std::vector<SearchAction> solution;
		while(true){
			auto tree_find = tree.find(parent_state);
			if(tree_find == tree.end()){
				break;
			}
			auto node = tree_find->second;
			solution.insert(solution.begin(),node.parent_act);
			parent_state = node.parent;
		}
		return solution;
	}
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	std::set<SearchState> closed;
	std::stack<std::shared_ptr<SearchState>> open;
	std::map<std::shared_ptr<SearchState>, Node> tree;
	if(init_state.isFinal()){
		return {};
	}

	bool reached_final = false;
	std::shared_ptr<SearchState> parent_state = std::make_shared<SearchState>(init_state);
	int current_depth = 0;
	Node init_node = {parent_state, init_state.actions()[0], 0};
	open.push(parent_state);
	tree.insert({parent_state,init_node});

	while(!open.empty()){
		auto current_parent = open.top();
		SearchState working_state(*current_parent);
		current_depth = tree.find(current_parent)->second.depth;
		open.pop();
		if(current_depth >= depth_limit_){
			continue;
		}
		auto actions = working_state.actions();
		for(auto act : actions){
			auto new_state = act.execute(working_state);
			if(closed.count(new_state) == 0){
				closed.insert(new_state);
				auto new_shared = std::make_shared<SearchState>(new_state);
				open.push(new_shared);
				Node parent_node = {current_parent, act, current_depth+1};		
				tree.insert({new_shared,parent_node});
				if(new_state.isFinal()){
					reached_final = true;
					parent_state = new_shared;
					break;
				}
			}
		}
		if(reached_final){
			break;
		}
	}

	if(reached_final){
		std::vector<SearchAction> solution;
		while(true){
			auto tree_find = tree.find(parent_state);
			if(tree_find->second.depth == 0){
				break;
			}
			auto node = tree_find->second;
			solution.insert(solution.begin(),node.parent_act);
			parent_state = node.parent;
		}
		return solution;
	}

	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    int cards_out_of_home = king_value * colors_list.size();
    for (const auto &home : state.homes) {
        auto opt_top = home.topCard();
        if (opt_top.has_value())
            cards_out_of_home -= opt_top->value;
    }

    return cards_out_of_home;
}

typedef struct {
	std::shared_ptr<SearchState> parent;
	SearchAction parent_act;
} Node_Assembly;

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	if (init_state.isFinal())
		return {};

	std::set<SearchState> closed;
	std::priority_queue<std::tuple<double, std::shared_ptr<SearchState>>, 
	std::vector<std::tuple<double, std::shared_ptr<SearchState>>>, 
	std::greater<std::tuple<double, std::shared_ptr<SearchState>>>> open;
	std::map<std::shared_ptr<SearchState>, Node_Assembly> tree;

	bool reached_final = false;
	int initial_value = 0;

	std::shared_ptr<SearchState> parent_state = std::make_shared<SearchState>(init_state);
	Node_Assembly init_node = {parent_state, init_state.actions()[0]};
	open.push({initial_value, parent_state});
	tree.insert({parent_state, init_node});

	while (!open.empty() && !reached_final) {
		double current_h = std::get<0>(open.top());
		auto current_parent = std::get<1>(open.top());
		
		SearchState working_state(*current_parent);

		std::vector<SearchAction> actions = working_state.actions();
		for (auto act : actions) {
			SearchState new_state = act.execute(working_state);
			
			if (closed.count(new_state) == 0) {
				closed.insert(new_state);

				auto new_shared = std::make_shared<SearchState>(new_state);
				double h = current_h + compute_heuristic(new_state, *heuristic_);
				open.push({h, new_shared});
				
				Node_Assembly parent_node = {current_parent, act};		
				tree.insert({new_shared, parent_node});
				if(new_state.isFinal()){
					reached_final = true;
					parent_state = new_shared;
					break;
				}
			}
		}
		open.pop();
	}
	
	if(reached_final){
		std::vector<SearchAction> solution;
		while(true) {
			auto tree_find = tree.find(parent_state);
			if(tree_find->second.parent == parent_state){
				break;
			}
			auto node = tree_find->second;
			solution.insert(solution.begin(), node.parent_act);
			parent_state = node.parent;
		}
		return solution;
	}
	return {};
}
