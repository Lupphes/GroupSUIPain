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

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	std::map<const SearchState, std::tuple<std::shared_ptr<const SearchState>, SearchAction>> closed;
	
	std::priority_queue<std::tuple<double, std::shared_ptr<SearchState>>, 
	std::vector<std::tuple<double, std::shared_ptr<SearchState>>>, 
	std::greater<std::tuple<double, std::shared_ptr<SearchState>>>> working_tree;

	std::shared_ptr<SearchState> final_state = std::make_shared<SearchState>(init_state);
	working_tree.push({0, final_state});
	
	std::cout << "Inicializovano" << std::endl;
	
	bool found_result = false;
	
	while (!working_tree.empty() && !found_result) {
		double working_state_h = std::get<0>(working_tree.top());
		SearchState working_state_ptr = *(std::get<1>(working_tree.top()));
		
		std::vector<SearchAction> actions = working_state_ptr.actions();
		
		for (size_t i = 0; i < actions.size(); i++) {
			SearchState new_state = actions[i].execute(working_state_ptr);
	
			auto it = closed.find(new_state);
			
			if (it == closed.end()) {
				std::tuple<std::shared_ptr<const SearchState>, SearchAction> new_tuple = {
					std::make_shared<const SearchState>(working_state_ptr), SearchAction(actions[i])
				};
				closed.insert({new_state, new_tuple});
				
				double h = working_state_h + compute_heuristic(new_state, *heuristic_);
				working_tree.push({h, std::make_shared<SearchState>(new_state)});
				// std::cout << h << std::endl;
			}
			
			if (new_state.isFinal()){
				found_result = true;
				SearchState working_state(new_state);
				final_state = std::make_shared<SearchState>(working_state);
				break;
			}
		}
		
		working_tree.pop();
	}
	
	std::cout << "hotovo" << std::endl;
	
	std::vector<SearchAction> solution;
	if (found_result) {
		std::shared_ptr<const SearchState> cur_state = final_state;
		while (cur_state != nullptr) {
			auto search = closed.find(*cur_state);
			if(search == closed.end()){
				break;
			}
			auto tuple = search->second;
			auto new_action = SearchAction(std::get<1>(search->second));
			solution.push_back(new_action);
			// std::cout << new_action << std::endl;
			cur_state = std::get<0>(tuple);
		}
		for (const auto & action : std::vector<SearchAction> (solution.rbegin(), solution.rend())){
			std::cout << action << std::endl;
		}
		return std::vector<SearchAction> (solution.rbegin(), solution.rend());
	}
	return {};
}
