#include "search-strategies.h"
#include <deque>
#include <queue>
#include <set>

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::map<const SearchState, std::tuple<const SearchState, SearchAction>>  closed;
	std::deque<std::shared_ptr<SearchState>> working_tree;
	std::shared_ptr<SearchState> final_state = std::make_shared<SearchState>(init_state);
	working_tree.push_back(final_state);
	bool found_result = false;
	while(!working_tree.empty()){
		SearchState working_state(*working_tree.front());
		working_tree.pop_front();
		auto actions = working_state.actions();
		for(size_t i = 0; i < actions.size(); i++){
			auto new_state = actions[i].execute(working_state);
			std::tuple<const SearchState, SearchAction> new_tuple = {working_state,actions[i]};
			auto check = closed.find(new_state);
			if(check == closed.end()){
				closed.insert({new_state,new_tuple});
				working_tree.push_back(std::make_shared<SearchState>(new_state));
			}
			if(new_state.isFinal()){
				found_result = true;
				SearchState working_state(new_state);
				final_state = std::make_shared<SearchState>(working_state);
				break;
			}
		}
		if(found_result)
			break;
	}
	
	std::vector<SearchAction> solution;
	if(found_result){
		std::shared_ptr<const SearchState> cur_state = final_state;
		while(true){
			auto search = closed.find(*cur_state);
			if(search == closed.end()){
				break;
			}
			auto tpl = search->second;
			auto old_state = cur_state;
			cur_state = std::make_shared<const SearchState>(std::get<0>(tpl));
			auto new_action = SearchAction (std::get<1>(search->second));
			solution.push_back(new_action);
			closed.erase(*old_state);
		}
		return std::vector<SearchAction> (solution.rbegin(), solution.rend());
	}

	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	std::map<const SearchState, std::tuple<const SearchState, SearchAction>>  closed;
	std::deque<std::shared_ptr<SearchState>> working_tree;
	std::shared_ptr<SearchState> final_state = std::make_shared<SearchState>(init_state);
	working_tree.push_back(final_state);
	bool found_result = false;
	while(!working_tree.empty()){
		SearchState working_state(*working_tree.front());
		working_tree.pop_front();
		
		auto actions = working_state.actions();
		for(size_t i = actions.size()-1; i >= 0 ; i--){
			auto new_state = actions[i].execute(working_state);
			std::tuple<const SearchState, SearchAction> new_tuple = {working_state,actions[i]};
			auto check = closed.find(new_state);
			if(check == closed.end()){
				closed.insert({new_state,new_tuple});
				working_tree.push_front(std::make_shared<SearchState>(new_state));
			}
			if(new_state.isFinal()){
				found_result = true;
				SearchState working_state(new_state);
				final_state = std::make_shared<SearchState>(working_state);
				break;
			}
		}
		if(found_result)
			break;
	}
	
	std::vector<SearchAction> solution;
	if(found_result){
		std::shared_ptr<const SearchState> cur_state = final_state;
		while(true){
			auto search = closed.find(*cur_state);
			if(search == closed.end()){
				break;
			}
			auto tpl = search->second;
			auto old_state = cur_state;
			cur_state = std::make_shared<const SearchState>(std::get<0>(tpl));
			auto new_action = SearchAction (std::get<1>(search->second));
			solution.push_back(new_action);
			closed.erase(*old_state);
		}
		return std::vector<SearchAction> (solution.rbegin(), solution.rend());
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
