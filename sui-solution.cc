#include "search-strategies.h"
#include <deque>
std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::map<const SearchState, std::tuple<const SearchState, SearchAction>>  closed;
	bool processing_odd = true;
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
	bool processing_odd = true;
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
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
