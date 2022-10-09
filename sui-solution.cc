#include "search-strategies.h"
#include <queue>
#include <stack>
#include <set>
#include "memusage.h"
#include <optional>
#include <climits>

// Structure for BFS and DFS
typedef struct
{
	std::shared_ptr<SearchState> parent; // Pointer to parent SearchState
	SearchAction parent_act;			 // Action which is used on parent SearchState to get current State
	int depth;							 // Used to track depth in Tree (only in DFS)
} Node;

// Structures used for priority queue in A star algorithm
// and its assembly
typedef struct
{
	std::shared_ptr<SearchState> parent;
	SearchAction parent_act;
} Node_Assembly;

struct Node_Queue
{
	double value;
	int depth;
	std::shared_ptr<SearchState> parent;
	bool operator<(const Node_Queue &rhs) const
	{
		return value > rhs.value;
	}
};

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state)
{
	std::set<SearchState> closed;					   // Closed list for SearchStates
	std::queue<std::shared_ptr<SearchState>> open;	   // Open Queue for Searchstates (not expanded nodes)
	std::map<std::shared_ptr<SearchState>, Node> tree; // SearchState tree

	if (init_state.isFinal())
	{
		return {};
	}

	bool reached_final = false;

	std::shared_ptr<SearchState> parent_state = std::make_shared<SearchState>(init_state);

	open.push(parent_state); // Pushing initial state to open list

	auto old_memory = getCurrentRSS();

	while (!open.empty() && !reached_final)
	{
		/* Getting SearchState from top of Queue */
		auto current_parent = open.front();
		SearchState working_state(*current_parent);
		open.pop();

		auto actions = working_state.actions();
		/* Tracking memory */
		auto taken_memory = getCurrentRSS();
		if ((taken_memory - old_memory) * 4 + taken_memory > mem_limit_)
		{
			//  Taken memory + 4 * difference between last round and current round
			return {};
		}
		old_memory = taken_memory;

		for (auto act : actions)
		{
			auto new_state = act.execute(working_state);
			if (closed.count(new_state) == 0)
			{ // if state is in closed, dont do anything
				closed.insert(new_state);
				auto new_shared = std::make_shared<SearchState>(new_state);
				open.push(new_shared);
				// Generating new node to the tree
				Node parent_node = {current_parent, act, 0};
				tree.insert({new_shared, parent_node});

				if (new_state.isFinal())
				{
					reached_final = true;
					parent_state = new_shared;
					break;
				}
			}
		}
	}

	if (reached_final)
	{
		/* Backtracking the result from the final node */
		std::vector<SearchAction> solution;
		while (true)
		{
			auto tree_find = tree.find(parent_state);
			if (tree_find == tree.end())
			{
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

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state)
{

	std::set<SearchState> closed;					   // Closed list for SearchStates
	std::stack<std::shared_ptr<SearchState>> open;	   // Open Stack for Searchstates (not expanded nodes)
	std::map<std::shared_ptr<SearchState>, Node> tree; // SearchState tree

	if (init_state.isFinal())
	{
		return {};
	}

	bool reached_final = false;
	std::shared_ptr<SearchState> parent_state = std::make_shared<SearchState>(init_state);
	int current_depth = 0;
	/** Inserting initial node **/
	Node init_node = {parent_state, init_state.actions()[0], 0};
	open.push(parent_state);
	tree.insert({parent_state, init_node});

	while (!open.empty())
	{
		/* Poping from the stack */
		auto current_parent = open.top();
		SearchState working_state(*current_parent);
		current_depth = tree.find(current_parent)->second.depth; // getting the depth of parent node
		open.pop();
		if (current_depth >= depth_limit_)
		{
			continue; // skipping the node expansion
		}

		auto actions = working_state.actions();
		for (auto act : actions)
		{
			auto new_state = act.execute(working_state);
			if (closed.count(new_state) == 0)
			{
				closed.insert(new_state);
				auto new_shared = std::make_shared<SearchState>(new_state);
				open.push(new_shared);
				Node parent_node = {current_parent, act, current_depth + 1}; // incrementing depth
				tree.insert({new_shared, parent_node});
				if (new_state.isFinal())
				{
					reached_final = true;
					parent_state = new_shared;
					break;
				}
			}
		}
		if (reached_final)
		{
			break;
		}
	}

	if (reached_final)
	{
		/* Backtracking the result */
		std::vector<SearchAction> solution;
		while (true)
		{
			auto tree_find = tree.find(parent_state);
			if (tree_find->second.depth == 0)
			{
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

double StudentHeuristic::distanceLowerBound(const GameState &state) const
{
	double number_of_cards_to_free = 0;
	double number_of_cards_in_foundation = 0;
	// The normal value 52 was picked by calculating this value
	// this can vary by number of stacks
	int cards_out_of_home = king_value * colors_list.size();

	int free_spaces = 0;
	// Count and remember the free spaces
	// if there is a card, try to play it
	std::vector<std::optional<Card>> free_cards = {};
	for (const auto &free : state.free_cells)
	{
		if (free.topCard() == std::nullopt)
		{
			free_spaces++;
		}
		else
		{
			free_cards.push_back(free.topCard());
		}
	}

	for (const auto &home : state.homes)
	{
		auto opt_top = home.topCard();
		// Check how many cards are in foundation
		if (opt_top.has_value())
			number_of_cards_in_foundation += opt_top->value;
		// Check if you can play card from free cell
		// and if yes, you should play it
		for (const auto &free_card : free_cards)
		{
			if (free_card.has_value() == opt_top->value + 1)
			{
				return cards_out_of_home - number_of_cards_in_foundation;
			}
		}

		// Check how many card are needed for next move and play
		// the move with the least amount of moves
		// also don't play a move when you know that there is a better one
		int best_move = INT_MAX;
		for (const auto &stack : state.stacks)
		{
			// Take a new card, if home stack cannot accept it
			// take next one and add counter
			int count = 0;
			for (int i = stack.storage().size(); i > 0; i--)
			{
				auto new_card = stack.storage()[i];
				//                printf("%d and %d\n", stack.storage()[i].value, stack.storage().back().value);
				//                printf("%b a %d|", home.canAccept(new_card), count);
				if (home.canAccept(new_card))
					break;
				count++;
			}
			if (count < best_move)
			{
				best_move = count;
			}
		}
		number_of_cards_to_free = best_move;
	}
	//    printf("%d", number_of_cards_to_free);
	return (cards_out_of_home - number_of_cards_in_foundation) * state.homes.size() + number_of_cards_to_free;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state)
{
	if (init_state.isFinal())
		return {};

	std::set<SearchState> closed;
	std::priority_queue<Node_Queue> open;
	std::map<std::shared_ptr<SearchState>, Node_Assembly> tree;

	bool reached_final = false;
	double initial_value = 0;
	int current_depth = 0;

	// Initialized variables and push them into specified lists
	std::shared_ptr<SearchState> parent_state = std::make_shared<SearchState>(init_state);
	Node_Assembly init_node = {parent_state, init_state.actions()[0]};
	Node_Queue init_queue = {initial_value, current_depth, parent_state};
	open.push(init_queue);
	tree.insert({parent_state, init_node});

	auto old_memory = getCurrentRSS();

	while (!open.empty() && !reached_final)
	{
		std::shared_ptr<SearchState> current_parent = open.top().parent;
		current_depth = open.top().depth;

		open.pop();
		SearchState working_state(*current_parent);

		std::vector<SearchAction> actions = working_state.actions();

		/* Tracking memory */
		auto taken_memory = getCurrentRSS();
		if ((taken_memory - old_memory) * 4 + taken_memory > mem_limit_)
		{
			//  Taken memory + 4 * difference between last round and current round
			// std::cout << "1";
			// std::cout.flush();
			return {};
		}
		old_memory = taken_memory;

		for (auto act : actions)
		{
			SearchState new_state = act.execute(working_state);

			if (closed.count(new_state) == 0)
			{
				closed.insert(new_state);

				// Use heuristics to compute new h, which will sort the values in the priority queue
				std::shared_ptr<SearchState> new_shared = std::make_shared<SearchState>(new_state);
				double h = current_depth + compute_heuristic(new_state, *heuristic_);
				open.push({h, current_depth + 1, new_shared});

				Node_Assembly parent_node = {current_parent, act};
				tree.insert({new_shared, parent_node});
				if (new_state.isFinal())
				{
					reached_final = true;
					parent_state = new_shared;
					break;
				}
			}
		}
	}

	if (reached_final)
	{
		/* Backtracking the result from the final node */
		std::vector<SearchAction> solution;
		while (true)
		{
			auto tree_find = tree.find(parent_state);
			if (tree_find->second.parent == parent_state)
			{
				break;
			}
			auto node = tree_find->second;
			solution.insert(solution.begin(), node.parent_act);
			parent_state = node.parent;
		}
		// std::cout << "0";
		// std::cout.flush();
		return solution;
	}
	// std::cout << "1";
	// std::cout.flush();
	return {};
}
