#include "search-strategies.h"
#include <queue>
#include <memory>
#include <algorithm> 


std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::queue<std::shared_ptr<SearchState>> q;
	std::map<SearchState, std::shared_ptr<SearchState>> parent;
	std::map<SearchState, SearchAction> actions;

	std::shared_ptr<SearchState> shared_init_state = std::make_shared<SearchState>(init_state);
	q.push(shared_init_state);
	parent[init_state] = std::make_shared<SearchState>(init_state); 
	
	while (!q.empty()) {
		std::shared_ptr<SearchState> current_state = q.front();
		q.pop();

		for (auto action : current_state->actions()) {
			std::shared_ptr<SearchState> new_state = std::make_shared<SearchState>(action.execute(*current_state));
			// If the key is not present, find returns an iterator to end
			if (parent.find(*new_state) == parent.end()) {
				q.push(new_state);
				parent[*new_state] = current_state;
				actions.insert(std::pair<SearchState, SearchAction>(*new_state, action));
			}
			
			if (new_state->isFinal()) {
				std::vector<SearchAction> solution;
				// Reconstruct the path of actions that led to the final state
				while (*new_state < init_state) {
					solution.push_back(actions.at(*new_state));
					new_state = parent[*new_state];
				}
				return solution;
			}

		}
	}
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
