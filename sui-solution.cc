#include "search-strategies.h"
#include <queue>
#include <set>
#include <limits>
#include <memory>
#include <algorithm> 
#include <stack>
#include "memusage.h"


std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::queue<std::shared_ptr<SearchState>> q;
	std::map<SearchState, std::shared_ptr<SearchState>> parent;
	std::map<SearchState, SearchAction> actions;

	std::shared_ptr<SearchState> shared_init_state = std::make_shared<SearchState>(init_state);
	q.push(shared_init_state);
	parent[init_state] = std::make_shared<SearchState>(init_state); 
	
	while (!q.empty()) {
		if (getCurrentRSS() > mem_limit_ * 0.9)
			return {};

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
				while (new_state != shared_init_state) {
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
	std::stack<std::shared_ptr<SearchState>> s;
	struct node_info {
		std::shared_ptr<SearchAction> action;
		std::shared_ptr<SearchState> parent;
		int depth;
	};
	std::map<SearchState, struct node_info> info;

	std::shared_ptr<SearchState> shared_init_state = std::make_shared<SearchState>(init_state);
	s.push(shared_init_state);
	info[init_state].action = nullptr;
	info[init_state].parent = shared_init_state;
	info[init_state].depth = 0;

	while (!s.empty()) {
		std::shared_ptr<SearchState> current_state = s.top();
		s.pop();

		int curr_depth = info[*current_state].depth;
		if (curr_depth <= depth_limit_) {
			int new_depth = curr_depth + 1;

			for (auto action : current_state->actions()) {
				std::shared_ptr<SearchState> new_state = std::make_shared<SearchState>(action.execute(*current_state));
				
				// Do not push states that were already seen
				if (info.find(*new_state) == info.end()) {
					s.push(new_state);
					struct node_info n_info = {
						std::make_shared<SearchAction>(action),
						current_state,
						new_depth
					};
					info.insert(std::pair<SearchState, node_info>(*new_state, n_info));
				}

				if (new_state->isFinal()) {
					std::vector<SearchAction> solution;
					// Reconstruct the path of actions that led to the final state
					while (new_state != shared_init_state) {
						solution.push_back(*(info.at(*new_state).action));
						new_state = info[*new_state].parent;
						// std::cout << *new_state;
						// std::cout << *(info.at(*new_state).action) << std::endl << std::endl;
					}
					std::reverse(solution.begin(), solution.end());
					return solution;
				}
			}
		}
	}

	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
	int stack_sum = 0;
	for (auto stack: state.stacks) {
		std::vector<Card> cards = stack.storage();			
		for (long unsigned i = 0; i + 1 < cards.size(); i++) {
			long unsigned depth = cards.size() - i;
			if (cards.at(i).value == cards.at(i + 1).value + 1) {
				if (render_color_map.at(cards.at(i).color) == render_color_map.at(cards.at(i + 1).color)) {
					stack_sum += 2;
				}
				else {
					stack_sum -= depth;
				}
			}
			else {
				if (cards.at(i).value < cards.at(i + 1).value) {
					stack_sum += (cards.at(i + 1).value - cards.at(i).value - 1) * depth;
				}
				else {
					stack_sum += cards.at(i).value - cards.at(i + 1).value + 1;
				}
			}
		}
	}

	for (const auto &cell: state.free_cells) {
		auto cell_top = cell.topCard();
		if (cell_top.has_value())
			stack_sum += 7;
	}

    for (const auto &home : state.homes) {
        auto home_top = home.topCard();
        if (home_top.has_value())
            stack_sum -= home_top->value * 13;
    }
    return stack_sum;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	std::set<std::shared_ptr<SearchState>> open;
	struct node_info {
		std::shared_ptr<SearchAction> action;
		std::shared_ptr<SearchState> parent;
		int depth;
	};
	std::map<std::shared_ptr<SearchState>, struct node_info> info;
	std::set<SearchState> closed;
	std::shared_ptr<SearchState> shared_init_state = std::make_shared<SearchState>(init_state);

	open.insert(shared_init_state);
	info[shared_init_state].parent = std::make_shared<SearchState>(init_state);
	info[shared_init_state].action = nullptr;
	info[shared_init_state].depth = 0;
	
	while (!open.empty()) {
		if (getCurrentRSS() > mem_limit_ * 0.9)
			return {};

		int best = std::numeric_limits<int>::max();
		std::shared_ptr<SearchState> best_choice = nullptr;

		// Looking for best candidate for expanding based on heuristic
		for (std::shared_ptr<SearchState> state_ptr: open) {
			int h = compute_heuristic(*state_ptr, *heuristic_);
			int g = info[state_ptr].depth;
			if (h + g < best) {
				best = h + g;
				best_choice = state_ptr;
			}
		}

		open.erase(best_choice);
		closed.insert(*best_choice);
		int new_depth = info[best_choice].depth + 1;

		for (auto action : best_choice->actions()) {
			std::shared_ptr<SearchState> new_state = std::make_shared<SearchState>(action.execute(*best_choice));
			// If the key is not present, find returns an iterator to end
			if (closed.find(*new_state) == closed.end()) {
				open.insert(new_state);
				struct node_info n_info = {
					std::make_shared<SearchAction>(action),
					best_choice,
					new_depth
				};
				info.insert(std::pair<std::shared_ptr<SearchState>, node_info>(new_state, n_info));
			}
			
			if (new_state->isFinal()) {
				std::vector<SearchAction> solution;
				// Reconstruct the path of actions that led to the final state
				while (new_state != shared_init_state) {
					solution.push_back(*(info.at(new_state).action));
					new_state = info[new_state].parent;
				}
				// reverse the solution
				std::reverse(solution.begin(), solution.end());
				return solution;
			}
		}
	}
	return {};
}
