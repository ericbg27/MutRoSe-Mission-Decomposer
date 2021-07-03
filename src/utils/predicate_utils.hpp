#ifndef __PREDICATE_UTILS
#define __PREDICATE_UTILS

#include <variant>
#include <vector>
#include <set>
#include <map>

#include "parsetree.hpp"
#include "domain.hpp"
#include "atgraph.hpp"

bool is_same_predicate(std::variant<literal,ground_literal> pred1, std::variant<literal,ground_literal> pred2);
bool check_decomposition_preconditions(std::vector<ground_literal> world_state, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions, Decomposition d);

void apply_effects_in_valid_decomposition(std::vector<ground_literal>& world_state, std::vector<std::pair<ground_literal,variant<int,float>>>& world_state_functions, std::pair<std::vector<std::pair<int,ATNode>>,std::set<int>> valid_mission_decomposition, 
                                            std::map<int,std::vector<std::variant<ground_literal,std::pair<ground_literal,variant<int,float>>>>> effects_to_apply);

#endif