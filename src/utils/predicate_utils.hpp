#ifndef __PREDICATE_UTILS
#define __PREDICATE_UTILS

#include <variant>
#include <vector>
#include <set>
#include <map>

#include "parsetree.hpp"
#include "domain.hpp"
#include "atgraph.hpp"
#include "constraint.hpp"

bool is_same_predicate(std::variant<literal,ground_literal> pred1, std::variant<literal,ground_literal> pred2);
bool is_same_non_ground_predicate(std::pair<literal,std::vector<std::string>> pred1, std::pair<literal,std::vector<std::string>> pred2);
bool check_decomposition_preconditions(std::vector<ground_literal> world_state, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions, std::map<int,std::vector<std::pair<literal,std::vector<std::string>>>> non_ground_world_state, std::pair<int,Decomposition> d_info, std::vector<Constraint> mission_constraints, std::set<int> current_decomposition, std::set<std::string> robot_related_sorts);

void apply_effects_in_valid_decomposition(std::vector<ground_literal>& world_state, std::vector<std::pair<ground_literal,variant<int,float>>>& world_state_functions, std::map<int,std::vector<std::pair<literal,std::vector<std::string>>>>& non_ground_world_state, std::pair<std::vector<std::pair<int,ATNode>>,std::set<int>> valid_mission_decomposition,
                                            std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>> effects_to_apply);

std::vector<std::string> get_predicate_argument_types(task t, literal pred);

#endif